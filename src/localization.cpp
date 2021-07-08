#include "localization.h"
#include "helpers.h"
#include "spline.h"
#include "buffer.h"

#include <thread>
#include <mutex>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <cmath>
#include <limits>

namespace PathPlanning {

using std::unique_lock;
using std::defer_lock;
using std::mutex;
using std::cout;
using std::endl;

Localization::Localization(Behavior& behavior, Trajectory& trajectory, 
    Map& map, double secs_per_update) 
    : behavior_ (behavior), trajectory_(trajectory), map_ (map) {
          
  dt_ = secs_per_update;
  processing_  = false;

  return;
}

Localization::~Localization() {
  
  if (processing_) {
    processing_ = false;
    if (thread_.joinable())
      thread_.join();
  }

}

// Ensures prev path has at least 2 waypoints.
// This prepares prev_path to be used in CalcLastPos, CalcLastVel, CalcLastAcc.
void Localization::VerifyPrevPath(InputData& in) {
  
  // verify that prev_path has at least 2 waypoints
  int num_waypoints = in.prev_path_x.size();
  
  if (num_waypoints < 2) {
    
    if (num_waypoints == 0) {
      
      // prev path has 0 waypoints in prev_path!
      // use current car position for first way point.
      in.prev_path_x.push_back(in.car_x);
      in.prev_path_y.push_back(in.car_y);
    }
    
    // add second waypoint based on car location, speed and yaw.
    Cartesian next_p;
    double car_speed = in_.car_speed*dt_;
    next_p.x = in.prev_path_x[0] + car_speed*cos(in.car_yaw);
    next_p.y = in.prev_path_y[0] + car_speed*sin(in.car_yaw);
    
    in.prev_path_x.push_back(next_p.x);
    in.prev_path_y.push_back(next_p.y);
  }
  
  cout << "Localization::ProcessUpdates() " << endl;
  cout << "Localization::should be 2 or more " << in_.prev_path_x.size() << endl;
  
  return;
}


void Localization::Input (InputData& in) {
  
  VerifyPrevPath(in);
  
  Trajectory::LocalizationInput loc_in = {in.prev_path_x, in.prev_path_y};
  trajectory_.Input(loc_in);

  in_buf_.Write(in);
  
  return;
}


// Returns frenet position of last waypoint in prev_path.
// Used in ProcessUpdates() to help calculate frenet kinematics
// of starting position for Behavior module.
Frenet Localization::CalcLastPos() {
  
  // index of last waypoint
  int last = in_.prev_path_x.size() - 1;

  // get last waypoint in cartesian
  Cartesian last_p = {in_.prev_path_x[last], in_.prev_path_y[last]};
  
  // covert last waypoint from cartesian point to frenet
  Frenet last_f = map_.CalcFrenet(last_p);
  
  cout << "Localization::last_f " << last_f.s() << " " << last_f.d() << endl;
  
  return last_f;
}


// Returns the Frenet velocity compenents of the last waypoint in prev_path_.
// Call after CalcLastPos(). Used in ProcessUpdates() to help calculate 
// frenet kinematics of starting position for Behavior module.
// Call after VerifyPrevPath to ensure prev_path_ has at least 2 waypoints.
//
// last_f : the last frenet position of the last waypoint in prev_path.
Frenet Localization::CalcLastVel(Frenet last_f) {

  int last = in_.prev_path_x.size() - 1;
  
  // use last 2 waypoints to calculate ds and dd, then velocities
  Cartesian prev_last_p;
  prev_last_p.x      = in_.prev_path_x[last-1];
  prev_last_p.y      = in_.prev_path_y[last-1];
  Frenet prev_last_f = map_.CalcFrenet(prev_last_p);
  
  // Calculate ds and dd for use in calculating velocity.
  // double ds = map_.Diff (last_f.s, prev_last_f.s);
  // TODO: check below for accurate difference
  //double ds = last_f.s - prev_last_f.s;
  //double dd = last_f.d - prev_last_f.d;
  Frenet df = last_f - prev_last_f;
  
  // Calculate velocity components using ds, dd, and dt.
  //Frenet last_v;
  //last_v.s = ds / dt_;  // dt_ is num secs per update (normally 0.02).
  //last_v.d = dd / dt_;
  Frenet last_v = df / dt_;
  
  return last_v;
}
      
// Returns the acceleration of the last waypoint in prev_path.
// Call after CalcVel(). Used in ProcessUpdates().
// 
// last_v: Velocity of last waypoint in prev_path. Should be
//         calculated from CalcVel();
Frenet Localization::CalcLastAcc(Frenet last_v) {
  
  // Use first two waypoints to help calculate initial velocities.
  Cartesian first_p  = {in_.prev_path_x[0], in_.prev_path_y[0]};
  Cartesian second_p = {in_.prev_path_x[1], in_.prev_path_y[1]};
  
  // Convert first two waypoints to frenet coordinates.
  Frenet first_f  = map_.CalcFrenet(first_p);
  Frenet second_f = map_.CalcFrenet(second_p);
  
  // Calculate initial velocity along s and d 
  // based on first and second  waypoints.
  //double first_vs = map_.Diff(second_f.s, first_f.s) / dt_;
  //double first_vd = (second_f.d - first_f.d) / dt_;
  Frenet first_v = (second_f - first_f) / dt_;
  
  // Calculate time over first and last velocities (entire path remaining).
  int num_waypoints = in_.prev_path_x.size();
  double dt_acc = num_waypoints*dt_;
  
  // Calculate the acc along s and d using a=dv/dt.
  Frenet acc = (last_v - first_v) / dt_acc;
  //acc.s = (last_v.s - first_vs) / dt_acc;
  //acc.d = (last_v.d - first_vd) / dt_acc;
  
  
  return acc;
}

 
void Localization::ProcessInputs () {

  while (processing_) {

    if (in_buf_.Read(in_)) {
      
      // verify that prev_path has at least 2 waypoints
      //VerifyPrevPath();
      
      Kinematic<Frenet> last_f;  // last point in frenet from previous path
      last_f.p = CalcLastPos();
      last_f.v = CalcLastVel(last_f.p);
      last_f.a = CalcLastAcc(last_f.v);
      
      cout << "Localization::Last s d for input to beh " << last_f.p.s() << endl;
      cout << "Localization::Last v for input to beh " << last_f.v.s()  << endl;
      // use prev_f as starting point for behavior module
      
      // Pass frenet kinematics of last waypoint
      // and number of waypoints of prev_path
      // to behavior module to generate new path.
      int num_waypoints = in_.prev_path_x.size();
      
      //test comment
      
      Behavior::InputData beh_in = {last_f, num_waypoints};
      behavior_.Input(beh_in);
    }
  }  // while
}
 
void Localization::Run() {
  
  processing_ = true;
  thread_ = thread( [this] {ProcessInputs ();} ); // thread
    
  return;
}


          /*
          // calculate seconds since last update
          static time_point prev_tp = localization_in_.tp;
          double secs = DiffTimePoint(localization_in_.tp, prev_tp);
          cout << "diff time update " << secs << endl;
          
          double theta = deg2rad(localization_in_.car_yaw);  // car yaw in radians
          
          // update kinetics on cartesian coordinates
          double prev_x_v = cart_.x.v;  
          cart_.x.p = localization_in_.car_x;
          cart_.x.v = localization_in_.car_speed * cos(theta);
          cart_.x.a = (cart_.x.v - prev_x_v) / secs;
          
          double prev_y_v = cart_.y.v;
          cart_.y.p = localization_in_.car_y,
          cart_.y.v = localization_in_.car_speed * sin(theta);
          cart_.y.a = (cart_.y.v - prev_y_v) / secs;
          
          // update kinetics in frenet system
          // calculate next sd position from next xy position
          // use next sd position to calculate velocity and acc in frenet system
          double  next_x = cart_.x.p + cart_.x.v*secs;
          double  next_y = cart_.y.p + cart_.y.v*secs;
          CartP   next_p = {next_x, next_y};
          FrenetP next_sd = CalcSD(next_p);
          
          double prev_s_v = frenet_.s.v;
          frenet_.s.p = localization_in_.car_s;
          frenet_.s.v = (next_sd.s - frenet_.s.p) / secs;
          frenet_.s.a = (frenet_.s.v - prev_s_v) / secs;
          
          double prev_d_v = frenet_.d.v;
          frenet_.d.p = localization_in_.car_d;
          frenet_.d.v = (next_sd.d - frenet_.d.p) / secs;
          frenet_.d.a = (frenet_.d.v - prev_d_v) / secs;
          
          // behavior.Update(frenet_);
          
          updated_ = false;
          prev_tp = localization_in_.tp;          
          */
          



void Localization::Test() {
  
  /*------------------- 

  
  cout << "test " << endl;
  
  CartP p = {950, 1130};
  vector<double> sd = getFrenet(p.x, p.y, 0, maps_x_, maps_y_);
  
  cout << "sd " << sd[0] << " " << sd[1] << endl;
  
  double lowest_d = std::numeric_limits<double>::infinity();
  int closest;
  
  // find closest map point by distance
  for (int i = 0; i < num_map_points_; i ++) {
    double d = distance(p.x, p.y, maps_x_[i], maps_y_[i]);
    if (d < lowest_d) {
      lowest_d = d;
      closest = i;
    }
  }
  
  cout << "closest i " << closest << endl;
  cout << "closest s " << maps_s_[closest] << endl;

  double precision = 0.000001;
  double delta = std::numeric_limits<double>::infinity();
  
  double s = (maps_s_[closest] + maps_s_[closest - 1]) / 2;
  int i = 0;
  while (abs(delta) > precision) {

    double xs = p.x - spline_sx_(s);
    double ys = p.y - spline_sy_(s);

    double sx_p  = spline_sx_.deriv(1, s);
    double sy_p  = spline_sy_.deriv(1, s);
    
    double sx_pp = spline_sx_.deriv(2, s);
    double sy_pp = spline_sy_.deriv(2, s);

    double D = distance (p.x, p.y, spline_sx_(s), spline_sy_(s));

    //double D_sqr_p = 2*(xs)*(-sx_p) + 2*(ys)*(-sy_p);
    double D_sqr_p = (-2) * (xs*sx_p + ys*sy_p);
    
    double D_p = (1/(2*D)) * (D_sqr_p);
    
    double D_neg_sqrt_prime = -0.5 * D_p / sqrt(D*D*D);
    
    //double D_sqr_pp = 2*(xs*(-sx_pp)+sx_p*sx_p) + 2*(ys*(-sy_pp)+ sy_p*sy_p);
    double D_sqr_pp = -2 * (xs*sx_pp - sx_p*sx_p + ys*sy_pp - sy_p*sy_p);
    
    double D_pp = 0.5*(sqrt(D) * D_sqr_pp + D_neg_sqrt_prime * D_sqr_p);
    
    // double m = (0.5*D_sqr_p/D);
    
    // double m_p = 0.5*(D_sqr_pp / sqrt(D) + D_neg_sqrt_prime*D_sqr_p);
    double m_p = 2*(D*D_pp + D_p*D_p);
    double m = 2*D*D_p;
    
    delta = D_sqr_p/D_sqr_pp;
    
    s = s - delta;
    

    cout << "s  " << s << endl;    
    cout << "m_prime  " << m_p << endl;    
    cout << "delta  " << delta << endl;    
    cout << "Distance   " << D << endl;    
    cout << "----" << D << endl;    
    
    i ++;
    //if (i > 10)
    //      break;
    
  }
  
  
  double d = distance (p.x, p.y, spline_sx_(s), spline_sy_(s));
  cout << "point x y " << p.x << " " << p.y << endl;
  cout << "best s " << s << endl;
  cout << "best x, y " << spline_sx_(s) << " " << spline_sy_(s) << endl;
  cout << "distance " << d << endl;
  cout << "iter i " << i << endl;
  
  double check_x = spline_sx_(s) + d * spline_sdx_(s);
  double check_y = spline_sy_(s) + d * spline_sdy_(s);
  cout << "check " << check_x << " " << check_y << endl;
  
  FrenetP f = CalcSD(p);
  
  cout << "frenet s " << f.s << endl;
  cout << "frenet d " << f.d << endl;
  
  CartP check = CalcXY(f);
  
  cout << "cart x " << check.x << endl;
  cout << "cart y " << check.y << endl;
  


  /*-------------------*/
  
  
  /*-------------------
  vector<double> sx_deriv_zero = spline_sx_.where_deriv_zero();
  vector<double> sy_deriv_zero = spline_sy_.where_deriv_zero();
  
  for (int i = 0; i < sx_deriv_zero.size(); i ++)
    cout << "s where deriv wrt x is 0 " << sx_deriv_zero[i] << endl;
  
  for (int i = 0; i <sx_deriv_zero.size(); i ++)
    cout << "s where deriv wrt y is 0 " << sy_deriv_zero[i] << endl;
  
  vector<double> s_deriv_zero;
  
  s_deriv_zero.insert(s_deriv_zero.end(), sx_deriv_zero.begin(), sx_deriv_zero.end());
  s_deriv_zero.insert(s_deriv_zero.end(), sy_deriv_zero.begin(), sy_deriv_zero.end());
  
  sort(s_deriv_zero.begin(), s_deriv_zero.end());
  

  for (int i = 0; i <s_deriv_zero.size(); i ++)
    cout << "s where deriv is 0 " << s_deriv_zero[i] << endl;
  
  ---------------------*/
  
  
  /*
  int i = 84;
  int j = (i + 1) % num_map_points_;
  
  cout << "Test i, j " << i << " " << j << endl;
  
  double s;
  if (j == 0) {
    s = (kMaxSVal_ + maps_s_[i]) / 2;
    cout << "Max S " << kMaxSVal_ << endl;
    cout << "maps_s_[i] " << maps_s_[i] << endl;
    cout << "s " << s << endl;
  }
  else
    s = (maps_s_[j] + maps_s_[i]) / 2;
  
  FrenetP sd = {s, 0};
  
  cout << "sd " << sd.s << endl;
  
  CartP xy = CalcXY (sd);
  
  FrenetP sd_new = CalcFrenet(xy);
  
  cout << sd.s << " --- " << sd_new.s << endl;
  cout << sd.d << " --- " << sd_new.d << endl;
  
  */

  /*

  double err = 0.0;
  
  for (int i = 0; i < num_map_points_; i ++) {
    
    cout << "------------" << endl;
    
    int j = (i + 1) % num_map_points_;
    
    double s;
    if (j == 0)
      s = (kMaxSVal_ + maps_s_[i]) / 2;
    else
      s = (maps_s_[j] + maps_s_[i]) / 2;
    
    for (int l = 0; l < 18; l ++) {
      
      double d;
      if (l % 2)
        d = -0.3*l;
      else
        d = 0.3*l;
      
      cout << "s d [" << i << "] " << s << ", " << d << endl;
      CartP xy = CalcXY({s, d});
      FrenetP sp_prime = CalcFrenet(xy);
      
      err += (s - sp_prime.s) + (d - sp_prime.d);
      cout << "total err so far " << err << endl;
    }
    if (abs(err) > 0.0001) break;
  }
  
  cout << "------------ total err " << err << endl;
  */
    
  
}

} //namespace PathPlanning