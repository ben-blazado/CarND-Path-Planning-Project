#include "localization.h"
#include "helpers.h"
#include "spline.h"
#include "ticker.h"

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
    Map& map, double max_s, double secs_per_update) 
    : behavior_(behavior), trajectory_(trajectory), map_(map) {
          
  max_s_ = max_s;
  dt_ = secs_per_update;
  
  processing_  = false;
  updated_     = false;
  buf_.updated = false;

  return;
}

Localization::~Localization() {
  
  if (processing_) {
    processing_ = false;
    if (thread_.joinable())
      thread_.join();
  }

}

void Localization::Receive (double car_x, double car_y, 
      double car_yaw, double car_speed, 
      const vector<double>& previous_path_x, 
      const vector<double>& previous_path_y) {

  if (buf_.m.try_lock()) {
    
    buf_.car_x        = car_x;
    buf_.car_y        = car_y;
    buf_.car_yaw      = car_yaw;
    buf_.car_speed    = car_speed;
    buf_.prev_path_x  = previous_path_x;
    buf_.prev_path_y  = previous_path_y;
    buf_.updated      = true;
    cout << "Localization::Update() " << endl;
    cout << "previous_path_x " << previous_path_x.size() << endl;
    
    buf_.m.unlock();
  }
  
  return;
}

void Localization::Update()
{
  buf_.m.lock();
  
  updated_ = buf_.updated;
  if (updated_) {
    
    car_x_       = buf_.car_x;
    car_y_       = buf_.car_y;
    car_yaw_     = buf_.car_yaw;
    car_speed_   = buf_.car_speed;
    prev_path_x_ = buf_.prev_path_x;
    prev_path_y_ = buf_.prev_path_y;
    
    buf_.updated = false;
  }
  
  buf_.m.unlock();
  
  return;
}
 
void Localization::ProcessUpdates () {

  while (processing_) {

    Update();

    if (updated_) {
      
      // calculate last point in frenet coordinates from previous path 
      Cartesian         last_p;  // last point from previous path
      Kinematic<Frenet> last_f;  // last point in frenet from previous path
      double ds;  // s distance for calculate v along s
      double dd;  // d distance for calculate v along d
      
      int last = prev_path_x_.size() - 1;

      if (last >= 0)
        last_p = {prev_path_x_[last], prev_path_y_[last]};
      else 
        last_p = {car_x_, car_y_};
      
      cout << "Localization::ProcessUpdates() : prev_p " << last_p.x << " " << last_p.y << endl;
      last_f.p = map_.CalcFrenet(last_p);
      
      // calculate ds and dd
      if (last > 0) {
        // use last 2 to calculate last car heading and veolocity
        Cartesian prev_last_p;
        prev_last_p.x      = prev_path_x_[last-1];
        prev_last_p.y      = prev_path_y_[last-1];
        Frenet prev_last_f = map_.CalcFrenet(prev_last_p);
          
        // calculate rest of frenet kinematics from previous path
        ds = last_f.p.s - prev_last_f.s;
        dd = last_f.p.d - prev_last_f.d;
        
      } 
      else {
        // project next point to calculate to have two points
        Cartesian next_p;
        next_p.x = last_p.x + car_speed_*cos(car_yaw_)*dt_;
        next_p.y = last_p.y + car_speed_*sin(car_yaw_)*dt_;;
        
        Frenet next_f = map_.CalcFrenet(next_p);
        ds = next_f.s - last_f.p.s;
        dd = next_f.d - last_f.p.d;
      }
      
      // calculate rest of kinematics
      last_f.v.s = ds / dt_;
      last_f.v.d = dd / dt_;
      last_f.a.s = 0;
      last_f.a.d = 0;
      
      cout << "Localization::ProcessUpdates() " << endl;
      // use prev_f as starting point for behavior module
      if (prev_path_x_.size() < 150)
        behavior_.Receive(last_f);
      trajectory_.Receive(prev_path_x_, prev_path_y_);
      
      updated_ = false;
      
    }
  }  // while
}
 
void Localization::Run() {
  
  processing_ = true;
  thread_ = thread( [this] {ProcessUpdates ();} ); // thread
    
  return;
}


          /*
          // calculate seconds since last update
          static time_point prev_tp = localization_data_.tp;
          double secs = DiffTimePoint(localization_data_.tp, prev_tp);
          cout << "diff time update " << secs << endl;
          
          double theta = deg2rad(localization_data_.car_yaw);  // car yaw in radians
          
          // update kinetics on cartesian coordinates
          double prev_x_v = cart_.x.v;  
          cart_.x.p = localization_data_.car_x;
          cart_.x.v = localization_data_.car_speed * cos(theta);
          cart_.x.a = (cart_.x.v - prev_x_v) / secs;
          
          double prev_y_v = cart_.y.v;
          cart_.y.p = localization_data_.car_y,
          cart_.y.v = localization_data_.car_speed * sin(theta);
          cart_.y.a = (cart_.y.v - prev_y_v) / secs;
          
          // update kinetics in frenet system
          // calculate next sd position from next xy position
          // use next sd position to calculate velocity and acc in frenet system
          double  next_x = cart_.x.p + cart_.x.v*secs;
          double  next_y = cart_.y.p + cart_.y.v*secs;
          CartP   next_p = {next_x, next_y};
          FrenetP next_sd = CalcSD(next_p);
          
          double prev_s_v = frenet_.s.v;
          frenet_.s.p = localization_data_.car_s;
          frenet_.s.v = (next_sd.s - frenet_.s.p) / secs;
          frenet_.s.a = (frenet_.s.v - prev_s_v) / secs;
          
          double prev_d_v = frenet_.d.v;
          frenet_.d.p = localization_data_.car_d;
          frenet_.d.v = (next_sd.d - frenet_.d.p) / secs;
          frenet_.d.a = (frenet_.d.v - prev_d_v) / secs;
          
          // behavior.Update(frenet_);
          
          updated_ = false;
          prev_tp = localization_data_.tp;          
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