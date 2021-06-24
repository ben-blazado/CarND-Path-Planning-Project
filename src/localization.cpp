#include "localization.h"
#include "helpers.h"

#include <thread>
#include <mutex>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <cmath>
#include <limits>

using std::unique_lock;
using std::defer_lock;
using std::mutex;
using std::cout;
using std::endl;


Origin::Origin (double x, double y, double rotation) {
  
  x_ = x;
  y_ = y;
  rotation_ = rotation;
  
  return;
}

void Origin::Transform(CartP& p) {
  
  // shift point relative to the origin
  p.x -= x_;
  p.y -= y_;
  
  // rotate point about the new orgin
  double sin_rot = sin(rotation_);
  double cos_rot = cos(rotation_);
  
  double x = p.x;
  double y = p.y;
  
  p.x = x*cos_rot - y*sin_rot;
  p.y = x*sin_rot + y*cos_rot;
  
  return;
}
  
  
void Origin::Restore(CartP& p) {
  
  // unrotate point
  double sin_rot = sin(-rotation_);
  double cos_rot = cos(-rotation_);
  
  double x = p.x;
  double y = p.y;
  
  p.x = x*cos_rot - y*sin_rot;
  p.y = x*sin_rot + y*cos_rot;
  
  // unshift point relative to new origin
  p.x += x_;
  p.y += y_;
  
  return;
}
  
  
void Origin::Transform(double& angle) {
  
  angle += rotation_;
  
  return;
}


void Origin::Restore(double& angle) {
  
  angle -= rotation_;
  
  return;
  
}


vector<double> quad_root(double a, double b, double c) {
  
  vector<double> root = {};
  
  double discriminant = b*b - 4*a*c;
  
  if (discriminant >= 0) {
  
    root.push_back ((-b + sqrt(discriminant)) / (2*a));
    root.push_back ((-b - sqrt(discriminant)) / (2*a));
  }
  
  return root;
}
  

vector<double> spline2::where_deriv_zero() {
  
  vector<double> x_where_deriv_zero;
  
  for (size_t i=0; i < m_x.size() - 1; i ++) {
    
    double a = 3*m_d[i];
    double b = 2*m_c[i];
    double c =   m_b[i];
    
    vector<double> root = quad_root(a, b, c);
    if (root.size() > 0)
      for (int r=0; r < 2; r ++) {
        double x_root = m_x[i] + root[r];
        if ((m_x[i] <= x_root) and (x_root < m_x[i+1]))
          x_where_deriv_zero.push_back(x_root);
      }
  }
  
  return x_where_deriv_zero;
}



Localization::Localization(vector<double>& maps_s, vector<double>& maps_x, 
        vector<double>& maps_y, vector<double>maps_dx, vector<double> maps_dy, 
        double secs_per_tick) {
          
  maps_s_ = maps_s;
  maps_x_ = maps_x;
  maps_y_ = maps_y;
  maps_dx_ = maps_dx;
  maps_dy_ = maps_dy;
  
  num_map_points_ = maps_s_.size();
  
  spline_sx_ = spline2(maps_s_, maps_x_);
  spline_sy_ = spline2(maps_s_, maps_y_);
  spline_sdx_ = spline2(maps_s_, maps_dx_);
  spline_sdy_ = spline2(maps_s_, maps_dy_);
  
  // spline_x_ = spline2(maps_x_, maps_y_);
  
  cart_.x.v   = 0;
  cart_.y.v   = 0;
  frenet_.s.v = 0;
  frenet_.d.v = 0;
  
  secs_per_tick_ = secs_per_tick;
  updated_       = false;

  return;
}

Localization::~Localization() {
  
  thread_alive_ = false;
  if (thread_.joinable())
    thread_.join();
}

int Localization::GetStartPoint(double s) {
  
  // use a linear search of maps_s since its pretty small
  // start search from last element and go backward
  int i = num_map_points_ - 1;
  while (maps_s_[i] > s and i > 0) 
    i --;
  
  return i;
}

int Localization::GetStartPoint(CartP p) {
  
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
  
  // use closest map point to find starting point of map segment
  // that contains p
  // set p as local origin rotated by bearing from p to closest point
  CartP closest_p = {maps_x_[closest], maps_y_[closest]};
  double bearing = atan2(closest_p.y - p.y, closest_p.x - p.x);
  Origin origin(p.x, p.y, -bearing);    // closest_p is now in front of p  
  
  // find next_p to closest and transform to local origin p
  int next = (closest + 1) % num_map_points_;
  CartP next_p = {maps_x_[next], maps_y_[next]};
  origin.Transform(next_p);
  
  int start;
  // check if the segment [closest, next_p] contains p
  if (next_p.x > 0) {
    // the segment [closest, next_p] can not contain p
    // select prev to closest as start point
    // https://stackoverflow.com/a/33664449
    start = (num_map_points_ + closest - 1) % num_map_points_;
  }
  else
    // the segment [closest, next_p] must contain p
    // select closest point as start point
    start = closest;
  
  return start; // closest point behind p
}

 
CartP Localization::CalcXY(FrenetP f) {
  
  CartP p;
  
  p.x = spline_sx_(f.s) + f.d*spline_sdx_(f.s);
  p.y = spline_sy_(f.s) + f.d*spline_sdy_(f.s);
  
  return p;
}

    
FrenetP Localization::CalcSD(CartP p) {
  
  FrenetP f;
  
  int start_point = GetStartPoint(p);
  
  double s = maps_s_[start_point];
  
  double precision = 0.000000001;
  double delta = std::numeric_limits<double>::infinity();
  
  int i = 0;
  while (abs(delta) > precision) {

    double xs = p.x - spline_sx_(s);
    double ys = p.y - spline_sy_(s);

    double sx_p  = spline_sx_.deriv(1, s);
    double sy_p  = spline_sy_.deriv(1, s);
    
    double sx_pp = spline_sx_.deriv(2, s);
    double sy_pp = spline_sy_.deriv(2, s);
    
    double D_sqr_p  = (-2) * (xs*sx_p + ys*sy_p);
    double D_sqr_pp = (-2) * (xs*sx_pp - sx_p*sx_p + ys*sy_pp - sy_p*sy_p);
    
    //double D = distance (p.x, p.y, spline_sx_(s), spline_sy_(s));    
    //double D_sqr_p = 2*(xs)*(-sx_p) + 2*(ys)*(-sy_p);
    //double D_p = (1/(2*D)) * (D_sqr_p);
    //double D_neg_sqrt_prime = -0.5 * D_p / sqrt(D*D*D);
    //double D_sqr_pp = 2*(xs*(-sx_pp)+sx_p*sx_p) + 2*(ys*(-sy_pp)+ sy_p*sy_p);
    //double D_pp = 0.5*(sqrt(D) * D_sqr_pp + D_neg_sqrt_prime * D_sqr_p);
    //double m = (0.5*D_sqr_p/D);
    //double m_p = 0.5*(D_sqr_pp / sqrt(D) + D_neg_sqrt_prime*D_sqr_p);
    //double m_p = 2*(D*D_pp + D_p*D_p);
    //double m = 2*D*D_p;
    
    delta = D_sqr_p/D_sqr_pp;
    
    s = s - delta;
    
    cout << "s  " << s << endl;    
    cout << "delta  " << delta << endl;    
    cout << "----" << endl;    
    
    i ++;
    if (i > 10) {
      cout << "s did not converge." << endl;
      break;
    }
    
  }
  
  f.s = s;
  f.d = distance (p.x, p.y, spline_sx_(s), spline_sy_(s));
 
  return f;
}


FrenetK Localization::frenet() {
  
  unique_lock<mutex> u_lock(mutex_, defer_lock);
  FrenetK frenet;
  
  u_lock.lock();
  frenet = frenet_;
  u_lock.unlock();
  
  return frenet;
}


void Localization::Update (double car_x, double car_y, double car_s, double car_d,
        double car_yaw, double car_speed) {

  unique_lock<mutex> u_lock(mutex_, defer_lock);
  
  if (u_lock.try_lock()) {
    time_point now = GetTimePoint();
    localization_data_ = LocalizationData{car_x, car_y, car_s, car_d, 
        car_yaw, car_speed, now};
    updated_ = true;
    u_lock.unlock();
  }
  
  return;
}
  

void Localization::Run () {
  
  thread_alive_ = true;
  
  thread_ = thread( [this] {
    
    unique_lock<mutex> u_lock(mutex_, defer_lock);
    
    while (thread_alive_) {
      
      if (u_lock.try_lock()) { 
        if (updated_) {
          
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
          
        }
        u_lock.unlock();
      }
    }  // while
  }); // thread
    
  return;
}


void Localization::Test() {
  
  /*-------------------  */

  
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
    
    double m = (0.5*D_sqr_p/D);
    
    double m_p = 0.5*(D_sqr_pp / sqrt(D) + D_neg_sqrt_prime*D_sqr_p);
    // double m_p = 2*(D*D_pp + D_p*D_p);
    // double m = 2*D*D_p;
    
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
  cout << "check " << check_x << " " << check_y;
  
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