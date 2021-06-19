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
  
  cout << "rotation " << rotation_ << endl;
  
  double x = p.x;
  double y = p.y;
  
  p.x = x*cos_rot - y*sin_rot;
  p.y = x*sin_rot + y*cos_rot;
  
  cout << "px " << p.x << endl;
  cout << "py " << p.y << endl;
  
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


Localization::Localization(vector<double>& maps_s, vector<double>& maps_x, 
        vector<double>& maps_y, vector<double>maps_dx, vector<double> maps_dy, 
        double secs_per_tick) {
          
  maps_s_ = maps_s;
  maps_x_ = maps_x;
  maps_y_ = maps_y;
  maps_dx_ = maps_dx;
  maps_dy_ = maps_dy;
  
  num_map_points_ = maps_s_.size();
  
  cout << std::fixed << std::setprecision(12);
  
  cout << "initializing loc mappoints " << num_map_points_ << endl; 
  for (int i = 0; i < num_map_points_; i ++) {
    
    int j = (i + 1) % num_map_points_;
    
    // origin_
    // generate local origins at each s position
    double start_normal = atan2(maps_dy_[i], maps_dx_[i]);
    double start_theta  = start_normal + pi()/2.0;
    Origin origin(maps_x[i], maps_y[i], -start_theta);
    origin_.push_back(origin);
    
    // direction_
    double end_normal = atan2(maps_dy_[j], maps_dx_[j]);
    double end_theta =  end_normal + pi()/2.0;   
    origin.Transform(end_theta);
    end_theta = fmod(end_theta, 2*pi());
    if (fabs(end_theta) > pi()) {
      cout << "end theta " << rad2deg(end_theta) << endl;
      if (end_theta > 0)
        end_theta = 2.0*pi() - end_theta;
      else
        end_theta = 2.0*pi() + end_theta;
      cout << "end theta " << rad2deg(end_theta) << endl;
    }
    Direction direction;
    if (0 > end_theta) 
      direction = kRight;
    else if (0 == end_theta) 
      direction = kStraight;
    else  // end_theta < 0
      direction = kLeft;
    direction_.push_back(direction);
      
    // end_theta_
    if (direction == kRight)
      end_theta *= -1.0;
    end_theta_.push_back(end_theta);
    
    //end_point_
    CartP end_point = {maps_x[j], maps_y[j]};
    origin.Transform(end_point);
    if (direction == kRight)
      end_point.y *= -1;
    end_point_.push_back(end_point);
    
    // yaw_rate_
    double s_ij;
    if (j == 0)   // implies that i == num_map_points_ - 1 (last map point)
      s_ij = kMaxSVal_ - maps_s_[i];
    else
      s_ij = maps_s_[j] - maps_s_[i];
    double yaw_rate = end_theta / s_ij;
    yaw_rate_.push_back(yaw_rate);

    // arc_vel_ (euclidean distance per degree)
    double dist_ij = distance (0, 0, end_point.x, end_point.y);
    double degs_ij = end_theta;
    double arc_vel = dist_ij / degs_ij;
    arc_vel_.push_back(arc_vel);

    // vel_ (euclidean distance per frenet s distance)
    double vel = dist_ij / s_ij;
    vel_.push_back(vel);
    
  }
  
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
  
  cout << "Get Start Point " << endl;
  cout << "s " << s << endl;
  int i = num_map_points_ - 1;
  while (maps_s_[i] > s and i > 0) 
    i --;
  
  cout << "closest i " << i << endl;
  
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
  
  cout << "closest i " << closest << endl;
  
  // use closest map point to find starting point of map segment
  // that contains p
  // set p as local origin rotated by bearing from p to closest point
  CartP closest_p = {maps_x_[closest], maps_y_[closest]};
  double bearing = atan2(closest_p.y - p.y, closest_p.x - p.x);
  cout << "bearing to closest point (deg) " << rad2deg(bearing) << endl;
  Origin origin(p.x, p.y, -bearing);    // closest_p is now in front of p  
  
  // find next_p to closest and transform to local origin p
  int next = (closest + 1) % num_map_points_;
  cout << "next point to try " << next << endl;
  CartP next_p = {maps_x_[next], maps_y_[next]};
  origin.Transform(next_p);
  
  int start;
  // check if the segment [closest, next_p] contains p
  if (next_p.x > 0) {
    cout << "next point is in front of p" << endl;
    // the segment [closest, next_p] does not contain p
    // select prev to closest as start point
    // https://stackoverflow.com/a/33664449
    start = (num_map_points_ + closest - 1) % num_map_points_;
  }
  else
    // the segment [closest, next_p] contains p
    // select closest point as start point
    start = closest;
  
  return start; // closest point behind p
}

 
CartP Localization::CalcXY(FrenetP frenet_p) {
  
  cout << "----- CalcXY ----- " << endl;

  CartP cart_p;
  
  int i = GetStartPoint(frenet_p.s);
  cout << "start point i " << i << endl;  
  
  switch (direction_[i]) {
    case kRight:
    case kLeft: {
      
      // calculate theta_f
      double s_f = frenet_p.s - maps_s_[i];
      double theta_f = yaw_rate_[i] * s_f;
      
      cout << "theta_f " << theta_f << endl;
      
      double sin_theta_f = sin(theta_f);
      double cos_theta_f = cos(theta_f);
      
      cout << "D" << distance(0, 0, end_point_[i].x, end_point_[i].y) << endl;
      cout << "end theta " << end_theta_[i] << endl;
      cout << "alpha (arcvel) " << arc_vel_[i] << endl;
      // find x,y on ref line based on theta_f
      double x_ref = arc_vel_[i] * sin_theta_f;
      double y_ref = arc_vel_[i] * (1 - cos_theta_f);
      
      cout << "x_ref " << x_ref << endl;
      cout << "y_ref " << y_ref << endl;
      
      // find x,y from ref line based on theta_f and d
      switch (direction_[i]) {
        case kRight:
          cout << "right" << endl;
          cart_p.x = x_ref - frenet_p.d * sin_theta_f;
          cart_p.y = (-1) * (y_ref + frenet_p.d * cos_theta_f);
          cout << "cart_p.x before transform " << cart_p.x << endl;
          cout << "cart_p.y before transform " << cart_p.y << endl;
          break;
        case kLeft:
          cout << "left" << endl;
          cart_p.x = x_ref + frenet_p.d * sin_theta_f;
          cart_p.y = y_ref - frenet_p.d * cos_theta_f;
          break;
      }
      break;
    }
    case kStraight:
      cout << "straight" << endl;
      cart_p.x = (frenet_p.s - maps_s_[i]) * vel_[i];
      cart_p.y = -frenet_p.d;
      break;
  }
  
  origin_[i].Restore(cart_p);
  cout << "cart p x " << cart_p.x << endl;
  cout << "cart p y " << cart_p.y << endl;
  
  return cart_p;
}

    
FrenetP Localization::CalcFrenet(CartP cart_p) {
  
  cout << "----- CalcFrenet ----- " << endl;
  
  FrenetP frenet_p;
  
  int i = GetStartPoint(cart_p);
  
  cout << "start point i " << i << endl;
  cout << "cart p x " << cart_p.x << endl;
  cout << "cart p y " << cart_p.y << endl;
  
  origin_[i].Transform(cart_p);
  
  switch (direction_[i]) {
    
    case kRight:
      cart_p.y *= -1;
    case kLeft: {
  
      cout << "cart p x after trans " << cart_p.x << endl;
      cout << "cart p y after trans " << cart_p.y << endl;
      cout << "alpha (arcvel) " << arc_vel_[i] << endl;
      double theta_f = atan2(cart_p.x, arc_vel_[i] - cart_p.y);
      double sin_theta_f = sin(theta_f);
      double cos_theta_f = cos(theta_f);
      
      cout << "xp - Cx " << cart_p.x << endl;
      cout << "arc_vel_[i] - cart_p.y " << arc_vel_[i] - cart_p.y << endl;
      cout << "theta_f " << theta_f << endl;
      
      // find x,y on s-reference line based on theta_f
      double x_ref = arc_vel_[i] * sin_theta_f;
      double y_ref = arc_vel_[i] * (1 - cos_theta_f);
      
      cout << "x_ref " << x_ref << endl;
      cout << "y_ref " << y_ref << endl;
      
      frenet_p.s = maps_s_[i] + theta_f / yaw_rate_[i];
      
      switch (direction_[i]) {
        case kRight : frenet_p.d = (x_ref - cart_p.x) / sin_theta_f; break;
        case kLeft  : frenet_p.d = (cart_p.x - x_ref) / sin_theta_f; break;
      }
      
      break;
    }
    case kStraight: 
    
      frenet_p.d = -cart_p.y;
      frenet_p.s = maps_s_[i] + cart_p.x/vel_[i];
      
      break;
  }
  
  return frenet_p;
}


FrenetKinematic Localization::frenet() {
  
  unique_lock<mutex> u_lock(mutex_, defer_lock);
  FrenetKinematic frenet;
  
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
  
  thread_ = std::thread( [this] {
    
    unique_lock<mutex> u_lock(mutex_, defer_lock);
    
    while (thread_alive_) {
      
      if (u_lock.try_lock()) {
        if (updated_) {
          
          // calculate seconds since last update
          static time_point prev_tp = localization_data_.tp;
          double secs = DiffTimePoint(localization_data_.tp, prev_tp);
          
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
          double next_x = cart_.x.p + cart_.x.v*secs;
          double next_y = cart_.y.p + cart_.y.v*secs;
          CartP next_p = {next_x, next_y};
          FrenetP next_sd = CalcFrenet(next_p);
          
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

  /**/

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
      
      CartP xy = CalcXY({s, d});
      FrenetP sp_prime = CalcFrenet(xy);
      
      err += (s - sp_prime.s) + (d - sp_prime.d);
      cout << "total err so far " << err << endl;
    }
    if (abs(err) > 0.0001) break;
  }
  
  cout << "------------ total err " << err << endl;
  /**/
    
  
}