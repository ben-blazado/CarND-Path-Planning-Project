#include "localization.h"
#include "helpers.h"

#include <thread>
#include <mutex>
#include <iostream>
#include <ctime>
#include <iomanip>

using std::unique_lock;
using std::defer_lock;
using std::mutex;
using std::cout;
using std::endl;

Localization::Localization(vector<double>& maps_s, vector<double>& maps_x, 
        vector<double>& maps_y, double secs_per_tick) {
          
  maps_s_ = maps_s;
  maps_x_ = maps_x;
  maps_y_ = maps_y;
  
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

vector<double> Localization::CalcXY (double s, double d) {
  
  return getXY (s, d, maps_s_, maps_x_, maps_y_);
}


vector<double> Localization::CalcFrenet(double x, double y, double theta) {
  
  return getFrenet(x, y, theta, maps_x_, maps_y_);
}

FrenetKinematic Localization::frenet(){
  
  return frenet_;
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
          
          static time_point prev_tp = localization_data_.tp;
          
          double secs = DiffTimePoint(localization_data_.tp, prev_tp);
          
          prev_tp = localization_data_.tp;          
          
          cout << std::fixed << std::setprecision(6) << "loc secs" << secs << endl;
          
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
          vector<double> next_sd = CalcFrenet(next_x, next_y, theta);
          
          double prev_s_v = frenet_.s.v;
          frenet_.s.p = localization_data_.car_s;
          frenet_.s.v = (next_sd[0] - frenet_.s.p) / secs;
          frenet_.s.a = (frenet_.s.v - prev_s_v) / secs;
          
          cout << frenet_.s.p << endl;

          double prev_d_v = frenet_.d.v;
          frenet_.d.p = localization_data_.car_d;
          frenet_.d.v = (next_sd[1] - frenet_.d.p) / secs;
          frenet_.d.a = (frenet_.d.v - prev_d_v) / secs;
          
          updated_ = false;
          
        }
        u_lock.unlock();
      }
    }  // while
    cout << "Localization::thread_ has ended." << endl;
  }); // thread
    
  return;
}