#include "trajectory.h"

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

namespace PathPlanning {
  
using std::cout;
using std::endl;

Trajectory::Trajectory(Map& map) : map_(map) {
  
  processing_ = false;
  
  updated_         = false;
  buf_.updated     = false;
  snd_buf_.updated = false;
  
  return;
}

Trajectory::~Trajectory() {
  
  if (processing_) {
    processing_ = false;
    if (thread_.joinable())
      thread_.join();
  }
  
  return;
}

void Trajectory::Receive(const vector<Frenet>& waypoints)  {
  
  if (buf_.m.try_lock()) {
    
    buf_.waypoints = waypoints;
    buf_.updated  = true;
    
    // cout << "Trajectory updated with best waypoints" << endl;
    //speed_options[1]
    cout << "Trajectory::Update () " << endl;
    cout << "waypoints size " << buf_.waypoints.size() << endl;
    
    
    buf_.m.unlock();
  }

  return;
}
  
void Trajectory::Receive(vector<double>& previous_path_x, 
    vector<double>& previous_path_y) {
    
  if (buf_.m.try_lock()) {
    
    buf_.prev_path_x = previous_path_x;
    buf_.prev_path_y = previous_path_y;
    buf_.updated = true;
    
    cout << "Travejectory::Update() " << endl;
    cout << "Previous path size " << previous_path_x.size() << endl;
    
    buf_.m.unlock();
  }

  return;
}

void Trajectory::Update() {
  
  buf_.m.lock();
  
  if (buf_.updated) {
    
    updated_     = true;
    waypoints_   = buf_.waypoints;
    prev_path_x_ = buf_.prev_path_x;
    prev_path_y_ = buf_.prev_path_y;
    
    buf_.updated = false;
  }
  
  buf_.m.unlock();
}

void Trajectory::Send() {
  
  if (snd_buf_.m.try_lock()) {
    
    snd_buf_.next_x_vals = next_x_vals_;
    snd_buf_.next_y_vals = next_y_vals_;
    snd_buf_.updated = true;
    snd_buf_.m.unlock();
  }
  
  return;
}

void Trajectory::GetNextXYVals (vector<double>& next_x_vals, 
    vector<double>& next_y_vals) {
      
  //cout << "GetXY trying to lock" << endl;
  
  if (snd_buf_.m.try_lock()) {
    
    //cout << "GetXY axquired lock" << endl;
    
    if (snd_buf_.updated) {
      next_x_vals = snd_buf_.next_x_vals;
      next_y_vals = snd_buf_.next_y_vals;
      snd_buf_.updated = false;
      
      cout << "Trajectory::GetXYVals() " << next_x_vals.size() << endl;
    }

    snd_buf_.m.unlock();
    
    //cout << "GetXY unlocked lock" << endl;
  }
}



void Trajectory::ProcessUpdates () {
  
  while (processing_) {
    
    Update();
      
    if (updated_) {
      
      if (prev_path_x_.size() > 0) {
        // add waypoints from previous path
        next_x_vals_ = prev_path_x_;
        next_y_vals_ = prev_path_y_;
      }
      
      // add waypoints from best path
      if (next_x_vals_.size() < 150)
        for (int i = 0; i < waypoints_.size(); i ++) {
          Cartesian p = map_.CalcCartesian(waypoints_[i]);
          next_x_vals_.push_back(p.x);
          next_y_vals_.push_back(p.y);
        } 
        
      Send();
      
      cout << "Trajectory::ProcessUpdates()" << endl;
      cout << "next_x_vals_ " << next_x_vals_.size() << endl;
      updated_ = false;
      
    }
    
  } // while
  
  return;
}


void Trajectory::Run () {
  
  processing_ = true;
  
  thread_ = thread( [this] { ProcessUpdates(); } );
  
}


} // namespace
    
 
 
/* 

void Trajectory::GetCPA (Trajectory& traj, 
        FrenetSeparation& min_sep, CPA& cpa) {
  
  cpa.found = false;
  
  for (ticker_.Start(); ticker_.IsTicking(), ticker_.Next()) {
    
    int    ticks   = ticker_.ticks();
    double d1     = d_vals_[ticks];
    double d2     = traj.d_vals_[ticks];
    double d_dist = abs(d1 - d2)
    
    if (d_dist < min_sep.d) {
      
      double s1     = s_vals_[ticks];
      double s2     = traj.s_vals_[ticks];
      double s_dist = abs(s1 - s2);
      
      if (s_dist < min_sep.s) {
        cpa.found = true;
        cpa.dist = dist;
        cpa.secs = ticker_.secs();
        break;
    }
  }
  
  return;
}

void Trajectory::GetXYVals(vector<double>& x_vals, vector<double>& y_vals) {
  
  x_vals = x_vals_;
  y_vals = y_vals_;
  
  return;
  
}
*/
