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

void Trajectory::Input(BehaviorInput& beh_in)  {
  
  beh_in_buf_.Write(beh_in);

  return;
}
  
void Trajectory::Input(LocalizationInput& loc_in) {
  
  loc_in_buf_.Write(loc_in);
  
  return;
}


void Trajectory::ProcessInputs () {
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  while (processing_) {

    bool loc_in_ready = loc_in_buf_.Read(loc_in_);
    bool beh_in_ready = beh_in_buf_.TryRead(beh_in_);
      
    if (loc_in_ready) {
      
      cout << "Trajectory::ProcessUpdates::prev path_x size from loc " << loc_in_.prev_path_x.size() << endl;;
      
      if (loc_in_.prev_path_x.size() > 0) {
        // add waypoints from previous path
        next_x_vals = loc_in_.prev_path_x;
        next_y_vals = loc_in_.prev_path_y;
      }
      
      if (beh_in_.start.s() == loc_in_.last_f.s()) {
        cout << "Trajectory::ProcessUpdates::waypoints size from beh " << beh_in_.waypoints.size() << endl;
        // add waypoints from best path
        for (int i = 0; i < beh_in_.waypoints.size(); i ++) {
          Cartesian p = map_.CalcCartesian(beh_in_.waypoints[i]);
          next_x_vals.push_back(p.x);
          next_y_vals.push_back(p.y);
        } 
      }
      else
        cout << "******* next vals last and waypoints start DO NOT MATCH *********" << endl;
      
      OutputData out = {next_x_vals, next_y_vals};
      out_buf_.Write(out);
      cout << "Trajectory::next_x_vals_ " << next_x_vals.size() << endl;
      
      next_x_vals.clear();
      next_y_vals.clear();
      beh_in_.waypoints.clear();
      
      
    }
    
  }
  
  return;
}


void Trajectory::Run () {
  
  processing_ = true;
  thread_     = thread( [this] { ProcessInputs(); } );
  
  return;
  
}


bool Trajectory::Output(OutputData& out) {

  bool successful = out_buf_.Read(out);
  
  return successful;
  
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
