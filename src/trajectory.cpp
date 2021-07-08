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
  
  loc_in_ = loc_in;
  
  return;
}


void Trajectory::ProcessInputs () {
  
  vector<Cartesian> waypoints;

  while (processing_) {

    if (beh_in_buf_.TryRead(beh_in_)) {
      
      vector<Cartesian> waypoints;
      cout << "Trajectory::ProcessUpdates::waypoints size from beh " << beh_in_.waypoints.size() << endl;
      for (int i = 0; i < beh_in_.waypoints.size(); i ++) {
        Cartesian p = map_.CalcCartesian(beh_in_.waypoints[i]);
        waypoints.push_back(p);
      } 
      wp_buf_.Write(waypoints);
    }
    
    // TODO: is this neded?
    beh_in_.waypoints.clear();
  }
  return;
}


void Trajectory::Run () {
  
  processing_ = true;
  thread_     = thread( [this] { ProcessInputs(); } );
  
  return;
  
}


bool Trajectory::Output(OutputData& out) {
  
  // TODO: can we get rid of this?
  if (loc_in_.prev_path_x.size() > 0) {
    out.next_x_vals = loc_in_.prev_path_x;
    out.next_y_vals = loc_in_.prev_path_y;
    
    cout << "*** size of prev wp ***" << out.next_x_vals.size() << endl;

    static vector<Cartesian> waypoints;
    static vector<Cartesian> old_waypoints;
    if (wp_buf_.TryRead(waypoints))
      cout << "********** using new way points  size: " << waypoints.size() << endl;
    else
      cout << "********** using old way points  size: " << old_waypoints.size() << endl;
    // find first waypoint that can connect to next x and y vals.
    int found = -1;
    for (int i = 0; i < waypoints.size(); i ++)
      if ((fabs(out.next_x_vals.back() - waypoints[i].x) < 0.001) and 
          (fabs(out.next_y_vals.back() - waypoints[i].y) < 0.001)) {
        found = i;
        break;
      }
    // add all waypoints after found to next x and y vals.
    if (found >= 0) {
      double max_waypoints = 0.8 / 0.02;
      double num_waypoints = max_waypoints - out.next_x_vals.size();        
      cout << "Trajectory::Output num waypoints " << num_waypoints << endl; 
      //TODO: is num_waypoints < 0 ok also?
      for (int i = 0; i < num_waypoints; i ++) {
        out.next_x_vals.push_back(waypoints[i + found + 1].x);
        out.next_y_vals.push_back(waypoints[i + found + 1].y);
      }
      old_waypoints = waypoints;
    }
    else {
      cout << "No Match " << endl; 
      // find first waypoint that can connect to next x and y vals.
      int found = -1;
      for (int i = 0; i < old_waypoints.size(); i ++)
        if ((fabs(out.next_x_vals.back() - old_waypoints[i].x) < 0.001) and 
            (fabs(out.next_y_vals.back() - old_waypoints[i].y) < 0.001)) {
          found = i;
          break;
        }
      // add all waypoints after found to next x and y vals.
      if (found >= 0) {
        double max_waypoints = 0.8 / 0.02;
        double num_waypoints = max_waypoints - out.next_x_vals.size();        
        cout << "Trajectory::Output num waypoints " << num_waypoints << endl; 
        //TODO: is num_waypoints < 0 ok also?
        for (int i = 0; i < num_waypoints; i ++) {
          out.next_x_vals.push_back(old_waypoints[i + found + 1].x);
          out.next_y_vals.push_back(old_waypoints[i + found + 1].y);
        }
      }
    }
  }
  
  return true;
  
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
