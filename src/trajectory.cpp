#include "trajectory.h"

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

namespace PathPlanning {
  
using std::cout;
using std::endl;
using std::min;

Trajectory::Trajectory(Map& map, double max_exe_secs, double secs_per_update,
    double max_v) 
    : map_(map) {
  
  processing_ = false;
  max_waypoints_ = max_exe_secs / secs_per_update;
  secs_per_update_ = secs_per_update;
  max_v_ = max_v;
  
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


bool Trajectory::FrenetToCartesian(const vector<Frenet>& f_waypoints, 
    vector<Cartesian>& waypoints) {

  bool valid = true;
  
  // covert each frenet waypoint to cartesian
  for (int i = 0; i < f_waypoints.size(); i ++) {
    
    Cartesian p;
    map_.CalcCartesian(f_waypoints[i], p);
    waypoints.push_back(p);
    
    // validate velocity
    if (i > 0) {
      Cartesian wp2 = waypoints[i];
      Cartesian wp1 = waypoints[i - 1];
      
      double d = distance (wp2.x, wp2.y, wp1.x, wp1.y);
      double v = d / secs_per_update_;
      
      //TODO: just check if last 2 points exceed? or cutoff remaining?
      // invalidate waypoints if v exceeds max_v 
      if (v >= max_v_) {
        valid = false;
        break;
      }
    }
    
  }
  
  //TODO: fix later
  return valid and (waypoints.size() > 1);
}


void Trajectory::ProcessInputs () {
  
  vector<Cartesian> waypoints;

  while (processing_) {

    if (beh_in_buf_.TryRead(beh_in_)) {
      
      bool selected = false;
      for (int i = 0; i < beh_in_.sorted_waypoints.size(); i ++) {

        vector<Cartesian> waypoints;
        
        // convert frenet waypoints (from behavior module) to cartesian.
        if (FrenetToCartesian(beh_in_.sorted_waypoints[i], waypoints)) {
          selected = true;
          // cout << "selected trajectory " << i << " of " << beh_in_.sorted_waypoints[i].size() << endl;
          double last_d = beh_in_.sorted_waypoints[i].back().d();
          static int lane = map_.D2Lane(last_d);
          if (lane != map_.D2Lane(last_d)) {
            lane = map_.D2Lane(last_d);
            cout << "Changing lane " << lane << endl;
          }
          wp_buf_.Write(waypoints);
          break;
        } 
        else {
          // cout << "trajectory " << i << " not selected." << endl;
          waypoints.clear();
        }
      }  // for
      if (not selected) {
        cout << "no paths selected " << endl;
        cout << "    received num paths from beh " << beh_in_.sorted_waypoints.size() << endl;
      }
    } // if
    
    // TODO: is this neded?
    beh_in_.sorted_waypoints.clear();
  }
  return;
}


// runs the ProcessInputs() method as a thread.
void Trajectory::Run () {
  
  processing_ = true;
  thread_     = thread( [this] { ProcessInputs(); } );
  
  return;
}


bool Trajectory::Output(OutputData& out) {
  
  // get last point of prev path which is the starting point 
  // of new waypoints that will be added to prev path.
  out.next_x_vals  = loc_in_.prev_path_x;
  out.next_y_vals  = loc_in_.prev_path_y;
  Cartesian last_p = {out.next_x_vals.back(), out.next_y_vals.back()};

  // try to find the index position of last point (last_p) in new_waypoints. 
  // if found, this index will be used to start
  // copying over coordinates from waypoints to next_x_vals and next_y_vals.
  int start_idx = -1;
  vector<Cartesian> new_waypoints;
  if (wp_buf_.TryRead(new_waypoints))
    for (int i = 0; i < new_waypoints.size(); i ++)
      if ((fabs(last_p.x - new_waypoints[i].x) < 0.001) and 
          (fabs(last_p.y - new_waypoints[i].y) < 0.001)) {
        start_idx = i;
        break;
      }
      
  //if (new_waypoints.size() == 0)
  //  cout << "traj output new waypoints size is zero" << endl;

  // if the last_p was found in new_waypoints, use new_waypoints
  // as the trajectory to add to next_x and next_y vals starting at start_idx.
  // if last_p was not found, then use the old waypoints.
  vector<Cartesian> waypoints;
  static vector<Cartesian> old_waypoints;
  if (start_idx >= 0) 
    waypoints = {new_waypoints.begin() + start_idx, new_waypoints.end()};
  else {
    waypoints = old_waypoints;
    // cout << "Old waypoints left " << waypoints.size() << endl;
  }

  // add the waypoints to next_x and next_y vals.
  // caculate num_waypoints to add to next_x and next_y vals;
  int num_waypoints = max_waypoints_ - out.next_x_vals.size();
  // ensure num_waypoints does not exceeed size of waypoints
  num_waypoints = min(num_waypoints, static_cast<int>(waypoints.size()));
  if (num_waypoints > 0) {
    // start at i=1. if i=0 is used, which is the last point of next vals
    // and the first point of waypoints, then that waypoint would be duplicated.
    for (int i = 1; i < num_waypoints; i ++) {
      out.next_x_vals.push_back(waypoints[i].x);
      out.next_y_vals.push_back(waypoints[i].y);
    }
    // copy over  waypoints starting at num_waypoints - 1 until the end.
    // the first waypoint of old_waypoints is now the last_p 
    // of prev_path on the next call and will allow old_waypoints to be used 
    // in case new_waypoints does not have last_p.
    old_waypoints = {waypoints.begin() + num_waypoints - 1, waypoints.end()};
  }
  
  // cout << "trajectory next x vals size " << out.next_x_vals.size() << endl;
  if (out.next_x_vals.size() == 0)
    cout << "next path is exhausted" << endl;
  // TODO: remove true, make method a void.
  return true;
}



} // namespace
    

/*
bool Trajectory::Output(OutputData& out) {
  
  // TODO: can we get rid of this?
  out.next_x_vals  = loc_in_.prev_path_x;
  out.next_y_vals  = loc_in_.prev_path_y;
  Cartesian last_p = {out.next_x_vals.back(), out.next_y_vals.back()};
  
  static vector<Cartesian> new_waypoints;
  static vector<Cartesian> old_waypoints;
  wp_buf_.TryRead(new_waypoints);
  
  vector<vector<Cartesian>*> plans;
  plans.push_back(&new_waypoints);
  plans.push_back(&old_waypoints);
  
  double num_waypoints = max_waypoints_ - out.next_x_vals.size();        
  for (int p = 0; p < plans.size(); p ++) {
    
    vector<Cartesian>& waypoints = *plans[p];
    
    // try to find last xy coordinate of next_x_vals and next_y_vals
    // in waypoints. this index will be used to start
    // copying over coordinates from waypoints to next_x_vals and next_y_vals.
    int found = -1;
    for (int i = 0; i < waypoints.size(); i ++)
      if ((fabs(last_p.x - waypoints[i].x) < 0.001) and 
          (fabs(last_p.y - waypoints[i].y) < 0.001)) {
        found = i;
        break;
      }
    
    // if found, copy waypoints starting at found + 1 
    // to next_x_vals and net_y_vals.
    if (found >= 0) {
      for (int i = 0; i < num_waypoints; i ++) {
        out.next_x_vals.push_back(waypoints[i + found + 1].x);
        out.next_y_vals.push_back(waypoints[i + found + 1].y);
      }
      // save rest of waypoints to old waypoints.
      old_waypoints = {waypoints.begin() + found + num_waypoints, waypoints.end()};
      break;
    }
  }
  
  return true;
  
}
*/


/*
bool Trajectory::Output(OutputData& out) {
  
  // TODO: can we get rid of this?
  if (loc_in_.prev_path_x.size() > 0) {
    out.next_x_vals = loc_in_.prev_path_x;
    out.next_y_vals = loc_in_.prev_path_y;
    
    cout << " " << endl;
    cout << " " << endl;
    cout << "*** *************** *** *************** ***************" << endl;
    cout << "*** size of prev wp ***" << out.next_x_vals.size() << endl;

    static vector<Cartesian> waypoints;
    static vector<Cartesian> old_waypoints;
    if (wp_buf_.TryRead(waypoints)) 
      cout << "NEW NEW NEW NEW NEW NEW way points read size: " << waypoints.size() << endl;
    else {
      cout << "waypoints NOT changed size: " << waypoints.size() << endl;
        //if (waypoints.front().x != old_waypoints.front().x)
    }
    if (old_waypoints.size() > 0)
      cout << "first old wp " << old_waypoints.front().x << endl;
    cout << "********** current way points  size: " << waypoints.size() << endl;
    if (waypoints.size() > 0) {
      cout << "first curr wp " << waypoints.front().x << endl;
      cout << "last curr wp " << waypoints.back().x << endl;      
    }
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
      cout << "Trajectory::Output num NEW waypoints " << num_waypoints <<" found " << found << endl; 
      cout << "NEW waypoints match at " << waypoints[found].x <<" found " << found << endl; 
      //TODO: is num_waypoints < 0 ok also?
      for (int i = 0; i < num_waypoints; i ++) {
        out.next_x_vals.push_back(waypoints[i + found + 1].x);
        out.next_y_vals.push_back(waypoints[i + found + 1].y);
      }
      //old_waypoints = waypoints;
      old_waypoints = {waypoints.begin() + found + 1, waypoints.end()};
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
        cout << "Trajectory::Output num OLD waypoints " << num_waypoints <<" found " << found << endl; 
        cout << "OLD waypoints match at " << old_waypoints[found].x <<" found " << found << endl; 
        //TODO: is num_waypoints < 0 ok also?
        for (int i = 0; i < num_waypoints; i ++) {
          out.next_x_vals.push_back(old_waypoints[i + found + 1].x);
          out.next_y_vals.push_back(old_waypoints[i + found + 1].y);
        }
        //if (found == 41)
        //  exit (0);
      }
    }
    cout << "Last pointx " << out.next_x_vals.back() << endl;
  }
  
  return true;
  
}

*/ 
 
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
