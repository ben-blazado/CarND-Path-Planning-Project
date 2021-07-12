#include "behavior.h"

#include <cmath>

namespace PathPlanning {
  
using std::cout;
using std::endl;
using std::sort;

Behavior::Behavior (Trajectory& trajectory, Map& map, double max_plan_secs, 
    double secs_per_update) 
    : trajectory_(trajectory), map_(map) {
  
  max_secs_      = max_plan_secs;
  processing_    = false;
  max_waypoints_ = max_plan_secs / secs_per_update;
  
  Path::SecsPerUpdate(secs_per_update);

  return;
  
}

Behavior::~Behavior() {
  
  if (processing_) {
    processing_ = false;
    if (thread_.joinable())
      thread_.join();
  }
  
}


void Behavior::Input(InputData& in) {
  
  in_buf_.Write(in);
  
  return;
}

void Behavior::GeneratePaths() {

  const double      max_v_s = 22.352;
  Kinematic<Frenet> end;

  paths_.clear();
  Path::ResetStats();
  double num_waypoints = max_waypoints_ - in_.prev_num_waypoints;
  double v_s = in_.start.v.s();
  
  // cout << "*** start v " << v_s << endl;
  if (num_waypoints <= 0) {
    cout << "error " << endl;
    exit(0);
  }
  //double t = num_waypoints*Path::secs_per_update_;
  double t = max_secs_;

  for (double a = -10; a <= 5; a += (a < 0 ? 2.0 : 0.125)) 
    for (int lane = 0; lane < 3; lane ++) {
    
      double end_v_s = v_s + a*t;
      if (end_v_s < 0)
        continue;
      
      double ds = v_s*t + 0.5*a*t*t; 
      
      // okay if s > max_s because s gets normalized in calc_xy
      // ensures that the end d is in middle of current lane
      end.p = {in_.start.p.s() + ds, map_.Lane2D(lane)};
      //end.p.d = in_.start.p.d;
      
      end.v = {end_v_s, 0};
      
      end.a = {a, 0};
      
      Path path(in_.start, end, t, max_waypoints_);
      //cout << "path ds_ " << path.ds_ << endl;
      //cout << "end_v_s " << end_v_s << endl;
      
      path.UpdateStats();
      
      paths_.push_back(path);
    }
  
  // cout << "num_paths " << paths_.size() << endl;
  
  return;
}

void Behavior::ScorePaths() {
  
  for (int i = 0; i < paths_.size(); i ++)
    paths_[i].UpdateScore();
  
  return;
}


vector<vector<Frenet>> Behavior::SortedWaypoints () {
  
  // sort path in order of descending score
  sort(paths_.begin(), paths_.end(), Path::GreaterThan);
  
  vector<vector<Frenet>> sorted_waypoints;
  for (int i = 0; i < paths_.size(); i ++) 
    sorted_waypoints.push_back(paths_[i].Waypoints());
  
  return sorted_waypoints;
}
  

  /*
  paths_.clear();
  
  Kinematic<Frenet> end;
  double target_vel = 21; 
  double max_distance = target_vel * max_secs_;
  
  // okay if s > max_s because s gets normalized in calc_xy
  end.p.s = in_.start.p.s + max_distance;
  // ensures that the end d is in middle of current lane
  end.p.d = map_.Lane2D(1);
  //end.p.d = in_.start.p.d;
  
  end.v.s = target_vel;
  end.v.d = 0;
  
  end.a.s = 0;
  end.a.d = 0;
  
  double max_waypoints = max_secs_ / 0.02;
  double num_waypoints = max_waypoints - in_.prev_num_waypoints;
  
  Path path(in_.start, end, max_secs_, num_waypoints);
  
  paths_.push_back(path);
  */

void Behavior::ProcessInputs () {

  while (processing_) {
    
    if (in_buf_.Read(in_)) {
      
      // cout << "Behavoir::input s d" << in_.start.p.s() << " " << in_.start.p.d() << endl;
      
      GeneratePaths();
      ScorePaths();

      Trajectory::BehaviorInput beh_in = {SortedWaypoints()};
      trajectory_.Input(beh_in);
    }
  } 
  
  return;
}


void Behavior::Run () {
  
  processing_ = true;
  thread_     = thread( [this] { ProcessInputs(); } );
  
  return;
}

          
        

  
  /*
  State::start(start_);
  int best_; // index for best state selected
  double lowest_cost = std::numeric_limits<double>::max();
  
  for (int i = 0; i < states_.size(); i ++) {
    
    State& state = states_[i];
    state.GenerateTrajectory();

    double cost = state.Cost();
    if (cost < lowest_cost) {
      lowest_cost = cost;
      best_ = i;
    }
  }
  */
  


} // namespace PathPlanning
