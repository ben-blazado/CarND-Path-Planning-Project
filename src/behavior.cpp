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


void Behavior::Input(LocalizationInput& loc_in) {
  
  loc_in_buf_.Write(loc_in);
  
  return;
}

void Behavior::GeneratePaths() {

  Path::ResetStats();
  paths_.clear();
  
  double num_waypoints = max_waypoints_ - loc_in_.prev_num_waypoints;
  double v_s = loc_in_.start.v.s();
  double t = max_secs_;
  double tdiv2 = 0.5*t;

  for (double a = -10; a <= 5; a += (a < 0 ? 2.0 : 0.125)) {
    
    double at = a*t;

    for (int lane = 0; lane < 3; lane ++) {
    
      double end_v_s = v_s + at;
      if (end_v_s < 0)
        // don't create a path that goes in reverse.
        continue;
      
      double ds = v_s*t + at*tdiv2; 
      
      Kinematic<Frenet> end;
      
      // okay if s > max_s because s gets normalized in calc_xy
      // ensures that the end d is in middle of current lane
      end.p = {loc_in_.start.p.s() + ds, map_.Lane2D(lane)};
      end.v = {end_v_s, 0};
      end.a = {a, 0};
      
      Path path(loc_in_.tp, loc_in_.start, end, t, max_waypoints_);
      //cout << "path ds_ " << path.ds_ << endl;
      //cout << "end_v_s " << end_v_s << endl;
      
      path.UpdateStats();
      
      paths_.push_back(path);
    }
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
  

void Behavior::ProcessInputs () {

  while (processing_) {
 
    loc_in_buf_.TryRead(loc_in_);
      
    // cout << "Behavoir::input s d" << in_.start.p.s() << " " << in_.start.p.d() << endl;
    
    GeneratePaths();
    ScorePaths();

    Trajectory::BehaviorInput beh_in = {loc_in_.tp, SortedWaypoints()};
    trajectory_.Input(beh_in);
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
