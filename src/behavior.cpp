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
  
  Path::secs_per_update(secs_per_update);

  return;
  
}

Behavior::~Behavior() {
  
  if (processing_) {
    processing_ = false;
    if (thread_.joinable())
      thread_.join();
  }
  
}


void Behavior::Input(const LocalizationInput& loc_in) {
  
  loc_in_buf_.Write(loc_in);
  
  return;
}


void Behavior::Input(const PredictionOutput& pre_out) {
  
  pre_in_buf_.Write(pre_out);
  
  return;
}


bool Behavior::Valid(Kinematic<Frenet>& goal, 
    LocalizationInput loc_in, PredictionOutput pre_out)
{
  bool valid = true;
  
  duration dt = loc_in.tp - pre_out.tp;
  double t = dt.count();
  // cout << "valid t " << t << " secs " << endl;
  
  double curr_pos   = loc_in.start.p.s();
  double curr_speed = loc_in.start.v.s();
  int curr_lane = map_.D2Lane(loc_in.start.p.d());
  int goal_lane = map_.D2Lane(goal.p.d());
  double goal_pos = goal.p.s();
  double goal_speed = goal.v.s();
  double goal_acc = goal.a.s();
  
  for (int i = 0; (i < pre_out.other_cars.size()); i ++) {
    
    Car car = pre_out.other_cars[i];
    // check if other car in front
    Frenet p = {car.p + car.v*t};
    int car_lane = map_.D2Lane(car.p.d());
    
    if ((goal_lane != curr_lane) 
      and ((p.s() - curr_pos) < 15)
      and (curr_lane == car_lane)) {
      valid = false;
      break;
    }
    
    if ((goal_lane < curr_lane)
        and (fabs (p.s() - loc_in.start.p.s()) < 12.5) 
        and (map_.D2Lane(car.p.d()) - curr_lane == -1)) {
      valid = false;
      break;
    }
    
    if ((goal_lane > curr_lane)
        and (fabs (p.s() - loc_in.start.p.s()) < 12.5) 
        and (map_.D2Lane(car.p.d()) - curr_lane == 1)) {
      valid = false;
      break;
    }
    
    //TODO: select closest car in lane
    if ((goal_lane == car_lane) // same lane
        //and (curr_lane == car_lane) // same lane
        and ((curr_speed > car.v.s()) or (goal.v.s() > car.v.s()))) {           // goal is faster than car
      double dt = 3;
      double target_v;
      if (p.s() - loc_in.start.p.s() < 15)      
          target_v = car.v.s() - 2.5;
      else
          target_v = car.v.s();
      goal.v = {target_v, goal.v.d()};
      goal.a = {0, goal.a.d()};
      double ds = target_v*dt + 0.5*goal.a.s()*dt*dt;       
      goal.p = {loc_in.start.p.s() + ds, goal.p.d()};
      // cout << "curr speed " << curr_speed << " other car " << car.v.s() << endl;
      break;
    }

    /*
    //TODO: select closest car in lane
    if ((goal_lane != curr_lane) // same lane
        and (goal_lane == car_lane) // same lane
        and (goal.v.s() > car.v.s())) {           // goal is faster than car
      double dt = 3;
      goal.v = {car.v.s(), goal.v.d()};
      goal.a = {0, goal.a.d()};
      double ds = car.v.s()*dt + 0.5*goal.a.s()*dt*dt;       
      goal.p = {loc_in.start.p.s() + ds, goal.p.d()};
      cout << "collision imminent " << curr_speed << " " << car.v.s() << endl;
      break;
    }
    */

    //if ((p.s() > loc_in.start.p.s())              // other car in front
    //    and (goal_pos > p.s() + 3*car.v.s() - 10)
    //    and (goal_lane == map_.D2Lane(car.p.d()))) { // same lane
    //  valid = false;
    //  break;
    //}

    
    
  }
      
  return valid;
}


void Behavior::GeneratePaths(LocalizationInput loc_in,
    PredictionOutput pre_out) {

  Path::ResetStats();
  paths_.clear();
  
  const double v_s = loc_in.start.v.s();
  // cout << "start vs " << v_s << endl;
  const double t = max_secs_;
  const double  tdiv2 = 0.5*t;
  const double  num_waypoints = max_waypoints_ - loc_in.prev_num_waypoints;
  
  for (double a = -10; a <= 5; a += (a < 0 ? 1 : 0.125)) {
    
    // acceleration, a, multiplied by time, t.
    double at = a*t;

    double end_v_s = v_s + at;
    //cout << "start v " << v_s << endl;
    if (end_v_s < 0) {
      // don't create a path that goes in reverse.
      // cout << "negative path." << endl;
      continue;
    }
    double ds = v_s*t + 0.5*a*t*t; 
    if (ds < 0)
      continue;

    for (int lane = 0; lane < map_.num_lanes(); lane ++) {
    
      // okay if s > max_s because s gets normalized in calc_xy
      // ensures that the end d is in middle of current lane
      // d = vt + at^2/2.
      //cout << "ds " << ds << endl;
      Kinematic<Frenet> goal;
      goal.p = {loc_in.start.p.s() + ds, map_.Lane2D(lane)};
      goal.v = {end_v_s, 0};
      goal.a = {a, 0};
      
      if (Valid(goal, loc_in, pre_out)) {
        Path path(loc_in.tp, loc_in.start, goal, t, max_waypoints_);
        path.UpdateStats();
        paths_.push_back(path);
      }
      
      //cout << "start " << loc_in.start.p.s() << " " << loc_in.start.p.d() << " " << loc_in.start.v.s() << " " << loc_in.start.v.d() << endl;
      //cout << "end_v_s " << end_v_s << endl;

      // cout << "----- Checking path [a " << a << ", end s " << loc_in.start.p.s() + ds << ", lane " << lane << "] for collision " << endl;
      //if (CollisionFree (path, pre_in)) {
      //}
      //else
      //cout << "      NOT ADDED: path [a " << a << ", end s " << loc_in.start.p.s() + ds << ", lane " << lane << "] for collision " << endl;        
    }
  }
  
  if (paths_.size() == 0) {
    cout << "no paths generated " << endl;
    // exit(0);
  }
  
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
    sorted_waypoints.push_back(paths_[i].waypoints());
  
  // cout << "sorted waypoints.size  " << sorted_waypoints.size() << endl;
  return sorted_waypoints;
}
  

void Behavior::ProcessInputs () {

  bool loc_in_ready = false;
  bool pre_in_ready = false;
  
  while (processing_) {
 
    LocalizationInput loc_in;
    PredictionOutput   pre_out;
    
    //TODO: try to eliminate need for flags
    loc_in_ready = loc_in_buf_.TryRead(loc_in) or loc_in_ready;
    //cout << "loc in " << endl;
    pre_in_ready = pre_in_buf_.TryRead(pre_out) or pre_in_ready;
    //cout << "pre in " << endl;
    
    if (loc_in_ready and pre_in_ready or true) {
      
      // cout << "Behavoir::input s d" << in_.start.p.s() << " " << in_.start.p.d() << endl;
      // cout << "cars nearby " << pre_in.predictions.size() << endl;
    
      GeneratePaths(loc_in, pre_out);
      // cout << "paths generated " << endl;
    
      ScorePaths();

      // cout << "Trajectory input " << endl;
      Trajectory::BehaviorInput beh_in = {loc_in.tp, SortedWaypoints()};
      trajectory_.Input(beh_in);
      
      loc_in_ready = false;
      pre_in_ready = false;
    }
    else
      cout << "loc and or pre not ready>" << endl;
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
