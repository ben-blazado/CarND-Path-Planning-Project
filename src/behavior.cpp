#include "behavior.h"

#include <cmath>

namespace PathPlanning {
  
using std::cout;
using std::endl;
using std::sort;
using std::min;
using std::max;

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
  
  duration dur_secs = loc_in.tp - pre_out.tp;
  double t_ahead = dur_secs.count();
  // cout << "valid t " << t << " secs " << endl;
  
  double curr_pos   = loc_in.start.p.s();
  double curr_speed = loc_in.start.v.s();
  double curr_acc   = loc_in.start.a.s();
  int    curr_lane  = map_.D2Lane(loc_in.start.p.d());
  
  const double goal_pos   = goal.p.s();
  const double goal_speed = goal.v.s();
  const double goal_acc   = goal.a.s();
  const int    goal_lane  = map_.D2Lane(goal.p.d());
  
  for (int i = 0; (i < pre_out.other_cars.size()); i ++) {
    
    Car car = pre_out.other_cars[i];
    double other_car_speed = car.v.s();
    double other_car_pos = car.p.s(); 
        + other_car_speed*t_ahead; // position sycronized to localizer time
    int other_car_lane = map_.D2Lane(car.p.d());
    double t_plan = 2.0;
    
    //double max_decel = min (curr_speed/t_plan, 7.5);
    //double relative_speed = curr_speed - other_car_speed;
    double max_decel = 8.5;  //10
    double breaking_distance = curr_speed*curr_speed / (2*max_decel);
    //double breaking_distance = curr_speed*t_plan - 0.5*max_decel*t_plan*t_plan;
    double safe_distance = breaking_distance + 5;  //+10
    double safe_curr_pos = other_car_pos - safe_distance;
    double safe_goal_pos = other_car_pos + other_car_speed*t_plan - safe_distance;
    
    
    //-----------------------------------------
    // rules for changing lanes based on goal speed and current speed
    //-----------------------------------------
    
    if ((goal_lane != curr_lane) and (goal_speed > curr_speed)) {
      valid = false;
      break;
    }
    
    //-----------------------------------------
    // rules for changing lanes based on current position of other cars
    //-----------------------------------------
    
    // car on left preventing lane change to left
    if ((goal_lane <= other_car_lane) and (other_car_lane < curr_lane)) {
      double s_dist = other_car_pos - curr_pos;
      if ((-12 <= s_dist) and (s_dist <= 12)) { //TODO: set as constant offsets
        valid = false;
        break;
      }
    }
    
    // car on right preventing lane change to right
    if ((curr_lane < other_car_lane) and (other_car_lane <= goal_lane)) {
      double s_dist = other_car_pos - curr_pos;
      double s_future_dist = (other_car_pos + other_car_speed*t_plan) - goal_pos;
      if ((-12 <= s_dist) and (s_dist <= 12)) { //TODO: set as constant offsets
        valid = false;
        break;
      }
    }
    
    
    //-----------------------------------------
    // rules for changing lanes based on current speeds of other cars
    //-----------------------------------------
    
    // car in left lane in front going slower or in back going faster than current speed
    if ((goal_lane <= other_car_lane) and (other_car_lane < curr_lane)     
        and ((other_car_pos > curr_pos) and (other_car_speed < curr_speed)
             or (other_car_pos < curr_pos) and (other_car_speed > curr_speed))) {
      valid = false;
      break;
    }

    // car in right lane in front going slower or in back going faster than current speed    
    if ((curr_lane < other_car_lane) and (other_car_lane <= goal_lane)
        and ((other_car_pos > curr_pos) and (other_car_speed < goal_speed)
             or (other_car_pos < curr_pos) and (other_car_speed > goal_speed))) {
      valid = false;
      break;
    }
    
    
    //-----------------------------------------
    // rules for changing lanes based on goal position and goal speed (future)
    //-----------------------------------------

    // car in right lane in front going slower or in back going faster than goal or goal in safe zone
    if ((goal_lane <= other_car_lane) and (other_car_lane < curr_lane)     
        and (((other_car_pos > curr_pos) and ((other_car_speed < goal_speed) or (goal_pos > safe_goal_pos))) // other car in front
             or ((other_car_pos < curr_pos) and (other_car_speed > goal_speed)))) {
      valid = false;
      break;
    }
    
    // car in right lane in front going slower or in back going faster or goal in safe zone
    if ((curr_lane < other_car_lane) and (other_car_lane <= goal_lane)
        and (((other_car_pos > curr_pos) and ((other_car_speed < goal_speed) or (goal_pos > safe_goal_pos))) // other car in front
             or ((other_car_pos < curr_pos) and (other_car_speed > goal_speed)))) {
      valid = false;
      break;
    }
    
    
    
    //TODO: collision still occurs with front car in safe zone can't change lanes, speed was high 48

    //TODO: slow start. sometimes. Need to kill thread?
    //TODO: Kill threads on error
    //TODO: reduce distance to allow for tight lane change (reduce rear distance?) goal - 8, reduce safe zone toleranace on lane change?
    //TODO: unreasonable suprios 2 lane change left and slow down at mile 2.33?, no traffic, lane ahead free is car behind causing lane change?
    //TODO: front car collision, left lane, after loop is complete. s-value too high?
    //TODO: no changing lanes when clear and after one loop is completet? s-value to high?
    //TODO: car takes too long to change lanes when behind another car and destination lane has traffic. lower safe distance?
    //TODO: side collision rule: clone oher car
    //TODO: rule with car yaw to prevent over speed?
    //TODO: collision change lane left accelerating?
    //TODO: emergency break even though car is behind
    //TODO: cant change lanes faster than current speed
    //TODO: can't change lane into safe zone of car in other lane
    //TODO: create function for best min safe speed
    //TODO: fix no paths selected.
    
    //Done: getting max jerks when lane following or when lanes are clear
            // - check spline dx=dy? for frenet2cart
    //DONE: car stops at end of loop... DONE
    //DONE: vefiry distance when one loop over and other target is behind or vice versa
    

    //-----------------------------------------
    // rules for when car is approaching or in safe zone
    //-----------------------------------------

    if ((goal_lane == other_car_lane) and (other_car_lane == curr_lane) // other car lane = goal lane = curr lane
        and (curr_pos < other_car_pos) // other car in front
        and (curr_pos > safe_curr_pos) // outside safe zone
        //and (goal_speed > curr_speed)) {
        and (goal_speed > ((other_car_pos - curr_pos) / safe_distance) * other_car_speed)) {
        //and (goal_speed > 0.45*other_car_speed)) {
        //and (curr_pos + curr_speed*t_plan + 0.5*curr_acc*t_plan*t_plan > (other_car_pos + other_car_speed*t_plan) - safe_distance)) { // faster than other car
      cout << "inside safe zone " << other_car_pos - curr_pos << " " << curr_speed << " other car speed " << other_car_speed << endl;
      //cout << "car ahead " << other_car_pos - curr_pos << " " << goal_acc << endl;
      valid = false;
      break;
    }
    

    if ((goal_lane == other_car_lane) and (other_car_lane == curr_lane) // other car in goal lane
        and (curr_pos < other_car_pos) // other car in front
        //and (goal_speed > curr_speed)
        and (goal_speed > ((other_car_pos - curr_pos) / safe_distance) * other_car_speed)
        //and (goal_speed > 0.70*other_car_speed)
        //and (curr_pos + curr_speed*t_plan + 0.5*curr_acc*t_plan*t_plan > (other_car_pos + other_car_speed*t_plan) - safe_distance)) { // faster than other car
        //and (goal_pos > (other_car_pos + other_car_speed*t_plan) - safe_distance)) { // faster than other car
        and (goal_pos > safe_goal_pos)) {
      cout << "approaching safe zone " << other_car_pos - curr_pos << " " << curr_speed << " other car speed " << other_car_speed << endl;
      //cout << "car ahead " << other_car_pos - curr_pos << " " << goal_acc << endl;
      valid = false;
      break;
    }
    
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
  
  for (double a = -10; a <= 5; a += (a < 0 ? 0.5 : 0.125)) {
  //for (double end_v_s = max(0.0, v_s - 7.5*t); end_v_s <= min (v_s + 7.5*t, 22.128); end_v_s += 2.2128) {
    // acceleration, a, multiplied by time, t.
    
    // this cap is needed. may prevent a no paths selected reduction of speed or jerk
    if ((v_s >= 20.9196) and (a > 0))
      continue;
    
    double at = a*t;

    double end_v_s = v_s + at;
    //cout << "start v " << v_s << endl;
    //if (end_v_s < 0) {
      // don't create a path that goes in reverse.
      // cout << "negative path." << endl;
      //continue;
    //}
    //double a = (end_v_s - v_s) / t;
    
    double ds = v_s*t + 0.5*at*t; 
    if (ds < 0)
      continue;
    
    //cout << end_v_s << " " << a << " " << ds << endl;

    for (int lane = 0; lane < map_.num_lanes(); lane ++) {
    
      // okay if s > max_s because s gets normalized in calc_xy
      // ensures that the end d is in middle of current lane
      // d = vt + at^2/2.
      //cout << "ds " << ds << endl;
      Kinematic<Frenet> goal;
      goal.p = {loc_in.start.p.s() + ds, map_.Lane2D(lane)};
      goal.v = {end_v_s, 0};
      goal.a = {0, 0};
      
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
