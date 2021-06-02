#include "planner.h"
#include <math.h>

Planner::Planner () {

  state_ = State();
}

Car::Update(double x,
            double y,
            double s,
            double d,
            double yaw,
            double speed,
            vector<double> prev_path_x,
            vector<double> prev_path_y,
            double end_s,
            double end_y) {
    
    loc_.Update();
    pred_.Update();
    beh_.Update();
}


void Localization::Update() {
  
  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  yaw_ = yaw;
  
}




  

Planner::Plan(vector<double> &x_vals, vector<double> &y_vals) {
  
  beh_.plan()
  loc_.Plan();
  tra_.Plan();
  
  x_vals = mot_.x_vals()
  y_vals = mot_.y_vals()
  
  
  
  
  behavior.GetBestState(pose_);
  
  vector<State> next_states = state_.GetNextStates (pose);
  
  double lowest_cost = INFINITY;
  
  State &best_next_state;
  
  for (int i = 0; i < options.size(); i ++) {
    
    State &curr_option = next_states[i];
    
    if (state.cost() < lowest_cost) {
      lowest_cost = cost;
      best_next_state = curr_option;
    }
  }
  create_trajectory(optimal_state, x_vals, y_vals);
}

  


Behavior::GetBestPose(Pose &current_pose)



void 
  
  
  
  
      
    
    
    
    
    
    
    
    
    