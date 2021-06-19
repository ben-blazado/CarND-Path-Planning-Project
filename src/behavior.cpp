#include <limits.h>

#include "behavior.h"
#include "state.h"
#include "trajectory.h"


void Behavior::Behavior(Trajectory& traj, int max_lanes, 
    double speed_limit, double max_acc) {
  
  max_lanes_ = max_lanes;
  max_secs_ = 2*(speed_limit/max_acc);
  speed_options[SpeedOption::kStop] = 0.0;
  
  states_.resize(1);
  //states_.resize(max_lanes_ * max_secs_ * speed_options_.size());
  
  //traj_ = traj;
  updated_ = false;
  
  return;
}

Behavior::Update(FrenetKinematic& start) {
  
  unique_lock<mutex> u_lock(mutex_, defer_lock);
  
  if (u_lock.try_lock()) {
    start_ = start;
    //speed_options[1]
    updated_ = true;
    u_lock.unlock();
  }
  
  return;
}

Behavior::GenerateStates {

  speed_options[SpeedOption::kMaintain] = start_.s.v;
  
  int i = 0;  // index for state
  
  int lane = DToLane(start_.s.d);
  int max_secs_ = 10;
  
  FrenetKinematic end;
        
  double speed = 35.0; 
  
  end.s.p = start_.s.p + speed*t
  end.s.v = speed;
  end.s.a = 0;
        
  end.d.p = LaneToD(lane);
  end.d.v = 0;
  end.d.a = 0;
  
  states_[0].frenet(end);

  /*
  // iterate over lane
  for (int lane = 0; lane < max_lanes_;l ++) {
    // max_lane_speed = MaxLaneSpeed(Lane);
    double max_lane_speed = 45;
    
    speed_options[SpeedOption::kMaxLane] = max_lane_speed;
    
    // iterate over time (seconds)
    for (int t = 1; t <= max_secs_; t++)
      
      // iterate over speed opions (stop, maintain, max lane)
      for (int v = 0; v < speed_options.size(); v++) {
        double speed = speed_options[v]; 

        FrenetKinematic end;
        
        end.s.p = start_.s.p + speed*t
        end.s.v = speed;
        end.s.a = 0;
        
        end.d.p = LaneToD(lane);
        end.d.v = 0;
        end.d.a = 0;
        
        states_[lane][t][v].end(end);
      }
  }
  return;
  */
}
        
State& Behavior::SelectBestState() {
  
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
  
  return state_[best_];
}

void Behavior::Run () {
  
  thread_alive_ = true;
  
  thread_ = std::thread([this]{
    
    unique_lock<mutex> u_lock(mutex_, defer_lock);
    
    while (thread_alive_) 
      if (u_lock.try_lock()) {
        if (updated_) {
          
          GenerateStates();
          State& best_state = state[0];
          best_state.identify();
          // State& best_state = SelectBestState();
          
          //trajectory_.Update(best_state.traj());
          
          updated_ = false;
        }
        u_lock.unlock();
      }
      
  });
  
  return;
}
