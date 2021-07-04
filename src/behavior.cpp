#include "behavior.h"

#include <cmath>

namespace PathPlanning {
  
using std::cout;
using std::endl;

Behavior::Behavior (Trajectory& trajectory, Map& map, double secs_per_update, 
    double max_secs) 
    : trajectory_(trajectory), map_(map) {
  
  max_secs_    = max_secs;
  processing_  = false;
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

  paths_.clear();
  
  Kinematic<Frenet> end;
  int lane = 1;
  for (double v_s = 0.0; v_s <= 22.5; v_s += 2.5) {
    
    double d_s = v_s * max_secs_; 
    
    // okay if s > max_s because s gets normalized in calc_xy
    // ensures that the end d is in middle of current lane
    end.p = {in_.start.p.s() + d_s, map_.Lane2D(lane)};
    //end.p.d = in_.start.p.d;
    
    end.v = {v_s, 0};
    
    end.a = {0, 0};
    
    double max_waypoints = max_secs_ / 0.02;
    double num_waypoints = max_waypoints - in_.prev_num_waypoints;
    
    Path path(in_.start, end, max_secs_, num_waypoints);
    cout << "path max_d_ " << path.max_d_.s() << endl;
    
    paths_.push_back(path);
  }
  
  cout << "num_paths " << paths_.size() << endl;
  
  
  return;
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
      
      cout << "Behavoir::input s d" << in_.start.p.s() << " " << in_.start.p.d() << endl;
      
      GeneratePaths();
      Path& best_path = SelectBestPath();
      
      vector<Frenet> waypoints = best_path.Waypoints();
      cout << "Behavoir::ProcessUpdates() " << waypoints.size() << endl;
      
      Trajectory::BehaviorInput beh_in = {in_.start.p, waypoints};
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

          
        

Path& Behavior::SelectBestPath() {
  
  int best=0;
  double max_a = 0;
  double max_d = 0;
  
  for (int i = 0; i < paths_.size(); i ++) {
    
    /*
    cout << "selectBestPath " << paths_[i].max_d_.s() << endl;
    if (paths_[i].max_d_.s() > max_d) {
      max_d = paths_[i].max_d_.s();
      best = i;
      cout << "best path [" << best << "] best width " << max_d << endl;
    }
    */
    
    if ((paths_[i].max_a_.s() < 8) and (max_a < paths_[i].max_a_.s())) {
      max_a = paths_[i].max_a_.s();
      best = i;
      cout << "best path [" << best << "] " << max_a << endl;
    }
  }
  
  return paths_[8];
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
