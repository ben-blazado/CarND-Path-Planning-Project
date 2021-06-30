#include "behavior.h"

namespace PathPlanning {
  
using std::cout;
using std::endl;

Behavior::Behavior (Trajectory& trajectory, double max_s, 
    double secs_per_update, double max_secs) : trajectory_(trajectory) {
  
  max_s_       = max_s;
  max_secs_    = max_secs;
  
  Path::SecsPerUpdate(secs_per_update);
  Path::MaxS(max_s);
  
  processing_  = false;
  updated_     = false;
  buf_.updated = false;
  processing_  = false;
  
  return;
  
}

Behavior::~Behavior() {
  
  if (processing_) {
    processing_ = false;
    if (thread_.joinable())
      thread_.join();
  }
  
}


void Behavior::Receive(Kinematic<Frenet>& start) {
  
  if (buf_.m.try_lock()) {
    
    buf_.start    = start;
    buf_.updated  = true;

    cout << "Behavior::Update() " << start_.p.s << " " << start_.p.d << endl;

    buf_.m.unlock();
  }
  
  return;
}

void Behavior::Update() {
  
  buf_.m.lock();
  
  if (buf_.updated) {
    
    updated_ = true;
    start_   = buf_.start;
    
    buf_.m.unlock();
  
  }
  
  buf_.m.unlock();
  
  return;
}

void Behavior::ProcessUpdates () {

  while (processing_) {
    
    Update();
    
    if (updated_) {
      GeneratePaths();
      Path& best_path = SelectBestPath();
      
      vector<Frenet> waypoints = best_path.Waypoints();
      cout << "Behavoir::ProcessUpdates() " << waypoints.size() << endl;
      trajectory_.Receive(waypoints);
      
      updated_ = false;
    }
  } // while
  
  return;
}


void Behavior::Run () {
  
  processing_ = true;
  thread_     = thread( [this] {ProcessUpdates();} );
  
  return;
}

          
int D2Lane (double d) {
  
  int lane = std::round ((d - 2.0) / 4.0);
  
  return lane;
}

double Lane2D (int lane) {
  
  double d = 4.0*lane + 2.0;
  
  return d;
}
          
void Behavior::GeneratePaths() {

  paths_.clear();
  
  Kinematic<Frenet> end;
  double target_vel = 21.90496; 
  
  end.p.s = fmod(start_.p.s + target_vel*max_secs_, max_s_);
  end.p.d = Lane2D(D2Lane(start_.p.d));
  
  end.v.s = target_vel;
  end.v.d = 0;
  
  end.a.s = 0;
  end.a.d = 0;
  
  Path path(start_, end, max_secs_);
  
  paths_.push_back(path);

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
        

Path& Behavior::SelectBestPath() {
  
  return paths_[0];
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
