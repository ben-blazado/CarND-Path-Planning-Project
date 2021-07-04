#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "helpers.h"
#include "path.h"
#include "trajectory.h"
#include "buffer.hpp"

#include <thread>
#include <mutex>
#include <iostream>

namespace PathPlanning {

using std::mutex;
using std::thread;


// CPA - closest point of approach
typedef struct {
  double dist;  // distance at CPA
  double secs;  // secs at CPA
} CPA;


class Behavior {
  
  public:
    Behavior(Trajectory& trajectory, Map& map, double secs_per_update, 
        double max_secs);
    ~Behavior();
    
    struct InputData {
      Kinematic<Frenet> start;
      int prev_num_waypoints;
    };
    void Input(InputData& in);
    
    void Run();
      
  private:
  
    Trajectory& trajectory_;
    Map&        map_;
    
    double max_s_;
    double max_secs_;
  
    mutex mutex_;
    bool  processing_;
    thread thread_;
    
    InputData         in_;
    Buffer<InputData> in_buf_;
    
    void GeneratePaths();
    Path& SelectBestPath();
    vector<Path> paths_;
    
    void ProcessInputs();
    
    //SelectBestState();
    
    //int max_lanes_;
    //int max_secs_;
    
    //vector<double> speed_options_(3);
    //enum class SpeedOption {
    //  kStop,
    //  kMaintain,
    //  kMaxLane
    //};
    
};

} // namespace PathPlanning


#endif