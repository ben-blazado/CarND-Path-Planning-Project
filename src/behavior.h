#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "helpers.h"
#include "path.h"
#include "trajectory.h"

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
    Behavior(Trajectory& trajectory, double max_s, double secs_per_update, 
        double max_secs);
    ~Behavior();
    
    void Run();
    void Receive(Kinematic<Frenet>& start);
      
  private:
  
    Trajectory& trajectory_;
    
    double max_s_;
    double max_secs_;
  
    mutex mutex_;
    bool  processing_;
    thread thread_;
    
    struct {
      mutex m;
      bool updated;
      Kinematic<Frenet> start;
    } buf_;
    bool updated_;
    Kinematic<Frenet> start_;
    
    void GeneratePaths();
    Path& SelectBestPath();
    vector<Path> paths_;
    
    void Update();
    void ProcessUpdates();
    
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