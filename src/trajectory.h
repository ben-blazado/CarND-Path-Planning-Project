#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "map.h"
#include "path.h"
#include "buffer.hpp"
#include "helpers.h"

#include <thread>
#include <mutex>

namespace PathPlanning {

using std::mutex;
using std::thread;

class Trajectory {
  
  public:
  
    Trajectory(Map& map, double max_exe_secs, 
        double secs_per_update, double max_v);
    ~Trajectory();
    
    struct BehaviorInput {
      time_point tp;
      vector<vector<Frenet>> sorted_waypoints;
    };
    struct LocalizationInput {
      vector<double> prev_path_x;
      vector<double> prev_path_y;
    };
    struct OutputData {
      vector<double> next_x_vals;
      vector<double> next_y_vals;
    };
    void Input(BehaviorInput& beh_in);
    void Input(LocalizationInput& loc_in);
    void Run();
    bool Output(OutputData& out);
  
  private:

    Map& map_;
    int max_waypoints_;
    double secs_per_update_;
    double max_v_;
    
    bool processing_;
    thread thread_;
    //TODO: do we still use MUTEX? delete if already be in buffer.h.
    mutex  mutex_;
    
    BehaviorInput             beh_in_;
    Buffer<BehaviorInput>     beh_in_buf_;
    Buffer<vector<Cartesian>> wp_buf_;
    
    LocalizationInput loc_in_;
    
    OutputData         out_;
    Buffer<OutputData> out_buf_;
    
    void ProcessInputs();
    bool FrenetToCartesian(const vector<Frenet>& f_waypoints, 
        vector<Cartesian>& waypoints);
};

} // PathPlanning

#endif
