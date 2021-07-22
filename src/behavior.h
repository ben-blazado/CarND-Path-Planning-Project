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
    Behavior(Trajectory& trajectory, Map& map, double max_plan_secs, 
        double secs_per_update);
    ~Behavior();
    
    // structure and method to receive inputs from localization module.
    struct LocalizationInput {
      time_point tp;
      Kinematic<Frenet> start;
      int prev_num_waypoints;
    };
    void Input(const LocalizationInput& in);
    
    struct Car {
      Frenet p;
      Frenet v;
    };
    // structure and method to receive inputs from prediction module.
    struct PredictionOutput {
      time_point tp;
      vector<Car> other_cars;
      /*
      PredictionInput& operator=(const PredictionInput& pre_rhs) {
        tp = pre_rhs.tp;
        predictions.clear();
        for (int i = 0; i < pre_rhs.predictions.size(); i ++)
          predictions.push_back(pre_rhs.predictions[i]);
        return *this;
      }
      */
    };
    
    /*
    struct PredictionInput {
      time_point tp;
      vector<vector<Frenet>> predictions;
      PredictionInput& operator=(const PredictionInput& pre_rhs) {
        tp = pre_rhs.tp;
        predictions.clear();
        for (int i = 0; i < pre_rhs.predictions.size(); i ++)
          predictions.push_back(pre_rhs.predictions[i]);
        return *this;
      }
     
    };
    */
    void Input(const PredictionOutput& pre_out);
    
    void Run();
      
  private:
  
    Trajectory& trajectory_;
    Map&        map_;
    
    double max_secs_;
    double max_waypoints_;
  
    mutex mutex_;
    bool  processing_;
    thread thread_;
    
    Buffer<LocalizationInput> loc_in_buf_;
    Buffer<PredictionOutput>   pre_in_buf_;
    
    void GeneratePaths(LocalizationInput loc_in, PredictionOutput pre_out);
    bool Valid(Kinematic<Frenet>& goal, LocalizationInput loc_in, 
            PredictionOutput pre_out);
    void ScorePaths();
    vector<vector<Frenet>> SortedWaypoints();
    vector<Path> paths_;
    
    void ProcessInputs();
    
};

} // namespace PathPlanning


#endif