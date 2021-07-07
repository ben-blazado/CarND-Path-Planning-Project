#ifndef PATH_H
#define PATH_H

#include "helpers.h"
#include "map.h"

namespace PathPlanning {
  
using std::vector;
  
class Path {
  
  friend class Behavior;

  public:
  
    Path (Kinematic<Frenet>& start, Kinematic<Frenet>& end, double t,
        int num_waypoints);
    static void SecsPerUpdate(double secs) { secs_per_update_ = secs; }
    
    const vector<Frenet>& Waypoints() { return waypoints_; }
    
    //int          Cost();
    //virtual void Plan()=0;
    //void         GetXYVals(vector<double>& x_vals, vector<double>& y_vals);
  private:
    static double secs_per_update_; 
    static double max_dd_;
    static double max_ds_;
    static double max_avg_v_s_;
    static double max_last_v_s_;
    
    static void ResetStats();
    static double DistanceScore (Path& path);
    static double LaneKeepingScore(Path& path);
    static double AverageVeloctityScore(Path &path); 
    static double LastVelocity(Path &path);
    
    vector<Frenet> waypoints_;
    
    double dd_;
    double ds_;
    double avg_v_s_;
    double last_v_s_;
    
    //typedef int    (*CostFunction)(State& state);  
    //static         vector<CostFunction> cost_functions_;
    //static int     SpeedCost(State& state);
    //static int     CollisionCost(State& state);
};

}
#endif