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
    Frenet max_d_;
    Frenet max_v_;
    Frenet max_a_;

    vector<Frenet> waypoints_;
    
    //typedef int    (*CostFunction)(State& state);  
    //static         vector<CostFunction> cost_functions_;
    //static int     SpeedCost(State& state);
    //static int     CollisionCost(State& state);
};

}
#endif