#ifndef PATH_H
#define PATH_H

#include "helpers.h"
#include "map.h"

namespace PathPlanning {
  
using std::vector;
  
class Path {

  public:
  
    Path (Kinematic<Frenet>& start, Kinematic<Frenet>& end, double t);
    ~Path ();
    static void SecsPerUpdate(double secs) { secs_per_update_ = secs; }
    static void MaxS(double max_s)         { max_s_           = max_s; }
    
    const vector<Frenet>& Waypoints() { return waypoints_; }
    
    //int          Cost();
    //virtual void Plan()=0;
    //void         GetXYVals(vector<double>& x_vals, vector<double>& y_vals);
  private:
    static double secs_per_update_; 
    static double max_s_;

    vector<Frenet> waypoints_;
    
    //typedef int    (*CostFunction)(State& state);  
    //static         vector<CostFunction> cost_functions_;
    //static int     SpeedCost(State& state);
    //static int     CollisionCost(State& state);
};

}
#endif