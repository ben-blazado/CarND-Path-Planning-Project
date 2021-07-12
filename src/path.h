#ifndef PATH_H
#define PATH_H

#include "helpers.h"
#include "map.h"

namespace PathPlanning {
  
using std::vector;
  
class Path {
  
  friend class Behavior;
  
  public:

    static void SecsPerUpdate(double secs) { secs_per_update_ = secs; }
  
    Path (Kinematic<Frenet>& start, Kinematic<Frenet>& end, double t,
        int num_waypoints);
    void UpdateStats();
    void UpdateScore();
    
    // getter for waypoints_
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
    static double DistanceScore(Path& path);
    static double LaneKeepingScore(Path& path);
    static double AverageVelocityScore(Path &path); 
    static double LastVelocityScore(Path &path);
    struct ScoreFunction {
      double (*f)(Path&);
      double weight;
    };
    static vector<ScoreFunction> score_functions_; 
    static bool GreaterThan(Path& p1, Path& p2);
    
    vector<Frenet> waypoints_;
    double dd_; // total accumulated lateral shift (i.e. along d axis)
    double ds_; // total accumulated longitudinal change (along s axis)
    double avg_v_s_;  // avg velocity along s axis
    double last_v_s_; // exit velocity
    double score_;
    
    //typedef int    (*CostFunction)(State& state);  
    //static         vector<CostFunction> cost_functions_;
    //static int     SpeedCost(State& state);
    //static int     CollisionCost(State& state);
};

}
#endif