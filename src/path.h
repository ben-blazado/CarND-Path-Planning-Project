#ifndef PATH_H
#define PATH_H

#include "helpers.h"
#include "map.h"

namespace PathPlanning {
  
using std::vector;
  
class Path {
  
  friend class Behavior;
  
  public:

    static void secs_per_update(double secs) { secs_per_update_ = secs; }
    static const double& secs_per_update() { return secs_per_update_ ; }
    
    static void ResetStats();
    static bool GreaterThan(Path& p1, Path& p2);
  
    Path (time_point tp, Kinematic<Frenet>& start, Kinematic<Frenet>& end, 
        double t, int num_waypoints);
    void UpdateStats();
    void UpdateScore();
    
    // getters
    const time_point& tp() const { return tp_; }
    const vector<Frenet>& waypoints() const { return waypoints_; }
    
    //int          Cost();
    //virtual void Plan()=0;
    //void         GetXYVals(vector<double>& x_vals, vector<double>& y_vals);
  private:
  
    static Map& map_;
    static double secs_per_update_; 
    
    static double max_dd_;
    static double max_ds_;
    static double max_avg_v_s_;
    static double max_last_v_s_;
    
    static double DistanceScore(Path& path);
    static double LaneKeepingScore(Path& path);
    static double AverageVelocityScore(Path &path); 
    static double LastVelocityScore(Path &path);
    struct ScoreFunction {
      double (*f)(Path&);
      double weight;
    };
    static vector<ScoreFunction> score_functions_; 
    
    time_point tp_;
    vector<Frenet> waypoints_;
    Kinematic<Frenet> end_;    
    double dd_; // total accumulated lateral shift (i.e. along d axis)
    double ds_; // total accumulated longitudinal change (along s axis)
    double avg_v_s_;  // avg velocity along s axis
    double last_v_s_; // exit velocity
    double closest_distance_; // closest dist
    double score_;
    
    //typedef int    (*CostFunction)(State& state);  
    //static         vector<CostFunction> cost_functions_;
    //static int     SpeedCost(State& state);
    //static int     CollisionCost(State& state);
};

}
#endif