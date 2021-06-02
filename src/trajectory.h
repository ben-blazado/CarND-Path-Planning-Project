#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ticker.h"


// CPA - closest point of approach
typedef struct {
  double dist;  // distance at CPA
  double secs;  // secs at CPA
} CPA;


class Trajectory {
  public:
  
    void ~Trajectory() {};
    
    static void SetMaps(vector<double>& maps_s, vector<double>& maps_x,
        vector<double>& maps_x
    
    void Generate(FrenetKinematic& start, FrenetKinematic& end);
    void Generate(CartP& start, CartV& vel);
    
    void GetCPA (Trajectory& traj, FrenetSeparation& min_sep, CPA& cpa);
    
    void GetXYVals(vector<double>& x_vals, vector<double>& y_vals);
  
  private:
  
    static vector<double> maps_s_; 
    static vector<double> maps_x_; 
    static vector<double> maps_y_;
  
    static const Ticker& ticker_;
    vector<double> s_vals_;
    vector<double> d_vals_;
    vector<double> x_vals_;
    vector<double> y_vals_;
};

#endif
