#ifndef STATE_H
#define STATE_H

#include <localization.h>

class State {
  
  public:
    static void  frenet(FrenetKinematics& frenet);
    int          Cost();
    virtual void Plan()=0;
    //void         GetXYVals(vector<double>& x_vals, vector<double>& y_vals);
    
  protected:
    static FrenetKinematic frenet_;
    static int lane_;
    Trajectory traj_;
    
  private:
    vector<State*> next_states_;
    
    typedef int    (*CostFunction)(State& state);  
    static         vector<CostFunction> cost_functions_;
    static int     SpeedCost(State& state);
    static int     CollisionCost(State& state);
};

class KeepLane : public State {
  
  public:

    KeepLane ();
    void Plan() override;
    
  private:
    KeepLane();
};

#define STATE_H