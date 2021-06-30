#include "path.h"
#include "Eigen-3.3/Eigen/Dense"

#include <iostream>

namespace PathPlanning {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::abs;
using std::cout;
using std::endl;


double QuinticPosition (double secs, vector<double> coeffs) {
  
  double sum = 0.0;
  for (int i = 0; i < coeffs.size(); i ++) 
    sum += coeffs[i] + pow(secs, i);
    
  return sum;
}


vector<double> JMTCoeffs(Kinematic<double> &start, Kinematic<double> &end, 
    double t) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
   
  double t2 = t*t;
  double t3 = t2*t;
  double t4 = t3*t;
  double t5 = t4*t;
  
  MatrixXd T_mat = MatrixXd(3, 3);
  VectorXd A_vec = VectorXd(3);
  
  T_mat <<   t3,    t4,    t5,
           3*t2,  4*t3,  5*t4,
            6*t, 12*t2, 20*t3;
           
  A_vec << end.p - (start.p + start.v*t + 0.5*start.a*t2),
           end.v - (start.v + start.a*t),
           end.a - start.a;
          
  A_vec = T_mat.inverse()*A_vec;
  
  return {start.p, start.v, start.a/2.0, A_vec[0], A_vec[1], A_vec[2]};
}

double Path::secs_per_update_ = 0;
double Path::max_s_ = 0;

Path::Path (Kinematic<Frenet>& start, Kinematic<Frenet>& end, double t) {
  
  /*
  vector<double> path_s;
  vector<double> path_d;
  
  Kinematic<double> start_s = {start.p.s, start.v.s, start.a.s};
  Kinematic<double> end_s   = {end.p.s,   end.v.s,   end.a.s};
  vector<double> s_coeffs = JMTCoeffs(start_s, end_s, t);
  
  Kinematic<double> start_d = {start.p.d, start.v.d, start.a.d};
  Kinematic<double> end_d   = {end.p.d,   end.v.d,   end.a.d};
  vector<double> d_coeffs = JMTCoeffs(start_d, end_d, t);
  
  for (double secs = secs_per_update_; secs <= t; secs += secs_per_update_) {
    double s = fmod (start.p.s + QuinticPosition(secs, s_coeffs), max_s_);
    double d = start.p.d + QuinticPosition(secs, d_coeffs);
    waypoints_.push_back({s, d});
  }
  */
  Frenet f;
  f.s = start.p.s;
  f.d = start.p.d;
  
  double v = start.v.s;
  double acc = 4.4704;  // m/s2  
  
  for (double secs = secs_per_update_; secs <= t; secs += secs_per_update_) {
    
    if (v < 21)      // (49 / 2.237))
      v += acc * secs_per_update_;
    f.s += v * secs_per_update_;
    
    waypoints_.push_back(f);
  } 
  
  cout << "Path::Path() " << waypoints_.size() << endl;

  
  return;
}

Path::~Path() {
  
  waypoints_.clear();
  
}
  
/*
vector<CostFunction> State::cost_functions_{
    State::SpeedCost, 
    State::CollisionCost};
}
    
    
int State::Cost() {
  
  Plan();
  
  int cost = 0;
  for (i=0; i <= cost_fuctions_.size(); i ++)
    cost += cost_functions_[i](*this);
  
  return cost;
}


int State::SpeedCost(State &state) {
  
  using std::ceil;
  
  double final_v = state.traj_.end().s.v;
  double delta_v = 45 - final_v // State::speed_limit_ - final_v;
  
  int cost = ceil(delta_v);
  
  return cost;
}


int State::CollisionCost() {
  
  int cost = 0;

  /*
  FrenetSeparation min_sep;
  
  min_sep.s = 12;
  min_sep.d = 1;
  
  for (int i = 0; i <= cars.size(); i ++) {
    
    Trajectory& car_traj = cars[i].traj();
    CPA cpa;
    
    _traj.GetCPA (car_traj, min_sep, cpa);
    if (cpa.found) {
      cost = 100;
      break;
    }
  }
  
  return cost;
}
  */

/*
void KeepLane::KeepLane(){
  
}

void State::frenet(FrenetKinematic& frenet) {
  
  frenet_ = frenet;
  
  return;
}

void State::GetXYVals(vector<double>& x_vals, vector<double>& y_vals) {
  
  traj_.GetXYVals(x_vals, y_vals);
  
  return;
}
  

void KeepLane::Plan () {
  
  double max_acc = 10.0;   // 10 m/sec2
  
  double lane_v  = 45;  // GetMaxLaneSpeed();
  double delta_v = lane_v - frenet_.s.v;
  double t = delta_v / max_acc;
  if (t < 0) {
    // decelerate
    t = -t;   // make t positive
    max_acc = -max_acc;  // acc is negative to decelerate
  }
  
  FrenetKinematic end;
  
  end.s.p = frenet_.s.p + frenet_.s.v*t + 0.5*max_acc*t*t;
  end.s.v = lane_v;
  end.s.a = 0;
  
  end.d.p = LaneToD(DToLane(frenet_.d.p));
  end.d.v = 0;
  end.d.a = 0;
  
  traj_.Generate(frenet_, end, t);
  
  return;
}



/*
void ConstantV::Plan () {

  double max_acc;
  
  double t = start__.s.v / start__.s.a;
  
  end.s.p = start__.s.p + start_.s.v*t + (0.5)*start_.s.a*t*t;
  end.s.v = start__.s.v + acc*t;
  end.s.a = start__.s.a;
  
  end.d.p = lane_to_d(start_lane);
  end.d.v = 0;
  end.d.a = 0;
  
  traj_.Generate(start, end, t);
}

*/

} // namespace