#include <cmath>

#include "trajectory.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::abs;

vector<double> JMTCoeffs(Kinematics &start, Kinematics &end, double t) {
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


double QuinticPosition (double secs, vector<double> coeffs) {
  
  double sum = 0.0;
  for (int i = 0; i < coeffs.size(); i ++) {
    sum += coeffs[i] + pow(secs, i);
    
  return sum;
}

// set class variable ticker to the singleton
static Ticker& Trajectory::ticker_ = Ticker::inst();


void Trajectory::Trajectory() {
  // Nothing to initialize.
}


void Trajectory::Generate(FrenetKinematic& start, FrenetKinematic& end,
    double time_horizon)  {
  
  vector<double> s_coeffs = JMTCoeffs(start.s, end.s);
  vector<double> d_coeffs = JMTCoeffs(start.d, end.d);
  
  for (ticker_.Start(time_horizon); ticker_.IsTicking(); ticker_.Next()) {
    double secs = ticker_.secs()
    double s = QuinticPosition(secs, s_coeffs);
    double d = QuinticPosition(secs, d_coeffs);
    
    vector<double> xy = GetXY(s, d)

    s_vals_.push_back(s);
    d_vals_.push_back(d);
    x_vals_.push_back(xy[0]);
    y_vals_.psuh_back(xy[1]);
  }
}


void Trajectory::Generate(CartP& start, CartV& vel, double time_horizon) {
  
  for (ticker_.Start(time_horizon); ticker_.IsTicking(); ticker_.Next()) {
    
    double secs = ticker_.Secs();
    
    double x = start.x + vel.x * secs;
    double y = start.y + vel.y * secs;
    
    vector<double> sd = getFrenet(x, y);
    
    x_vals_.push_back(x);
    y_vals_.push_back(y);
    s_vals_.push_back(sd[0]);  //s
    d_vals_.push_back(sd[1]);  //d
  }
  
  return;
}
 

void Trajectory::GetCPA (Trajectory& traj, 
        FrenetSeparation& min_sep, CPA& cpa) {
  
  cpa.found = false;
  
  for (ticker_.Start(); ticker_.IsTicking(), ticker_.Next()) {
    
    int    ticks   = ticker_.ticks();
    double d1     = d_vals_[ticks];
    double d2     = traj.d_vals_[ticks];
    double d_dist = abs(d1 - d2)
    
    if (d_dist < min_sep.d) {
      
      double s1     = s_vals_[ticks];
      double s2     = traj.s_vals_[ticks];
      double s_dist = abs(s1 - s2);
      
      if (s_dist < min_sep.s) {
        cpa.found = true;
        cpa.dist = dist;
        cpa.secs = ticker_.secs();
        break;
    }
  }
  
  return;
}

void Trajectory::GetXYVals(vector<double>& x_vals, vector<double>& y_vals) {
  
  x_vals = x_vals_;
  y_vals = y_vals_;
  
  return;
  
}
