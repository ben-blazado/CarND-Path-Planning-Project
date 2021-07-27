#include "path.h"
#include "Eigen-3.3/Eigen/Dense"

#include <iostream>

namespace PathPlanning {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::abs;
using std::min;
using std::max;
using std::cout;
using std::endl;


double QuinticPosition (double secs, vector<double> coeffs) {
  
  double sum = 0.0;
  for (int i = 0; i < coeffs.size(); i ++) 
    sum += (coeffs[i] * pow(secs, i));
    
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
  VectorXd S_vec = VectorXd(3);
  VectorXd A_vec = VectorXd(3);
  
  T_mat <<   t3,    t4,    t5,
           3*t2,  4*t3,  5*t4,
            6*t, 12*t2, 20*t3;
           
  S_vec << end.p - (start.p + start.v*t + 0.5*start.a*t2),
           end.v - (start.v + start.a*t),
           end.a - start.a;
          
  A_vec = T_mat.inverse()*S_vec;
  
  return {start.p, start.v, start.a/2.0, A_vec[0], A_vec[1], A_vec[2]};
}

double Path::secs_per_update_;

// class stats. used in calculating scores per path instance.
double Path::max_dd_;
double Path::max_ds_;
double Path::max_avg_v_s_;
double Path::max_last_v_s_;

// resets stats for entire class. 
// call before calculating stats for each instance.
void Path::ResetStats() {
  
  double secs_per_update_ = 0;
  double max_dd_ = 0;
  double max_ds_ = 0;
  double max_avg_v_s_ = 0;
  double max_last_v_s_ = 0;
  
  return;
}


Path::Path (time_point tp, Kinematic<Frenet>& start, Kinematic<Frenet>& end, double t,
    int num_waypoints) {
      
  tp_ = tp;
  end_ = end;
  
  //Kinematic<double> start_s = {start.p.s(), start.v.s(), 0};
  //Kinematic<double> end_s   = {end.p.s(),   end.v.s(),   0};

  Kinematic<double> start_s = {start.p.s(), start.v.s(), start.a.s()};
  Kinematic<double> end_s   = {end.p.s(),   end.v.s(),   end.a.s()};
  vector<double> s_coeffs = JMTCoeffs(start_s, end_s, t);
  
  Kinematic<double> start_d = {start.p.d(), 0, 0};
  Kinematic<double> end_d   = {end.p.d(),   0,   0};

//  Kinematic<double> start_d = {start.p.d(), start.v.d(), start.a.d()};
//  Kinematic<double> end_d   = {end.p.d(),   end.v.d(),   end.a.d()};
  vector<double> d_coeffs = JMTCoeffs(start_d, end_d, t);

  // use starting position as first waypoint.
  // TODO: explain why start point is included
  waypoints_.push_back(start.p);
  
  double secs = 0;

  for (int i = 0; i < num_waypoints; i ++) {

    // calulate next waypoint {s, d}.
    secs += secs_per_update_;
    double s = QuinticPosition(secs, s_coeffs); 
    double d = QuinticPosition(secs, d_coeffs);

    // waypoints should automatically be normalized between [0, max_s_).
    // see class def of Frenet.
    Frenet waypoint = {s, d};
    waypoints_.push_back(waypoint); 
  }
  
  UpdateStats();
  return;
}


// update instance stats.
// path must have at least 2 waypoints.
void Path::UpdateStats() {
  
  // initialize instance stats.
  ds_       = 0;
  dd_       = 0;
  avg_v_s_  = 0;
  last_v_s_ = 0;
  
  // used in averageing velocity over entire path.
  double sum_v_s = 0;
  
  int num_waypoints = waypoints_.size();

  for (int i = 1; i < num_waypoints; i ++) {

    // need at least two waypoints
    Frenet wp2 = waypoints_[i];
    Frenet wp1 = waypoints_[i-1];  // TODO: whatif i == 0.
    
    // accumulate shift along d-axis
    dd_ += fabs(wp2.d() - wp1.d());
    Path::max_dd_ = max(Path::max_dd_, dd_);
    
    // acculate total velocity along s axis for averaging.
    // used to calculate avg velocity along entire path.
    sum_v_s += ((wp2.s() - wp1.s()) /  secs_per_update_);
  }
  
  // calculate remaining stats
  int last = num_waypoints - 1;
  
  // calculate total longitudinal distance
  ds_ += waypoints_[last].s() - waypoints_[0].s();
  Path::max_ds_ = max(Path::max_ds_, ds_);
  
  // calculate average velocity along entire path
  avg_v_s_ = sum_v_s / waypoints_.size();
  Path::max_avg_v_s_ = max(Path::max_avg_v_s_, avg_v_s_);
  
  // calculate exit velocity of path (velocity at last waypoint)
  last_v_s_ = (waypoints_[last].s() - waypoints_[last - 1].s()) 
      / secs_per_update_;
  Path::max_last_v_s_ = max(Path::max_last_v_s_, last_v_s_);
  
  return;
}


void Path::UpdateScore() {
  
  score_ = 0.0;
  for (int i = 0; i < Path::score_functions_.size(); i ++) {
    double w = Path::score_functions_[i].weight;
    score_ += w*Path::score_functions_[i].f(*this);
  }
  
  return;
}

  
double Path::DistanceScore (Path& path) {
  return path.ds_ / Path::max_ds_;
}


double Path::LaneKeepingScore(Path& path) {
  return fabs (Path::max_dd_ - path.dd_) / Path::max_dd_;
}


double Path::AverageVelocityScore(Path &path) {
  return (path.avg_v_s_ / Path::max_avg_v_s_);
}

  
double Path::LastVelocityScore(Path &path) {
  return path.last_v_s_ / Path::max_last_v_s_;
}

//TODO: need sccore for being in lane
//TODO: can't touch right side lane 2.0
vector<Path::ScoreFunction> Path::score_functions_ = {
  {
    Path::DistanceScore, 
    0.10   //15
  },
  {
    Path::LaneKeepingScore, 
    0.05 //10  //15
  },
  {
    Path::AverageVelocityScore, 
    0.05  //10
  },
  {
    Path::LastVelocityScore,
    0.80 //70 //60
  }
};

bool Path::GreaterThan(Path& p1, Path& p2) {
  return p1.score_ > p2.score_;
}
 
} // namespace




  /**/
  /*
  Frenet f;
  f.s = start.p.s;
  f.d = start.p.d;
  
  double v = start.v.s;
  double acc = 4.4704;  // m/s2  
  cout << "Path()::start v " << v << endl;

  double secs = 0;
  for (int i = 1; i <= num_waypoints && secs <= t; i ++) {
    secs += secs_per_update_;
  //for (double secs = secs_per_update_; secs <= t; secs += secs_per_update_) {
    
    if (v < 21)      // (49 / 2.237))
      v += acc * secs_per_update_;
    f.s += v * secs_per_update_;
    
    waypoints_.push_back(f);
  } 
  */


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
