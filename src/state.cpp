#include <state.h>

vector<CostFunction> State::cost_functions_{
    State::SpeedCost, 
    State::CollisionCost};
    
State::frenet(FrenetKinematic& frenet) {
  
  frenet_ = frenet;
  
  return;
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
  */
  
  return cost;
}

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
  

int DToLane (double d) {
  
  int lane = std::round ((d - 2.0) / 4.0)
  
  return (lane > 0) ? lane : 1;
}

double LaneToD (int lane) {
  
  double d = 4.0*lane + 2.0;
  
  return d;
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
  
  lane = DToLane(frenet_.d.p)
  end.d.p = LaneToD(lane);
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
