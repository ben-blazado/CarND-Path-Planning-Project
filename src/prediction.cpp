#include "prediction.h"
#include "map.h"

#include <iostream>

namespace PathPlanning {
  
using std::cout;
using std::endl;


Prediction::Prediction (Behavior& behavior, Map& map, double prediction_secs, 
    double secs_per_update, double near_distance) 
    : behavior_(behavior), map_(map) {
  
  processing_      = false;
  near_distance_   = near_distance;
  prediction_secs_ = prediction_secs;
  secs_per_update_ = secs_per_update;
  
  return;
}


Prediction::~Prediction () {
  
  processing_ = false;
  
  if (thread_.joinable())
    thread_.join();
  
  return;
}


void Prediction::Input (InputData& in) {
  
  //cout << "prediction input" << endl;
  // cout << "num cars " << in.sensor_fusion.size() << endl;
  //cout << "input car x" << in.x << endl;
  in_buf_.Write(in);
  // cout << "num cars in buffer " << in_buf_.data_.sensor_fusion.size() << endl;
  return;
}


void Prediction::SelectCarsNearby(const double car_x, const double car_y, 
    const vector<vector<double>>& sensor_fusion, 
    vector<vector<double>>& cars_nearby) {
      
  //cout << "car x " << car_x << endl;
  cars_nearby.clear();
  
  //cout << "SelectCarsNearby sensor_fusion size " << sensor_fusion.size() << endl;
  for (int i = 0; i < sensor_fusion.size(); i ++) {
    const vector<double>& other_car = sensor_fusion[i];
    const double& other_car_x = other_car[1];
    const double& other_car_y = other_car[2];
    
    const double dist = distance (car_x, car_y, other_car_x, other_car_y);
    //cout << "other car x " << other_car_x << endl;
    if (dist < near_distance_)
      cars_nearby.push_back(other_car);
  }
  
  // cout << "cars nearby " << cars_nearby.size() << endl;
  
  return;
}


void Prediction::CreateOutput(time_point tp, const vector<vector<double>>& 
    cars_nearby, Behavior::PredictionOutput& pre_out) {

  pre_out.tp = tp;
  
  for (int i=0; i < cars_nearby.size(); i ++) {

    // aliases for readability
    const vector<double>& other_car = cars_nearby[i];
    const double& x = other_car[1];
    const double& y = other_car[2];
    const double& vx = other_car[3];
    const double& vy = other_car[4];

    Cartesian p1 = {x, y};
    Cartesian p2 = {x + vx, y + vy};
    
    Frenet f1, f2;
    map_.CalcFrenet(p1, f1);
    map_.CalcFrenet(p2, f2);
    Frenet v = f2 - f1;  // velocity in frenet system
    
    Behavior::Car car = {f1, v};
    
    pre_out.other_cars.push_back(car);

    //vf.d(0); // TODO: check v on d
    
    
    // cout << i << " car " << vf.s() << " " << vf.d() << endl;
    // cout << i << " fre " << f1.s() << " " << f1.d() << " " << map_.D2Lane(f1.d()) << endl;
    
    /*
    vector<Frenet> predicted_path;

    // predict frenet waypoints
    Frenet f;
    for (double secs = t; secs < prediction_secs_; secs += t) {
      // d = di + v*t
      f = f1 + vf*secs;
      predicted_path.push_back(f);
    }
    
    predictions.push_back(prediction);
    prediction.clear();
    */
  }
  
  return;
}


void Prediction::ProcessInputs() {
  
  while (processing_) { 
    
    InputData in;
    if (in_buf_.TryRead(in)) {
      
      vector<vector<double>> cars_nearby;
      SelectCarsNearby(in.x, in.y, in.sensor_fusion, cars_nearby);
      
      Behavior::PredictionOutput pre_out;
      CreateOutput (in.tp, cars_nearby, pre_out);
      // cout << "prediction module size " << pre_in.predictions.size() << endl;
      behavior_.Input(pre_out);
      
    }
  }
}

void Prediction::Run() {
  
  processing_ = true;
  thread_ = thread( [this] {ProcessInputs ();} ); // thread
    
  return;
}



} // namespace
