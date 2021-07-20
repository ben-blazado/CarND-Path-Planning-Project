#ifndef PREDICTION_H
#define PREDICTION_H

#include "helpers.h"
#include "buffer.hpp"
#include "behavior.h"
#include "map.h"

#include <thread>
#include <vector>


namespace PathPlanning {
  
using std::vector;
using std::thread;
using std::cout;
using std::endl;

class Prediction {

  public: 
  
    Prediction (Behavior& behavior, Map& map, double prediction_secs, 
        double secs_per_update, double near_distance);
    ~Prediction();
    
    struct InputData {
      time_point tp;
      double x;
      double y;
      vector<vector<double>> sensor_fusion;
      // TODO: remove comments below on operator overloading after testing.
      // assignment overload needed for buffer reads to work
      // properly due to vector of vector of double member (sensor_fusion).
      InputData& operator=(const InputData& in_rhs) {
        tp = in_rhs.tp;
        x = in_rhs.x;
        y = in_rhs.y;
        sensor_fusion.clear();
        for (int i = 0; i < in_rhs.sensor_fusion.size(); i ++)
          sensor_fusion.push_back(in_rhs.sensor_fusion[i]);
        //cout << "sensor fusion size " << sensor_fusion.size() << endl;
        return *this;
      }
    };
    void Input(InputData& in);
    void Run();
    
  private:
  
    Behavior& behavior_;
    Map& map_;
    double near_distance_;
    double prediction_secs_;
    double secs_per_update_;
  
    bool processing_;
    thread thread_;
    
    Buffer<InputData> in_buf_;
    
    void ProcessInputs();
    void SelectCarsNearby(const double x, const double y, 
        const vector<vector<double>>& sensor_fusion,
        vector<vector<double>>& cars_nearby);
    void CreateOutput(time_point tp, const vector<vector<double>>& 
        cars_nearby, Behavior::PredictionOutput& pre_out);
        
    
};

} // namespace


#endif // PREDICTION_H