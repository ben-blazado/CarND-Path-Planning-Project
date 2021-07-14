#ifndef PREDICTION_H
#define PREDICTION_H

#include "helpers.h"
#include "buffer.hpp"

#include <thread>
#include <vector>


namespace PathPlanning {
  
using std::vector;
using std::thread;

struct SensorFusion {
  double id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};
  
  
class Prediction {

  public: 
  
    Prediction ();
    ~Prediction();
    
    struct InputData {
      time_point tp;
      vector<vector<double>> sensor_fusion;
    };
    void Input(const InputData& in);
    void Run();
    
  private:
  
    bool processing_;
    thread thread_;
    
    InputData         in_;
    Buffer<InputData> in_buf_;
    
    void ProcessInputs();
    
};

} // namespace


#endif // PREDICTION_H