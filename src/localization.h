#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <mutex>
#include <thread>
#include <vector>

#include "helpers.h"
#include "behavior.h"
#include "map.h"
#include "buffer.hpp"

namespace PathPlanning {

using std::vector;
using std::mutex;
using std::thread;


class Localization {
  public:
  
    Localization(Behavior& behavior, Trajectory& trajectory, Map& map,
        double secs_per_update);
    ~Localization();
        
    // input from main.cpp
    struct InputData {
      time_point     tp;
      double         car_x;
      double         car_y;
      double         car_yaw;
      double         car_speed;
      vector<double> prev_path_x;
      vector<double> prev_path_y;
    };
    void Input(InputData& in);
    void Run();
    
    //TODO: remove tests.
    void Test();
    
  private:
  
    Behavior&   behavior_;
    Trajectory& trajectory_;
    Map&        map_;
    
    thread thread_;
    bool   processing_;
    
    InputData         in_;
    Buffer<InputData> in_buf_;
    
    double dt_;  // secs per update (delta t); should be 0.02 
    
    void   Update();
    void   VerifyPrevPath(InputData& in);
    Frenet CalcLastPos();
    //TODO: change input params to const&!!!.
    Frenet CalcLastVel(Frenet last_f);
    Frenet CalcLastAcc(Frenet last_v);
    void   ProcessInputs();     
    
};

} // namespace PathPlanning

#endif