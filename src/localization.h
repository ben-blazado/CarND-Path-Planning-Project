#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>

#include "helpers.h"
#include "behavior.h"
#include "map.h"

namespace PathPlanning {

using std::vector;
using std::mutex;
using std::thread;
using std::condition_variable;


class Localization {
  public:
  
    Localization(Behavior& behavior, Trajectory& trajectory, Map& map,
        double max_s, double secs_per_tick);
    ~Localization();
        
    void Receive(double car_x, double car_y,  double car_yaw, 
        double car_speed, const vector<double>& prev_path_x, 
        const vector<double>& prev_path_y);
    void Run();
    
    void Test();
    
  private:
  
    Behavior&   behavior_;
    Trajectory& trajectory_;
    Map&        map_;
    double      max_s_;
    
    thread thread_;
    bool   processing_;
    struct {
      mutex          m;
      bool           updated;
      double         car_x;
      double         car_y;
      double         car_yaw;
      double         car_speed;
      vector<double> prev_path_x;
      vector<double> prev_path_y;
    } buf_;
    bool           updated_;
    double         car_x_;
    double         car_y_;
    double         car_yaw_;
    double         car_speed_;
    vector<double> prev_path_x_;
    vector<double> prev_path_y_;
    double         dt_;  // secs per interval update; should be 0.02 
    
    void Update();
    void ProcessUpdates();     
    
};

} // namespace PathPlanning

#endif