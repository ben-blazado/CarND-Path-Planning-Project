#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "map.h"
#include "path.h"

#include <thread>
#include <mutex>

namespace PathPlanning {

using std::mutex;
using std::thread;

class Trajectory {
  
  public:
  
    Trajectory(Map& map);
    ~Trajectory();
    
    void Receive(const vector<Frenet>& waypoints);
    void Receive(vector<double>& prev_path_x, vector<double>& prev_path_y);
    void Run();
    void GetNextXYVals(vector<double>& next_x_vals, vector<double>& next_y_vals);
  
  private:

    Map& map_;
    
    bool processing_;
    thread thread_;
    mutex  mutex_;
    struct {
      mutex m;
      bool  updated;
      vector<Frenet> waypoints;
      vector<double> prev_path_x;
      vector<double> prev_path_y;
    } buf_;
    bool updated_;
    struct {
      mutex m;
      bool  updated;
      vector<double> next_x_vals;
      vector<double> next_y_vals;
    } snd_buf_;
    
    vector<Frenet> waypoints_;
    vector<double> prev_path_x_;
    vector<double> prev_path_y_;
    vector<double> next_x_vals_;
    vector<double> next_y_vals_;
    
    void Update();
    void ProcessUpdates();
    void Send();
    
};

} // PathPlanning

#endif
