#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <thread>
#include <vector>
#include <mutex>

#include "helpers.h"

using std::vector;
using std::mutex;

// PVA - position, velocity, acceleration
typedef struct {
  double p;
  double v;
  double a;
} Kinematic;

// SD - Frenet s and d coordinates
typedef struct {
  Kinematic s;
  Kinematic d;
} FrenetKinematic;

typedef struct {
  Kinematic x;
  Kinematic y;
} CartesianKinematic;

typedef struct {
  double x;
  double y;
} CartP;

typedef struct {
  double x;
  double y;
} CartV;

typedef struct {
  double s;
  double d;
} FrenetSeparation;

typedef struct {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  time_point tp;
} LocalizationData;

class Localization {
  public:
  
    Localization(vector<double>& maps_s, vector<double>& maps_x, 
        vector<double>& maps_y, double secs_per_update);
        
    ~Localization();

    vector<double> CalcXY (double s, double d);
    vector<double> CalcFrenet (double x, double y, double theta);
    
    FrenetKinematic frenet();
    
    void Update (double car_x, double car_y, double car_s, double car_d,
        double car_yaw, double car_speed);
        
    void Run();

    
  private:
  
    std::thread thread_;
    bool thread_alive_;
    std::mutex mutex_;
    bool updated_;
    
    vector<double> maps_s_;
    vector<double> maps_x_;
    vector<double> maps_y_;
    
    LocalizationData localization_data_;
    CartesianKinematic cart_;
    FrenetKinematic frenet_;
    
    double secs_per_tick_;
    
};

#endif