#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <thread>
#include <vector>
#include <mutex>

#include "helpers.h"
#include "spline.h"

using std::vector;
using std::mutex;
using std::thread;

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
} FrenetK;

typedef struct {
  Kinematic x;
  Kinematic y;
} CartK;

typedef struct {
  double s;
  double d;
} FrenetP;

typedef struct {
  double x;
  double y;
} CartP;

typedef struct {
  double x;
  double y;
} CartV;

typedef struct {
  double delta_s;
  double delta_d;
} FrenetDiff;

typedef struct {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  time_point tp;
} LocalizationData;


class Origin {
  public:
    Origin (double origin_x, double origin_y, double rotation);   
  
    void Transform (CartP& p);
    void Restore (CartP& p);
    
    void Transform(double& angle);
    void Restore(double& angle);
  
  private:
    double x_;
    double y_;
    double rotation_;
};
    

class spline2 : public tk::spline {
  
  
  public:
    spline2() : spline() {
      ;
    }
    
    spline2(const std::vector<double>& X, const std::vector<double>& Y) :
        tk::spline(X, Y) {
      ;
    }
        
    vector<double> where_deriv_zero();
    vector<double> closest_x(double x, double y);
};


class Localization {
  public:
  
    Localization(vector<double>& maps_s, vector<double>& maps_x, 
        vector<double>& maps_y, vector<double>maps_dx, vector<double> maps_dy, 
        double secs_per_tick);
        
    ~Localization();
    
    static constexpr double kMaxSVal_ = 6946.0;   // per project readme.md

    CartP   CalcXY(FrenetP frenet_p);
    FrenetP CalcSD(CartP cart_p);
    
    FrenetK frenet();
    
    void Update (double car_x, double car_y, double car_s, double car_d,
        double car_yaw, double car_speed);
        
    void Run();
    
    void Test();
    
  private:
  
    vector<double> maps_s_;
    vector<double> maps_x_;
    vector<double> maps_y_;
    vector<double> maps_dx_;
    vector<double> maps_dy_;
    vector<double> maps_s_theta_;
    
    int num_map_points_;
    
    spline2 spline_sx_;
    spline2 spline_sy_;
    spline2 spline_sdx_;
    spline2 spline_sdy_;
    spline2 spline_y_;
    
    int GetStartPoint(double s);
    int GetStartPoint(CartP p);
    
    thread thread_;
    bool   thread_alive_;
    mutex  mutex_;
    bool   updated_;
    
    LocalizationData localization_data_;
    CartK            cart_;
    FrenetK          frenet_;
    
    double secs_per_tick_;
    
};

#endif