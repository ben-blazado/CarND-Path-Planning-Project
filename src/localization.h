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
  double s;
  double d;
} FrenetP;


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
    

enum Direction {
    kRight,
    kStraight,
    kLeft
};

class Localization {
  public:
  
    Localization(vector<double>& maps_s, vector<double>& maps_x, 
        vector<double>& maps_y, vector<double>maps_dx, vector<double> maps_dy, 
        double secs_per_tick);
        
    ~Localization();

    CartP   CalcXY(FrenetP frenet_p);
    FrenetP CalcFrenet(CartP cart_p);
    
    FrenetKinematic frenet();
    
    void Update (double car_x, double car_y, double car_s, double car_d,
        double car_yaw, double car_speed);
        
    void Run();
    
    void Test();
    
  private:
  
    std::thread thread_;
    bool thread_alive_;
    std::mutex mutex_;
    bool updated_;
    
    vector<double> maps_s_;
    vector<double> maps_x_;
    vector<double> maps_y_;
    vector<double> maps_dx_;
    vector<double> maps_dy_;
    vector<double> maps_s_theta_;
    
    int num_map_points_;
    
    vector<Origin>    origin_;
    vector<Direction> direction_;
    vector<CartP>     end_point_;
    vector<double>    end_theta_;
    vector<double>    yaw_rate_;
    vector<double>    arc_vel_;
    vector<double>    vel_;
    
    static constexpr double kMaxSVal_ = 6946.0;   // per project readme.md
    
    int GetStartPoint(double s);
    int GetStartPoint(CartP p);
    
    LocalizationData   localization_data_;
    CartesianKinematic cart_;
    FrenetKinematic    frenet_;
    
    double secs_per_tick_;
    
};

#endif