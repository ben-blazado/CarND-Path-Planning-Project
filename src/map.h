#ifndef MAP_H
#define MAP_H

#include "spline.h"

namespace PathPlanning {
  
using std::vector;
using tk::spline;
  
  
// PVA - position, velocity, acceleration
template <typename T>
struct Kinematic{
  T p;
  T v;
  T a;
};

struct Frenet{
  double s;
  double d;
};


struct Cartesian{
  double x;
  double y;
};


class Origin {
  public:
    Origin (double origin_x, double origin_y, double rotation);   
  
    void Transform (Cartesian& p);
    void Restore (Cartesian& p);
    
    void Transform(double& angle);
    void Restore(double& angle);
  
  private:
    double x_;
    double y_;
    double rotation_;
};

  
class Map {
  
  public:
  
    Map (vector<double>& maps_s, vector<double>& maps_x, vector<double>& maps_y, 
        vector<double>& maps_nx, vector<double>& maps_ny, double max_s);
    
    Cartesian CalcCartesian (Frenet f);
    Frenet    CalcFrenet    (Cartesian p);
    
    double Add(double s, double ds);
    double Diff(double s2, double s1);
    double Normalize(double s);
    
    double Lane2D(int lane);
    int D2Lane(double d);
        
  private:
  

    int GetStartPoint(double s);
    int GetStartPoint(Cartesian p);

    vector<double>& maps_s_;
    vector<double>& maps_x_;
    vector<double>& maps_y_;
    vector<double>& maps_nx_;
    vector<double>& maps_ny_;
    
    int    num_map_points_;
    double max_s_;
    
    spline sx_;
    spline sy_;
    spline snx_;
    spline sny_;
  
};

 
  
} // namespace

#endif // MAP_H