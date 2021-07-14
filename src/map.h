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


class Frenet{
  
  friend class Map;
  
  public:
  
    Frenet();
    Frenet(double s, double d);

    double s() { return s_; };
    double d() { return d_; };
  
    Frenet operator =(Frenet f);
    Frenet operator +(Frenet f);
    Frenet operator -(Frenet f);
    Frenet operator /(double divisor);
    void Max(Frenet f);
  
  protected:
  
    double s_;
    double d_;
  
    // initialized by Map
    static double max_s_;  
    static double half_max_s_;
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
    
    void CalcCartesian (const Frenet& f, Cartesian& p);
    void CalcFrenet    (const Cartesian& p, Frenet& f);
    
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