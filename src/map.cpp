#include "map.h"
#include "helpers.h"

#include <iostream>

namespace PathPlanning {
  
  using std::cout;
  using std::endl;
  
Origin::Origin (double x, double y, double rotation) {
  
  x_ = x;
  y_ = y;
  rotation_ = rotation;
  
  return;
}

void Origin::Transform(Cartesian& p) {
  
  // shift point relative to the origin
  p.x -= x_;
  p.y -= y_;
  
  // rotate point about the new orgin
  double sin_rot = sin(rotation_);
  double cos_rot = cos(rotation_);
  
  double x = p.x;
  double y = p.y;
  
  p.x = x*cos_rot - y*sin_rot;
  p.y = x*sin_rot + y*cos_rot;
  
  return;
}
  
void Origin::Restore(Cartesian& p) {
  
  // unrotate point
  double sin_rot = sin(-rotation_);
  double cos_rot = cos(-rotation_);
  
  double x = p.x;
  double y = p.y;
  
  p.x = x*cos_rot - y*sin_rot;
  p.y = x*sin_rot + y*cos_rot;
  
  // unshift point relative to new origin
  p.x += x_;
  p.y += y_;
  
  return;
}
  
void Origin::Transform(double& angle) {
  
  angle += rotation_;
  
  return;
}

void Origin::Restore(double& angle) {
  
  angle -= rotation_;
  
  return;
  
}

Map::Map(vector<double>& maps_s, 
    vector<double>& maps_x, vector<double>& maps_y, 
    vector<double>& maps_nx, vector<double>& maps_ny, 
    double max_s) : 
    maps_s_(maps_s), 
    maps_x_(maps_x),
    maps_y_(maps_y),
    maps_nx_(maps_nx),
    maps_ny_(maps_ny) {
  
  // append last point data for max S to wrap back around to first element
  maps_s_.push_back(max_s);
  maps_x_.push_back(maps_x_[0]);
  maps_y_.push_back(maps_y_[0]);
  maps_nx_.push_back(maps_nx_[0]);
  maps_ny_.push_back(maps_ny_[0]);
  
  num_map_points_ = maps_s_.size();
  
  sx_  = spline(maps_s_, maps_x_);
  sy_  = spline(maps_s_, maps_y_);
  snx_ = spline(maps_s_, maps_nx_);
  sny_ = spline(maps_s_, maps_ny_);
  
  max_s_ = max_s;
  
}

int Map::GetStartPoint(double s) {
  
  // use a linear search of maps_s since its pretty small
  // start search from last element and go backward
  int i = num_map_points_ - 1;
  while (maps_s_[i] > s and i > 0) 
    i --;
  
  return i;
}

int Map::GetStartPoint(Cartesian p) {
  
  double lowest_d = std::numeric_limits<double>::infinity();
  int closest;
  
  // find closest map point by distance
  for (int i = 0; i < num_map_points_; i ++) {
    double d = distance(p.x, p.y, maps_x_[i], maps_y_[i]);
    if (d < lowest_d) {
      lowest_d = d;
      closest = i;
    }
  }
  
  // use closest map point to find starting point of map segment
  // that contains p
  // set p as local origin rotated by bearing from p to closest point
  Cartesian closest_p = {maps_x_[closest], maps_y_[closest]};
  double bearing = atan2(closest_p.y - p.y, closest_p.x - p.x);
  Origin origin(p.x, p.y, -bearing);    // closest_p is now in front of p  
  
  // find next_p to closest and transform to local origin p
  int next = (closest + 1) % num_map_points_;
  Cartesian next_p = {maps_x_[next], maps_y_[next]};
  origin.Transform(next_p);
  
  int start;
  // check if the segment [closest, next_p] contains p
  if (next_p.x > 0) {
    // the segment [closest, next_p] can not contain p
    // select prev to closest as start point
    // https://stackoverflow.com/a/33664449
    start = (num_map_points_ + closest - 1) % num_map_points_;
  }
  else
    // the segment [closest, next_p] must contain p
    // select closest point as start point
    start = closest;
  
  return start; // closest point "behind" along reference
}
 
Cartesian Map::CalcCartesian(Frenet f) {
  
  Cartesian p;
  
  p.x = sx_(f.s) + f.d*snx_(f.s);
  p.y = sy_(f.s) + f.d*sny_(f.s);
  
  return p;
}

Frenet Map::CalcFrenet(Cartesian p) {
  
  int    start_point = GetStartPoint(p);
  double s = maps_s_[start_point];
  
  double dx;
  double dy;
  double snx;
  double sny;
  
  int attempts = 0;
  double delta = std::numeric_limits<double>::infinity(); 
  const double kPrecision = 0.000000001;
  
  while (abs(delta) > kPrecision) {

    dx = p.x - sx_(s);
    dy = p.y - sy_(s);

    snx = snx_(s);
    sny = sny_(s);

    double sx_p  = sx_.deriv(1, s);  // s sub-x prime (first derivative)
    double sy_p  = sy_.deriv(1, s);

    double snx_p = snx_.deriv(1, s);
    double sny_p = sny_.deriv(1, s);
    
    double f = dx*sny - dy*snx;
    double f_p = dx*sny_p - sx_p*sny - dy*snx_p + sy_p*snx;
    
    delta = f/f_p;
    s = s - delta;
    
    attempts ++;
    if (attempts > 10) {
      cout << "s did not converge." << endl;
      break;
    }
    
  }
  
  Frenet f;
  f.s = s;
  f.d = (dx - dy) / (snx - sny);
  
  return f;
}
  

}