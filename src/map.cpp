#include "map.h"
#include "helpers.h"

#include <iostream>

namespace PathPlanning {
  
using std::cout;
using std::endl;
  
double Frenet::max_s_;
double Frenet::half_max_s_;

Frenet::Frenet() {

  return;
}


Frenet::Frenet(double s, double d) {

  if (s >=  max_s_)
    s_ = fmod(s, max_s_);
  else
    s_ = s;
  d_ = d;
  
  return;
}

Frenet Frenet::operator =(Frenet f) {
  
  if (f.s_ >=  max_s_)
    s_ = fmod(f.s_, max_s_);
  else
    s_ = f.s_;
  
  d_ = f.d_;
  
  return *this;
}

Frenet Frenet::operator +(Frenet f) {
  Frenet sum;
  
  sum.s_ = s_ + f.s_;
  if (sum.s_ >= max_s_)
    sum.s_ = fmod (sum.s_, max_s_);
  
  sum.d_ = d_ + f.d_;
  
  return sum;
}


// Gets shorter distance on the s-axis between this and  f 
// since the x axis wraps around at max_s.
Frenet Frenet::operator -(Frenet f) {
  Frenet diff;
  
  diff.s_ = s_ - f.s_;
  // get shorter distance if the difference is greater than half of the max s.
  if (fabs(diff.s_) > half_max_s_)
    if (s_ < f.s_)
      diff.s_ = (s_ + max_s_) - f.s_;
    else
      diff.s_ = s_ - (f.s_ + max_s_);
  diff.d_ = d_ - f.d_;
  return diff;
}

Frenet Frenet::operator /(double divisor) {
  Frenet q;
  q.s_ = s_ / divisor;
  q.d_ = d_ / divisor;
  return q;
}

void Frenet::Max(Frenet f) {
  if (f.s_ > s_)
    s_ = f.s_;
  if (f.d_ > d_)
    d_ = f.d_;
  return;
}

Origin::Origin (double x, double y, double rotation) {
  
  x_ = x;
  y_ = y;
  rotation_ = rotation;
  
  return;
}

void Origin::Transform(Cartesian& p) {
  
  // shift point relative to the instance's origin x_, y_
  p.x -= x_;
  p.y -= y_;
  
  // rotate point about the instance's orgin
  double sin_rot = sin(rotation_);
  double cos_rot = cos(rotation_);
  
  // store x and y coordinates temporarily before transforming!!!!
  double x = p.x;  
  double y = p.y;
  
  p.x = x*cos_rot - y*sin_rot;
  p.y = x*sin_rot + y*cos_rot;
  
  return;
}
  

// Transforms p from local origin system back to the cartesian system.
// p should have been previously transformed with Origin::Transform().
void Origin::Restore(Cartesian& p) {
  
  // unrotate point
  double sin_rot = sin(-rotation_);
  double cos_rot = cos(-rotation_);
  
  double x = p.x;
  double y = p.y;
  
  p.x = x*cos_rot - y*sin_rot;
  p.y = x*sin_rot + y*cos_rot;
  
  // unshift point relative to origin 0,0
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
  
  Frenet::max_s_ = max_s;
  Frenet::half_max_s_ = max_s/2;
  
  return;
}


double Map::Add(double s, double ds) {
  
  return fmod (s + ds, max_s_);
}


double Map::Diff(double s2, double s1) {
  
  if (s1 > s2)
    s2 += max_s_;
  
  return s2 - s1;
}

// Returns s such that s in [0, max_s_].
// Typically used after adding some value to s.
double Map::Normalize(double s) {
  
  return fmod (s, max_s_);
}

int Map::D2Lane (double d) {
  
  int lane = std::round ((d - 2.0) / 4.0);
  
  return lane;
}

double Map::Lane2D (int lane) {
  
  double d = 4.0*lane + 2.0;
  
  return d;
}

// Returns an index to maps_s where s in [maps_s[i], maps_s[i++]).
// TODO: delete function? no longer needed?
int Map::GetStartPoint(double s) {
  
  // use a linear search of maps_s since its pretty small
  // start search from last element and go backward
  int i = num_map_points_ - 1;
  while (maps_s_[i] > s and i > 0) 
    i --;
  
  return i;
}

// Returns an index to maps_s where p is in between maps_s[i] and maps_s[i+1].
// Used in CalcFrenet in finding starting point for interpolating the best
// s and d values.
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
 
void Map::CalcCartesian(const Frenet& f, Cartesian& p) {
  
  //TODO: check this condition.
  if (f.s_ >= max_s_) {
    cout << "*** f.s is greater than max_s. exiting... ***" << endl;
    exit(1);
  }
  
  p.x = sx_(f.s_) + f.d_*snx_(f.s_);
  p.y = sy_(f.s_) + f.d_*sny_(f.s_);
  
  return;
}

void Map::CalcFrenet(const Cartesian& p, Frenet& f) {
  
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
  
  f.s_ = s;
  //f.s = (s > max_s_) ? Normalize(s) : s;
  f.d_ = (dx - dy) / (snx - sny);   // check for zeros?
  
  return;
}
  

}