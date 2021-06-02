#ifndef PLANNER_H
#define PLANNER_H

class Planner {
  public:
    Planner();
    
    ~Planner();
    
    void Update(double x,
                double y,
                double s,
                double d,
                double yaw,
                double speed,
                vector<double> prev_path_x,
                vector<double> prev_path_y,
                double end_s,
                double end_d;
                
  private:
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double speed_;
    vector<double> prev_path_x_;
    vector<double> prev_path_y_;
    double end_s_;
    double end_y_;
    
    State state_;
};

                
                