#ifndefine BEHAVIOR_H
#define BEHAVIOR_H

#include "state.h"

class Behavior {
  
  public:
    Behavior(Trajectory& trajectory, 
        int max_lanes, double speed_limit, double max_acc);
    
    void Run();
      
  private:
    SelectBestState();
    
    Trajectory trajectory_;
    
    int max_lanes_;
    int max_secs_;
    
    vector<double> speed_options_(3);
    enum class SpeedOption {
      kStop,
      kMaintain,
      kMaxLane
    };
    
    vector<State> states_;
};


#endif