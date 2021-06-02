#ifndefine BEHAVIOR_H
#define BEHAVIOR_H

#include <localization.h>

class Behavior {
  
  public:
    Behavior(Localization& localization);
    void Plan(vector<double>& xvals, vector<double>& yvals);
      
  private:
    SelectBestState();
    
    vector<State> states_;
    int           i_curr_state_;
    
    Localization& localization;
};


#endif