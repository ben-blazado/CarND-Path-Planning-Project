#include <limits.h>

#include <behavior.h>
#include <state.h>

void Behavior::Behavior(Localization& localization) {
  
  /*
  vector<State> states_(
      KeepLane(),
      ChangeLaneLeft(),
      ChangeLaneRight()
  );
  */
  
  vector<State> states_(
      KeepLane()
  );
  
  localization_ = localization;
  
  return;
}


void Behavior::SelectBestState(){
  
  State::SetCurrSD(localization_.sd());
  
  int lowest_cost = INT_MAX;   //limits.h
  int i_best_state;
  
  for (int i=0; i < states.size(); i ++) {
    State& state = states[i];
    
    state.Plan();
    cost = state.Cost()
    if (cost < lowest_cost) {
      lowest_cost = cost;
      i_best_state = i;
    }
  }
  
  i_curr_state_ = i_best_state;
  
  return;
}

void Behavior::CreatePlan(vector<double>& x_vals, vector<double>& y_vals){
  
  SelectBestState();
  
  states_[i_curr_state_].GetXYVals(x_vals, y_vals);
}