#include "ticker.h"

#include <math.h>

using std::ceil;


Ticker& Ticker::Instance() {
  
  static Ticker ticker_;
  
  return ticker_;
}


Ticker::Ticker() {
  
  ticks_         = 0;
  secs_          = 0;
  num_ticks_     = 5;
  secs_per_tick_ = 0.2;
  
  return;
}


void Ticker::Start(double secs) {
  
  SetTimeHorizon(secs);
  
  Start();
  
  return;
}

void Ticker::Start() {
  
  ticks_ = 0;
  secs_ = 0;
  
  return;
}


bool Ticker::IsTicking() {
  return ticks_ < num_ticks_;
}


void Ticker::Next() {
  
  ticks_ ++;
  secs_ += secs_per_tick_;
  
  return;
}


void Ticker::SetTimeHorizon(double secs) {
  
  int num_ticks_ = ceil(secs / secs_per_tick_);
  
  return;
};

double Ticker::secs() {
  
  return secs_;
}


int Ticker::ticks() {
  
  return ticks_;
}