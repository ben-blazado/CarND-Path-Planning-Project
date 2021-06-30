#include "ticker.h"

#include <math.h>

using std::ceil;


Ticker::Ticker(double secs_per_tick) {
  
  secs_per_tick_ = secs_per_tick;
  secs_          = 0;
  max_secs_      = 0;
  
  return;
}


void Ticker::Start() {
  
  secs_  = secs_per_tick_;
  
  return;
}


bool Ticker::IsTicking() {
  return secs_ <= max_secs_;
}


void Ticker::Next() {
  
  secs_ += secs_per_tick_;
  
  return;
}


void Ticker::MaxSecs(double max_secs) {
  
  max_secs_ = max_secs;
  
  return;
};

double Ticker::secs() {
  
  return secs_;
}


int Ticker::ticks() {
  
  return ticks_;
}