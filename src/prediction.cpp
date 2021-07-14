#include "prediction.h"

#include <iostream>

namespace PathPlanning {
  
using std::cout;
using std::endl;
  
Prediction::Prediction () {
  
  processing_ = false;
  
  return;
}


Prediction::~Prediction () {
  
  processing_ = false;
  
  if (thread_.joinable())
    thread_.join();
  
  return;
}



void Prediction::Input (const InputData& in) {
  
  in_buf_.Write(in);
  
  return;
}


void Prediction::ProcessInputs() {
  
  while (processing_) {
    if (in_buf_.TryRead(in_)) {
      cout << "Prediction processing input" << endl;
    }
  }
}

void Prediction::Run() {
  
  processing_ = true;
  thread_ = thread( [this] {ProcessInputs ();} ); // thread
    
  return;
}



} // namespace
