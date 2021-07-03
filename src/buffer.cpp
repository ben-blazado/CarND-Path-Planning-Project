#include "buffer.h"

#include <mutex>

namespace PathPlanning {
  
using std::unique_lock;
using std::mutex;
using std::defer_lock;

template <typename T>
Buffer<T>::Buffer () {
  
  updated_ = false;
  
  return;
}


template <typename T>
bool Buffer<T>::Write(T& d) {
  
  bool successful;
  unique_lock<mutex> l(m_, defer_lock);
  
  if (l.try_lock()) {
    data_      = d;
    updated_   = true;
    l.unlock();
    successful = true;
  }
  else
    successful = false;
  
  return successful;
}

template <typename T>
bool Buffer<T>::Read(T& d) {
  
  bool successful;
  unique_lock<mutex> l(m_, defer_lock);
  
  if (l.try_lock()) {
    if (updated_) {
      d = data_;
      updated_ = false;
      successful  = true;
    }
    else
      successful = false;
    l.unlock();
  }
  else
    successful = false;
  
  
  return successful;
}

}