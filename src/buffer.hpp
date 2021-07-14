#ifndef BUFFER_H
#define BUFFER_H

#include <mutex>

namespace PathPlanning {

using std::unique_lock;
using std::mutex;
using std::defer_lock;

// T should be a struct
template <typename T>
class Buffer {
  
  public:
    Buffer();
    bool Write(const T& d);
    bool TryRead(T& d);
    bool Read(T& d);    
    bool ReadDirty(T& d);    
    
  private:
  
    mutex m_;
    T data_;
    bool updated_;
};

template <typename T>
Buffer<T>::Buffer () {
  
  updated_ = false;
  
  return;
}

// write data from d to the buffer.
template <typename T>
bool Buffer<T>::Write(const T& d) {
  
  bool successful;
  unique_lock<mutex> l(m_, defer_lock);
  
  l.lock();
  data_      = d;
  updated_   = true;
  l.unlock();
  
  successful = true;

  /*
  if (l.try_lock()) {
    data_      = d;
    updated_   = true;
    l.unlock();
    successful = true;
  }
  else
    successful = false;
  */
  
  return successful;
}

// try to read data from the buffer to d.
template <typename T>
bool Buffer<T>::TryRead(T& d) {
  
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


template <typename T>
bool Buffer<T>::Read(T& d) {
  
  bool successful;
  unique_lock<mutex> l(m_, defer_lock);
  
  l.lock();

  if (updated_) {
    d = data_;
    updated_ = false;
    successful  = true;
  }
  else
    successful = false;

  l.unlock();
  
  return successful;
}

template <typename T>
bool Buffer<T>::ReadDirty(T& d) {
  
  bool successful;
  unique_lock<mutex> l(m_, defer_lock);
  
  l.lock();

  d = data_;
  updated_ = false;
  successful  = true;

  l.unlock();
  
  return successful;
}



}

#endif