#ifndef TICKER_H
#define TICKER_H

class Ticker {
  public:
  
    Ticker(double secs_per_tick);
    
    void Start();
    bool IsTicking();
    void Next();
    
    void MaxSecs(double secs);
    
    // setters getters
    double           secs();
    int              ticks();
    void             secs_per_tick(double secs) {secs_per_tick_ = secs;};
    constexpr double secs_per_tick() {return secs_per_tick_;};
    
  private:
  
    // private constructor for singleton
    Ticker();

    int ticks_;
    double secs_;
    
    double max_secs_;
    double secs_per_tick_;
};

#endif