#ifndef TICKER_H
#define TICKER_H

class Ticker {
  public:
  
    static Ticker& Instance();
    
    void Start(double secs);
    void Start();
    bool IsTicking();
    void Next();
    
    void SetTimeHorizon(double secs);
    
    // setters getters
    double secs();
    int    ticks();
    void   secs_per_tick(double secs) {secs_per_tick_ = secs;};
    double secs_per_tick() {return secs_per_tick_;};
    
  private:
  
    // private constructor for singleton
    Ticker();

    int ticks_;
    double secs_;
    
    int num_ticks_;
    double secs_per_tick_;
};

#endif