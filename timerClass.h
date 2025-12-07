// Timer that doesn't halt the processor class 
// Programmer: Bruno Villalobos 
// Date: 15/10/2025

#ifndef TIMERCLASS_H
#define TIMERCLASS_H
#include <Arduino.h>


class timerClass {
      
  private:
  unsigned long previousMillis;    
  unsigned long currentMillis;
  bool TimingFlag = false;
  bool timeDone;
  void ChangeDetected(bool bit);
  
  public: 
  int time;
  struct TimerData {
    bool EN;
    bool TT;
    bool DN;
    unsigned long PRE;
    unsigned long ACC;
  };

    TimerData timerData;
  
  //unsigned long runMillis;
  timerClass ();
  // Timer function block
  bool timer_LOOP( bool enabled , unsigned long time );
  bool timer_TON( bool enabled , unsigned long time );
  bool timer_TOF( bool enabled , unsigned long time );

};

#endif //TIMERCLASS_H