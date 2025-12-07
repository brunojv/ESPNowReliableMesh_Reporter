#include "timerClass.h"

//Constructor 
timerClass::timerClass (){
timerData.DN = false;
previousMillis = 0; 
}

//Detect the bit chang function 

 void timerClass::ChangeDetected(bool bit){
  bool currentState = false;
  bool previousState = false;
  currentState = bit;  

  // Detect state change
  if (currentState != previousState) {
    Serial.print("State changed to: ");
    Serial.println(currentState ? "TRUE" : "FALSE");

    // Do something when the state changes
    if (currentState) {
      // State changed to true
    } else {
      // State changed to false
    }

    // Update previous state
    previousState = currentState;
  }
  
 } 


// timerLoop function block
bool timerClass::timer_LOOP(bool enabled , unsigned long time){
  
  if (enabled){
    unsigned long currentMillis = millis();
      timerData.DN = false;
      if ((currentMillis - previousMillis) > time) {
      timerData.DN = true;
      if (previousMillis != currentMillis){
        timerData.ACC++;
      }
      previousMillis = currentMillis;
      timerData.TT = true;              
    }
    return timerData.DN;
  }
  else {
    timerData.DN = false;
    timerData.ACC = 0;
    return timerData.DN;
  }
}

//timerOneshoot fucntion block
bool timerClass::timer_TON(bool enabled, unsigned long time){
  bool lastDN;
  if (enabled){
    unsigned long currentMillis = millis();
    
    //TT, Enlasep time
    if (!TimingFlag && ((currentMillis - previousMillis) <= time)){
      timerData.TT = true;
      timerData.ACC = currentMillis - previousMillis;
    }
    else {
      timerData.TT = false;
      timerData.ACC = time;
      
    }
                
    //Timing Done 
    if (!TimingFlag && ((currentMillis - previousMillis) >= time)) {
      previousMillis = currentMillis;      
      TimingFlag = true;
      timerData.DN = true;
      //lastDN = timerData.DN;}
      return timerData.DN;

    }else {
      if (!TimingFlag){
        timerData.DN = false;   
      }
      return timerData.DN;
    }
    
  }
  else {
    TimingFlag = false;
    timerData.DN = false;
    timerData.TT = false;
    timerData.ACC = 0;
    previousMillis = millis();
    lastDN = false;
    return timerData.DN;
  }
}

//timerOneshoot fucntion block
bool timerClass::timer_TOF(bool enabled, unsigned long time){
  bool lastDN;

  if (enabled){
    
    unsigned long currentMillis = millis();
    
    //TT, Enlasep time
    if (!TimingFlag && ((currentMillis - previousMillis) <= time)){
      timerData.TT = true;
      timerData.ACC = currentMillis - previousMillis;
    }
    else {
      timerData.TT = false;
      timerData.ACC = time;
      
    }
                
    //Timing Done 
    if (!TimingFlag && ((currentMillis - previousMillis) >= time)) {
      previousMillis = currentMillis;      
      TimingFlag = true;
      timerData.DN = true;
      //lastDN = timerData.DN;}
      return timerData.DN;

    }
    else {
      if (!TimingFlag){
        timerData.DN = false;   
      }
      return timerData.DN;
    }
    
  }
  else {
    TimingFlag = false;
    timerData.DN = false;
    timerData.TT = false;
    timerData.ACC = 0;
    previousMillis = millis();
    lastDN = false;
    return timerData.DN;
  }  
  
  }
