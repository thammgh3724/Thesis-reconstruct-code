// Gripper.h
#ifndef GRIPPER_H
#define GRIPPER_H

#include "Arduino.h"
#include "global.h"
#include "Communication.h"

class Gripper {
  private:
    Servo gripperServo;       
    int currentState;       
    float currentAngle;       
    float nextAngle;        

    const int servoPin = 11;   
    const float MAX_ANGLE = 180.0;
    const float MIN_ANGLE = 90.0;

  public:
    Gripper();  
    void setupGripper();     
    void initGripper();  
    float getCurrentAngle();      
    void moveGripper(double* input); 
    int getCurrentState(); 
    void setCurrentState(int state);
    void gripperOpen();
    void gripperClose();  
};

#endif
