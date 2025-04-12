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

  public:
    const float MAX_ANGLE = 90;
    const float MIN_ANGLE = 0;
    const unsigned long MOVING_TIME = 500000; // 0.5s

  public:
    Gripper();  
    void setupGripper();     
    void initGripper();  
    float getCurrentAngle();
    void setCurrentAngle(float angle); 
    float getNextAngle();
    void setNextAngle(float angle); 
    void moveGripper(float input); 
    int getCurrentState(); 
    void setCurrentState(int state);
    void gripperOpen();
    void gripperClose();  
};

#endif
