#ifndef __MOVING__H
#define __MOVING__H

#include "Arduino.h"
#include "communication.h"
#include "kinematics.h"

class ArmMoving {
private:
  float currJoint[6]; // current joints angle
  float currX[6]; // current {x, y, z, ZYZ Euler angles}
  float buffer[6];
  bool isFirstmove; 
  bool isHorizontalMove;
  bool isLengthwiseMove;

public:
  ArmMoving();
  void wakeUp();
  void goHomeFromManual();
  void goFoldFromManual();
  void singleJointMove(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530);
  void manualControl(char *DATA, float vel0, float acc0, float velini, float velfin);
  void autoMove(float* Xnext, float vel0, float acc0, float velini, float velfin);
  void listen();
  void move();

  void printCurJoint();
  void printCurPos();

  bool getDataModel(float* output);
};

#endif
