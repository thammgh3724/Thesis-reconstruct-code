#ifndef __MOVING__H
#define __MOVING__H

#include "Arduino.h"
#include "communication.h"
#include "kinematics.h"

class ArmMoving {
private:
  double currJoint[6]; // current joints angle
  double currX[6]; // current {x, y, z, ZYZ Euler angles}
  double buffer[6];
  bool isFirstmove; 
  bool isHorizontalMove;
  bool isLengthwiseMove;

public:
  ArmMoving();
  void wakeUp();
  void goHomeFromManual();
  void goFoldFromManual();
  void singleJointMove(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530);
  void manualControl(char *DATA, double vel0, double acc0, double velini, double velfin);
  void autoMove(double* Xnext, double vel0, double acc0, double velini, double velfin);
  void listen();
  void move();

  void printCurJoint();
  void printCurPos();

  bool getDataModel(double* output);
};

#endif
