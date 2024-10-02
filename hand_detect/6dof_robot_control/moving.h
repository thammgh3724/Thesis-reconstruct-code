#ifndef __MOVING__H
#define __MOVING__H

#include "Arduino.h"
#include "communication.h"
#include "kinematics.h"

int inductiveSrDetect();

class ArmMoving {
private:
  double currPosSlider; // add for slider
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
  void manualMove(double* Jnext, double vel0, double acc0, double velini, double velfin);
  void autoMove(double* Xnext, double vel0, double acc0, double velini, double velfin);
  void autoMove_detectHand(double* Xnext, double vel0, double acc0, double velini, double velfin);
  void listen();
  void move();
  int validateJoint(double* input);

  void printCurJoint();
  void printCurPos();

  bool getDataModel(double* output);
  bool getAxis(double* output);
  bool getDataManual(double* output);
  // add for slider
  void sliderInit();
  void sliderMove(double SliderPosNext);
  void ArmMoving::singleSliderMove( uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 400, int incValue = 15, int accRate = 20);
};

#endif
