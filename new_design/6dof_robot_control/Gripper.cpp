#include "Gripper.h"

Gripper::Gripper() : currentState(INIT), currentAngle(100.0), nextAngle(100.0) {}

void Gripper::setupGripper() {
  this->gripperServo.attach(servoPin);
}

void Gripper::initGripper() {
  this->currentAngle = 100.0;
  this->gripperServo.write(static_cast<int>(this->currentAngle));
}

void Gripper::moveGripper(double* input) {
  int mode = static_cast<int>(input[0]); 
  int adjustment = static_cast<int>(input[1]);           

  if (mode == 0) {
    if (adjustment == 1) {
      this->nextAngle = this->currentAngle + 1;
    } else if (adjustment == 2) {
      this->nextAngle = this-> currentAngle - 1; 
    }

    if (this->nextAngle > this->MAX_ANGLE) {
      this->nextAngle = this->MAX_ANGLE;
    } else if (this->nextAngle < this->MIN_ANGLE) {
      this->nextAngle = this->MIN_ANGLE;
    }

    this->currentAngle = this->nextAngle;
    this->gripperServo.write(static_cast<int>(this->currentAngle)); 
  }
  else {

  }
}

void Gripper::gripperOpen() {
  this->gripperServo.write(150); 
}

void Gripper::gripperClose() {
  this->gripperServo.write(100); 
}

float Gripper::getCurrentAngle() {
  return this->currentAngle; 
}

int Gripper::getCurrentState() {
  return this->currentState;
}

void Gripper::setCurrentState(int state) {
  this->currentState = state;
}
