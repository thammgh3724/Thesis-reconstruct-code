#include "Gripper.h"

Gripper::Gripper() : currentState(INIT), currentAngle(120.0), nextAngle(120.0) {}

void Gripper::setupGripper() {
  this->gripperServo.attach(servoPin);
}

void Gripper::initGripper() {
  this->currentAngle = 40.0;
  this->gripperServo.write(static_cast<int>(this->currentAngle));
}

void Gripper::moveGripper(float input) {
  this->gripperServo.write(static_cast<int>(input)); 
}

void Gripper::gripperOpen() {
  this->gripperServo.write(static_cast<int>(this->MAX_ANGLE)); 
}

void Gripper::gripperClose() {
  this->gripperServo.write(static_cast<int>(this->MIN_ANGLE)); 
}

float Gripper::getCurrentAngle() {
  return this->currentAngle; 
}

void Gripper::setCurrentAngle(float angle) {
  this->currentAngle = angle;
}

float Gripper::getNextAngle() {
  return this->nextAngle; 
}

void Gripper::setNextAngle(float angle) {
  this->nextAngle = angle;
}

int Gripper::getCurrentState() {
  return this->currentState;
}

void Gripper::setCurrentState(int state) {
  this->currentState = state;
}
