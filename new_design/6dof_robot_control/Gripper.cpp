#include "Gripper.h"

Gripper::Gripper() : currentState(INIT), currentAngle(90.0), nextAngle(90.0) {}

void Gripper::setupGripper() {
  gripperServo.attach(servoPin);
}

void Gripper::initGripper() {
  currentAngle = 90.0;
  gripperServo.write(currentAngle);
}

void Gripper::moveGripper(double* input) {
  int mode = static_cast<int>(input[0]); 
  float adjustment = input[1];           

  if (mode == 0) { 
    nextAngle = currentAngle + adjustment;

    if (nextAngle > MAX_ANGLE) {
      nextAngle = MAX_ANGLE;
    } else if (nextAngle < MIN_ANGLE) {
      nextAngle = MIN_ANGLE;
    }

    currentAngle = nextAngle;
    gripperServo.write(currentAngle); 
  }
  else {

  }
}

int Gripper::getCurrentState() {
  return currentState;
}

void Gripper::setCurrentState(int state) {
  currentState = state;
}
