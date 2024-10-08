/*
Script to move my tiny 6dof robotic arm
*/
#include "System.h"

void setup() {
  // Define pin in and output
  pinMode(PUL1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PUL2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PUL3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(PUL4_PIN, OUTPUT);
  pinMode(DIR4_PIN, OUTPUT);
  pinMode(PUL5_PIN, OUTPUT);
  pinMode(DIR5_PIN, OUTPUT);
  pinMode(PUL6_PIN, OUTPUT);
  pinMode(DIR6_PIN, OUTPUT);

  pinMode(EN321_PIN, OUTPUT);
  pinMode(EN4_PIN, OUTPUT);
  pinMode(EN5_PIN, OUTPUT);
  pinMode(EN6_PIN, OUTPUT);

  pinMode(PUMP_PIN, OUTPUT);

  // All pin initial signal
  // digitalWrite(PUL1_PIN, LOW);  // gear ratio = 96/20 = 4.8
  // digitalWrite(DIR1_PIN, LOW);  //LOW = negative direction

  // digitalWrite(PUL2_PIN, LOW);  // gear ratio = 4
  // digitalWrite(DIR2_PIN, LOW);  //LOW = positive direction

  // digitalWrite(PUL3_PIN, LOW);  // gear ratio = 5
  // digitalWrite(DIR3_PIN, LOW);  //LOW = negative direction

  // digitalWrite(PUL4_PIN, LOW);  // gear ratio = 56/20 = 2.8
  // digitalWrite(DIR4_PIN, LOW);  //LOW = positive direction

  // digitalWrite(PUL5_PIN, LOW);  // gear ratio = 42/20 = 2.1
  // digitalWrite(DIR5_PIN, LOW);  //LOW = positive direction

  // digitalWrite(PUL6_PIN, LOW);  // gear ratio = 1
  // digitalWrite(DIR6_PIN, LOW);  //LOW = positive direction

  // all joints disabled!
  // digitalWrite(EN321_PIN, HIGH);
  // digitalWrite(EN4_PIN, HIGH);
  // digitalWrite(EN5_PIN, HIGH);
  // digitalWrite(EN6_PIN, HIGH);
  digitalWrite(PUMP_PIN, LOW);  //PUMP OFF

  // SET UP FOR SLIDERS
  pinMode(SLIDER_DIR, OUTPUT); 
  pinMode(SLIDER_PUL, OUTPUT);
  pinMode(INDUCTIVE_SR, INPUT); 

  pinMode(SLIDER_NEG_DIR, OUTPUT); 
  digitalWrite(SLIDER_NEG_DIR, LOW);

  Serial.begin(115200);
}

System mySystem;
// Main function go here
void loop() {

  //--------------------------------------------------------GoGoGo-------------------
  mySystem.communicate(); // read data -> return action if have completed command -> distribute action to the rights object 
  mySystem.arm_fsm(); // do things due to next action need to do
  mySystem.slider_fsm(); // do things due to next action need to do
}