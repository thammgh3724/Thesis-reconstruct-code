#include "Slider.h"

Slider::Slider(){
    this->state = INIT;
    this->position = 0.0;
};

Slider::~Slider(){};

double Slider::double_abs(double num) { 
  if ( num < 0.1 ) {
    num = 0 - num;
  }
  return num;
}
int Slider::onStart(){
    // go home here
    this->position = this->MIN_POSITION;
    digitalWrite(SLIDER_DIR,HIGH);
    if(inductiveSrDetect() == LOW) return 0;
    while (inductiveSrDetect() != LOW){
      digitalWrite(this->PUL_PINS,HIGH);
      this->PULstat = 1;
      delayMicroseconds(100);
      digitalWrite(this->PUL_PINS,LOW);
      this->PULstat = 0;
      delayMicroseconds(100); 
    }
    return 1;
    // digitalWrite(SLIDER_DIR,LOW);
    // int count =100;
    // for (int i = 0 ; i< 100;i++ ){
    //   digitalWrite(SLIDER_PUL,HIGH); 
    //   delayMicroseconds(100);
    //   digitalWrite(SLIDER_PUL,LOW); 
    //   delayMicroseconds(100); 
    // }
    // do {
    //   digitalWrite(SLIDER_PUL,HIGH); 
    //   delayMicroseconds(100);
    //   digitalWrite(SLIDER_PUL,LOW); 
    //   delayMicroseconds(100); 
    //   count ++;
    // } while (inductiveSrDetect() != LOW);
    // return count;
}

int Slider::getCurrentState(){
    return this->state;
}

void Slider::setState(int state){
    this->state = state;
}

double Slider::getCurrentPosition(){
    return this->position;
}

void Slider::setPosition(double position){
    this->position = position;
}

int Slider::inductiveSrDetect() {
  int inductive = digitalRead(INDUCTIVE_SR);
  return inductive;
}

void Slider::manualMove(double input){
    if ( (input >= 0.9) && (input <= 1.1) && (this->position - 0.5 >= MIN_POSITION)) {
      //Rotate positive direction
      digitalWrite(this->DIR_PINS, HIGH);
      if (PULstat == 0) {
        digitalWrite(this->PUL_PINS, HIGH);
        PULstat = 1;
      } else {
        digitalWrite(this->PUL_PINS, LOW);
        PULstat = 0;
      }
      this->position = this->position - 0.5;
    } 
    else if ( (input >= 1.9) && (input <= 2.1) && (this->position + 0.5 <= MAX_POSITION )) {
      //Rotate negative direction
      digitalWrite(this->DIR_PINS, LOW);
      if (PULstat == 0) {
        digitalWrite(this->PUL_PINS, HIGH);
        PULstat = 1;
      } else {
        digitalWrite(this->PUL_PINS, LOW);
        PULstat = 0;
      }
      this->position = this->position + 0.5;
    }
}
int Slider::validatePosition(double input){
    if(input <0 || input > 28246) return 1;
    return 0; 
}
void Slider::setNextPosition(int newPosition){
  this->nextPosition = newPosition;
}
void Slider::calculateTotalSteps(){
  this->numberStepToGo = this->nextPosition - this->position;
}
void Slider::updatePosition(){
  this->position = this->nextPosition;
}
void Slider::initStepDone(){
  this->numberStepDone = 0.0;
}
bool Slider::isAutoMoveDone(){
  if (this->numberStepDone == this->numberStepToGo) return true;
  return false;
}
double Slider::getNumberStepToGo(){
  return this->numberStepToGo;
}
void Slider::generalAutoMove(unsigned long &delValue, int incValue = 15, int accRate = 20){
  if((double_abs(this->numberStepToGo) > 0.2) && (double_abs(this->numberStepToGo) - this->numberStepDone) > 0.2){
    if (double_abs(this->numberStepToGo) > (2*accRate + 1)){
      if (this->numberStepDone < accRate){
        //acceleration
        if(delValue -incValue > 1.0) delValue = delValue - incValue;
      } else if (this->numberStepDone > (double_abs(this->numberStepToGo) - accRate)){
        //decceleration
        if(delValue + incValue < 4000.0) delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (this->numberStepDone < (double_abs(this->numberStepToGo)/2)){
        //acceleration
          if(delValue > 1.0) delValue =  delValue - incValue;
      } else if (this->numberStepDone > (double_abs(this->numberStepToGo)/2)){
        //decceleration
          if(delValue < 4000.0) delValue =  delValue +  incValue;
      }
    }
    if ( this->numberStepToGo > 0.2 ) {
      //Rotate positive direction
      digitalWrite(this->DIR_PINS, HIGH);
      if (this->PULstat == 0) {
        digitalWrite(this->PUL_PINS, HIGH);
        this->PULstat = 1;
      } else {
        digitalWrite(this->PUL_PINS, LOW);
        this->PULstat = 0;
      }
    }
    else if (this->numberStepToGo < -0.2 ) {
      //Rotate negative direction
      digitalWrite(this->DIR_PINS, LOW);
      if (PULstat == 0) {
        digitalWrite(this->PUL_PINS, HIGH);
        PULstat = 1;
      } else {
        digitalWrite(this->PUL_PINS, LOW);
        PULstat = 0;
      }
    }
    this->numberStepDone = this->numberStepDone + 0.5;
  }
  else if ( this->isAutoMoveDone()){
    #ifdef DEBUG
    String data_print = "!SLIDER ";
    data_print += " move done";
    // this->sender->sendData(data_print);
    #endif
  }
}