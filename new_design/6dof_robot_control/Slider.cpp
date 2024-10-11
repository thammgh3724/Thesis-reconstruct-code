#include "Slider.h"

Slider::Slider(){
    this->state = INIT;
    this->position = 0.0;
};

Slider::~Slider(){};

int Slider::onStart(){
    // go home here
    this->position = 0;
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
    if ( (input >= 0.9) && (input <= 1.1) && (this->position - 0.5 >= MIN_POSITION) ) {
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
void Slider::generalAutoMove(uint8_t DIR, int totSteps, int delValue = 400, int incValue = 15, int accRate = 20){
  digitalWrite(this->DIR_PINS, DIR);
  for (int i = 0; i < totSteps; i++) // need to remove this for loop
  {
   if (totSteps > (2*accRate + 1)){
      if (i < accRate){
        //acceleration
        delValue = delValue - incValue;
      } else if (i > (totSteps - accRate)){
        //decceleration
        delValue = delValue + incValue;
      }
    } else {
      //no space for proper acceleration/decceleration
      if (i < ((totSteps - (totSteps % 2))/2)){
        //acceleration
        delValue = delValue - incValue;
      } else if (i > ((totSteps + (totSteps % 2))/2)){
        //decceleration
        delValue = delValue + incValue;
      }
    }
    for (int j=0; j < 100 ; j++){
      digitalWrite(this->PUL_PINS, HIGH);
      delayMicroseconds(delValue);
      digitalWrite(this->PUL_PINS, LOW);
      delayMicroseconds(delValue);
    }
  }
}