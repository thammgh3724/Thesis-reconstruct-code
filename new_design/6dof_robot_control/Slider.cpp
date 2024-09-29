#include "Slider.h"

Slider::Slider(){
    this->state = INIT;
    this->position = 0.0;
};

Slider::~Slider(){};

void Slider::onStart(){
    // go home here
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