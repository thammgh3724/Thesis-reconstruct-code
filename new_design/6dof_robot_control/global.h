#ifndef __GLOBAL__H
#define __GLOBAL__H

#include "Arduino.h"
#define PI 3.1415926535897932384626433832795

// inductive sensor for sliders
#define INDUCTIVE_SR    33

// Driver for slider control
#define SLIDER_PUL      31
#define SLIDER_DIR      29
#define SLIDER_NEG_DIR  27

//driver for the axis 1
#define PUL1_PIN 39
#define DIR1_PIN 37
//driver for the axis 2
#define PUL2_PIN 43
#define DIR2_PIN 41
//driver for the axis 3
#define PUL3_PIN 47
#define DIR3_PIN 45
//driver for the axis 4
#define PUL4_PIN 46
#define DIR4_PIN 48
//driver for the axis 5
#define PUL5_PIN A6
#define DIR5_PIN A7
//driver for the axis 6
#define PUL6_PIN A0
#define DIR6_PIN A1

//enable pin for the axis 3, 2 and 1
#define EN321_PIN 32
#define EN4_PIN   A8
#define EN5_PIN   A2
#define EN6_PIN   38

#define PUMP_PIN  63

// Define global state
#define INIT 0
#define HOME 1
#define MANUAL_MOVING 2 
#define GENERAL_AUTO_MOVING 3
#define STOP 4
#define DETECT_HAND_AUTO_MOVING 5

// Define action to do
#define NO_ACTION    0
#define UNKNOW_ACTION    1
#define INIT_ACTION    2 // use for 2 object
#define ARM_GOHOME_ACTION    3
#define ARM_AUTO_MOVE_POSITION_ACTION    4
#define ARM_AUTO_MOVE_DETECT_HAND_ACTION    5
#define ARM_MANUAL_MOVE_DISTANCE_ACTION    6
#define SLIDER_MANUAL_MOVE_DISTANCE_ACTION    7
#define SLIDER_AUTO_MOVE_FREE_ACTION    8
#define ARM_STOP_ACTION    9
#define SLIDER_STOP_ACTION    10
#define ARM_INIT_ACTION       11
#define SLIDER_INIT_ACTION    12
#define SLIDER_AUTO_MOVE_DETECT_HAND_ACTION     13
#define SLIDER_AUTO_MOVE_POSITION_ACTION        14

//Debug flag
#define DEBUG

#endif