#include "Communication.h"
//−−−−−−−−−−VARIABLES USED FOR THE SERIAL DATA TRANSFER−−−−−−−−−−//

#define startChar   '!'         //Message start
#define endChar     '#'         //Message end
#define MAX_DATA_SIZE 30

int numCharRead = 0;
char rc;                               //Received character
String data_print = "";

Listener::Listener(){
  this->command = "";
}

Listener::~Listener(){};

void Listener::getDoubleArrayData(double* output, int numDataToGet, int commandLength, char charToPassBy){
  int posLen = commandLength;
  String token = "";
  int i = 0;
  for (int idx = 0; idx < posLen + 1; idx++) { // May be need a new clear logic here 
    if (i == numDataToGet) break;
    if (this->command[idx] != ':' && this->command[idx] != '\0' && this->command[idx] != charToPassBy) {
        token += this->command[idx];
    } 
    else {
        // Convert String to char array
        char charArray[token.length() + 1]; // +1 for null terminator
        token.toCharArray(charArray, token.length() + 1);
        //Convert to double
        double doubleValue = atof(charArray); 
        output[i++] = doubleValue;
        token = "";
    }
  }
}

void Listener::consumeCommand(int action, double* output){
  int commandLength;
  switch (action)
  {
  case ARM_INIT_ACTION:
    break;
  case ARM_GOHOME_ACTION:
    break;
  case ARM_MANUAL_MOVE_DISTANCE_ACTION:
    // get data move arm from gamepad
    for (int i = 0; i < 6; i++) 
    {
        output[i] = 0.0; 
    }
    commandLength = this->command.length(); 
    getDoubleArrayData(output, 6, commandLength, 'M');
    break;
  case ARM_AUTO_MOVE_POSITION_ACTION:
    // get new arm position
    for (int i = 0; i < 6; i++) 
    {
        output[i] = 0.0; 
    }
    commandLength = this->command.length(); 
    getDoubleArrayData(output, 6, commandLength, 'A');
    break;
  case ARM_AUTO_MOVE_DETECT_HAND_ACTION:
    // get data of distance from model detect hand
    for (int i = 0; i < 2; i++) 
    {
        output[i] = 0.0; 
    }
    commandLength = this->command.length(); 
    getDoubleArrayData(output, 2, commandLength, 'H');
    break;
  case SLIDER_MANUAL_MOVE_DISTANCE_ACTION:
    // get data move arm from gamepad
    for (int i = 0; i < 1; i++) 
    {
        output[i] = 0.0; 
    }
    commandLength = this->command.length(); 
    getDoubleArrayData(output, 1, commandLength, 'S');
    break;
  case SLIDER_AUTO_MOVE_FREE_ACTION:
    break;
  case ARM_STOP_ACTION:
    break;
  case SLIDER_STOP_ACTION:
    break;
  case UNKNOW_ACTION:
    break;
  default:
    break;
  }
  this->command = "";
}

int Listener::parseCommandToAction(){ // need 1 command "STOP"
  if(this->isCommandReady){
    if(this->command == "init") {
      return ARM_INIT_ACTION;
    }
    // !gohome#
    // not working
    else if(this->command == "gohome"){
      return ARM_GOHOME_ACTION;
    }
    // !0:0:0:0:0:0A#
    else if(this->command.endsWith("A")){
      return ARM_AUTO_MOVE_POSITION_ACTION;
    }
    // !0:0H#
    else if(this->command.endsWith("H")){
      return ARM_AUTO_MOVE_DETECT_HAND_ACTION;
    }
    // !0:0:0:0:0:0M#
    else if(this->command.endsWith("M")){
      return ARM_MANUAL_MOVE_DISTANCE_ACTION;
    }
    // Sliders Control
    else if (this->command.endsWith("S")) {
      return SLIDER_MANUAL_MOVE_DISTANCE_ACTION;  
    }
    else {
      return UNKNOW_ACTION;
    }
  }
  else {
    return NO_ACTION;
  }
}

void Listener::readData() {
  if(numCharRead > MAX_DATA_SIZE){
    this->command = "";
    numCharRead = 0;
  }
  if(Serial.available() > 0){
    rc = Serial.read();     //Reads one char of the data
    if (rc == startChar) {  //Start data transfer if startChar found
      this->isdataTransferring = true;
      this->command = "";
    } 
    else if (this->isdataTransferring) {
      if (rc != endChar) {  //Transfer data as long as endChar notfound
        this->command += rc;       //Save data
        numCharRead++;
      } 
      else {                //Stop data transfer if endChar found
        this->isdataTransferring = false;  //Stop data transfer
        numCharRead = 0;
        this->isCommandReady = true;
        data_print = "!Receive data : ";
        data_print += this->command;
      }
    }
  }
}

//*************************************//
Sender::Sender(){};
Sender::~Sender(){};

void Sender::sendData(String data){
  Serial.println(data);
}