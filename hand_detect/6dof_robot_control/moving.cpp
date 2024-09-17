#include "moving.h"

//−−−−−−−−−−VARIABLES USED FOR CONTROLLING ARM−−−−−−−−−−//
#define POSITIVE_DIRECTION '1'
#define NEGATIVE_DIRECTION '2'
#define ANGLE_PER_COMMAND   2

const float velG = 0.25e-4;
float start_vel = 1 * velG;
float end_vel = 1 * velG; 

#define NUM_BYTES_BUFFER    (6 * sizeof(float))


int validateJoint(float* input){
  for (int i = 0; i < 6; ++i){
    switch (i){
      case 0:
        if (input[i] > 90 || input[i] < -90) return 1;
        break; 
      case 1:
        if (input[i] > 87 || input[i] < -60) return 2;
        break; 
      case 2:
        if (input[i] > 60 || input[i] < -60) return 3;
        break;
      case 3:
        if (input[i] > 180 || input[i] < -180) return 4;
        break;
      case 4:
        if (input[i] > 120 || input[i] < -90) return 5;
        break;
      case 5:
        break;
      default:
        break;
    }
  }
  return 0;
}

ArmMoving::ArmMoving(){
  memset(this->currJoint, 0, NUM_BYTES_BUFFER);
  memset(this->buffer, 0, NUM_BYTES_BUFFER);
  memset(this->currX, 0, NUM_BYTES_BUFFER);
  this->isFirstmove = true;
  this->isHorizontalMove = false;
  this->isLengthwiseMove = false;
}

void ArmMoving::printCurJoint(){
  String result = "!Curr joint : ";
  for (int i = 0; i < 6; ++i){
    result += String(this->currJoint[i]);  
    if (i < 5) {
        result += ":"; 
    }
  }
  Serial.println(result);
}

void ArmMoving::printCurPos(){
  String result = "!Curr pos : ";
  for (int i = 0; i < 6; ++i){
    result += String(this->currX[i]);  
    if (i < 5) {
        result += ":"; 
    }
  }
  Serial.println(result);
}

void ArmMoving::wakeUp(){
  // enable all joints
  digitalWrite(EN321_PIN, LOW);
  digitalWrite(EN4_PIN, LOW);
  digitalWrite(EN5_PIN, LOW);
  digitalWrite(EN6_PIN, LOW);
  // joint #2
  singleJointMove(DIR2_PIN, HIGH, PUL2_PIN, 5582);
  // joint #3
  singleJointMove(DIR3_PIN, LOW, PUL3_PIN, 6569);
  // joint #5
  singleJointMove(DIR5_PIN, HIGH, PUL5_PIN, (int)((180-10) / dl5)); // minus 10 in initial 
  // as by default, the position of pump is tilted by the camera wire
  //Serial.println("Arm go home");
  
  memset(this->currJoint, 0, NUM_BYTES_BUFFER);

  this->currJoint[4] = 90;
  setCurPos(0, 0, 0, 0, 90, 0);
  memcpy(this->buffer, this->currJoint, NUM_BYTES_BUFFER);
  // First move
  float output[6] = { 190.0, -0.0, 260.0, 0.0, 90.0, 180.0 };
  this->autoMove(output, 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);
  setCurPos(0.0, -2.15, 6.47, 180.0, 4.32, -180.0);
  ForwardK(this->currJoint, this->currX); // calculate Xcurr by FK
  this->isFirstmove = false;
}

void ArmMoving::goHomeFromManual(){
  float tmp[6];
  memcpy(tmp, this->currJoint, NUM_BYTES_BUFFER);  //Store current joints
  memset(this->currJoint, 0, NUM_BYTES_BUFFER);
  //Rotate Joint5 90 degree
  currJoint[4] = 90;
  //Moving using kinematics
  goStrightLine(tmp, this->currJoint, 0.25e-4, 0.75e-10, 0.0, 0.0);
  setCurPos(0, 0, 0, 0, 90, 0);
  memcpy(this->buffer, this->currJoint, NUM_BYTES_BUFFER);
}

void ArmMoving::singleJointMove(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530)
{
  digitalWrite(DIR_PIN, DIR);
  for (int i = 0; i < totSteps; i++)
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
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(delValue);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(delValue);
  }
}

void ArmMoving::autoMove(float* Xnext, float vel0, float acc0, float velini, float velfin){
  Serial.println("!Start Calculate new Position");
  float Jcurr[6]; // tmp for this->currJoint;
  float Xcurr[6]; // current //{x, y, z, ZYZ Euler angles}
  float Jnext[6]; // target joints
  memcpy(Jcurr, this->currJoint, NUM_BYTES_BUFFER);
  ForwardK(Jcurr, Xcurr); // calculate Xcurr by FK
  InverseK(Xnext, Jnext); // calculate Jnext by IK
  // just move joint 4 in initial move
  if(this->isFirstmove == false) {
    Jnext[3] = Jcurr[3];
    Serial.println("!Dont rotate joint 4");
    if (isHorizontalMove) {
      Jnext[1] = Jcurr[1];
      Jnext[2] = Jcurr[2];
      Jnext[3] = Jcurr[3];
      Jnext[4] = Jcurr[4];
      Jnext[5] = Jcurr[5];
      Serial.println("!Move only joint 1 in horizontal");
    }
    if (isLengthwiseMove) {
      // joint 5 alway move up when robot go down
      // need rework
      // if ( temp_Jnext4 - Jcurr[3] < -1.0 ) {
      //   if ((Xnext[2] < Xcurr[2]) && (Jnext[4] - Jcurr[4] <= 1) ) {
      //     Jnext[4] = Jcurr[4] + (Jcurr[4] - Jnext[4]); // parse joint5 move down -> move up
      //     Serial.println("!joint5 should move up");
      //   }
      //   else if ( (Xnext[2] > Xcurr[2]) && (Jnext[4] - Jcurr[4] >= 1)) {
      //     Jnext[4] = Jcurr[4] - (Jnext[4] - Jcurr[4]); // parse joint5 move up -> move down
      //     Serial.println("!joint5 should move down");
      //   }
      // }
      Jnext[0] = Jcurr[0];
      Jnext[2] = Jcurr[2];
      Jnext[3] = Jcurr[3];
      Jnext[4] = Jcurr[4];
      Jnext[5] = Jcurr[5];
      Serial.println("!Move only joint 2 in lengthwise");
    }
  }
  memcpy(Jcurr, Jnext, NUM_BYTES_BUFFER); //Store Jnext
  String data_print = "!JNEXT: ";
  for (int i = 0; i < 6; ++i){
    data_print += Jnext[i];
    data_print += ":";
    if (!isfinite(Jnext[i])) {
        Serial.println("!Danger! The number is not finite");
        return;
    }
  }
  Serial.println(data_print);
  //Move
  int canMove = validateJoint(Jnext);
  if (canMove == 0){
    Serial.println("!MOVING...");
    goStrightLine(this->currJoint, Jnext, vel0, acc0, velini, velfin);
    memcpy(this->currJoint, Jnext, NUM_BYTES_BUFFER); //Update currJoint
    ForwardK(this->currJoint, this->currX); //Update currX
    Serial.println("!MOVE DONE");
  }
  else{
    data_print = "!Joint out of range : Joint ";
    data_print += canMove;
    Serial.println(data_print);
  }
}

void ArmMoving::listen(){
    read();
}

bool getAxis(float* output){
    // INIT AXIS ARRAY;
    for (int i = 0; i < 6; i++) 
    {
        output[i] = 0.0; 
    }
    int posLen = position.length(); 
    String token = "";
    int i = 0;
    for (int idx = 0; idx < posLen + 1; idx++) {
        if (i == 6) break;
        if (position[idx] != ':' && position[idx] != '\0' && position[idx] != 'A') {
            token += position[idx];
        } 
        else {
            // Convert String to char array
            char charArray[token.length() + 1]; // +1 for null terminator
            token.toCharArray(charArray, token.length() + 1);
            //Convert to float
            float floatValue = atof(charArray); 
            output[i++] = floatValue;
            token = "";
        }
    }
    String data_print = "!GET SUCCESS POSITION  ";
    for (int j = 0; j < 6; j++)
    {
      data_print += output[j];
      data_print += ":";
      // data_print += i; 
    }
    Serial.println(data_print);
    return true;
}


void ArmMoving::move(){
  if(position == "init") {
    Serial.println(position); 
    position = "";
    Serial.println("!INIT START");
    this->wakeUp();
    this->printCurJoint();
    this->printCurPos();
    Serial.println("!INIT DONE");
  }
  else if(position == "gohome"){
    position = "";
    Serial.println("!GOHOME START");
    this->goHomeFromManual();
    this->printCurJoint();
    this->printCurPos();

    Serial.println("!GOHOME DONE");
  }
  else if(position.endsWith("A")){
    // set position
    Serial.println("!AUTO START");
    float output[6];
    if(getAxis(output)){
      // Move lengthwise and horizontal seperately
      float lengthwise = output[2];
      float horizontal = output[1];
      // horizontal move
      if( (horizontal - this->currX[1] >= 1.0) || (horizontal - this->currX[1] < -1.0) ){
        output[1] = horizontal;
        output[2] = currX[2]; // keep lengthwise unchanged
        this->isHorizontalMove = true;
        this->autoMove(output, 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);
        this->printCurJoint();
        this->printCurPos();
        this->isHorizontalMove = false;
        Serial.println("!HORIZONTAL DONE");
      }
      // lengthwise move
      if( (lengthwise - this->currX[2] >= 1.0) || (lengthwise - this->currX[2] < -1.0) ){
        // update new position after horizontal move
        memcpy(output, this->currX, NUM_BYTES_BUFFER);
        output[2] = lengthwise;
        this->isLengthwiseMove = true;
        this->autoMove(output, 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);
        this->printCurJoint();
        this->printCurPos();
        this->isLengthwiseMove = false;
        Serial.println("!LENGHTWISE DONE");
      }
    }
    Serial.println("!AUTO DONE");
    position = "";
  } 
  else {
    // float output[6]; 
    // if (getAxis(output)) {
    //     Serial.println("!TEST DONE");
    // }
    // position = "";
  }
}