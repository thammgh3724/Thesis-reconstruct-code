#include "moving.h"

//−−−−−−−−−−VARIABLES USED FOR CONTROLLING ARM−−−−−−−−−−//
#define POSITIVE_DIRECTION '1'
#define NEGATIVE_DIRECTION '2'
#define ANGLE_PER_COMMAND   2

const double velG = 0.25e-4;
double start_vel = 1 * velG;
double end_vel = 1 * velG; 

#define NUM_BYTES_BUFFER    (6 * sizeof(double))

/* read inductive sensor for sliders
 */
int inductiveSrDetect() {
  int inductive = digitalRead(INDUCTIVE_SR);
  return inductive;
}

ArmMoving::ArmMoving(){
  memset(this->currJoint, 0, NUM_BYTES_BUFFER);
  memset(this->buffer, 0, NUM_BYTES_BUFFER);
  memset(this->currX, 0, NUM_BYTES_BUFFER);
  this->isFirstmove = true;
  this->isHorizontalMove = false;
  this->isLengthwiseMove = false;
}

void ArmMoving::listen(){
    read();
}

void ArmMoving::sliderInit(){
  this->currPosSlider = 0;
  if(inductiveSrDetect() == LOW) return;
  while (inductiveSrDetect() != LOW){
    digitalWrite(SLIDER_DIR,HIGH);
    for (int i = 0 ; i <100 ;i++){
      digitalWrite(SLIDER_PUL,HIGH); 
      delayMicroseconds(250); 
      digitalWrite(SLIDER_PUL,LOW); 
      delayMicroseconds(250); 
    }
  }
  Serial.println("!SLIDER INIT DONE");
}
void ArmMoving::move(){
  // !init#
  if(position == "init") {
    Serial.println(position);
    position = "";
    Serial.println("!INIT START");
    this->wakeUp();
    this->printCurJoint();
    this->printCurPos();
    Serial.println("!INIT DONE");
    // Init for Slider
    this->sliderInit();
  }
  // !gohome#
  // not working
  else if(position == "gohome"){
    position = "";
    Serial.println("!GOHOME START");
    this->goHomeFromManual();
    this->printCurJoint();
    this->printCurPos();
    Serial.println("!GOHOME DONE");
  }
  // !0:0:0:0:0:0A#
  else if(position.endsWith("A")){
    // set positionAUTO
    Serial.println("!AUTO START");
    double output[6];
    if(getAxis(output)){
      this->autoMove(output, 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);
    }
    Serial.println("!AUTO DONE");
    position = "";
  }
  // !0:0H#
  else if(position.endsWith("H")){
    // set position
    Serial.println("!GO HAND START");
    double output[6];
    if(getDataModel(output)){
      // Move lengthwise and horizontal seperately
      double lengthwise = output[2];
      double horizontal = output[1];
      // horizontal move
      if( (horizontal - this->currX[1] >= 1.0) || (horizontal - this->currX[1] < -1.0) ){
        output[1] = horizontal;
        output[2] = currX[2]; // keep lengthwise unchanged
        this->isHorizontalMove = true;
        this->autoMove_detectHand(output, 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);
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
        this->autoMove_detectHand(output, 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);
        this->printCurJoint();
        this->printCurPos();
        this->isLengthwiseMove = false;
        Serial.println("!LENGHTWISE DONE");
      }
    }
    Serial.println("!GO HAND DONE");
    position = "";
  }
  // !0:0:0:0:0:0:0M#  add more one value for slider pos
  else if(position.endsWith("M")){
    // set position
    #ifdef DEBUG
    Serial.println("!GO MANUAL START");
    #endif
    double output[7];
    if(getDataManual(output)){
      double armOutput[6];
      for (int i = 0 ; i < 6;i++){
        armOutput[i] = output[i];
      }
      this->manualMove(armOutput, 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);

    }
    //add for slider
    this->sliderMove(output[6]);
    Serial.println("!GO MANUAL DONE");
    position = "";
  }
  // Sliders Control
  else if (position.endsWith("S")) {
    int direct = digitalRead(SLIDER_DIR);
    if (position == "LEFTS"){
      if (inductiveSrDetect() == LOW && direct == HIGH){
        Serial.println("!LIMIT MOVE DETECT");
        return;
      }

      digitalWrite(SLIDER_DIR,HIGH);
      for (int i = 0 ; i < 100 ; i++){
        digitalWrite(SLIDER_PUL,HIGH); 
        delayMicroseconds(250); 
        digitalWrite(SLIDER_PUL,LOW); 
        delayMicroseconds(250); 
      }
      Serial.println("!GO LEFT");
    }
    else if (position == "RIGHTS"){
      if (inductiveSrDetect() == LOW && direct == LOW){
        Serial.println("!LIMIT MOVE DETECT");
        return;
      }
      digitalWrite(SLIDER_DIR,LOW);
      for (int i = 0 ; i < 100; i++){
        digitalWrite(SLIDER_PUL,HIGH); 
        delayMicroseconds(250); 
        digitalWrite(SLIDER_PUL,LOW); 
        delayMicroseconds(250); 
      }
      Serial.println("!GO RIGHT");
    }
    else if (position == "AUTOS"){ 
      while(true){
        if (inductiveSrDetect() == 0){
          digitalWrite(SLIDER_DIR, !digitalRead(SLIDER_DIR));
          for (int i = 0 ;i <100 ;i++){
            digitalWrite(SLIDER_PUL, HIGH);
            delayMicroseconds(250);
            digitalWrite(SLIDER_PUL,LOW);
            delayMicroseconds(250);
          }
          continue;
        }
        digitalWrite(SLIDER_PUL, HIGH);
        delayMicroseconds(250);
        digitalWrite(SLIDER_PUL,LOW);
        delayMicroseconds(250);
        read();
        if (position == "STOPS"){
          break;
        }
      }
    }
  }
  else {
    // double output[6]; 
    // if (getAxis(output)) {
    //     Serial.println("!TEST DONE");
    // }
    // position = "";
  }
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
  setcurJoint(0, 0, 0, 0, 90, 0);
  memcpy(this->buffer, this->currJoint, NUM_BYTES_BUFFER);
  // First move
  double output[6] = { 190.0, -0.0, 260.0, 0.0, 90.0, 180.0 };
  this->autoMove(output, 0.25e-4, 0.1 * 0.75e-10, start_vel, end_vel);
  setcurJoint(0.0, -2.15, 6.47, 180.0, 4.32, -180.0);
  ForwardK(this->currJoint, this->currX); // calculate Xcurr by FK
  this->isFirstmove = false;
}

void ArmMoving::goHomeFromManual(){
  double tmp[6];
  memcpy(tmp, this->currJoint, NUM_BYTES_BUFFER);  //Store current joints
  memset(this->currJoint, 0, NUM_BYTES_BUFFER);
  //Rotate Joint5 90 degree
  this->currJoint[1] = -2.15;
  this->currJoint[2] = 6.47;
  this->currJoint[3] = 180.0;
  this->currJoint[4] = 4.32;
  this->currJoint[5] = -180.0;  
  //Moving using kinematics
  goStrightLine(tmp, this->currJoint, 0.25e-4, 0.75e-10, 0.0, 0.0);
  setcurJoint(0.0, -2.15, 6.47, 180.0, 4.32, -180.0);
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

void ArmMoving::autoMove(double* Xnext, double vel0, double acc0, double velini, double velfin){
  Serial.println("!Start Calculate new Position");
  double Jcurr[6]; // tmp for this->currJoint;
  double Xcurr[6]; // current //{x, y, z, ZYZ Euler angles}
  double Jnext[6]; // target joints
  memcpy(Jcurr, this->currJoint, NUM_BYTES_BUFFER);
  ForwardK(Jcurr, Xcurr); // calculate Xcurr by FK
  InverseK(Xnext, Jnext); // calculate Jnext by IK
  String data_print = "!JNEXT: ";
  for (int i = 0; i < 6; ++i){
    if (!isfinite(Jnext[i])) {
        Serial.println("!Danger! The number is not finite");
        return;
    }
    data_print += Jnext[i];
    data_print += ":";
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

void ArmMoving::autoMove_detectHand(double* Xnext, double vel0, double acc0, double velini, double velfin){
  Serial.println("!Start Calculate new Position");
  double Jcurr[6]; // tmp for this->currJoint;
  double Xcurr[6]; // current //{x, y, z, ZYZ Euler angles}
  double Jnext[6]; // target joints
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
      int angleJ2Move = Jnext[1] - Jcurr[1];
      int angleJ3Move = Jnext[2] - Jcurr[2];
      Jnext[4] = Jcurr[4] + angleJ2Move + angleJ3Move;
    }
  } 
  memcpy(Jcurr, Jnext, NUM_BYTES_BUFFER); //Store Jnext
  String data_print = "!JNEXT: ";
  for (int i = 0; i < 6; ++i){
    if (!isfinite(Jnext[i])) {
        Serial.println("!Danger! The number is not finite");
        return;
    }
    data_print += Jnext[i];
    data_print += ":";
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
void ArmMoving::sliderMove(double sliderPosNext){
  //Move
  if (sliderPosNext > 280 || sliderPosNext < 0 ) return;
  if (sliderPosNext > this->currPosSlider){
    double stepMove = sliderPosNext - this->currPosSlider;
    digitalWrite(SLIDER_DIR,LOW);
    for (int i = 0 ; i < stepMove*100 ; i++){
      digitalWrite(SLIDER_PUL,HIGH); 
      delayMicroseconds(250); 
      digitalWrite(SLIDER_PUL,LOW); 
      delayMicroseconds(250); 
    }
  }
  else if (sliderPosNext < this->currPosSlider){
    double stepMove = this->currPosSlider - sliderPosNext;
    digitalWrite(SLIDER_DIR,HIGH);
    for (int i = 0 ; i < stepMove*100 ; i++){
      digitalWrite(SLIDER_PUL,HIGH); 
      delayMicroseconds(250); 
      digitalWrite(SLIDER_PUL,LOW); 
      delayMicroseconds(250); 
    }
  }
  this->currPosSlider = sliderPosNext;
}
void ArmMoving::manualMove(double* Jnext, double vel0, double acc0, double velini, double velfin){
  //Move
  int canMove = validateJoint(Jnext);
  if (canMove == 0){
    #ifdef DEBUG
    Serial.println("MOVING...");
    #endif
    goStrightLine(this->currJoint, Jnext, vel0, acc0, velini, velfin);
    memcpy(this->currJoint, Jnext, NUM_BYTES_BUFFER); //Update currJoint
    ForwardK(this->currJoint, this->currX); //Update currX
    #ifdef DEBUG
    Serial.println("!MOVE DONE");
    #endif
  }
  else{
    String data_print = "!Joint out of range : Joint ";
    data_print += canMove;
    Serial.println(data_print);
  }
}

bool ArmMoving::getAxis(double* output){
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
            //Convert to double
            double doubleValue = atof(charArray); 
            output[i++] = doubleValue;
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

bool ArmMoving::getDataModel(double* output){
  // get data from model
    double data[2];
    for (int i = 0; i < 2; i++) 
    {
        data[i] = 0.0; 
    }
    int posLen = position.length(); 
    String token = "";
    int i = 0;
    for (int idx = 0; idx < posLen + 1; idx++) {
        if (i == 2) break;
        if (position[idx] != ':' && position[idx] != '\0' && position[idx] != 'H') {
            token += position[idx];
        } 
        else {
            // Convert String to char array
            char charArray[token.length() + 1]; // +1 for null terminator
            token.toCharArray(charArray, token.length() + 1);
            //Convert to double
            double doubleValue = atof(charArray); 
            data[i++] = doubleValue;
            token = "";
        }
    }
    String data_print1 = "!GET SUCCESS DATA  ";
    for (int j = 0; j < 2; j++)
    {
      data_print1 += data[j];
      data_print1 += ":";
    }
    Serial.println(data_print1);
    // INIT AXIS ARRAY;
    for (int i = 0; i < 6; i++) 
    {
        output[i] = this->currX[i]; 
    }
    // caculate new position
    output[1] = output[1] + ( (-40.0/177.0)*(data[0]-320.0) );
    output[2] = output[2] + ( (-75.0/73.0)*(data[1]-240.0) );
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

bool ArmMoving::getDataManual(double* output){
  // get data from gamepad
    double data[7];
    for (int i = 0; i < 7; i++) 
    {
        data[i] = 0.0; 
    }
    int posLen = position.length(); 
    String token = "";
    int i = 0;
    for (int idx = 0; idx < posLen + 1; idx++) {
        if (i == 7) break;
        if (position[idx] != ':' && position[idx] != '\0' && position[idx] != 'M') {
            token += position[idx];
        } 
        else {
            // Convert String to char array
            char charArray[token.length() + 1]; // +1 for null terminator
            token.toCharArray(charArray, token.length() + 1);
            //Convert to double
            double doubleValue = atof(charArray); 
            data[i++] = doubleValue;
            token = "";
        }
    }
    #ifdef DEBUG
    String data_print1 = "!GET SUCCESS DATA  ";
    for (int j = 0; j < 7; j++)
    {
      data_print1 += data[j];
      data_print1 += ":";
    }
    Serial.println(data_print1);
    #endif
    // INIT NEW JOINT ARRAY;
    for (int j = 0; j < 6; j++) 
    {
        output[j] = this->currJoint[j]; 
    }
    // caculate new joint
    for (int j = 0; j < 6; ++j) {
      if ( (data[j] >= 0.9) && (data[j] <= 1.1)) {
        //Rotate positive direction
       output[j] += ANGLE_PER_COMMAND;
      } 
      else if ( (data[j] >= 1.9) && (data[j] <= 2.1) ) {
        //Rotate negative direction
        output[j] -= ANGLE_PER_COMMAND;
      }
    }
    // add for slider
    output[6] = data[6];
    #ifdef DEBUG
    String data_print = "!GET SUCCESS JOINT  ";
    for (int j = 0; j < 6; j++)
    {
      data_print += output[j];
      data_print += ":";
      // data_print += i; 
    }
    Serial.println(data_print);
    #endif
    return true;
}

int ArmMoving::validateJoint(double* input){
  for (int i = 0; i < 6; ++i){
    switch (i){
      case 0:
        if (input[i] > MAX_JOINT_ANGLE[0] || input[i] < MIN_JOINT_ANGLE[0]) return 1;
        break; 
      case 1:
        if (input[i] > MAX_JOINT_ANGLE[1] || input[i] < MIN_JOINT_ANGLE[1]) return 2;
        break; 
      case 2:
        if (input[i] > MAX_JOINT_ANGLE[2] || input[i] < MIN_JOINT_ANGLE[2]) return 3;
        break;
      case 3:
        if (input[i] > MAX_JOINT_ANGLE[3] || input[i] < MIN_JOINT_ANGLE[3]) return 4;
        break;
      case 4:
        if (input[i] > MAX_JOINT_ANGLE[4] || input[i] < MIN_JOINT_ANGLE[4]) return 5;
        break;
      case 5:
        if (input[i] > MAX_JOINT_ANGLE[5] || input[i] < MIN_JOINT_ANGLE[5]) return 6;
        break;
      default:
        break;
    }
  }
  return 0;
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