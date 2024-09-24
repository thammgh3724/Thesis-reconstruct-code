#define startChar   '!'         //Message start
#define endChar     '#'         //Message end
#define MAX_DATA_SIZE 100
const int DIR_PIN = 5; 
const int PUL_PIN = 2; 
const int NEG_DIR_PIN = 8; 
const int INDUCTIVE_SR = 4;

int cmd = 0;
String position = "";
int move_flag = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_PIN, OUTPUT); 
  pinMode(PUL_PIN, OUTPUT);
  pinMode(INDUCTIVE_SR, INPUT); 

  pinMode(NEG_DIR_PIN, OUTPUT); 
  digitalWrite(NEG_DIR_PIN, LOW);

  Serial.begin(9600); 
}

int inductiveSrDetect() {
  int inductive = digitalRead(INDUCTIVE_SR);
  return inductive;
}

void read(){
  static bool dataTransferring = false;  //true if data transfer in progress
  char rc;                               //Received character
  static int i = 0;
  String data_print = "!Receive data : ";
  while (Serial.available() > 0 && i < MAX_DATA_SIZE) {
    rc = Serial.read();     //Reads one char of the data
    if (rc == startChar) {  //Start data transfer if startChar found
      dataTransferring = true;
      position = "";
    } 
    else if (dataTransferring) {
      if (rc != endChar) {  //Transfer data as long as endChar notfound
        position += rc;       //Save data
        i++;
      } 
      else {                //Stop data transfer if endChar found
        dataTransferring = false;  //Stop data transfer
        i = 0;
        data_print += position;
        Serial.println(data_print);
        move_flag = 1;
      }
    }
  }
}
void move(){
  int direct = digitalRead(DIR_PIN);
  if (position == "LEFT"){
    if (inductiveSrDetect() == LOW && direct == HIGH){
      Serial.println("!LIMIT MOVE DETECT");
      return;
    }
    digitalWrite(DIR_PIN,HIGH);
    for (int i = 0 ; i < 100 ; i++){
      digitalWrite(PUL_PIN,HIGH); 
      delayMicroseconds(250); 
      digitalWrite(PUL_PIN,LOW); 
      delayMicroseconds(250); 
    }
    Serial.println("!GO LEFT");
  }
  else if (position == "RIGHT"){
    if (inductiveSrDetect() == LOW && direct == LOW){
      Serial.println("!LIMIT MOVE DETECT");
      return;
    }
    digitalWrite(DIR_PIN,LOW);
    for (int i = 0 ; i < 100; i++){
      digitalWrite(PUL_PIN,HIGH); 
      delayMicroseconds(250); 
      digitalWrite(PUL_PIN,LOW); 
      delayMicroseconds(250); 
    }
    Serial.println("!GO RIGHT");
  }
  else if (position == "AUTO"){ 
    while(true){
      if (inductiveSrDetect() == 0){
        digitalWrite(DIR_PIN, !digitalRead(DIR_PIN));
        for (int i = 0 ;i <100 ;i++){
          digitalWrite(PUL_PIN, HIGH);
          delayMicroseconds(250);
          digitalWrite(PUL_PIN,LOW);
          delayMicroseconds(250);
        }
        continue;
      }
      digitalWrite(PUL_PIN, HIGH);
      delayMicroseconds(250);
      digitalWrite(PUL_PIN,LOW);
      delayMicroseconds(250);
      read();
      if (position == "STOP"){
        break;
      }
    }
  }
  else{
    
  }
}

void loop() {
  read();
  if (move_flag == 1){
    move_flag = 0;
    move();
  }
}
