#include "communication.h"
//−−−−−−−−−−VARIABLES USED FOR THE SERIAL DATA TRANSFER−−−−−−−−−−//

#define AUTO_CMD    "goauto"
#define MANUAL_CMD  "manual"
#define GO_HOME_CMD "gohome"
#define STOP_CMD    "000000"
#define INITIAL_CMD "Initon"
#define GO_FOLD_CMD "gofold"
#define startChar   '!'         //Message start
#define endChar     '#'         //Message end
#define PUMP_ON     "PUMP:ON"
#define PUMP_OFF    "PUMP:OFF"
#define MAX_DATA_SIZE 100

int cmd = 0;
String position = "";
int move_flag = 0;

void read() {
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