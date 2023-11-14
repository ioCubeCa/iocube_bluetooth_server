#include "IOCUBE.h"

IOCUBE myCube;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myCube.begin();
  myCube.setCallback(cubeEventCheck);
  myCube.ble_server_on();
}

void loop() {
  // put your main code here, to run repeatedly:  
}

void cubeEventCheck(byte eventCode,byte eventValue,byte inputID){
    switch(eventCode){
      case EC_INPUT_S:
        break;
      case EC_INPUT_O:
        break;
      case EC_INPUT_X:
        break;
    }
}
