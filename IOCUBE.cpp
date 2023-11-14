#include <Arduino.h>
#include <Wire.h>
#include "IOCUBE.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "FS.h"
#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED true

#define flashTempSize 1024
byte IOCUBE::m_script[flashTempSize] = {};
byte IOCUBE::m_sprite[flashTempSize] = {};

const byte animate[][8] =
{
  {0,0,0,0,0,0,0,0},  // 0:00000000 1
  {0,0,0,0,0,0,0,1},  // 1:00000001 2_1
  {0,1,0,1,0,1,0,1},  // 2:01010101 2_2 
  {0,0,1,1,0,0,1,1},  // 3:00110011 2_3
  {0,0,0,0,1,1,1,1},  // 4:00001111 2_4
  
  {0,1,2,1,0,1,2,1},  // 5:01210121 3_1
  {0,1,0,2,0,1,0,2},  // 6:01020102 3_2
  {1,2,3,1,2,3,1,2},  // 7:12312312 3_3
  {1,3,1,2,1,3,1,2},  // 8:13121312 3_4
  {2,3,4,3,2,3,4,3},  // 9:23432343 3_5
  {5,6,7,6,5,6,7,6},  //10:56765676 3_6

  {0,1,2,3,0,1,2,3},  //11:01230123 4_1
  {0,3,2,1,0,3,2,1},  //12:03210321 4_2
  {1,2,3,4,1,2,3,4},  //13:12341234 4_3
  {0,0,1,1,2,2,3,3},  //14:00112233 4_4

  {0,1,2,3,4,5,6,7},  //15:01234567 8
};

const byte listRandom[16] = {0x09,0xb2,0xd5,0x48,0x54,0xf0,0x23,0x67,0x81,0x36,0xa9,0xc4,0x72,0x17,0xe3,0x98};
static byte listRandomPair[16] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

const byte dirOrdinal[8] = {DIRECTION_1,DIRECTION_3,DIRECTION_5,DIRECTION_6,DIRECTION_7,DIRECTION_9,DIRECTION_11,DIRECTION_12};

CALLBACK IOCUBE::eventCallback       = NULL;
byte IOCUBE::stSerialIndex           = 0;
byte IOCUBE::stRecvIndex             = 0;
byte IOCUBE::stRecvBuffer[dRecvSize] ={};
byte IOCUBE::stSendIndex             = 0;
byte IOCUBE::stSendBuffer[dSendSize] ={};
byte IOCUBE::st_bMode = MODE_SCRIPT;

byte IOCUBE::stageLoopStart          = 0;
byte IOCUBE::oiSelected              = OI_PASS;
byte IOCUBE::m_random               = 0;
byte IOCUBE::m_ordinal              = 0;
#define STATE_VALUE_MAX 6
byte IOCUBE::m_state[STATE_VALUE_MAX] = {0,0,0,0,0,0};

//const byte *IOCUBE::st_ptSprite;
//const byte *IOCUBE::st_ptStoryboard;
const byte *IOCUBE::st_ptAnimate;
//uint16_t IOCUBE::stTaskRunInterval = 100;
uint16_t IOCUBE::timeCount = 0;
byte IOCUBE::m_at_index  = 0;
byte IOCUBE::m_sum  = 0;
bool IOCUBE::m_flag = false;
byte IOCUBE::m_boundary_cd = 0x00;
int8_t IOCUBE::m_left    = -1;
int8_t IOCUBE::m_bottom  = -1;
int8_t IOCUBE::m_right   =  8;
int8_t IOCUBE::m_top     =  8;
int8_t IOCUBE::m_result  = 0;
int8_t IOCUBE::m_u = 0;
int8_t IOCUBE::m_v = 0;
int8_t IOCUBE::m_w = 0;
int8_t IOCUBE::m_x = 0;
int8_t IOCUBE::m_y = 0;
int8_t IOCUBE::m_z = 0;
int8_t IOCUBE::m_object_x[dObjectLimit] ={};
int8_t IOCUBE::m_object_y[dObjectLimit] ={};
int8_t IOCUBE::m_object_xDest[dObjectLimit] ={};
int8_t IOCUBE::m_object_yDest[dObjectLimit] ={};
int8_t IOCUBE::m_object_xVel[dObjectLimit] ={};
int8_t IOCUBE::m_object_yVel[dObjectLimit] ={};
int8_t IOCUBE::m_object_value[dObjectLimit] ={};
byte IOCUBE::m_object_dir[dObjectLimit] ={};
byte IOCUBE::m_object_width[dObjectLimit] ={};
byte IOCUBE::m_object_height[dObjectLimit] ={};
byte IOCUBE::m_object_cd[dObjectLimit] ={};
byte IOCUBE::m_object_image[dObjectLimit] ={};
byte IOCUBE::m_object_at[dObjectLimit] ={};
//byte IOCUBE::m_cd_now   = 0;
//byte IOCUBE::m_cd_last  = 0;
//byte IOCUBE::m_cd_count = 0;
//byte IOCUBE::m_map[120]={};

//for function direction turn_left & turn_right
const byte dirClock[4]= {3,6,9,12};
int8_t IOCUBE::dirCount[dObjectLimit]={};

byte IOCUBE::matrixA[8]={};
byte IOCUBE::matrixB[8]={};

byte IOCUBE::m_event_code=0;    //storage input event code
byte IOCUBE::m_event_value=0;   //storage input event value
byte IOCUBE::myId=0xFE;         //HOST ID
byte IOCUBE::m_device_id  =0;   //storage input event device id

byte IOCUBE::cap1293_count[3] ={};
byte IOCUBE::cap1293_old = 0;

//bluetooth variable
#define SERVICE_UUID           "00001101-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID_RX "ac94d26c-4bff-11ec-81d3-0242ac130003"
#define CHARACTERISTIC_UUID_TX "ce85b106-4c00-11ec-81d3-0242ac130003"

#define bleStateLed 10
static bool bleEnable = false;
static bool ledState  = false;
static uint8_t bleBuffer[136]={};
static uint8_t bleBufferIndex=0;
static bool bleTransfer=false;
//bluetooth server variable
static BLEServer *pServer = NULL;
//static BLECharacteristic* pCharacteristic = NULL;
static BLECharacteristic * pTxCharacteristic = NULL;
static bool deviceConnected=false;
static bool oldDeviceConnected=false;
//#define bleClientMax 4
#define bleClientMax 2
//static uint8_t ble_clientList[bleClientMax] = {0,0,0,0};
static uint8_t ble_clientIndex = 0;
//static uint16_t iSum = 0;
//=====================================================================
//bluetooth server callback class
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(bleStateLed,HIGH);
      BLEDevice::startAdvertising();  //for multiconnect
      //Serial.println("BLE device Connected.");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      //Serial.println("BLE device Disconnected.");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      byte bLength   = value.length();
      byte bByteX    = 0;
      byte bTmpEvent = 0;
      byte bSum      = 0;
      
      if (bLength > 0) {
        for(byte i=0;i<(bLength-1);i++)bSum += value[i];
        //check sum equal
        if(bSum== value[(bLength-1)]){
          //bluetooth data transfer to Serial1
          for (int i = 0; i < bLength; i++){
            Serial.print(value[i]);
            //Serial1.print(value[i]);
            bleBuffer[i]=value[i];
          }
          bleTransfer=true;
          bleBufferIndex=bLength;
          //print log
          //Serial.print(bLength);
          //Serial.println(" BLE Received.");  
        }

      }
    }
};
//=====================================================================
void IOCUBE::ble_server_on(){
    //bluetooth is server
    bleEnable=true;
    pinMode(bleStateLed,OUTPUT);
    digitalWrite(bleStateLed,HIGH);
    delay(1000);
    digitalWrite(bleStateLed,LOW);
    //Create the BLE Device
    BLEDevice::init("IOCUBE");
    
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
  
    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
  
    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                       );
    pTxCharacteristic->addDescriptor(new BLE2902());
                       
    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                        );
    pRxCharacteristic->setCallbacks(new MyCallbacks());
  
    // Start the service
    pService->start();
    
    //Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();      
}
//=====================================================================
void IOCUBE::ble_check(){
    //bluetooth is server
    //notify data get from Serial1
    if(deviceConnected){
      if(bleBufferIndex)  {
          pTxCharacteristic->setValue((uint8_t*)&bleBuffer[0],bleBufferIndex);
          pTxCharacteristic->notify();
          bleBufferIndex=0;      
      }
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        //Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }   
}
//=====================================================================
void IOCUBE::deleteFile(fs::FS &fs, const char * path){
    //Serial.printf("Deleting file: %s\r\n", path);
    //if(fs.remove(path)){
    if(SPIFFS.remove(path)){
        //Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}
//=====================================================================
void IOCUBE::writeFile(fs::FS &fs,const char * path,unsigned char * inputData ){
  //Serial.printf("Writing file: %s\r\n", path);

  //File file = fs.open(path, FILE_WRITE);
  File fileToWrite = SPIFFS.open(path,"wb");
  if(!fileToWrite){
      Serial.println("- failed to open file for writing");
      return;
  }
  if(fileToWrite.write(inputData,flashTempSize)){
    //Serial.println("- file was written");
  }else{
    //Serial.println("- write fail!");
  }
  fileToWrite.close();
}
//=====================================================================
void IOCUBE::readFile(fs::FS &fs, const char * path,byte * outputBuf){
    //Serial.printf("Reading file: %s\r\n", path);
    File fileToRead = SPIFFS.open(path);
    if(!fileToRead){
        Serial.println("Failed to open file for reading");
        return;
    }
    //Serial.println("File Content:");
    int i=0;
    while(fileToRead.available()){
        byte tmp = fileToRead.read();
        outputBuf[i] = tmp;

        //if(outputBuf[i] < 16)Serial.print(0,HEX);
        //Serial.print(outputBuf[i],HEX);  
        
        i++;
        if(i > flashTempSize)break;
    }
    Serial.println();
    fileToRead.close();   
}


//=====================================================================
void IOCUBE::cap1293_read(){
  byte tmp;
  byte rIndex;
  byte inputValue;
  byte recvBuffer[3]={};
  byte button_state;
  byte button_index;
  byte error;

  Wire.beginTransmission(0x28);
  Wire.write(0x10);
  delay(1);
  error = Wire.endTransmission(true);

  uint8_t bytesReceived = Wire.requestFrom(0x28, 3);
  if((bool)bytesReceived){   //If received more than zero bytes
    Wire.readBytes(recvBuffer, bytesReceived);
  } 
  for(uint8_t i=0;i<3;i++)if(recvBuffer[i] > 0x7F)recvBuffer[i]=0;
    
  inputValue=0;
  if(recvBuffer[0] > 0x40){
    inputValue += 0x01;
  }
  if(recvBuffer[1] > 0x40){
    inputValue += 0x02;
  }
  if(recvBuffer[2] > 0x40){
    inputValue += 0x04;
  }

    if(inputValue != cap1293_old){
      m_random  = random(256);    
      m_state[0] = recvBuffer[0]; //input S
      m_state[1] = recvBuffer[1]; //input O
      m_state[2] = recvBuffer[2]; //input X

      if(inputValue > cap1293_old){
        switch(inputValue){
          case 0x01:
            cap1293_count[0]++;
            m_ordinal = cap1293_count[0];
            break;
          case 0x02:
            cap1293_count[1]++;
            m_ordinal = cap1293_count[1];
            break;
          case 0x04:
            cap1293_count[2]++;
            m_ordinal = cap1293_count[2];
            break;
        }
        button_index = inputValue;
        button_state = 1;
      }else{
        button_index = cap1293_old;
        button_state = 0;
      }
      
      sendChar(0);                  //IIC Address(broadcast)
      sendChar(10);                  //length
      sendChar(0xFF);               //TargetId:0x00~0xFE 0xFF->broadcast
      //sendChar(myId);               //sourceId myId=0xFE
      sendChar(myId);               //sourceId myId=0x00
      sendChar(cmdInputState);      //command 
      sendChar(button_index);       //button index
      sendChar(button_state);       //button state
      sendChar(m_random);           //random value
      sendChar(m_ordinal);          //button Count
      sendChar(m_state[0]);         //m_state[0] square
      sendChar(m_state[1]);         //m_state[1] circle
      sendChar(m_state[2]);         //m_state[2] cross
      sendSum();  

      bleTransfer=false;
      tmp = inputValue;
      stDispatchEvent(button_index,button_state,myId);

      //Serial.print("cap1293:");
      //Serial.println(inputValue,HEX);
    }
  
  cap1293_old = inputValue;

  //print cap1293 buffer value
  //Serial.print("cap1293:");
  //Serial.print(recvBuffer[0],HEX);
  //Serial.print(recvBuffer[1],HEX);
  //Serial.println(recvBuffer[2],HEX);
  
}
//=====================================================================
byte IOCUBE::stEventSet  =0;
byte IOCUBE::stEventGet  =0;
byte IOCUBE::stEventCode[dEventQueueSize]={};
byte IOCUBE::stEventValue[dEventQueueSize]={};
byte IOCUBE::stEventID[dEventQueueSize]={};
//=====================================================================
void IOCUBE::stDispatchEvent(byte bEventCode,byte bEventValue,byte bEventId){
  stEventCode[stEventSet] = bEventCode;
  stEventValue[stEventSet] = bEventValue;
  stEventID[stEventSet] = bEventId;
  stEventSet++;
  if(stEventSet >= dEventQueueSize)stEventSet=0;
}
//=====================================================================
void IOCUBE::eventTrigger(){
  if(stEventSet != stEventGet){
    byte bEventCode = stEventCode[stEventGet];
    byte bEventValue = stEventValue[stEventGet];
    byte bEventId   = stEventID[stEventGet];
    stEventGet++;
    if(stEventGet >= dEventQueueSize)stEventGet=0;
    appScriptEngine(bEventCode,bEventValue,bEventId);
    if(eventCallback)eventCallback(bEventCode,bEventValue,bEventId);    
  }
}
//=====================================================================
void IOCUBE::drawBitmap(byte id,const byte aBitmap[]){
  
  sendChar(0);      //IIC Address(broadcast)
  sendChar(11);     //length
  sendChar(id);     //TargetId
  sendChar(myId);   //sourceId
  sendChar(cmdSetAttribute);   //command 0x05:setAttribute
  sendChar(OI_LED8x8);         //object id
  for(byte i=0;i<8;i++)sendChar(aBitmap[i]);
  sendSum();
}
//=====================================================================
void IOCUBE::setMode(byte bMode){
    
  sendChar(0);            //IIC Address(broadcast)
  sendChar(4);            //length
  sendChar(0xFF);         //TargetId
  sendChar(myId);         //TargetId
  sendChar(cmdSetMode);   //command
  sendChar(bMode);        //mode data
  sendSum();

  st_bMode=bMode;

}
//=====================================================================
void IOCUBE::setBrightness(byte bValue){

  if(bValue > 15)bValue=15;
  if(bValue <= 0)bValue=0;
  sendChar(0);            //IIC Address(broadcast)
  sendChar(4);            //length
  sendChar(0xFF);         //TargetId
  sendChar(myId);         //TargetId
  sendChar(cmdSetBrightness);   //command
  sendChar(bValue);        //mode data
  sendSum();

}

//=====================================================================
void IOCUBE::stSerial(){
   byte bByteX;
   byte bByteL=1;
   int  iReadTimeOut;
   byte bSum=0;
   byte bTmpEvent;
   int addr;
   int addrIndex;
   
   if(Serial.available()) stSerialIndex = 1;
   if(Serial1.available()) stSerialIndex = 2;
   
 
      if(stSerialIndex == 1){
        bByteX = Serial.read();
        if(bByteX==0x00){
          stRecvBuffer[stRecvIndex]=bByteX;
          stRecvIndex++;
          if(Serial.available()){
            bleTransfer=false;
            bByteL = Serial.read();
            stRecvBuffer[stRecvIndex]=bByteL;
            stRecvIndex++;
            iReadTimeOut=1000;
            bByteL++;
            while(bByteL){
              if(Serial.available()){
                bByteX = Serial.read();
                stRecvBuffer[stRecvIndex]=bByteX;
                stRecvIndex++;              
                bByteL--;
                iReadTimeOut=1000;
              }
              iReadTimeOut--;
              if(iReadTimeOut==0)break;
            }
          }
        }
     }

 
      if(stSerialIndex == 2){
        bByteX = Serial1.read();
        if(bByteX==0x00){
          stRecvBuffer[stRecvIndex]=bByteX;
          stRecvIndex++;
          if(Serial1.available()){
            bleTransfer=false;
            bByteL = Serial1.read();
            stRecvBuffer[stRecvIndex]=bByteL;
            stRecvIndex++;
            iReadTimeOut=1000;
            bByteL++;
            while(bByteL){
              if(Serial1.available()){
                bByteX = Serial1.read();
                stRecvBuffer[stRecvIndex]=bByteX;
                stRecvIndex++;              
                bByteL--;
                iReadTimeOut=1000;
              }
              iReadTimeOut--;
              if(iReadTimeOut==0)break;
            }
          }
        }
     }

   if(bleTransfer){
      stRecvIndex=bleBufferIndex;
      for(byte i=0;i<bleBufferIndex;i++)stRecvBuffer[i]=bleBuffer[i];
      bleBufferIndex=0;
      stSerialIndex = 1;
   }

   if(stRecvIndex){
      for(byte i=0;i<(stRecvIndex-1);i++)bSum += stRecvBuffer[i];
      if(bSum== stRecvBuffer[(stRecvIndex-1)]){
        if(stRecvIndex==1){
          //Serial.print("noise.");
        }else{
            //if not trasferdata from ble.save data to bleBuffer
            if(bleTransfer==false){
              bleBufferIndex=stRecvIndex;
              for(byte i=0;i<stRecvIndex;i++)bleBuffer[i]=stRecvBuffer[i];
            }  
            //print log
            /*
            Serial.print("Recv(");
            Serial.print(stRecvIndex);
            Serial.print("):");
            for(byte i=0;i<stRecvIndex;i++){
              if(stRecvBuffer[i] < 16)Serial.print(0,HEX);
              Serial.print(stRecvBuffer[i],HEX);
            }
            Serial.println();  
            */
            //print data
            //if(stSerialIndex == 1){
              
            //}
            
            if(stSerialIndex == 2){
              for(byte i=0;i<stRecvIndex;i++)Serial.write(stRecvBuffer[i]);
            }else{
              for(byte i=0;i<stRecvIndex;i++){
                Serial1.write(stRecvBuffer[i]);
              }
            }


            
           //trigger callback event
           switch(stRecvBuffer[4]){
              case cmdInputState: //button input event
                m_random = stRecvBuffer[7];
                m_ordinal= stRecvBuffer[8];
                bByteX = stRecvBuffer[3]; //sourceId
                for(byte i=0;i<STATE_VALUE_MAX;i++)m_state[i] = stRecvBuffer[9+i];
                //bTmpEvent = (stRecvBuffer[6]<<4)|(stRecvBuffer[5] & 0x0F);
                //bTmpEvent = stRecvBuffer[5];
                stDispatchEvent(stRecvBuffer[5],stRecvBuffer[6],bByteX);
                break;
              case cmdStoryBoardErase:
                //iSum = 0;
                break;
              case cmdStoryBoardWrite:
                addrIndex=stRecvBuffer[5]*8;
                for(byte i=0;i<8;i++){
                  addr = addrIndex+i;
                  bByteX = stRecvBuffer[i+6];
                  m_script[addr] = bByteX;
                  //iSum+=bByteX;
                }
                break;
              case cmdSpriteErase:
          
                break;
              case cmdSpriteWrite:
                addrIndex=stRecvBuffer[5]*8;
                for(byte i=0;i<8;i++){
                  addr = addrIndex+i;
                  bByteX = stRecvBuffer[i+6];
                  m_sprite[addr] = bByteX;
                }
                break;
              case cmdSetMode:
                st_bMode = stRecvBuffer[5];
                //Serial.print("MODE=");
                //Serial.println(st_bMode);
                if(st_bMode == MODE_SCRIPT){
                  deleteFile(SPIFFS, "/script.txt");
                  writeFile(SPIFFS, "/script.txt",m_script);
                  deleteFile(SPIFFS, "/sprite.txt");
                  writeFile(SPIFFS, "/sprite.txt",m_sprite); 
                   
                }
                appInit();
                break;
              case cmdRequestID:
                //dispatch ID to client
                bleBuffer[0] = 0;
                bleBuffer[1] = 7;
                bleBuffer[2] = stRecvBuffer[3]; //targetID
                bleBuffer[3] = DI_ID_HOST;
                bleBuffer[4] = cmdSetID;
                bleBuffer[5] = ble_clientIndex;
                bleBuffer[6] = 0;
                bleBuffer[7] = stRecvBuffer[5];
                bleBuffer[8] = stRecvBuffer[6];
                bByteX = 0; //caculate check sum
                for(byte i=0;i<9;i++)bByteX += bleBuffer[i];
                bleBuffer[9] = bByteX;
                bleBufferIndex = 10;
                //Serial.print("Dispatch New ID:");
                //Serial.println(ble_clientIndex);
                ble_clientIndex++;
                if(ble_clientIndex >= bleClientMax)ble_clientIndex=0;
                break;
           }
        }
      }else{
        //Serial.print("sum error! ");
      }
      stRecvIndex=0;     
   }

   //check sendIndex
   if(stSendIndex)
   {
    //if is not trasferdata from ble.save data to bleBuffer
    if(bleTransfer==false){
      bleBufferIndex=stSendIndex;
      for(byte i=0;i<stSendIndex;i++)bleBuffer[i]=stSendBuffer[i];
    }  

    //send data to serial
    for(byte i=0;i<stSendIndex;i++){
      Serial.write(stSendBuffer[i]);
      Serial1.write(stSendBuffer[i]);
    }

    //print log
    /*
    Serial.print("Send(");
    Serial.print(stSendIndex);
    Serial.print("):");
    for(byte i=0;i<stSendIndex;i++){
      if(stSendBuffer[i] < 16)Serial.print(0,HEX);
      Serial.print(stSendBuffer[i],HEX);
    }
    Serial.println();  
    */
    stSendIndex=0;
   }  
}
//=====================================================================
// CONSTRUCTORS, DESTRUCTOR
IOCUBE::IOCUBE()
{
  
}

IOCUBE::~IOCUBE()
{

}
//=====================================================================
void IOCUBE::setCallback(CALLBACK CB){
  IOCUBE::eventCallback = CB;
}
//=====================================================================
//ESP32_C3 ioCube
//LED io_10
//IIC SCL io_4
//IIC SDA io_5
//uart1 TXD io_19
//uart1 RXD io_18
void IOCUBE::begin(){
  Serial1.begin(57600, SERIAL_8N1, 18, 19);

  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      Serial.println("SPIFFS Mount Failed");
      SPIFFS.format();
      return;
  }
  readFile(SPIFFS,"/script.txt",m_script);
  readFile(SPIFFS,"/sprite.txt",m_sprite);
  Serial.println("read file finish.");
  Wire.begin(5,4);
  Serial.println("Wire begin.");
  xTaskCreatePinnedToCore(taskSerial,"Task1",8192,NULL,2,NULL,0);
  Serial.println("task run.");
  //xTaskCreatePinnedToCore(taskRun,"Task2",8192,NULL,5,NULL,0); 
  //Serial.println("task main run.");
  st_ptAnimate = &animate[0][0];
  st_bMode = MODE_SCRIPT;
  appInit();
  Serial.println("app start.");
}
//=====================================================================
unsigned char taskCount;

void IOCUBE::taskSerial(void *pvParameters){
  //(void) pvParameters;
  for (;;){
    stSerial();
    ble_check();

    
    taskCount++;
    if(taskCount >=10){
      taskCount = 0;
      if(bleEnable){
        if(deviceConnected==false){
          ledState = !ledState;
          digitalWrite(bleStateLed,ledState);   
        }
      }

      if(st_bMode==MODE_SCRIPT){
        timeCount++;
        appScriptEngine(EC_TIME_100MS,0,myId);
        if((timeCount % 10)==0){
          appScriptEngine(EC_TIME_1SECOND,0,myId);
        }
        if((timeCount % 600)==0)appScriptEngine(EC_TIME_1MINUTE,0,myId);
        //3600 second = 60 minutes = 1 hour
        if(timeCount >=36000){
          appScriptEngine(EC_TIME_1HOUR,0,myId);
          timeCount=0;
        }    
        
        eventTrigger();
    
        appScriptEngine(EC_ROUTINE,0,myId);
        object_update_xy();
        object_cd_boundary();
        object_cd_object();
        object_update_to_map(); 
      }

      if(st_bMode != MODE_UPLOAD)cap1293_read();
    }
    

    vTaskDelay(10);
  }
}
//=====================================================================
/*
void IOCUBE::taskRun(void *pvParameters){  
  for (;;){
    if(bleEnable){
      if(deviceConnected==false){
        ledState = !ledState;
        digitalWrite(bleStateLed,ledState);   
      }
    }

    if(st_bMode==MODE_SCRIPT){
      timeCount++;
      appScriptEngine(EC_TIME_100MS,0,myId);
      if((timeCount % 10)==0){
        appScriptEngine(EC_TIME_1SECOND,0,myId);
      }
      if((timeCount % 600)==0)appScriptEngine(EC_TIME_1MINUTE,0,myId);
      //3600 second = 60 minutes = 1 hour
      if(timeCount >=36000){
        appScriptEngine(EC_TIME_1HOUR,0,myId);
        timeCount=0;
      }    
      
      eventTrigger();
  
      appScriptEngine(EC_ROUTINE,0,myId);
      object_update_xy();
      object_cd_boundary();
      object_cd_object();
      object_update_to_map(); 
    }

    if(st_bMode != MODE_UPLOAD)cap1293_read();
    
    vTaskDelay(100);
  }
}
*/
//=====================================================================
void IOCUBE::appInit(){
  stageLoopStart=0;
  oiSelected = OI_PASS;
  timeCount = 0;

  m_ordinal=0;
  m_random =0;

  m_left    = -1;
  m_bottom  = -1;
  m_right   =  8;
  m_top     =  8;

  m_boundary_cd = 0x00;

  for(byte i;i<dObjectLimit;i++)object_init(i);
  
  m_at_index  = 0;
  m_u = 0;
  m_v = 0;
  m_w = 0;
  m_x = 0;
  m_y = 0;
  m_z = 0;
  m_result=0;
  m_flag = false;
  
  //delay(2000);
  timeCount = 3;
}
//=====================================================================
void IOCUBE::object_init(byte bId){
  if(bId < dObjectLimit){
    m_object_x[bId]      = 0;
    m_object_y[bId]      = 0;
    m_object_xDest[bId]  = dBypassValue; //X destination
    m_object_yDest[bId]  = dBypassValue; //Y destination
    m_object_xVel[bId]   = 0;            //X velocity
    m_object_yVel[bId]   = 0;            //Y velocity
    m_object_width[bId]  = 1;
    m_object_height[bId] = 1;
    m_object_value[bId]  = 0;
    m_object_cd[bId]     = 0;                  //Collision Detection
    m_object_image[bId]  = dBypassImage;       //load sprite image
    m_object_at[bId]     = dBypassAnimate;     //animation  
    m_object_dir[bId]    = 0;                  //direction 
  }else{
   doObjectSet(bId,PN_VALUE,0,0);
  }
}
//=====================================================================

void IOCUBE::object_update_xy(){
  byte i;
  
  for(i=0;i<dObjectLimit;i++){
    if(m_object_value[i] > 0){
      //check X destination. set X velocity
      if(m_object_xDest[i] != dBypassValue){
        if(m_object_xDest[i]  > m_object_x[i])m_object_xVel[i] =  1;
        if(m_object_xDest[i]  < m_object_x[i])m_object_xVel[i] = -1;
      }
      //check Y destination. set Y velocity
      if(m_object_yDest[i] != dBypassValue ){
        if(m_object_yDest[i]  > m_object_y[i])m_object_yVel[i] =  1;
        if(m_object_yDest[i]  < m_object_y[i])m_object_yVel[i] = -1;
      }
      //set object X Y with velocity
      if(m_object_xVel[i] != 0)m_object_x[i] += m_object_xVel[i];
      if(m_object_yVel[i] != 0)m_object_y[i] += m_object_yVel[i];
      m_object_dir[i] = object_check_direction(i);

      //if object X Y. equal destination X Y.set velocity=0
       if(m_object_xDest[i] != dBypassValue){
         if(m_object_xDest[i] == m_object_x[i]){
           m_object_xVel[i] = 0;
           m_object_xDest[i] = dBypassValue;
         }
       }
       
       if(m_object_yDest[i] != dBypassValue){
         if(m_object_yDest[i] == m_object_y[i]){
          if((m_object_cd[i] & PN_CD_GRAVITY) &&
             (m_object_y[i] > 0)){
                m_object_yDest[i] = 0;
                m_object_yVel[i]  = -1;
             }
          else{
            m_object_yVel[i] = 0;
            m_object_yDest[i] = dBypassValue;
          }
         }
       }
    }
  }
}

//=====================================================================

byte IOCUBE::object_check_direction(byte bId){
/* O=Ojbect
 **-X+Y   0X+Y   +X+Y
 *    \    |   /
 *      11 12 1
 *-X0Y- 9  O  3- +X0Y
 *      7  6  5
 * -X-Y  0X-Y    +X-Y
 */
  if((m_object_xVel[bId] == 0)&&(m_object_yVel[bId] == 0))return DIRECTION_0;
  if((m_object_xVel[bId]  < 0)&&(m_object_yVel[bId]  < 0))return DIRECTION_7;
  if((m_object_xVel[bId]  < 0)&&(m_object_yVel[bId] == 0))return DIRECTION_9;
  if((m_object_xVel[bId]  < 0)&&(m_object_yVel[bId]  > 0))return DIRECTION_11;
  if((m_object_xVel[bId] == 0)&&(m_object_yVel[bId]  > 0))return DIRECTION_12;
  if((m_object_xVel[bId]  > 0)&&(m_object_yVel[bId]  > 0))return DIRECTION_1;
  if((m_object_xVel[bId]  > 0)&&(m_object_yVel[bId] == 0))return DIRECTION_3;
  if((m_object_xVel[bId]  > 0)&&(m_object_yVel[bId]  < 0))return DIRECTION_5;
  if((m_object_xVel[bId] == 0)&&(m_object_yVel[bId]  < 0))return DIRECTION_6;

  return DIRECTION_0;
}
//=====================================================================
/* object collision detection with other objects
 * define collision direction value
 * object_A collision with object_B
 * A        A        A
 *  +X-Y   -Y    -X-Y
 *      \   |   /
 *       11 12  1
 * A +X -9  B   3 - -X A
 *       7  6   5
 *      /   |   \
 *  +X+Y   +Y    -X+Y
 *A         A        A
 */    
void IOCUBE::object_cd_object(){
  byte A;
  byte B;
  byte cd_A=0x00;
  byte cd_B=0x00;
  
  for(A=0;A<dObjectLimit;A++){
    if(m_object_cd[A]){
      if((m_object_cd[A] & 0x07) != PN_CD_INVERT_VXorVY_OB){
        for(B=0;B<dObjectLimit;B++){
            if(object_pixel_collision(A,B))object_cd_behavior(A,B);
        }         
      }
    }
  }

/*
  //unlock collision object
  m_cd_now = ((cd_A<<4)|cd_B);
  if(m_cd_now)
  {
    if(m_cd_now == m_cd_last){
      m_cd_count++;
      if(m_cd_count > 200){
        m_cd_count=0;
        m_object_x[cd_A]+=1;
        m_cd_now=0x00;
        //Serial.println("unlock collision object");
      }
    }else{
      m_cd_count=0;
    }
    m_cd_last=m_cd_now;
  }
*/

  
}

//=====================================================================
byte IOCUBE::object_pixel_collision(byte A,byte B){
    int8_t z=0;
    int8_t Ax=0;
    int8_t Ay=0;
    int8_t Bx=0;
    int8_t By=0;

    if(m_object_value[A]<=0)return(0);
    if(m_object_value[B]<=0)return(0);
    if(A==B)return(0);

    Ax = m_object_x[A];
    Ay = m_object_y[A];
    Bx = m_object_x[B];
    By = m_object_y[B];
    if(Ax<Bx)z=Ax; else z=Bx;
    if(z>0){
        while(z){
            Ax--;
            Bx--;
            z--;
        }        
    }
    if(z<0){
        while(z){
            Ax++;
            Bx++;
            z++;
        }
    }
    if(Ay<By)z=Ay; else z=By;
    if(z>0){
        while(z){
            Ay--;
            By--;
            z--;
        }        
    }
    if(z<0){
        while(z){
            Ay++;
            By++;
            z++;
        }
    }

    for(z=0;z<8;z++){
      matrixA[z] = 0;
      matrixB[z] = 0;
    }

    object_zero_draw(A,Ax,Ay,0);
    z=object_zero_draw(B,Bx,By,1);
    
    return(z);
}
//=====================================================================
byte IOCUBE::object_zero_draw(byte i,byte input_x,byte input_y,byte mIndex){
    byte tmp=0;
    byte tmpW;
    byte tmpX;
    byte tmpByte=0x00;
    int addr;

    
    if(m_object_image[i]== dBypassImage){
        for(tmpW=0;tmpW < m_object_height[i];tmpW++)tmpByte |= (0x01<<tmpW);
    }else{
        if(m_object_at[i] != dBypassAnimate) tmp = *(st_ptAnimate + ((m_object_at[i] & 0x7F) * 8) + m_at_index);
        object_measure_size(i,tmp); 
    }
    for(tmpW=0;tmpW < m_object_width[i];tmpW++){
        //check image is need mirror
        if(m_object_image[i]!= dBypassImage){
            if((m_object_at[i] & 0x80)&&(m_object_at[i] != dBypassAnimate)) {
              addr = ((m_object_image[i]+tmp) * 8) + (m_object_width[i]-1-tmpW);
              tmpByte = m_sprite[addr]; //讀數據 
            }else{
              addr = ((m_object_image[i]+tmp) * 8) + tmpW;
              tmpByte = m_sprite[addr]; //讀數據 
            }    
        } 



        tmpX = input_x + tmpW;
        if(tmpX < 8){
          if(mIndex==0)matrixA[tmpX] |= (tmpByte<<input_y);  else matrixB[tmpX] |= (tmpByte<<input_y); 
        }
    }

    if(mIndex==1){
        for(tmpW=0;tmpW<8;tmpW++)if(matrixA[tmpW] & matrixB[tmpW])return(1);
    }
    return(0);
    
}

//=====================================================================
//object collision detection with other boundary
void IOCUBE::object_cd_boundary(){
  
  unsigned char i;
//  unsigned char bTmpEvent;
  
  for(i=0;i<dObjectLimit;i++){
    if(m_object_value[i] > 0){
       //is object over map boundary
      if(m_left != dBypassValue){
         if(m_object_x[i] <= m_left){
            if(m_boundary_cd & PN_BOUNDARY_CD_LEFT_LINK_RIGHT)
            {
              m_object_x[i] = m_right - 1;
            }
            else
            {
              //check collision setting value
              object_cd_behavior(i,OI_BOUNDARY_L);
              //Trigger collision boundary event
              appScriptEngine(EC_CD_LEFT,0,i);
            }
         }
      }
       
      if(m_bottom != dBypassValue){
         if(m_object_y[i] <= m_bottom){
            if(m_boundary_cd & PN_BOUNDARY_CD_BOTTOM_LINK_TOP)
            {
              m_object_y[i] = m_top - 1;
            }
            else
            {
              object_cd_behavior(i,OI_BOUNDARY_B); 
              //Trigger collision boundary event
              appScriptEngine(EC_CD_BOTTOM,0,i);
            }
         }
      }
      
      if(m_right != dBypassValue){
         if((m_object_x[i] + (m_object_width[i]-1)) >= m_right){
            if(m_boundary_cd & PN_BOUNDARY_CD_RIGHT_LINK_LEFT)
            {
              if(m_object_x[i] >= m_right)m_object_x[i] = m_left + 1;
            }
            else
            {
              object_cd_behavior(i,OI_BOUNDARY_R);
              //Trigger collision boundary event
              appScriptEngine(EC_CD_RIGHT,0,i);
            }
         } 
      }

      if(m_top != dBypassValue){
         if((m_object_y[i] + (m_object_height[i]-1) ) >= m_top){
            if(m_boundary_cd & PN_BOUNDARY_CD_TOP_LINK_BOTTOM)
            {
              if(m_object_y[i] >= m_top)m_object_y[i] = m_bottom + 1;
            }
            else
            {
            object_cd_behavior(i,OI_BOUNDARY_T); 
            //Trigger collision boundary event
            appScriptEngine(EC_CD_TOP,0,i);
            }
          } 
      }
    }
  }
}
//=====================================================================
/* object collision with boundary. change velocity 
 *  if define compare value then compare value
 *  
 * O=OBJECT
 **-X+Y   0X+Y   +X+Y
 *    \    |   /
 *      11 12 1
 *-X0Y- 9  O  3- +X0Y
 *      7  6  5
 * -X-Y  0X-Y    +X-Y
 *
 */ 
void IOCUBE::object_cd_behavior(byte bId,byte bIdB){
  //check collision detection setting then change object.value
  //only get low byte 3bit.
  byte bCdValue = ((m_object_cd[bId] >> 4) & 0x07);

  //if F-bit is set.don't affect value
    switch(bCdValue){
      case 0:   //Reserved
        if(bIdB < dObjectLimit){
          bCdValue = m_object_cd[bIdB] & 0x70;
          if((m_object_cd[bId] & PN_CD_VALUE_FIXED)==0)
          {
            switch(bCdValue)
            {
              case PN_CD_VALUE_B_MINUS_A_A0:
                m_object_value[bId] -= m_object_value[bIdB];
                if(m_object_value[bId] < 0)m_object_value[bId] = 0;
                m_object_value[bIdB] = 0;              
                break;
              case PN_CD_VALUE_B_PLUS_A_A0:
                m_object_value[bId] += m_object_value[bIdB];
                if(m_object_value[bId] > 99)m_object_value[bId] = 99;
                m_object_value[bIdB] = 0;              
                break;
            }
          }else{
              if((m_object_cd[bIdB] & PN_CD_VALUE_FIXED)==0){
                  switch(bCdValue){
                      case PN_CD_VALUE_AB_MINUS_BA:
                          m_object_value[bIdB] -= m_object_value[bId];
                          break;
                      case PN_CD_VALUE_B_ZERO:
                          m_object_value[bIdB] = 0;
                          break;
                  }
              }              
          }
        }
        break;
      case 1:   //A=0
        if((m_object_cd[bId] & PN_CD_VALUE_FIXED)==0)m_object_value[bId]=0;
        break;
      case 2:   //A-=B
        if((m_object_cd[bId] & PN_CD_VALUE_FIXED)==0){
          if(bIdB < dObjectLimit)m_object_value[bId] -= m_object_value[bIdB];
        }
        break;
      case 3:   //A+=B
        if((m_object_cd[bId] & PN_CD_VALUE_FIXED)==0){
          if(bIdB < dObjectLimit)m_object_value[bId] += m_object_value[bIdB];
        }
        if(m_object_value[bId] >=100)m_object_value[bId] = 99;
        break;
      case 4:   //B=0
        if((m_object_cd[bIdB] & PN_CD_VALUE_FIXED)==0){
          if(bIdB < dObjectLimit)m_object_value[bIdB] = 0;
        }
        break;
      case 5:   //B-=A PN_CD_VALUE_B_MINUS_A_A0
        if((m_object_cd[bIdB] & PN_CD_VALUE_FIXED)==0){
          if(bIdB < dObjectLimit)m_object_value[bIdB] -= m_object_value[bId];
          if(m_object_value[bIdB] < 0)m_object_value[bIdB]=0;
        }
        m_object_value[bId] = 0;
        break;
      case 6:   //B+=A PN_CD_VALUE_B_PLUS_A_A0
        if((m_object_cd[bIdB] & PN_CD_VALUE_FIXED)==0){
          if(bIdB < dObjectLimit)m_object_value[bIdB] += m_object_value[bId];
        }
        if(m_object_value[bIdB] >=100)m_object_value[bIdB] = 99;   
        m_object_value[bId] = 0;
        break;
      case 7:   //A-=B & B-=A
        if(bIdB < dObjectLimit){
          bCdValue = m_object_value[bId];
          if((m_object_cd[bId] & PN_CD_VALUE_FIXED)==0)m_object_value[bId] -= m_object_value[bIdB];
          if((m_object_cd[bIdB] & PN_CD_VALUE_FIXED)==0)m_object_value[bIdB] -= bCdValue;
        }
        break;
    }    
  
    
    
  //Offset to last coordinate
  if((m_object_cd[bId] & 0x07) !=0){
    switch(m_object_dir[bId]){
      case DIRECTION_1:
        m_object_x[bId] -= 1 ;
        m_object_y[bId] -= 1 ;
        break;
      case DIRECTION_3:
        m_object_x[bId] -= 1 ;
        break;
      case DIRECTION_5:
        m_object_x[bId] -= 1 ;
        m_object_y[bId] += 1 ; 
        break;
      case DIRECTION_6:
        m_object_y[bId] += 1 ; 
        break;
      case DIRECTION_7:
        m_object_x[bId] += 1 ; 
        m_object_y[bId] += 1 ; 
        break;
      case DIRECTION_9:
        m_object_x[bId] += 1 ; 
        break;
      case DIRECTION_11:
        m_object_x[bId] += 1 ; 
        m_object_y[bId] -= 1 ;
        break;
      case DIRECTION_12:
        m_object_y[bId] -= 1 ;
        break;
    }
  }   
    
    
  //check collision set x,y,xVel,yVel
  switch(m_object_cd[bId] & 0x07){
    case PN_CD_NONE: //no check
      break;
    case PN_CD_INVERT_VXorVY_OB:
      //only check boundary.don't change velocity with object collision
    case PN_CD_INVERT_VXorVY: 
      //if x collision then xVel invert
      //if y collision then yVel invert
        if(bIdB >= OI_BOUNDARY_L){
            if(bIdB == OI_BOUNDARY_L)m_object_xVel[bId] = 1;
            if(bIdB == OI_BOUNDARY_B)m_object_yVel[bId] = 1;
            if(bIdB == OI_BOUNDARY_R)m_object_xVel[bId] = -1;
            if(bIdB == OI_BOUNDARY_T)m_object_yVel[bId] = -1;
        }else{
            if(m_object_dir[bId] == DIRECTION_1)m_object_yVel[bId] = -1;
            else if(m_object_dir[bId] == DIRECTION_5)m_object_xVel[bId] = -1;
            else if(m_object_dir[bId] == DIRECTION_7)m_object_yVel[bId] = 1;
            else if(m_object_dir[bId] == DIRECTION_11)m_object_xVel[bId] = 1;
            else {
                m_object_xVel[bId]= -(m_object_xVel[bId]); 
                m_object_yVel[bId]= -(m_object_yVel[bId]);                     
            }
        }

      break;
    case PN_CD_INVERT_VX_VY: //invert xVel,yVel
      m_object_xVel[bId]= -(m_object_xVel[bId]); 
      m_object_yVel[bId]= -(m_object_yVel[bId]); 
      break;
    case PN_CD_INVERT_VX_VY0: //invert xVel,yVel=0
      m_object_xVel[bId]= -(m_object_xVel[bId]); 
      m_object_yVel[bId]=0;
      if(m_object_yDest[bId] != dBypassValue){
        m_object_yDest[bId] = m_object_y[bId];
      }
      break;
    case PN_CD_INVERT_VY_VX0: //invert yVel,xVel=0
      m_object_yVel[bId]= -(m_object_yVel[bId]); 
      m_object_xVel[bId]=0;
      if(m_object_xDest[bId] != dBypassValue){
        m_object_xDest[bId] = m_object_x[bId];
      }
      break;
    case PN_CD_VX0_VY0: //set xVel=0,yVel=0
      m_object_xVel[bId] = 0;
      m_object_yVel[bId] = 0;
      if(m_object_xDest[bId] != dBypassValue){
        m_object_xDest[bId] = m_object_x[bId];
      }
      if(m_object_yDest[bId] != dBypassValue){
        m_object_yDest[bId] = m_object_y[bId];
      }
      break;
    case PN_CD_RANDOM: //randomize direction
      m_random++;
      bCdValue = (m_random % 10);
      bCdValue = listRandom[bCdValue] & 0x0F;
      bCdValue %= 8;
      bCdValue = dirOrdinal[bCdValue];
      object_set_direction(bId,bCdValue);
      break;  
  } 
  
}
//=====================================================================
void IOCUBE::object_update_to_map(){
  
  unsigned char i;
  unsigned char tmpY;
  //increase m_at_index for change object image
  m_at_index++;
  if(m_at_index > 7)m_at_index=0;
  //draw object to map
  for(i=0;i<dObjectLimit;i++){
    if(m_object_value[i] > 0){
      if(m_object_image[i] == dBypassImage){

      }else{
        //load animation index then select image
        tmpY=0;
        if(m_object_at[i] != dBypassAnimate)tmpY = *(st_ptAnimate +((m_object_at[i] & 0x7F) * 8) + m_at_index);
        object_measure_size(i,tmpY);
      }
    }
  }  
}
//=====================================================================

void IOCUBE::object_measure_size(unsigned char bId,unsigned char bOffset)
{
  int8_t i;
  unsigned char tmpW=0;
  unsigned char tmpH=0;
  unsigned char tmpByte=0;
  int addr=0;
  
  if(bId < dObjectLimit){
    for(i=7;i>=0;i--){
      addr = ((m_object_image[bId] + bOffset)*8) + i;
      tmpByte = m_sprite[addr]; //讀數據      
      if(tmpByte > tmpH)tmpH = tmpByte;
      if((tmpByte > 0x00)&&(tmpW==0))tmpW=i+1;
    }
    tmpByte = tmpH;
    tmpH=8;
    for(i=0;i<8;i++){
      if(tmpByte & (0x80>>i))break;
      tmpH--; 
    }
    m_object_width[bId] = tmpW;
    m_object_height[bId] = tmpH;
  }
}
//=====================================================================
void IOCUBE::object_set_velocity(byte bId,int8_t i8X,int8_t i8Y){
  if(bId < dObjectLimit){
    m_object_xVel[bId] = i8X;
    m_object_yVel[bId] = i8Y;
  }
}
//=====================================================================
void IOCUBE::object_set_direction(byte bId,byte bValue){
  byte tmpDir = dBypassValue;

  if(bId < dObjectLimit){
    //set xDest & yDest bypassvalue
    //object_set_destination(bId,dBypassValue,dBypassValue);
    m_object_xDest[bId] = dBypassValue;
    m_object_yDest[bId] = dBypassValue;
    //change xVel or yVel
    switch(bValue){
      case DIRECTION_0: //STOP
        object_set_velocity(bId,0,0);
        break;
      case DIRECTION_1: //turn 1 o'clock direction
        object_set_velocity(bId,1,1);
        break;  
      case DIRECTION_3: //turn 3 o'clock direction
        object_set_velocity(bId,1,0);
        break;          
      case DIRECTION_5: //turn 5 o'clock direction
        object_set_velocity(bId,1,-1);
        break;         
      case DIRECTION_6: //turn 6 o'clock direction
        object_set_velocity(bId,0,-1);
        break;    
      case DIRECTION_7: //turn 7 o'clock direction
        object_set_velocity(bId,-1,-1);
        break;    
      case DIRECTION_9: //turn 9 o'clock direction
        object_set_velocity(bId,-1,0);
        break;
      case DIRECTION_11: //turn 11 o'clock direction
        object_set_velocity(bId,-1,1);
        break;      
      case DIRECTION_12: //turn 12 o'clock direction
        object_set_velocity(bId,0,1);
        break;
    }

    m_object_dir[bId] = object_check_direction(bId);
  }
}
//=====================================================================

//return newer id
byte IOCUBE::object_new(){
  byte i;
  for(i=0;i<(dObjectLimit-1);i++){
    if(m_object_value[i] <=0){
      object_init(i);
      m_object_value[i] = 1;
      //Serial.print("Object new:");
      //Serial.println(i);
      return i; 
    }      
  }
  return dBypassValue;
}
//=====================================================================
//return removed id
byte IOCUBE::object_remove(byte bId){
  
  byte i;
  
  switch(bId)
  {
    case OI_ALL:
      for(i=0;i<dObjectLimit;i++)object_init(i);
      break;
    case OI_SELECTED:
      object_init(oiSelected);
      break;
    default:
      for(i=(dObjectLimit-1);i >=0;i--){
        if(m_object_value[i] >0){
          //m_object_value[i] = 0;
          object_init(i);
          return i; 
        }      
      }      
  }
  

  return bId;
}
//=====================================================================

void IOCUBE::appScriptEngine(byte eventCode,byte eventValue,byte inputID){

  //byte nextStage = 1;
  byte tmpEvent;
  byte tmpByte;
  byte i;

  m_event_code  = eventCode;
  m_event_value = eventValue;
  m_device_id   = inputID;
  
  for(i=stageLoopStart;i<128;i++)
  {
    tmpEvent=eventCode;
    tmpByte = getStoryboardByte(i,EVENT_CODE);
    if(tmpByte == EC_NULL)break;
    if(tmpByte == EC_END)break;
    if(tmpByte == EC_STAGE_END)break;
    
    if(eventCode <= EC_INPUT_ANY){
      if((tmpByte == EC_INPUT_ANY)||(tmpByte == eventCode)){
        tmpByte = EC_INPUT_ANY;
        tmpEvent = EC_INPUT_ANY;
      }
    }

    if(tmpEvent == tmpByte){
      switch(tmpEvent)
      {
        case EC_CD_LEFT:        
        case EC_CD_BOTTOM:
        case EC_CD_RIGHT:
        case EC_CD_TOP:
          oiSelected = inputID;
        case EC_INPUT_ANY:
          tmpByte = getStoryboardByte(i,DEVICE_ID);
          if((tmpByte == DI_ID_ALL)||(inputID == tmpByte)){
              tmpByte = getStoryboardByte(i,EVENT_VALUE);
              if((tmpByte == eventValue)||(tmpByte ==0xFF))doStoryboardAction(i);
            }
          break;
        case EC_TIME_100MS:
          tmpByte = getStoryboardByte(i,EVENT_VALUE);
          if(tmpByte==0)tmpByte=1;
          if((timeCount % tmpByte)==0)
          {
            doStoryboardCheckID(i);
          }
          break;
        case EC_TIME_1SECOND:
          tmpByte = getStoryboardByte(i,EVENT_VALUE);
          if(tmpByte==0)tmpByte=1;
          if(((timeCount/10) % tmpByte)==0)doStoryboardCheckID(i);
          break;          
        case EC_TIME_1MINUTE:
          tmpByte = getStoryboardByte(i,EVENT_VALUE);
          if(tmpByte==0)tmpByte=1;
          if(((timeCount/600) % tmpByte)==0)doStoryboardCheckID(i);
          break;
        case EC_TIME_1HOUR:
        case EC_ROUTINE:
          doStoryboardCheckID(i);
          break;
      }
    }else{
      if(eventCode == EC_ROUTINE)
      {
        if((tmpByte==EC_FLAG_TRUE)&&(m_flag==1))doStoryboardCheckID(i);
        if((tmpByte==EC_FLAG_FALSE)&&(m_flag==0))doStoryboardCheckID(i);
      }      
    }
  }
}

//=====================================================================
void IOCUBE::doStoryboardCheckID(byte index){
   byte tmpByte;
   tmpByte = getStoryboardByte(index,DEVICE_ID);
   if((myId == tmpByte)||(tmpByte==DI_ID_ALL))doStoryboardAction(index);
}
//=====================================================================
void IOCUBE::doStoryboardAction(byte index){

  byte i;
  byte tmpByte;
  byte tmpObjectID;
  byte tmpName;
  byte tmpParam1;
  byte tmpParam2;
  
  tmpByte = getStoryboardByte(index,ACTION_CODE); //
  tmpObjectID = getStoryboardByte(index,OBJECT_ID);
  tmpName = getStoryboardByte(index,PARAM_NAME);
  tmpParam1= getStoryboardByte(index,PARAM_1);
  tmpParam2= getStoryboardByte(index,PARAM_2);
  
  switch(tmpByte)
  {
    case AC_STAGE_GOTO:
      stageLoopStart = getStoryboardByte(index,PARAM_1);
      break;
    /*
    case AC_OI_VARIABLE_GET_ID:
        switch(tmpParam1){
            case OI_MYID:
                doObjectSet(tmpObjectID,tmpName,myId,0);
                break;
            case OI_EVENTID:
                doObjectSet(tmpObjectID,tmpName,inputID,0);
                break;
            case OI_SELECTED:
                doObjectSet(tmpObjectID,tmpName,oiSelected,0);
                break;
        }
        break;
        */
    case AC_OI_VARIABLE_SET:
        tmpObjectID = doObjectGet(OI_VARIABLE,tmpObjectID);
        doObjectSet(tmpObjectID,tmpName,tmpParam1,tmpParam2);
        break;
    case AC_OI_VARIABLE_GET_FROM:
        tmpObjectID = doObjectGet(OI_VARIABLE,tmpObjectID);
        tmpByte = doObjectGet(tmpParam1,tmpParam2);
        doObjectSet(tmpObjectID,tmpName,tmpByte,0);        
        break;
    case AC_OI_VARIABLE_SET_TO:
        tmpObjectID = doObjectGet(OI_VARIABLE,tmpObjectID);
        tmpByte = doObjectGet(tmpObjectID,tmpName);
        doObjectSet(tmpParam1,tmpParam2,tmpByte,0);        
        break;
    case AC_OIV1_EQUAL_OIV2:
        tmpObjectID = doObjectGet(OI_VARIABLE,tmpObjectID);
        tmpByte = doObjectGet(tmpObjectID,tmpName);
        tmpObjectID = doObjectGet(OI_VARIABLE,tmpParam1);
        tmpParam1 = doObjectGet(tmpObjectID,tmpParam2);
        if(tmpByte == tmpParam1)m_flag = 1; else m_flag = 0;
        break;
      
    case AC_OBJECT_SYNC:
      object_sync_send();
      break;
    
//    case AC_TIME_SYNC:
//      time_sync_send();
//      break;
    case AC_OI_GET_FROM:
      //object1.attr = object2.attr
      tmpByte = doObjectGet(tmpParam1,tmpParam2);
      doObjectSet(tmpObjectID,tmpName,tmpByte,0);
      break;
    case AC_OI_SET_TO:
      //object2.attr = object1.attr
      tmpByte = doObjectGet(tmpObjectID,tmpName);
      doObjectSet(tmpParam1,tmpParam2,tmpByte,0);
      break;
    case AC_OI_SET:
      //object1.attr = param1
      if(tmpObjectID == OI_ALL){
          for(i=0;i<dObjectLimit;i++)doObjectSet(i,tmpName,tmpParam1,tmpParam2);
      }else{
        doObjectSet(tmpObjectID,tmpName,tmpParam1,tmpParam2);
      }
      break;
    case AC_OI_INIT:
      for(i=0;i<dObjectLimit;i++)object_init(i);
      m_u=0;
      m_v=0;
      m_w=0;
      m_x=0;
      m_y=0;
      m_z=0;  
      m_flag=0;
      timeCount=0;
      break;
    case AC_OBJECT_NEW:
      oiSelected=object_new();
      if(tmpObjectID == OI_SELECTED)
      {
        doObjectSet(oiSelected,tmpName,tmpParam1,0);
      }
      break;
    case AC_OBJECT_REMOVE:
      object_remove(tmpObjectID);
      break;
    case AC_OI_PLUS:
      i = doObjectGet(tmpObjectID,tmpName);
      i+=tmpParam1;
      doObjectSet(tmpObjectID,tmpName,i,0);     
      break;
    case AC_OI_MINUS:
      i = doObjectGet(tmpObjectID,tmpName);
      i-=tmpParam1;
      doObjectSet(tmpObjectID,tmpName,i,0);         
      break;
    case AC_OI_TIMES:
      i = doObjectGet(tmpObjectID,tmpName);
      i*=tmpParam1;
      doObjectSet(tmpObjectID,tmpName,i,0);       
      break;
    case AC_OI_DIVIDE:
      i = doObjectGet(tmpObjectID,tmpName);
      if(tmpParam1>0)
      {
        i/=tmpParam1;
        doObjectSet(tmpObjectID,tmpName,i,0); 
      }
      break;
    case AC_OI_MOD:
      i = doObjectGet(tmpObjectID,tmpName);
      i%=tmpParam1;
      doObjectSet(tmpObjectID,tmpName,i,0); 
      break;
      
    case AC_OI_RANDOM_LOCAL:
    case AC_OI_RANDOM_FROM_EI:
    case AC_OI_RANDOM_FROM_EI_H:
    case AC_OI_RANDOM_FROM_EI_L:
        tmpByte = getStoryboardByte(index,EVENT_CODE);  
        if(tmpByte >= EC_TIME_100MS)m_random = (random(255) % tmpParam1) + tmpParam2; 
        randomPairs(tmpParam2,tmpParam1);
        //Max=param1,Min=param2   Min <= tmpByte <= Max
        //tmpByte = (input % (Max-Min+1)) + Min  
        tmpParam1 = (tmpParam1-tmpParam2+1);          
        tmpByte = getStoryboardByte(index,ACTION_CODE);
        switch(tmpByte){
            case AC_OI_RANDOM_LOCAL:
                tmpByte=(random(255) % tmpParam1) + tmpParam2; 
                if(tmpObjectID == OI_ALL){
                    for(i=0;i<dObjectLimit;i++){
                        doObjectSet(i,tmpName,tmpByte,0);
                        tmpByte=(random(255) % tmpParam1) + tmpParam2; 
                    }
                }else doObjectSet(tmpObjectID,tmpName,tmpByte,0);
                return;
            case AC_OI_RANDOM_FROM_EI:
                tmpByte=(m_random % tmpParam1) + tmpParam2; 
                break;
            case AC_OI_RANDOM_FROM_EI_H:
                tmpByte=((m_random >> 4) % tmpParam1) + tmpParam2;
                break;
            case AC_OI_RANDOM_FROM_EI_L:
                tmpByte=((m_random & 0x0F) % tmpParam1) + tmpParam2;
                break;
        }
        if(tmpObjectID == OI_ALL){
            tmpObjectID = getStoryboardByte(index,ACTION_CODE);
            for(i=0;i<dObjectLimit;i++){
                tmpByte = (m_random % 16);
                switch(tmpObjectID){
                    case AC_OI_RANDOM_FROM_EI:
                        tmpByte = listRandomPair[i];
                        break;
                    case AC_OI_RANDOM_FROM_EI_H:
                        tmpByte=((listRandom[tmpByte] >> 4) % tmpParam1) + tmpParam2;
                        break;
                    case AC_OI_RANDOM_FROM_EI_L:
                        tmpByte=((listRandom[tmpByte] & 0x0F) % tmpParam1) + tmpParam2;
                        break;
                }
                doObjectSet(i,tmpName,tmpByte,0);
                m_random++;
            }
            
        }else doObjectSet(tmpObjectID,tmpName,tmpByte,0);
      break;    
    case AC_OI_ORDINAL_LOCAL:
    case AC_OI_ORDINAL_FROM_EI:
    case AC_OI_ORDINAL_FROM_EI_H:
    case AC_OI_ORDINAL_FROM_EI_L:
            tmpByte = getStoryboardByte(index,EVENT_CODE);  
            //Max=param1,Min=param2   Min <= tmpByte <= Max
            //tmpByte = (input % (Max-Min+1)) + Min  
            tmpParam1 = (tmpParam1-tmpParam2+1); 
            if(tmpByte < EC_TIME_100MS){
                tmpByte = getStoryboardByte(index,ACTION_CODE);
                switch(tmpByte){
                case AC_OI_ORDINAL_LOCAL:
                    tmpByte=(timeCount % tmpParam1) + tmpParam2; 
                    break;
                case AC_OI_ORDINAL_FROM_EI:
                    tmpByte=(m_ordinal % tmpParam1) + tmpParam2; 
                    break;
                case AC_OI_ORDINAL_FROM_EI_H:
                    tmpByte=((m_ordinal >> 4) % tmpParam1) + tmpParam2;
                    break;
                case AC_OI_ORDINAL_FROM_EI_L:
                    tmpByte=((m_ordinal & 0x0F) % tmpParam1) + tmpParam2;
                    break;
                }
                if(tmpObjectID == OI_ALL){
                    for(i=0;i<dObjectLimit;i++){
                        doObjectSet(i,tmpName,tmpByte,0);
                        tmpByte++;
                        if(tmpByte > tmpParam1)tmpByte = tmpParam2;                          
                    }
                }else doObjectSet(tmpObjectID,tmpName,tmpByte,0);
            }else{
                switch(tmpByte){
                    case EC_TIME_1SECOND:
                        tmpByte = ((timeCount/10) % tmpParam1) + tmpParam2; 
                        break;
                    case EC_TIME_1MINUTE:
                        tmpByte = ((timeCount/600) % tmpParam1) + tmpParam2;
                        break;
                    case EC_TIME_1HOUR:
                        tmpByte = (((timeCount/3600)/10) % tmpParam1) + tmpParam2;
                        break;
                    default:
                        tmpByte = (timeCount % tmpParam1) + tmpParam2; 
                        break;
                }
                if(tmpObjectID == OI_ALL){
                        for(i=0;i<dObjectLimit;i++){
                            doObjectSet(i,tmpName,tmpByte,0);
                            tmpByte++;
                            if(tmpByte > tmpParam1)tmpByte = tmpParam2; 
                        }
                }else doObjectSet(tmpObjectID,tmpName,tmpByte,0);
            }
      break;
      
    case AC_OI_EQUAL:
    case AC_OI_MORE_THAN_EQUAL:
    case AC_OI_LESS_THAN_EQUAL:
    case AC_OI_MORE_THAN:
    case AC_OI_LESS_THAN:
        if(tmpObjectID==OI_ALL){
            for(i=0;i<dObjectLimit;i++){
                tmpParam2 = doObjectGet(i,tmpName);
                logic_check(i,tmpByte,tmpParam2,tmpParam1);
                if(m_flag==1)i=dObjectLimit;
            }
        }else{
            tmpParam2 = doObjectGet(tmpObjectID,tmpName);
            logic_check(tmpObjectID,tmpByte,tmpParam2,tmpParam1); 
        } 
        break;
    case AC_OI_POSITION_EQUAL:
        if(tmpObjectID < dObjectLimit){
            if((m_object_x[tmpObjectID]==tmpParam1)&&(m_object_y[tmpObjectID]==tmpParam2))m_flag=1; else m_flag=0;
        }else{
            m_flag=0;
            oiSelected=OI_PASS;
            for(i=0;i<dObjectLimit;i++){
                if((m_object_x[i]==tmpParam1)&&(m_object_y[i]==tmpParam2)){
                    oiSelected=i;
                    m_flag=1;
                    i=dObjectLimit;
                }
            }
        }
        break;
    case AC_OI1_EQUAL_OI2:
    case AC_OI1_MORE_THAN_OI2:
    case AC_OI1_LESS_THAN_OI2:
    case AC_OI1_MORE_THAN_EQUAL_OI2:
    case AC_OI1_LESS_THAN_EQUAL_OI2:
        if(tmpObjectID==OI_ALL){
            tmpParam2 = doObjectGet(tmpParam1,tmpParam2);
            for(i=0;i<dObjectLimit;i++){
                tmpParam1 = doObjectGet(i,tmpName);
                logic_check(i,tmpByte,tmpParam1,tmpParam2);
                if(m_flag==1)i=dObjectLimit;
            }                
        }else{
            tmpParam2 = doObjectGet(tmpParam1,tmpParam2);
            tmpParam1 = doObjectGet(tmpObjectID,tmpName);
            logic_check(tmpObjectID,tmpByte,tmpParam1,tmpParam2); 
        }
        break;

    case AC_OI1_POSITION_EQUAL_OI2:
        m_flag=0;
        tmpParam1 = getStoryboardByte(index,DEVICE_ID);
        if(tmpObjectID == OI_ALL){
            for(i=0;i<dObjectLimit;i++){
                if(object_pixel_collision(i,tmpParam1)){
                    oiSelected=i;
                    m_flag=1;
                    break;
                }
            }
        }else{
            if(object_pixel_collision(tmpObjectID,tmpParam1)){
                oiSelected=tmpObjectID;
                m_flag=1;
            }
        }            
        break;
    case AC_OI1_PLUS_EQUAL_OI2:
    case AC_OI1_MINUS_EQUAL_OI2:
    case AC_OI1_TIMES_EQUAL_OI2:
    case AC_OI1_DIVIDE_EQUAL_OI2:
    case AC_OI1_MOD_EQUAL_OI2:
        tmpParam2 = doObjectGet(tmpParam1,tmpParam2);
        tmpParam1 = doObjectGet(tmpObjectID,tmpName);            
        tmpParam1 = math_caculate(tmpByte,tmpParam1,tmpParam2); 
        doObjectSet(tmpObjectID,tmpName,tmpParam1,0);
        break;        
    case AC_RESULT_EQUAL_OI1_PLUS_OI2:
    case AC_RESULT_EQUAL_OI1_MINUS_OI2:
    case AC_RESULT_EQUAL_OI1_TIMES_OI2:
    case AC_RESULT_EQUAL_OI1_DIVIDE_OI2:
    case AC_RESULT_EQUAL_OI1_MOD_OI2:  
        tmpParam2 = doObjectGet(tmpParam1,tmpParam2);
        tmpParam1 = doObjectGet(tmpObjectID,tmpName);            
        tmpParam1 = math_caculate(tmpByte,tmpParam1,tmpParam2); 
        doObjectSet(OI_VARIABLE,PN_VARIABLE_RESULT,tmpParam1,0);
        break;   
    case AC_OI_SHOOT:
      if(tmpObjectID < dObjectLimit)
      {
        oiSelected=object_new(); 
        m_object_value[oiSelected] = tmpParam2;
        tmpByte = getStoryboardByte(index,EVENT_CODE);
        switch(tmpParam1)
        {
          case P1_DIR_RANDOM:
            if(tmpByte >= EC_TIME_100MS){
                m_random++;
                tmpByte = (m_random % 10);
                m_random = listRandom[tmpByte] & 0x0F;
            }
            i = (m_random % 8);
            i = dirOrdinal[i];
            object_set_direction(oiSelected,i);
            break;
          case P1_DIR_ORDINAL:
            if(tmpByte >= EC_TIME_100MS)m_ordinal++;
            i = (m_ordinal % 8);
            i = dirOrdinal[i];
            object_set_direction(oiSelected,i);           
            break;
          default:
            object_set_direction(oiSelected,tmpParam1);   
            break;
        }
        m_object_cd[oiSelected] = PN_CD_VALUE_B_MINUS_A_A0; 
        switch(m_object_dir[oiSelected])
        {
          case DIRECTION_1: //shoot 1 o'clock direction
            m_object_x[oiSelected] = m_object_x[tmpObjectID] + m_object_width[tmpObjectID]  -1;
            m_object_y[oiSelected] = m_object_y[tmpObjectID] + m_object_height[tmpObjectID] -1;

            break;  
          case DIRECTION_3: //shoot 3 o'clock direction
            m_object_x[oiSelected] = m_object_x[tmpObjectID] + m_object_width[tmpObjectID] -1;
            m_object_y[oiSelected] = m_object_y[tmpObjectID] + ((m_object_height[tmpObjectID]-1)/2);
            break;          
          case DIRECTION_5: //shoot 5 o'clock direction
            m_object_x[oiSelected] = m_object_x[tmpObjectID] + m_object_width[tmpObjectID] -1;
            m_object_y[oiSelected] = m_object_y[tmpObjectID];
            break;         
          case DIRECTION_6: //shoot 6 o'clock direction
            m_object_x[oiSelected] = m_object_x[tmpObjectID] + ((m_object_width[tmpObjectID]-1)/2);   
            m_object_y[oiSelected] = m_object_y[tmpObjectID]; 
            break;    
          case DIRECTION_7: //shoot 7 o'clock direction
            m_object_x[oiSelected] = m_object_x[tmpObjectID];   
            m_object_y[oiSelected] = m_object_y[tmpObjectID];
            break;  
          case DIRECTION_9: //shoot 9 o'clock direction
            m_object_x[oiSelected] = m_object_x[tmpObjectID];   
            m_object_y[oiSelected] = m_object_y[tmpObjectID] + ((m_object_height[tmpObjectID]-1)/2);
            break;
          case DIRECTION_11: //shoot 11 o'clock direction
            m_object_x[oiSelected] = m_object_x[tmpObjectID]; 
            m_object_y[oiSelected] = m_object_y[tmpObjectID] + m_object_height[tmpObjectID] -1;

            break;      
          case DIRECTION_12: //shoot 12 o'clock direction
            m_object_x[oiSelected] = m_object_x[tmpObjectID] + ((m_object_width[tmpObjectID]-1)/2);   
            m_object_y[oiSelected] = m_object_y[tmpObjectID] + m_object_height[tmpObjectID] -1;   
  
            break;
        }
      }
      break;
  }  
}
//=====================================================================
void IOCUBE::randomPairs(byte bMin,byte bMax){
  byte i;
  byte val=bMin;
  byte iRand=0;
  byte temp;
  byte pairCount = bMax - bMin;
  
  if(pairCount > 0){
    pairCount = (pairCount+1)*2;

    if(pairCount > 16)pairCount=16;
    
    for(i=0;i<16;i++){
      listRandomPair[i] = val;
      val++;
      if(val > bMax)val=bMin;      
    }
   
    for(i=0;i<pairCount;i++){
      iRand = random(pairCount);
      temp = listRandomPair[i];
      listRandomPair[i] = listRandomPair[iRand];
      listRandomPair[iRand] = temp;
    }
    for(i=pairCount;i<16;i++){
      iRand = random(pairCount,16);
      temp = listRandomPair[i];
      listRandomPair[i] = listRandomPair[iRand];
      listRandomPair[iRand] = temp;
    }
  }else{
    for(i=0;i<16;i++)listRandomPair[i] = val;
  }
}
//=====================================================================
void IOCUBE::logic_check(byte index,
                          byte logic,
                          int8_t value1,
                          int8_t value2){
    m_flag=0;
    //oiSelected=OI_PASS;
    //if(index < dObjectLimit)if(m_object_value[index]<1)return;
    switch(logic){
    case AC_OI_EQUAL:
        case AC_OI1_EQUAL_OI2:
            if(value1==value2)m_flag=1;
            break;
        case AC_OI_MORE_THAN_EQUAL:
        case AC_OI1_MORE_THAN_EQUAL_OI2:
            if(value1>=value2)m_flag=1;
            break;
        case AC_OI_LESS_THAN_EQUAL:
        case AC_OI1_LESS_THAN_EQUAL_OI2:
            if(value1<=value2)m_flag=1;
            break;
        case AC_OI_MORE_THAN:
        case AC_OI1_MORE_THAN_OI2:
            if(value1>value2)m_flag=1;
            break;
        case AC_OI_LESS_THAN:
        case AC_OI1_LESS_THAN_OI2:
            if(value1<value2)m_flag=1;
            break;
    }
    if((m_flag==1)&&(index < dObjectLimit))oiSelected=index;                         
}
//=====================================================================
int8_t IOCUBE::math_caculate(byte operatorX,
                             int8_t value1,
                             int8_t value2){

    switch(operatorX){
      case AC_OI1_PLUS_EQUAL_OI2:
      case AC_RESULT_EQUAL_OI1_PLUS_OI2:
          value1+=value2;
          break;
      case AC_OI1_MINUS_EQUAL_OI2:
      case AC_RESULT_EQUAL_OI1_MINUS_OI2:
          value1-=value2;
          break;
      case AC_OI1_TIMES_EQUAL_OI2:
      case AC_RESULT_EQUAL_OI1_TIMES_OI2:
          value1*=value2;
          break;
      case AC_OI1_DIVIDE_EQUAL_OI2:
      case AC_RESULT_EQUAL_OI1_DIVIDE_OI2:
          if(value2 > 0)value1/=value2;
          break;
      case AC_OI1_MOD_EQUAL_OI2:
      case AC_RESULT_EQUAL_OI1_MOD_OI2:
          value1%=value2;
          break;                   
    }
    return value1;                         
}
//=====================================================================
void IOCUBE::doObjectSet(byte bId,
                          byte param_name,
                          byte param_1,
                          byte param_2){

  if(bId == OI_SELECTED)bId = oiSelected;
                            
  if(bId < dObjectLimit)
  {
    switch(param_name)
    {
      case PN_X:
        m_object_x[bId] = (char)param_1;
        break;
      case PN_Y:
        m_object_y[bId] = (char)param_1;
        break;
      case PN_XDEST:
        m_object_xDest[bId] = (char)param_1;
        break;
      case PN_YDEST:
        m_object_yDest[bId] = (char)param_1;
        break;    
      case PN_XVEL:
        m_object_xVel[bId] = (char)param_1;
        break;
      case PN_YVEL:
        m_object_yVel[bId] = (char)param_1;
        break;    
      case PN_VALUE:
        m_object_value[bId] = param_1;
        break;
      case PN_DIR:
        switch(param_1){
            case DIRECTION_TURN_RIGHT:
                dirCount[bId]++;
                if(dirCount[bId]>3)dirCount[bId]=0;
                param_1 = dirClock[dirCount[bId]];
                break;
            case DIRECTION_TURN_LEFT:
                dirCount[bId]--;
                if(dirCount[bId]<0)dirCount[bId]=3;
                param_1 = dirClock[dirCount[bId]];
                break;
        }      
        object_set_direction(bId,param_1);
        break;
      case PN_WIDTH:
        m_object_width[bId] = param_1;
        break;
      case PN_HEIGHT:
        m_object_height[bId] = param_1;
        break;    
      case PN_CD:
        m_object_cd[bId] = param_1;
        break;      
      case PN_IMAGE:
        m_object_image[bId] = param_1;
        break;
      case PN_AT:
        m_object_at[bId] = param_1;
        break;  
      case PN_XOFFSET:
        m_object_x[bId] += (char)param_1;
        break;
      case PN_YOFFSET:
        m_object_y[bId] += (char)param_1;
        break;
      case PN_POSITION:
        //object_set_position(bId,param_1,param_2);
        m_object_x[bId] += (char)param_1;
        m_object_y[bId] += (char)param_2;
        break;
      case PN_XYOFFSET:
        m_object_x[bId] += (char)param_1;
        m_object_y[bId] += (char)param_2;
        break;
      case PN_XYDEST:
        m_object_xDest[bId] = (char)param_1;
        m_object_yDest[bId] = (char)param_2;
        break;
      case PN_XYOFFSETDEST:
        m_object_xDest[bId] = m_object_x[bId] + (char)param_1;
        m_object_yDest[bId] = m_object_y[bId] + (char)param_2;
        break;      
      case PN_SIZE:
        m_object_width[bId] = param_1;
        m_object_height[bId] = param_2;
        break;      
    }
  }

  if(bId == OI_VARIABLE){
    switch(param_name)
    {
      case PN_VARIABLE_RESULT:
        m_result=(char)param_1;
        break;
      case PN_VARIABLE_U:
        m_u=param_1;
        break;      
      case PN_VARIABLE_V:
        m_v=param_1;
        break;
      case PN_VARIABLE_W:
        m_w=param_1;
        break;
      case PN_VARIABLE_X:
        m_x=param_1;
        break;
      case PN_VARIABLE_Y:
        m_y=param_1;
        break;
      case PN_VARIABLE_Z:
        m_z=param_1;
        break;
    }
  }

  if(bId == OI_BOUNDARY){
    switch(param_name){
        case PN_BOUNDARY_LEFT:
            m_left = (char)param_1;
            break;
        case PN_BOUNDARY_BOTTOM:
            m_bottom = (char)param_1;
            break;
        case PN_BOUNDARY_RIGHT:
            m_right = (char)param_1;
            break;
        case PN_BOUNDARY_TOP:
            m_top = (char)param_1;
            break;
        case PN_BOUNDARY_CD:
            m_boundary_cd = param_1;
            break;
      }
    }

  
}
//=====================================================================
byte IOCUBE::doObjectGet(byte bId,
                          byte param_name){
  byte i = 0;                         

  if(bId == OI_SELECTED)bId = oiSelected;
  
  if(bId < dObjectLimit)
  {                       
    switch(param_name)
    {
      case PN_X:
        return m_object_x[bId];
      case PN_Y:
        return m_object_y[bId];
      case PN_XDEST:
        return m_object_xDest[bId];
      case PN_YDEST:
        return m_object_yDest[bId]; 
      case PN_XVEL:
        return m_object_xVel[bId];
      case PN_YVEL:
        return m_object_yVel[bId];  
      case PN_VALUE:
        return m_object_value[bId];
      case PN_DIR:
        //object_set_direction(bId,param_1);
        return (object_check_direction(bId));
      case PN_WIDTH:
        return m_object_width[bId];
      case PN_HEIGHT:
        return m_object_height[bId];  
      case PN_CD:
        return m_object_cd[bId];    
      case PN_IMAGE:
        return m_object_image[bId];
      case PN_AT:
        return m_object_at[bId];
      case PN_PASS: //if pass param_name return object.value
        return PN_PASS;
      //case PN_POSITION:
        //object_set_position(bId,param_1,param_2);
        //break;
      //case PN_XOFFSET:
        //m_object[bId].x += (char)param_1;
        //break;
      //case PN_YOFFSET:
        //m_object[bId].y += (char)param_1;
      default:
        return (0);
        //break;
    }
  }

  if(bId == OI_TRIGGER){
    switch(param_name){
      case PN_EVENT_CODE:
        return m_event_code;
      case PN_EVENT_VALUE:
        return m_event_value;
      case PN_DEVICE_ID:
        return m_device_id;
      case PN_MYID:
        return myId;
      case PN_SELECTED:
        return oiSelected;
      case PN_STATE_VALUE_1:
        return m_state[0];
      case PN_STATE_VALUE_2:
        return m_state[1];
      case PN_STATE_VALUE_3:
        return m_state[2];
      case PN_STATE_VALUE_4:
        return m_state[3];
      case PN_STATE_VALUE_5:
        return m_state[4];
      case PN_STATE_VALUE_6:
        return m_state[5];
    }
  }
  
  if(bId == OI_VARIABLE){
    switch(param_name)
    {
      case PN_VARIABLE_RESULT:
        return m_result;
      case PN_VARIABLE_U:
        return m_u;
      case PN_VARIABLE_V:
        return m_v;
      case PN_VARIABLE_W:
        return m_w;
      case PN_VARIABLE_X:
        return m_x;
      case PN_VARIABLE_Y:
        return m_y;
      case PN_VARIABLE_Z:
        return m_z;
    }
  }
  return(0);
}
//=====================================================================
byte IOCUBE::getStoryboardByte(byte index,byte offset)
{
  //byte tmpByte = *(st_ptStoryboard +(index * 8)+offset);
  int addr = (index * 8)+offset; 
  byte tmpByte = m_script[addr];
  return(tmpByte);
}
//=====================================================================
void IOCUBE::object_sync_send()
{
    byte i;
    byte bLength = (dObjectLimit * 4)+12; 
    byte timeH = (timeCount >> 8);
    byte timeL = (timeCount & 0xFF); 
    
    //Serial.println("Object Sync...");
    /*
    //Serial.print("m_left");
    //Serial.print(m_left);
    //Serial.print("m_bottom");
    //Serial.print(m_bottom);
    //Serial.print("m_right");
    //Serial.print(m_right);
    //Serial.print("m_top");
    //Serial.println(m_top);    
    */
  /*
    for(i=0;i<dObjectLimit;i++)
    {
      if(m_object_value[i] > 0)
      {
        Serial.print(i);
        Serial.print(" X:");
        Serial.print(m_object_x[i]);
        Serial.print(" Y:");
        Serial.print(m_object_y[i]);
        Serial.print(" VALUE:");
        Serial.print(m_object_value[i]);
        //Serial.print(" xDest:");
        //Serial.print(m_object_xDest[i]);
        //Serial.print(" yDest:");
        //Serial.print(m_object_yDest[i]);   
        //Serial.print(" xVel:");
        //Serial.print(m_object_xVel[i]);
        //Serial.print(" yVel:");
        //Serial.print(m_object_yVel[i]);        
        Serial.print(" DIR:");
        Serial.println(m_object_dir[i]);  
              
      }
    }
    */

    
    sendChar(0);                    //IIC Address(broadcast)
    sendChar(bLength);              //length
    sendChar(0xFF);                 //TargetId
    sendChar(myId);                 //sourceId
    sendChar(cmdObjectSync);        //command 
    for(i=0;i<dObjectLimit;i++)
    {
      sendChar(m_object_x[i]);
      sendChar(m_object_y[i]);
      sendChar(m_object_value[i]);
      sendChar(m_object_dir[i]);
      object_set_direction(i,m_object_dir[i]);
    }
    sendChar(m_result);
    sendChar(m_u);
    sendChar(m_v);
    sendChar(m_w);
    sendChar(m_x);
    sendChar(m_y);
    sendChar(m_z);    

    sendChar(timeH);
    sendChar(timeL);  
 
    sendSum();  
    //delay(500);
}

//=====================================================================
void IOCUBE::time_sync_send()
{  
    //byte bLength = (dObjectLimit * 3)+8;  
    /*
    byte timeH = (timeCount >> 8);
    byte timeL = (timeCount & 0xFF);
  
    sendChar(0);                  //IIC Address(broadcast)
    sendChar(5);                  //length
    sendChar(0xFF);               //srcId & TargetId
    sendChar(cmdTimeSync);        //command 
    sendChar(timeH);   
    sendChar(timeL);              //time value
    sendChar(0x00);               //forward count
    sendSum();  
    //Serial.println("Time Sync...");
    */
    /*
    //Serial.print("Time Sync...");
    //Serial.print(timeCount,HEX);
    //Serial.print(',');
    //Serial.print(timeH,HEX);
    //Serial.print(timeL,HEX);
    //Serial.println();
    */

    
}
//=====================================================================

void IOCUBE::sendChar(byte bChar){
  m_sum+=bChar;
  stSendBuffer[stSendIndex]=bChar;
  stSendIndex++;
  if(stSendIndex >= dSendSize)stSendIndex = dSendSize - 1;
}
//=====================================================================
void IOCUBE::sendSum(){
  stSendBuffer[stSendIndex]=m_sum;
  stSendIndex++;  
  m_sum=0;
}
//=====================================================================
