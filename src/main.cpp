#include <Arduino.h>
#include <DataConfig.h>
#include <FlashFAT.h>
#include <SerialBuffer.h>
#include <CircularBufferIndexer.h>

#include <HostCommunicator.h>
#include <SensorPackage.h>
#include <FlightRecorder.h>

//#include <MPU9250_IMU.h>

//MPU9250_IMU imu;

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

FlashFAT storageController;
HostCommunicator hostCommunicator;
SensorPackage sensorPackage;
FlightRecorder flightRecorder;
int status = 0;

/*Todo list:
//Sensor Package Testing
//Storage Controller Testing
Host communicator finishing
Intermediate information routing
File Send testing

*/

/*
  handles all sensors on the rocket
  handles proper timing of data collection
  requires high update speeds
*/


int sendDataFile(uint fileIndex);
int recoverFile(uint fileIndex);


class FlightKinematics{

};



class FlightController{

};

//these are important for converting floats and larger values into bytes. Do lose resolution, but so what?

bool serialInterrupt = false;


bool checkForLaunch(){
  return false;
}

FLIGHT_PHASES currentPhase;

void handleStandDown(){
  switch(currentPhase){
    case(WAITING_FOR_LAUNCH):
      break;
    case(BOOST_PHASE):
      break;
    case(COAST_PHASE):
      break;
    case(APOGEE):
      break;
    case(RECOVERY):
      break;
    case(LANDING):
      break;
    case(STANDBY):
      break;
  }
}



void checkPhaseChange(){
  switch(currentPhase){
    case(WAITING_FOR_LAUNCH):
      break;
    case(BOOST_PHASE):
      break;
    case(COAST_PHASE):
      break;
    case(APOGEE):
      break;
    case(RECOVERY):
      break;
    case(LANDING):
      break;
    case(STANDBY):
      break;
  }
}
String hostMessage;
int sendFileAllocationTable(){
  //go read the fat table
  FlashFAT::FileAllocationTable tempFATTable;
  storageController.getFileAllocationTable(&tempFATTable);
  //create the message
  hostMessage = "" + String(tempFATTable.numFiles) + ";";
  for(uint i = 0; i < tempFATTable.numFiles; i ++){
    if(i != 0) hostMessage += ";";
    hostMessage += String(i) + ":" + String(tempFATTable.files[i].size) + "," + String(tempFATTable.files[i].startAddress);
  }
  hostCommunicator.sendData(&hostMessage, HostCommunicator::VERBOSE);
  return 0;
}


#define REQUEST_FAT_TABLE 30
#define REQUEST_FILE_SEND 31

struct HostRequest{
  int requestFlag;
  int *data;
  uint numMembers = 0;
};
HostRequest recentRequest;
int handleComputerCommunication(){
  while(hostCommunicator.available()>0){
    hostCommunicator.readLine(&hostMessage);
    Serial.println("Recieved Message: " + hostMessage);
    if(hostMessage.indexOf(SEND_FAT_TABLE) >= 0){
      recentRequest.requestFlag = REQUEST_FAT_TABLE;
      recentRequest.numMembers = 0;
      return REQUEST_FAT_TABLE;
    }
    if(hostMessage.indexOf(SEND_FILE) >= 0){
      Serial.println("Sending over file: ");
      int index = hostMessage.indexOf(':');
      int fileIndex = atoi(hostMessage.substring(index+1).c_str());
      Serial.println("Index: " + String(fileIndex));
      return sendDataFile(fileIndex);
    }
  }
  return 0;
}

String dataString;
String rocketDataToString(RocketData *data){
  dataString = String(data->tag) + "," + String(data->timeStamp);
  int numDataMembers = 0;
  switch(data->tag){
    case(FLIGHT_ACCEL_DATA_TAG):
      numDataMembers = 3;
      break;
    case(FLIGHT_GYRO_DATA_TAG):
      numDataMembers = 3;
      break;
    case(FLIGHT_MAG_DATA_TAG):
      numDataMembers = 3;
      break;
    case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
      numDataMembers = 1;
      break;
  }
  for(int i = 0; i < numDataMembers; i ++){
    dataString += "," + String(data->data[i]);
  }
  return dataString;
}

/*
  sendDataFile
  handles sending a file by communicating between the storageController and hostCommunicator
  send the data one member at a time
*/

int recoverFile(uint fileIndex){
  //double check the fileIndex
  status = storageController.open(FlashFAT::READ,fileIndex);
  if(status != 0){
    Serial.println("No such file!" + String(status));
    return -1;
  }
  ulong address;
  FlashFAT::FileAllocationTable table;
  storageController.getFileAllocationTable(&table);
  address = table.files[fileIndex].startAddress;
  Serial.println("Recovering from: " + String(address));
  byte readBuff[1024];
  storageController.recoveryRead(address, &readBuff[0], 1024);
  for(int i = 0; i < 1024; i ++){
    if(i%16 == 0) Serial.println();
    Serial.print(String(readBuff[i]) + "\t");
  }
  Serial.println("\n\n");
}

int sendDataFile(uint fileIndex){
  //double check the fileIndex
  status = storageController.open(FlashFAT::READ,fileIndex);
  if(status != 0){
    Serial.println("No such file!" + String(status));
    return -1;
  }
  byte readBuff[512];
  RocketData data;
  data.data = new float[5];
  //Serial.println("Peek Status: " + String(storageController.peek()));
  while(storageController.peek() > 0){
      status = storageController.read(&readBuff[0],1);
      int length = flightRecorder.determineEncodingByteSize(readBuff[0]);
      //Serial.println("Length: " + String(length));
      storageController.read(&readBuff[1],length-1);
      status = flightRecorder.decodeBuffer(&readBuff[0], length, &data);
      //Serial.println("Temp status " + String(status));
      Serial.println(rocketDataToString(&data));
      delay(1);

  }


}

/*
handles the component updates
also handles the components speaking between another
In order of component:
1. update sensorPackage
  Grab all new data available
2. update flight algorithms
  includes the parachute deployment and airbrake algorithms
  send all new flight data relevent and check for requests
3. check for telemetry and overrides
  respond accordingly
4. update flightRecorder
  Send all new flight data and flight events, check if write request is recieved
    if write request is recieved, execute at end of the loop. This is done as to ensure flight
    dependent algorithms, such as parachute deployment, airbrakes, etc. aren't lagged out
5. update storageController
  send any flight recorder data over to here
*/

RocketData masterTempStorage[10];
int masterTempStorageMembers = 0;

/*
handles updating the sensor package and grabbing new data if available
*/
int updateSensorPackage(){
  status = sensorPackage.update();
  if(status > 0){
    masterTempStorageMembers += sensorPackage.getNewSensorData(&masterTempStorage[masterTempStorageMembers], 10 - masterTempStorageMembers);
  }
  return masterTempStorageMembers;
}

/*
handles a storage request from the flightRecorder to the storageController
assume at this point it is safe to take complete control of writing the entirety of the flightRecorder
*/
byte flightRecorderStorageBuffer[256];
int flightRecorderStorageIndex = 0;
int handleStorageRequest(){
  bool done = false;
  while(!done){
    flightRecorderStorageIndex = flightRecorder.getRequestedBufferToStore(&flightRecorderStorageBuffer[0],256);
    if(flightRecorderStorageIndex > 0){
      #ifndef STORE_DATA
        //Serial.println("New data to be stored:");
        for(int i = 0; i < flightRecorderStorageIndex; i ++){
          if(i%16 == 0) Serial.println();
          Serial.print(String(flightRecorderStorageBuffer[i]) + "\t");
        }
      #endif
      #ifdef STORE_DATA
        status = storageController.write(&flightRecorderStorageBuffer[0], flightRecorderStorageIndex);
        //Serial.println("Storage Status: " +String(status))
      #endif
    }
    else{
      done = true;
    }
  }
  return 0;
}

int updateSystems(){
 int numDataAvail = updateSensorPackage();
 if(numDataAvail > 0){
   //Serial.println("New sensor data avail!");
   for(int i = 0; i < numDataAvail; i ++)
   status = flightRecorder.update(masterTempStorage[i]);
 }
 if(status > 0){
   //Serial.println("Storage request made!");
   //handle recording request
   handleStorageRequest();
 }
 status = handleComputerCommunication();
 switch(status){
  case(REQUEST_FAT_TABLE):
    sendFileAllocationTable();
    break;
  case(REQUEST_FILE_SEND):
    break;
 }
 masterTempStorageMembers = 0;
 return 0;
}

/*
The computer has requsted data files, so we are standing down from any launch events
*/
void handleDataRecoverMode(){
  while(true){
    status = handleComputerCommunication();
    switch(status){
     case(REQUEST_FAT_TABLE):
       sendFileAllocationTable();
       break;
     case(REQUEST_FILE_SEND):
       break;
    }
  }
}

struct PreFlightDataStorage{
  byte *buf;
  uint length;
};

PreFlightDataStorage preFlightCircularBuffer[128];
bool preFlightCircularBufferFilledOnce = false;
uint preFlightCircularBufferIndex = 0;

void copyBuffer(byte *source, byte *target, uint length){
  for(uint i = 0; i < length; i ++){
    target[i] = source[i];
  }
}

/*
Controlls what to do for pre-launch purposes
Also checks if the computer requests a data recovery mode and enters it appropriately
*/
void preFlightWait(){
  Serial.println("Storage status: " + String(storageController.open(FlashFAT::WRITE)));
  long startM = millis();
  while(millis() - startM < 15000){
    //update sensors
    int numDataAvail = updateSensorPackage();
    //update flightRecorder
    if(numDataAvail > 0){
      for(int i = 0; i < numDataAvail; i ++){
        status = flightRecorder.update(masterTempStorage[i]);
        //Serial.println("Time Stamp on Sensor: " + String(masterTempStorage[i].timeStamp));
      }
    }
    //if flight recorder is requesting datas, place in the preflight buffer
    if(status > 0){
      bool done = false;
      while(!done){
        flightRecorderStorageIndex = flightRecorder.getRequestedBufferToStore(&flightRecorderStorageBuffer[0],256);
        if(flightRecorderStorageIndex > 0){
          //Serial.println("Requested store, storing to tempBuffer!");
          preFlightCircularBuffer[preFlightCircularBufferIndex].buf = new byte[flightRecorderStorageIndex];
          copyBuffer(&flightRecorderStorageBuffer[0],&preFlightCircularBuffer[preFlightCircularBufferIndex].buf[0], flightRecorderStorageIndex);
          preFlightCircularBuffer[preFlightCircularBufferIndex].length = flightRecorderStorageIndex;
          //Serial.println("Buffer to be recorded in index: " + String(preFlightCircularBufferIndex));
          for(int i = 0; i < flightRecorderStorageIndex; i ++){
            if(i%16 == 0) Serial.println();
            Serial.print(String(preFlightCircularBuffer[preFlightCircularBufferIndex].buf[i]) + "\t");
          }
          //Serial.println("Free Memory: " + String(freeMemory()));
          preFlightCircularBufferIndex ++;
          if(preFlightCircularBufferIndex >= 128){
            preFlightCircularBufferIndex = 0;
            if(!preFlightCircularBufferFilledOnce) preFlightCircularBufferFilledOnce = true;
          }
        }
        else{
          done = true;
        }
      }
    }

    masterTempStorageMembers = 0;
    status = handleComputerCommunication();
  }
  //go and dump the data
  Serial.println("Dumping data!");

  if(preFlightCircularBufferFilledOnce){
    for(int i = 0; i < 128; i ++){
      if(i + preFlightCircularBufferIndex >= 128){
        storageController.write(&preFlightCircularBuffer[i + preFlightCircularBufferIndex - 128].buf[0], preFlightCircularBuffer[i + preFlightCircularBufferIndex - 128].length);
      }
      else{
        storageController.write(&preFlightCircularBuffer[i + preFlightCircularBufferIndex].buf[0], preFlightCircularBuffer[i + preFlightCircularBufferIndex].length);
      }
    }
  }
  else{
    for(uint i = 0; i < preFlightCircularBufferIndex; i ++){
      Serial.println("Writing deprecated:");
      Serial.println("Status: " + String(storageController.write(&preFlightCircularBuffer[i].buf[0], preFlightCircularBuffer[i].length)));
      for(uint p = 0; p < preFlightCircularBuffer[i].length; p ++){
        if(p%16 == 0) Serial.println();
        Serial.print(String(preFlightCircularBuffer[i].buf[p]) + "\t");
      }
      Serial.println();
    }
  }
  Serial.println("Wrote!");
  storageController.close();
}



void flight(){
  int openStatus = storageController.open(FlashFAT::WRITE);
  Serial.println("Open Status: " + String(openStatus));
  long startMillis = millis();
  Serial.println("Running for 5 seconds to create dummy file");
  while(millis() - startMillis < 5000){
    updateSystems();
  }
  storageController.close();
  Serial.println("Done!");
  while(true);
}

void wipeStorage(){
  Serial.println("Wiping storage, kill power to stop!");
  delay(10000);
  Serial.println("Wiping, please wait");
  storageController.eraseAllFiles();
  Serial.println("Done!");
}

void wipeLastFile(){
  Serial.println("Wiping last file!");
  delay(5000);
  storageController.eraseLastFile();
  delay(2000);
  Serial.println("Done!");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Free Memory: " + String(freeMemory()));
  flightRecorder.init();
  storageController.init();
  status = sensorPackage.init();
  //storageController.eraseAllFiles();
  //wipeStorage();
  //wipeLastFile();
  Serial.println("Sensor Package init: " + String(status));
  //testCircularBuffer();
  //while(true);
  Serial.println("Beginning!");
  Serial.println("Free Memory: " + String(freeMemory()));
  preFlightWait();
  String message;
  //ComputerCommunication coms;
  //coms.init();
  String msg;
  //make up a couple of sensor packages
  RocketData sensorData[5];
  //sensorData[0].data = new float[3]{};
  //flight();
  while(true){
    updateSystems();
  }
  if(status != 0){
    while(true);
  }
  Serial.println("Enting loop");
  while(true){
    if(sensorPackage.update() > 0){
      Serial.println("New sensor update!");
      status = sensorPackage.getNewSensorData(&sensorData[0], 5);
      for(int i = 0; i < status; i ++){
        Serial.println("New Data:\nTag:" + String(sensorData[i].tag) + "\nTimeStamp:" + String(sensorData[i].timeStamp) + "\nData0:" + String(sensorData[0].data[0]));
      }
    }
    //delay(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
