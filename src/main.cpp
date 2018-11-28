#include <Arduino.h>
#include <DataConfig.h>
#include <FLASH_FAT.h>
#include <SerialBuffer.h>
#include <CircularBufferIndexer.h>

#include <StorageController.h>
#include <HostCommunicator.h>
#include <SensorPackage.h>
#include <FlightRecorder.h>

//#include <MPU9250_IMU.h>

//MPU9250_IMU imu;



StorageController storageController;
HostCommunicator hostCommunicator;
SensorPackage sensorPackage;
FlightRecorder flightRecorder;
int status = 0;

/*Todo list:
//Sensor Package Testing
Storage Controller Testing
Host communicator finishing
Intermediate information routing
File Send testing

*/

/*
  handles all sensors on the rocket
  handles proper timing of data collection
  requires high update speeds
*/






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
/*
  sendDataFile
  handles sending a file by communicating between the storageController and hostCommunicator
  send the data one member at a time
*/
int sendDataFile(uint fileIndex){
  //double check the fileIndex
  status = storageController.openToRead(fileIndex);
  if(status < 0) return -1;
  byte buff[32];
  status = storageController.read(&buff[0],1);
  while(status > 0){
    switch(status){
      case(1):
      break;
    }
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
        storageController.write(&flightRecorderStorageBuffer[0], flightRecorderStorageIndex);
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
 masterTempStorageMembers = 0;
 return 0;
}


void setup() {
  Serial.begin(115200);
  delay(2000);
  flightRecorder.init();
  storageController.init();
  status = sensorPackage.init();
  Serial.println("Sensor Package init: " + String(status));
  //testCircularBuffer();
  //while(true);
  Serial.println("Beginning!");
  String message;
  //ComputerCommunication coms;
  //coms.init();
  String msg;
  //make up a couple of sensor packages
  RocketData sensorData[5];
  //sensorData[0].data = new float[3]{};
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

  RocketData dummyAccel, dummyGyro;
  dummyAccel.tag = FLIGHT_ACCEL_DATA_TAG;
  dummyAccel.timeStamp = 100;
  dummyAccel.data = new (float[3]){10,20,40};
  dummyGyro.tag = FLIGHT_GYRO_DATA_TAG;
  dummyGyro.timeStamp = 120;
  dummyGyro.data = new float[3]{100,200,400};
  ulong startTime = micros();
  flightRecorder.update(dummyAccel);
  ulong endTime = micros();
  Serial.println("Elapsed: " + String(endTime - startTime));
  flightRecorder.update(dummyGyro);
  delay(1000);
  int status = flightRecorder.update(dummyGyro);
  Serial.println("FlightRecorder status: " + String(status));
  byte tempBuffer[256];
  startTime = micros();
  int size = flightRecorder.getRequestedBufferToStore(&tempBuffer[0], 256);
  endTime = micros();
  Serial.println("Elapsed: " + String(endTime - startTime));
  for(int i = 0; i < size; i ++){
    if(i%16 == 0) Serial.println();
    Serial.print(String(tempBuffer[i]) + "\t");
  }
  Serial.println("Done");
  while(true);
  /*
  while(true){
    if(coms.available()){
      coms.readLine(&msg);
      Serial.println("RECEIVED: " + msg);
    }
  }
  //initFileSystem();
  //makeDummyFlightFile();
  //fileSystem.makeFileAllocationTable();

  while(true){
    while(computerBuffer.available() <=0){
      delay(1);
    }
    computerBuffer.readLine(&message);
    Serial.println("Recieved Message: " + message);
    handleComputerCommunication(message);
  }
  */
  /*
  status = initFileSystem();
  if(status < 0){
    //major error detected
    while(true);
  }
  if(status > 0){
    //minor error detected
    while(true);
  }
*/
}

void loop() {
  // put your main code here, to run repeatedly:
}
