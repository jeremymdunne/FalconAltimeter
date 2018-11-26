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



void setup() {
  Serial.begin(115200);
  delay(2000);
  flightRecorder.init();
  //testCircularBuffer();
  //while(true);
  Serial.println("Beginning!");
  String message;
  //ComputerCommunication coms;
  //coms.init();
  String msg;
  //make up a couple of sensor packages
  RocketData dummyAccel, dummyGyro;
  dummyAccel.tag = FLIGHT_ACCEL_DATA_TAG;
  dummyAccel.timeStamp = 100;
  dummyAccel.data = new float[3]{10,20,40};
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
