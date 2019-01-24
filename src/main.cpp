#include <Arduino.h>
/*
main control for the rocket.

Flight mode:
  initialize hardware
    report errors if necessary
  initialize software
    report errors if necessary
  begin collecting data and wait for launch detection
    store data in 'circular buffer' as to not kill memory and last and indefinate time
  store data upon launch detection
  begin flight loop
    collect new sensor data
    pass data to necessary modules to 'build' the rocket's state
    check for phase changes
      act if necessary
    execute any additional modules
    pass completed state to the flight recorder
      record to storage as requested
  when landing is determined, keep recording and storing data for n seconds
    during this time, ensure that the state does not change significantly
      if it does, the rocket may still be in flight
  safe all pyros, unused hardware

Data recall mode:
  if serial connection is recognized, enter this mode
  respond to user requests for data
    send all necessary data

Simulation mode:
  if the user sends appropriate data, enter this mode
  request the 'safe-state' of all hardware components
    i.e. should we fire pyros?
  request what data and at what rate should be sent over serial
  Run following the Flight mode routine, but only use faked sensor data given over serial
    report data back to user over serial as specified

*/

//Mode definitions
//#define STORE_DATA //actually store data to the flash data
#define ECHO_STORAGE_DATA //echo the storage data to the serial monitor
#define DEBUG_LEVEL_1 //full verbose logging
//#define DEBUG_LEVEL_2 //
#define SAFE_PYROS //don't burn/hurt yourself...
#define DEBUG_SENSOR_UPDATE



#include <header.h>
#include <FlashFat.h>
#include <SensorPackage.h>
#include <FlightRecorder.h>
#include <KinematicEngine.h>


using namespace RocketHelper;

SensorPackage sensorPackage;
FlightRecorder flightRecorder;
FlashFAT storage;
KinematicEngine kinematics;


/*
initSystems
initialize all hardware/software, make sure no critical errors are appearing
return an error code for what fails, as defined in errors.h

@return int, 0 = no errors, any other number as defined in errors.h
*/
int initSystems(){
  return 0;
}

/*
checkForSerial
check if the serial line has a valid message, return a code if it does
a valid message is defined by the global start end chars

@return int, >0 means message available
*/
int checkForSerial(){
  return 0;
}

/*
getSerialMessage
returns a fifo of the available messages from serial
*/
String getSerialMessage(){
  return "";
}

/*
printBuffer

prints a buffer out ot the serial, 16 members a line
*/
void printBuffer(uint8_t *buffer, uint size){
  Serial.println();
  for(uint i = 0; i < size; i ++){
    Serial.print(String(buffer[i]) + "\t");
    if(i%16 == 0 && i != 0) Serial.println();
  }
  Serial.println();
}

/*
handleFlightRecorderStorageRequest

writes all data from the flight recorder to the storage medium
*/
int handleFlightRecorderStorageRequest(){
  uint8_t tempStorageBuffer[256];
  int used = -1;
  //go until the flightRecorder has given us all data
  while(used < 0){
    used = flightRecorder.getRequestedDataToStore(&tempStorageBuffer[0], 256);
    //write to storage
    #ifdef STORE_DATA
      storage.write(&tempStorageBuffer[0], abs(used));
    #endif
    #ifdef ECHO_STORAGE_DATA
      printBuffer(&tempStorageBuffer[0], abs(used));
    #endif
  }

  //write to

}

/*
flightRoutine
tight control-loop of the program
handles the main flight following launch detection and preceeding landing
*/
int flightRoutine(){
  RawRocketSensorData rawSensorData;
  Rocket_State rocketState;
  UpdateScheduler printStateUpdater;
  printStateUpdater.setUpdateFrequency(5);
  long startMicros = 0;
  while(true){
    startMicros = micros();
    //Serial.println("loop");
    //get new sensor data
    int newSensorData = sensorPackage.update();

    if(newSensorData > 0){
        //Serial.println("New data!");
        sensorPackage.getNewSensorData(&rawSensorData);
        //copy
        copyBarometricData(&rocketState.primaryBarometricData, &rawSensorData.barometricData);
        copyIMUdata(&rocketState.primaryIMUdata, &rawSensorData.imuData);
        copyGPSdata(&rocketState.primaryGPSdata, &rawSensorData.gpsData);
        //update kinematics
        //kinematics.update(&rawSensorData);
        //update the storage controller
        int newFlightRecorderData = flightRecorder.update(&rocketState);
        if(newFlightRecorderData > 0){
          handleFlightRecorderStorageRequest();
        }
    }
    //update the loopMillis
    rocketState.updateMicros = (rocketState.updateMicros + micros()-startMicros)/2;
    if(printStateUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      printRocketState(&rocketState);
    }
  }
}

/*
preFlightWait
handles the circular buffer data collection will waiting for lanch parameters to be met

*/
int preFlightWait(){
  return 0;
};

/*
landingRoutine
handles all routines after landing
*/
int landingRoutine(){
  return 0;
}



void setup() {
  // put your setup code here, to run once:
  delay(3000);
  Serial.begin(115200);
  int errorCode = sensorPackage.init();
  if(errorCode != 0){
    Serial.println("Sensor Package Init error!");
    while(true);
  }
  errorCode = flightRecorder.init();
  if(errorCode != 0){
    Serial.println("Flight Recorder Init error!");
    while(true);
  }
  errorCode = storage.init();
  if(errorCode != 0){
    Serial.println("Storage Init error!");
    while(true);
  }
  Serial.println("Init Successful");
  flightRoutine();
}

void loop() {
  // put your main code here, to run repeatedly:
}
