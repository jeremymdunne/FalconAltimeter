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
//#define SERIAL_COMMUNICATION_DEBUG


#include <header.h>
#include <FlashFat.h>
#include <SensorPackage.h>
#include <FlightRecorder.h>
#include <KinematicEngine.h>
#include <SerialLineBuffer.h>
#include <CommunicationTags.h>


using namespace RocketHelper;

SensorPackage sensorPackage;
FlightRecorder flightRecorder;
FlashFAT storage;
KinematicEngine kinematics;
SerialLineBuffer hostCommunication;

//used for calling simulation and overriding flight sensor data
bool flightSimulationMode = false;
//initial state of the rocket, interrupted by serial communication
bool inFlightMode = true; //
//secondary state of the rocket when in communication with the host computer
bool inDataRetrevialMode = false;


//store the rocket phase at each of the major flight phases
//these are used to sanity check phase changes, i.e., coast should not start 100ms after boost
Rocket_State acceptedGroundLevelState; 
Rocket_State stateAtLaunch;
Rocket_State stateAtCoastStart;
Rocket_State stateAtApogee;
Rocket_State stateAtRecovery;
Rocket_State stateAtLanding;

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
  int size = hostCommunication.available();
  if(size > 0) return 1;
  return 0;
}

/*
getSerialMessage
returns a fifo of the available messages from serial
*/
String getSerialMessage(){
  int size = hostCommunication.available();
  if(size <= 0) return "";
  char buffer[size];
  //Serial.println("S: " + String(size));
  size = hostCommunication.getLine(&buffer[0], size);
  //Serial.println("S: " + String(size));
  String message = String(buffer);
  return message;
}



/*
validateMessage
Validates the string and 'cuts' it down to size (i.e. removes checksums)

@return int see error codes in CommunicationTags.h
*/

int validateMessage(String *message){
  //first check and remove the start + end char
  int size = message->length();
  if(size == 0) return SERIAL_COMMUNICATION_ERROR_MESSAGE_TOO_SMALL; //no message should be this small
  int start = message->indexOf(SERIAL_COMMUNICATION_START_CHAR);
  if(start < 0){
    return SERIAL_COMMUNICATION_ERROR_MISSING_DELIMINATOR_CHARS;
  }
  int end = message->indexOf(SERIAL_COMMUNICATION_END_CHAR);
  if(end < 0){
    return SERIAL_COMMUNICATION_ERROR_MISSING_DELIMINATOR_CHARS;
  }

  //now check for checksums and handle
  int checkSumStart = message->indexOf('*');
  if(checkSumStart < 0){
    //modify the message
    *message = message->substring(start + 1,end);
    //dosen't contain a checksum, report back a sucess
    return SERIAL_COMMUNICATION_VALIDATION_NO_CHECKSUM;
  }
  int checksum = computeChecksum(message, start+1, checkSumStart); //what we think it should be
  String checksumSubstring = message->substring(checkSumStart+1, end);
  int parsedCheckSum = stringToHex(&checksumSubstring);

  #ifdef SERIAL_COMMUNICATION_DEBUG
    Serial.println("Isolated Message: " + message->substring(start+1, end));
    Serial.println("Checksum substring: " + checksumSubstring);
    Serial.println("Parsed: " + String(parsedCheckSum));
    Serial.println("Believed: " + String(checksum));
  #endif

  if(parsedCheckSum != checksum){
    //report an error
    return SERIAL_COMMUNICATION_ERROR_CHECKSUM_FAILURE;
  }
  //modify the string to only be the 'middle' part
  *message = message->substring(start + 1, checkSumStart);
  return SERIAL_COMMUNICATION_VALIDATION_WITH_CHECKSUM;
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
}

/*
checkForPhaseChange

checks the flight data to see if we have switched phases
phase changes are determined by the current phase
*/

Rocket_State comparisonState;
bool checkPhaseChange = false;
int checkForPhaseChange(Rocket_State *state){
  switch(state->currentFlightPhase){
    case(WAITING_FOR_LAUNCH_PHASE):
      //check if a previous flag has been raised
      if(!checkPhaseChange){
        //check acceleration
        if(getLinearAcceleration(&state->primaryIMUdata) >= LAUNCH_DETECTION_MINUMUM_ACCELERATION){
          //mark the data as a possible launch
          checkPhaseChange = true;
          copyRocketState(&comparisonState, state);
          Serial.println("Possible State Change!");
        }
      }
      else{
        if(state->sysMillisTimeStamp - comparisonState.sysMillisTimeStamp >= LAUNCH_DETECTION_ACCELERATION_ELAPSED_TIME){
          if(getLinearAcceleration(&state->primaryIMUdata) >= LAUNCH_DETECTION_MINUMUM_ACCELERATION){
            //phase change! Yay!
            Serial.println("Elapsed: " + String(state->sysMillisTimeStamp - comparisonState.sysMillisTimeStamp));
            checkPhaseChange = false;
            return 1;
          }
          else{
            //no phase change...
            checkPhaseChange = false;
          }
        }
      }
      break;
  }
  return 0;
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
    rocketState.sysMillisTimeStamp = millis();
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
        //handle any phase changes
        int phaseCode = checkForPhaseChange(&rocketState);
        if(phaseCode > 0){
          Serial.println("Phase change! ");
          delay(2000);
        }
        //update the storage controller only if we are in the correct phase
        if(rocketState.currentFlightPhase != WAITING_FOR_LAUNCH_PHASE){
          int newFlightRecorderData = flightRecorder.update(&rocketState);
          if(newFlightRecorderData > 0){
            handleFlightRecorderStorageRequest();
          }
        }
    }
    //update the loopMillis
    rocketState.updateMicros = (rocketState.updateMicros + micros()-startMicros)/2;
    if(printStateUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      Serial.println();
      printRocketState(&rocketState);
      Serial.println();
    }
  }
}

/*
preFlightWait

Currently, this runs all the systems in order to wait for launch conditions and set an accepted
initial state. There is no need to store flight data before the launch, so it will not.

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

int checkForSerialInterrupt(){
  if(checkForSerial()){
    String message = getSerialMessage();
    Serial.println("Possible message: " + message);
    if(!message.equals("")){
      if(validateMessage(&message)>0){
        Serial.println("Recieved Message: " + String(message));
      }
      else{
        Serial.println("Message Garbage");
      }
    }
  }
}


void setup() {
  // put your setup code here, to run once:
  delay(3000);
  Serial.begin(115200);
  hostCommunication.init(&Serial);
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
/*
  uint8_t tempBuffer[256];
  float fakeAccel[3] = {-1,200.92,12.3};
  uint size = flightRecorder.encodeData(RAW_ACCEL_DATA_TAG, 1002, &fakeAccel[0], &tempBuffer[0]);
  Serial.println();
  for(uint i = 0; i < size; i ++){
    Serial.print(String(tempBuffer[i]) + "\t");
  }
  Serial.println();
  float target[10];
  size = flightRecorder.translateNext(&tempBuffer[0], 256, &target[0], 10);
  Serial.println("Size: " + String(size));
  for(uint i = 0; i < size; i ++){
    Serial.print(String(target[i]) + "\t");
  }
*/
  flightRoutine();
}

void loop() {
  // put your main code here, to run repeatedly:
  //checkForSerialInterrupt();
}
