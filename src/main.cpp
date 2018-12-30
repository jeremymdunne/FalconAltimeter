/*
Master TODO
Flight data storage
Storage Controller interaction
Host Computer interaction
File Transfer
Host Computer Program


WIBNI
  Host communicator char* only, no strings
  selective flight recorder data update rates
    No reason to have 100% hertz before flight and after apogee
  Ground altitude determination
  Better launch detection algorithm
    currently relies on an overall delta altitude or and acceleration and then a smaller delta altitude
*/



#include <Arduino.h>

#include <DataConfig.h>
#include <SensorPackage.h>
#include <FlightRecorder.h>
#include <MemoryFree.h>
#include <FlashFAT.h>
#include <HostCommunicator.h>
SensorPackage sensorPackage;
FlightRecorder flightRecorder;
FlashFAT storage;
HostCommunicator host;

#define PRE_FLIGHT_BUFFER_ROCKET_DATA_SIZE 128

String rocketDataToString(RocketData *data);
int recoverFile(uint fileIndex);
int sendDataFile(uint fileIndex);
int sendFileAllocationTable();
int handleComputerCommunication();
String rocketDataToString(RocketData *data);
bool checkForLaunch(RocketData *rawSensorData, uint length);
void copyRocketDataToTarget(RocketData *target, RocketData *source);
int updateSensors(RocketData *sensorData, uint max);
int handleRocketDataStorage(RocketData *data, uint n = 1);
int preFlightWait();
int handleHostCommunications();
bool checkForLanding();
int handleFlight();
void flightController();

int recoverFile(uint fileIndex){
  //double check the fileIndex
  int status = storage.open(FlashFAT::READ,fileIndex);
  if(status != 0){
    Serial.println("No such file!" + String(status));
    return -1;
  }
  ulong address;
  FlashFAT::FileAllocationTable table;
  storage.getFileAllocationTable(&table);
  address = table.files[fileIndex].startAddress;
  Serial.println("Recovering from: " + String(address));
  byte readBuff[1024];
  storage.recoveryRead(address, &readBuff[0], 1024);
  for(int i = 0; i < 1024; i ++){
    if(i%16 == 0) Serial.println();
    Serial.print(String(readBuff[i]) + "\t");
  }
  Serial.println("\n\n");
}

int sendDataFile(uint fileIndex){
  //double check the fileIndex
  int status = storage.open(FlashFAT::READ,fileIndex);
  if(status != 0){
    Serial.println("No such file!" + String(status));
    return -1;
  }
  byte readBuff[512];
  RocketData data;
  data.data = new float[5];
  //Serial.println("Peek Status: " + String(storageController.peek()));
  //send a begin file message
  String startMessage = String(SEND_FILE) + ":" + String(fileIndex);
  host.sendData(&startMessage, HostCommunicator::VERBOSE);
  while(storage.peek() > 0){
      status = storage.read(&readBuff[0],1);
      int length = flightRecorder.determineEncodingByteSize(readBuff[0]);
      if(length <= 0){
        Serial.println("Possibly corrupt file! " + String(readBuff[0]));
        Serial.println(storage.peek());

        return -1;
      }
      //Serial.println("Length: " + String(length));
      storage.read(&readBuff[1],length-1);
      status = flightRecorder.decodeBuffer(&readBuff[0], length, &data);
      //Serial.println("Temp status " + String(status));
      //Serial.println(rocketDataToString(&data));
      String dataLine = rocketDataToString(&data);
      host.sendData(&dataLine, HostCommunicator::VERBOSE);
      //delay(1);
  }
  startMessage = "END";
  host.sendData(&startMessage, HostCommunicator::VERBOSE);
}

int sendFileAllocationTable(){
  //go read the fat table
  FlashFAT::FileAllocationTable tempFATTable;
  storage.getFileAllocationTable(&tempFATTable);
  //create the message
  String hostMessage = "" + String(tempFATTable.numFiles) + ";";
  for(uint i = 0; i < tempFATTable.numFiles; i ++){
    if(i != 0) hostMessage += ";";
    hostMessage += String(i) + ":" + String(tempFATTable.files[i].size) + "," + String(tempFATTable.files[i].startAddress);
  }
  host.sendData(&hostMessage, HostCommunicator::VERBOSE);
  return 0;
}


#define REQUEST_FAT_TABLE 30
#define REQUEST_FILE_SEND 31
#define REQUEST_FILE_ERASE 32
#define REQUEST_LAST_FILE_ERASE 33
#define REQUEST_ERASE_ALL 34

struct HostRequest{
  int requestFlag;
  int data[3];
  uint numMembers = 0;
};

HostRequest recentRequest;

int handleComputerCommunication(){
  String hostMessage;
  while(host.available()>0){
    host.readLine(&hostMessage);
    //Serial.println("Recieved Message: " + hostMessage);
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
      recentRequest.requestFlag = REQUEST_FILE_SEND;
      recentRequest.data[0] = fileIndex;
      return REQUEST_FILE_SEND;
    }
    if(hostMessage.indexOf(ERASE_LAST_FILE) >= 0){
      Serial.println("Erase Last File request: ");
      recentRequest.requestFlag = REQUEST_LAST_FILE_ERASE;
      return REQUEST_LAST_FILE_ERASE;
    }
    if(hostMessage.indexOf(ERASE_ALL_FILES) >= 0){
      Serial.println("Erase All Files request: ");
      recentRequest.requestFlag = REQUEST_ERASE_ALL;
      return REQUEST_ERASE_ALL;
    }
  }
  return 0;
}

/*
Converts a peice of rocketData to a printable string
*/
String rocketDataToString(RocketData *data){
  String dataString = String(data->tag) + "," + String(data->timeStamp);
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




bool launchSuspected = false;
AccelData accelAtSuspectedLaunch;
PressureAltitudeData altitudeAtSuspectedLaunch;
PressureAltitudeData firstAltitudeRecorded;
bool firstAltitudeRecordedFilled = false;
bool checkForLaunch(RocketData *rawSensorData, uint length){
  if(!launchSuspected){
    float linearAccel;
    for(uint i = 0; i < length; i ++){
      switch(rawSensorData[i].tag){
        case(FLIGHT_ACCEL_DATA_TAG):
          linearAccel = pow(pow(rawSensorData[i].data[0],2) + pow(rawSensorData[i].data[1],2) + pow(rawSensorData[i].data[2],2),.5);
          if(linearAccel > LAUNCH_ACCELERATION_DETECTION_THRESHOLD){
            //go copy the data rq
            accelAtSuspectedLaunch.timeStamp = rawSensorData[i].timeStamp;
            accelAtSuspectedLaunch.data[0] = rawSensorData[i].data[0];
            accelAtSuspectedLaunch.data[1] = rawSensorData[i].data[1];
            accelAtSuspectedLaunch.data[2] = rawSensorData[i].data[2];
            Serial.println("Launch Acceleration reached, checking altitude change");
            launchSuspected = true;
            return false;
          }
          break;
        case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
          if(!firstAltitudeRecordedFilled){
            firstAltitudeRecorded.timeStamp = rawSensorData[i].timeStamp;
            firstAltitudeRecorded.data[0] = rawSensorData[i].data[0];
            firstAltitudeRecordedFilled = true;
          }
          altitudeAtSuspectedLaunch.timeStamp = rawSensorData[i].timeStamp;
          altitudeAtSuspectedLaunch.data[0] = rawSensorData[i].data[0];
          //do a safety check, just in case we did not detect an acceleration event
          if(rawSensorData[i].data[0] - firstAltitudeRecorded.data[0] > LAUNCH_DETECTION_ALTITUDE_WITHOUT_ACCELERATION_DETECTION) return true;
      }
    }
    return false;
  }
  else{
    for(uint i = 0; i < length; i ++){
      switch(rawSensorData[i].tag){
        case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
          if(rawSensorData[i].timeStamp - altitudeAtSuspectedLaunch.timeStamp > LAUNCH_DETECTION_WAIT_MILLIS){
            if(rawSensorData[i].data[0] - altitudeAtSuspectedLaunch.data[0] > LAUNCH_DETECTION_ALTITUDE_THRESHOLD_WITH_ACCELERATION_DETECTION){
              Serial.println("Launch detected!");
              return true;
            }
            else{
              launchSuspected = false;
              return false;
            }
          }
        break;
      }
    }
  }
  return false;
}

void copyRocketDataToTarget(RocketData *target, RocketData *source){
  //Serial.println("hello!");
  target->tag = source->tag;
  target->timeStamp = source->timeStamp;
  switch(target->tag){
    case(FLIGHT_ACCEL_DATA_TAG):
      target->data[0] = source->data[0];
      target->data[1] = source->data[1];
      target->data[2] = source->data[2];
      break;
    case(FLIGHT_GYRO_DATA_TAG):
      target->data[0] = source->data[0];
      target->data[1] = source->data[1];
      target->data[2] = source->data[2];
      break;
    case(FLIGHT_MAG_DATA_TAG):
      target->data[0] = source->data[0];
      target->data[1] = source->data[1];
      target->data[2] = source->data[2];
      break;
    case(FLIGHT_GPS_DATA_TAG):

      break;
    case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
      target->data[0] = source->data[0];
      break;
  }
  //Serial.println("Goodbye!");
}

/*
This is used as an intermediary between the sensorPackage on the core modules
this is done so a host-simulation can be easily added here, bypassing the sensors
in favor for a simulation

TODO simulation bypassing
*/

int updateSensors(RocketData *sensorData, uint max){
  return sensorPackage.getNewSensorData(sensorData, max);
}

/*
Handles storing the data (if applicable)
*/
int handleRocketDataStorage(RocketData *data, uint n = 1){
  int writeStatus = 0;
  int expectedSize;
  for(uint i = 0; i < n; i ++){
    expectedSize = flightRecorder.determineEncodingByteSize(&data[i]);
    //Serial.println(expectedSize);
    byte writeBuffer[expectedSize];
    //Serial.println(freeMemory());
    flightRecorder.encodeRocketData(&data[i], &writeBuffer[0]);
    //Serial.println(freeMemory());
    #ifdef STORE_DATA
      //actually store it
      writeStatus = storage.write(&writeBuffer[0], expectedSize);
      if(writeStatus != 0){
        //we gots an error!
        Serial.println("Writing data error!" + String(writeStatus));
      }
    #endif
    #ifdef PRINT_STORAGE_BUFFERS
      for(int p = 0; p < expectedSize; p ++){
        Serial.print(String(writeBuffer[p]) + "\t");
      }
      Serial.println();
    #endif
  }
  return writeStatus;
}


#define HOST_INTERRUPT_TAG 200

int preFlightWait(){
  Serial.println(freeMemory());
  PreFlightAllocation preFlightBuffer[PRE_FLIGHT_BUFFER_ROCKET_DATA_SIZE];
  uint8 bufferIndex = 0;
  bool filledOnce = false;
  Serial.println(freeMemory());
  bool launchDetected = false;
  while(!launchDetected && !host.available()){
    int numSensorData = sensorPackage.update();
    if(numSensorData > 0){
      RocketData newSensorData[numSensorData];
      int numNewSensorData = updateSensors(&newSensorData[0], numSensorData);
      //give this information to the flight recorder
      //Serial.println("Pre Flight Recorder Memory " + String(freeMemory()));
      launchDetected = checkForLaunch(&newSensorData[0], numNewSensorData);
      int flightRecorderUpdates = flightRecorder.update(&newSensorData[0], numNewSensorData);
      if(launchDetected){
        EventData launchEvent;
        launchEvent.timeStamp = millis();
        launchEvent.tag = FLIGHT_LAUNCH_DETECTION_EVENT_TAG;
        flightRecorderUpdates = flightRecorder.update(&launchEvent, 1);
        Serial.println("Adding launch detection to preflight buffer1; " + String(flightRecorderUpdates));
      }
      if(flightRecorderUpdates > 0){
        //Serial.println(freeMemory());
        RocketData newFlightRecorderData[flightRecorderUpdates];
        //Serial.println(sizeof(newFlightRecorderData[0].data[0]));
        //Serial.println(freeMemory());
        int numFilled = flightRecorder.getRequestedDataToStore(&newFlightRecorderData[0], flightRecorderUpdates);
        //Serial.println("Mid Flight Recorder Memory " + String(freeMemory()));
        //Serial.println("Filled: " + String(numFilled) + " Updates: " + String(flightRecorderUpdates));
        //preFlightBuffer[bufferIndex] = buffer[i];
        for(int i = 0; i < numFilled; i ++){
          copyRocketDataToTarget(&preFlightBuffer[bufferIndex], &newFlightRecorderData[i]);
          /*
          preFlightBuffer[bufferIndex].timeStamp = newFlightRecorderData[i].timeStamp;
          preFlightBuffer[bufferIndex].tag = newFlightRecorderData[i].tag;
          switch(preFlightBuffer[bufferIndex].tag){
            case(FLIGHT_ACCEL_DATA_TAG):
              preFlightBuffer[bufferIndex].data[0] = newFlightRecorderData[i].data[0];
              preFlightBuffer[bufferIndex].data[1] = newFlightRecorderData[i].data[1];
              preFlightBuffer[bufferIndex].data[2] = newFlightRecorderData[i].data[2];
              break;
            case(FLIGHT_GYRO_DATA_TAG):
              preFlightBuffer[bufferIndex].data[0] = newFlightRecorderData[i].data[0];
              preFlightBuffer[bufferIndex].data[1] = newFlightRecorderData[i].data[1];
              preFlightBuffer[bufferIndex].data[2] = newFlightRecorderData[i].data[2];
              break;
            case(FLIGHT_MAG_DATA_TAG):
              preFlightBuffer[bufferIndex].data[0] = newFlightRecorderData[i].data[0];
              preFlightBuffer[bufferIndex].data[1] = newFlightRecorderData[i].data[1];
              preFlightBuffer[bufferIndex].data[2] = newFlightRecorderData[i].data[2];
              break;
            case(FLIGHT_GPS_DATA_TAG):

              break;
            case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
              preFlightBuffer[bufferIndex].data[0] = newFlightRecorderData[i].data[0];
              break;
          }
          */
          bufferIndex ++;
          //Serial.print(bufferIndex);
          if(bufferIndex >= PRE_FLIGHT_BUFFER_ROCKET_DATA_SIZE) {
            if(!filledOnce) filledOnce = true;
            bufferIndex = 0;
          }
        }
        //Serial.println();
      }
      //Serial.println("Post Sensor Memory " + String(freeMemory()));
    }
  }
  if(host.available()){
    return HOST_INTERRUPT_TAG;
  }
  //Serial.println("Filled Once:" + String(filledOnce));
  Serial.println("Done!");
  if(filledOnce){
    int bufferCounter = 0;
    for(int i = 0; i < PRE_FLIGHT_BUFFER_ROCKET_DATA_SIZE; i ++){
      bufferCounter = i + bufferIndex;
      if(bufferCounter >= PRE_FLIGHT_BUFFER_ROCKET_DATA_SIZE) bufferCounter -= PRE_FLIGHT_BUFFER_ROCKET_DATA_SIZE;
      //Serial.println(rocketDataToString(&preFlightBuffer[bufferCounter]));
      handleRocketDataStorage(&preFlightBuffer[bufferCounter]);
    }
  }
  else{
    for(int i = 0; i < bufferIndex; i ++){
      handleRocketDataStorage(&preFlightBuffer[i]);
    }
  }
  Serial.println("Post Pre Flight Recorder Memory " + String(freeMemory()));
}
/*
handles sending and deleting files and such
Will later be used for initiating simulations and such
TODO on erase, request confirmation
*/
int handleHostCommunications(){
  bool loop = true;
  while(loop){
    int flag = handleComputerCommunication();
    switch (flag) {
      case REQUEST_FAT_TABLE:
        sendFileAllocationTable();
        break;
      case REQUEST_FILE_SEND:
        sendDataFile(recentRequest.data[0]);
        break;
      case REQUEST_ERASE_ALL:
        storage.eraseAllFiles();
        sendFileAllocationTable();
        break;
      case REQUEST_LAST_FILE_ERASE:
        storage.eraseLastFile();
        sendFileAllocationTable();
        break;
    }
  }
  return 0;
}


/*
this will later be the job of a phase checker
right now this is a simple 15 second timer
*/
long flightStart = 0;
bool checkForLanding(){
  if(flightStart == 0){
    flightStart = millis();
    return false;
  }
  if(millis() - flightStart < 15000){
    return false;
  }
  return true;
}

/*
  At this point, pre-flight has already ocurred
  Handles all components of the flights using the provided modules
*/
int handleFlight(){
  while(!checkForLanding()){
    int numSensorUpdates = sensorPackage.update();
    if(numSensorUpdates > 0){
      RocketData newSensorData[numSensorUpdates];
      numSensorUpdates = sensorPackage.getNewSensorData(&newSensorData[0],numSensorUpdates);
      //go check the flight recorder
      int flightRecorderUpdates = flightRecorder.update(&newSensorData[0], numSensorUpdates);
      if(flightRecorderUpdates > 0){
        //go and grab them and print them out
        RocketData newRecorderData[flightRecorderUpdates];
        flightRecorderUpdates = flightRecorder.getRequestedDataToStore(&newRecorderData[0], flightRecorderUpdates);
        for(int i = 0; i < flightRecorderUpdates; i ++){
          //Serial.println(rocketDataToString(&newRecorderData[i]));
          handleRocketDataStorage(&newRecorderData[i]);
        }
      }
    }
  }
  return 0;
}

void flightController(){
  #ifdef STORE_DATA
    storage.open(FlashFAT::WRITE);
  #endif
  int status = preFlightWait();
  //check if the host just wants to talk to us...
  if(status == HOST_INTERRUPT_TAG){
    //close the file if necessary
    #ifdef STORE_DATA
      storage.close();
      storage.eraseLastFile();
    #endif
    Serial.println("Host wants to talk! Exited preflight");
    Serial.println(freeMemory());
    //go and converse with the host
    status = handleHostCommunications();
    //TODO handle this return flag
  }
  else{
    //going into flight mode
    status = handleFlight();
    Serial.println("Flight Concluded");
  }
  #ifdef STORE_DATA
    storage.close();
  #endif
}


void setup() {
  delay(2000);
  Serial.begin(115200);
  Serial.println(freeMemory());
  //Serial.println(h);
  sensorPackage.init();
  flightRecorder.init();
  storage.init();
  host.init();
  //AccelData data;
  //rocketData[0] = data;
  //rocketData[1] = data;
  //rocketData[2] = data;
  //Serial.println(p);
  //hello(&tempData1);
  //Serial.println(sizeof(tempData));
  Serial.println(freeMemory());
  Serial.println(sizeof(sensorPackage));
  Serial.println(sizeof(flightRecorder));
  Serial.println(sizeof(storage));
  Serial.println(sizeof(host));
  flightController();
  Serial.println("Post all memory " + String(freeMemory()));
  Serial.println(sizeof(sensorPackage));
  Serial.println(sizeof(flightRecorder));
  Serial.println(sizeof(storage));
  Serial.println(sizeof(host));

}

void loop() {
  // put your main code here, to run repeatedly:
}
