#include <Arduino.h>
#include <DataConfig.h>
#include <FLASH_FAT.h>
#include <BMP280.h>
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


//example: SF:0 //send file 0
//flight time communications

/*
void sendErrorToComputer(String message){
  //encode it
  message = SEND_ERROR+message;
  handleComputerSend(&message);
}
int getFileIndexFromMessage(String message, int *index){
  int start = message.indexOf(":") + 1;
  if(start < 0) {
    sendErrorToComputer("NO FILE INDEX");
    return -1;
  }
  int end = message.length();
  //if(message.indexOf('\n') >= 0) end = message.indexOf('\n');
  Serial.println("Parsed index: " + message.substring(start,end));
  *index = atoi(message.substring(start, end).c_str());
  return 0;
}

int handleFileSendRequest(String message){
  //go parse the message for a file index
  int index;
  if(getFileIndexFromMessage(message, &index) != 0) return -1;
  if(index < 0 || index > fileStructure.numberOfFiles-1){
    //error!
    sendErrorToComputer("INVALID FILE INDEX");
    return -1;
  }
  Serial.println("Opening file index: " + String(index));
  //otherwise, lets start a stream
  fileSystem.openForRead(index);
  //go and send
  status = reportOpenedFileContents();
}


int reportOpenedFileContents(){
  //go grab space for the maximum message size ~32 bytes
  byte primary[32];
  //go and grab the first byte in the file, this is the data descriptor
  //need some look ahead
  //byte dataDescriptor = 0;
  String computerMessage;
  while(fileSystem.readStream(&primary[0], 1) > 0){
    switch((int)primary[0]){
      case (RAW_FLIGHT_DATA_DATA_DESCRIPTOR):
        //Serial.println("RAW FLIGHT DATA");
        //grab the remaining length
        status = fileSystem.readStream(&primary[1],RAW_FLIGHT_DATA_SIZE-1);

        for(int i = 0; i < RAW_FLIGHT_DATA_SIZE; i ++){
          Serial.print(String(primary[i]) + "\t");
        }

        status = decodeRawFlightDataFromDataStorage(&primary[0], &buffferedFlightData);
        computerMessage = encodeRawFlightDataForComputer(&buffferedFlightData);
        status = handleComputerSend(&computerMessage);
        if(status == COMMS_FAILURE){
          sendErrorToComputer("FILE_SEND_SERIAL_FAILURE");
          return -1;
        }
        break;
      default:
        //todo
        Serial.println("Unrecognized!");
        break;
      }
  }
  return 0;
}

void updateFileStructure(){
  fileSystem.readLookupTable(&fileStructure);
}

void handleFatRequest(String message){
  //send over the fat table
  //send each index and length.

  index,size;index,size;index,size

  updateFileStructure();
  String totalMessage;
  for(int i = 0; i < fileStructure.numberOfFiles; i ++){
    totalMessage+=String(i) + ",";
    totalMessage += String(fileStructure.files[i].size) + ";";
  }
  //go and send it
  status = handleComputerSend(&totalMessage);
  if(status == COMMS_FAILURE){
    sendErrorToComputer("FILE_FAT_SERIAL_FAILURE");
  }
}

int handleEraseRequest(String message){
  //get the file index
  //int index;
  //if(getFileIndexFromMessage(message, &index) != 0) return -1;
  //
  fileSystem.eraseLastFile();
  fileSystem.readLookupTable(&fileStructure);
  return 0;
}

//this is how we communicate with the computer
int handleComputerCommunication(String message){
  //main handler
  //check what the message is, handle appropriately
  if(message.indexOf(SEND_FILE)>=0){
    //Serial.println("Sending file!");
    handleFileSendRequest(message);
  }
  else if(message.indexOf(SEND_FAT_TABLE)>=0){
    handleFatRequest(message);

  }
  else if(message.indexOf(ERASE_FILE)>=0){
    handleEraseRequest(message);
  }
  else{

  }
  return -1;
}



int initFileSystem(){
  //first check if we can communicate with the flash
  status = fileSystem.init(PA4,true,false);
  if(status != 0){
    Serial.println("Init Filesystem error! " + String(status));
    while(true);
  }
  else{
    Serial.println("Init Filesystem success! ");
  }
  //then make sure we have a filesystem
  status = fileSystem.readLookupTable(&fileStructure);
  //go check the number of files and make sure we're not full
  if((fileStructure.numberOfFiles > 250) | ((fileStructure.files[fileStructure.numberOfFiles-1].startAddress + fileStructure.files[fileStructure.numberOfFiles-1].size) >= 8000000L)){
    Serial.println("Running out of space!");
  }
  //otherwise, we're good!
  return 0;
}


  recordFlight
  main function for flight routines
  records all available data and controls data storage
  six distinct flight phases: waitingForLaunch, Boost, Coast, Apogee, Recovery, Landed
  serial interupt pauses all flight systems as this is a sign we are not actually in flight, or that recovery is happening

  the waitForLaunch uses a circular buffer of ~5 seconds
*/

//Raw_Flight_Data flightDataPreLaunchBuffer[PRE_LAUNCH_RAW_FLIGHT_BUFFER_SIZE];
//GPS_data gpsDataPreLaunchBuffer[PRE_LAUNCH_GPS_BUFFER_SIZE];

//CircularBufferIndexer flightDataPreLaunchBufferIndexer;
//CircularBufferIndexer gpsDataPreLaunchBufferIndexer;


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
void recordFlight(){
  //main idea is to check and handle user commands
  if(computerBuffer.available() > 0){
    if(!serialInterrupt){
      serialInterrupt = true;
      handleStandDown();
    }
    String message;
    computerBuffer.readLine(&message);
    handleComputerCommunication(message);
  }
  if(!serialInterrupt){
    //standard data collection
    checkPhaseChange();
  }

}
*/
/*
void makeDummyFile(){
  fileSystem.openToWrite();
  byte buffer[256];
  for(int i = 0; i < 256; i ++){
    buffer[i] = i;
  }
  fileSystem.write(&buffer[0], 256);
  fileSystem.close();
}

void makeDummyFlightFile(){
  Raw_Flight_Data data;
  fileSystem.openToWrite();
  byte tempHold[RAW_FLIGHT_DATA_SIZE];
  for(int i = 0; i < 256; i ++){
    data.timeStamp = i*10;
    data.acceleration = i/100.0;
    data.pressureAlt = i*2;
    encodeRawFlightDataForDataStorage(&tempHold[0], &data);
    fileSystem.write(&tempHold[0],RAW_FLIGHT_DATA_SIZE);
  }
  fileSystem.close();
}

*/
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
  RocketData dummyAccel, dummyGyro, dummyMag;
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
  /*
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
