#include <Arduino.h>
#include <FLASH_FAT.h>
#include <BMP280.h>
#include <SerialBuffer.h>
//#include <MPU9250_IMU.h>

FLASH_FAT fileSystem;
FILE_ALLOCATION_TABLE_STRUCTURE fileStructure;
BMP280 pressureSensor;
//MPU9250_IMU imu;

int status = 0;

struct Raw_Flight_Data{
  ulong timeStamp;
  float pressureAlt;
  float acceleration;
};

struct Rocket_Attitude_Estimations{
  ulong timeStamp;
  float xOrientation;
  float yOrientation;
  float zOrientation;
};

struct Flight_Estimations{
  ulong timeStamp;
  float estimatedVelocity;
  float estimatedApogee;
};

struct GPS_data{
  ulong timeStamp;
  String latitude;
  String longitude;
  ulong altitude;
  ulong gpsTime;
};

struct Event_Data{
  ulong timeStamp;
  int dataMembers;
  float *data;
};


Raw_Flight_Data buffferedFlightData;



//these are important for converting floats and larger values into bytes. Do lose resolution, but so what?

#define ALTITUDE_SCALE_FACTOR 550  //~2^24/(30480)
#define TIME_SCALE_FACTOR 1000
#define ACCELERATION_SCALE_FACTOR 300 //~2^16/(20*9.81)
#define VELOCITY_SCALE_FACTOR 90  //~2^16/(343*2)
#define ORIENTATION_SCALE_FACTOR

#define RAW_FLIGHT_DATA_DATA_DESCRIPTOR 1
#define RAW_FLIGHT_DATA_SIZE 9
#define FLIGHT_ESTIMATION_DATA_TAG 2
#define GPS_DATA_TAG 3


#define SEND_FILE "SF"
#define SEND_FAT_TABLE "ST"
#define WRITE_CONFIGURATION "WC"
#define ERASE_FILE "FE"
#define SEND_CONFIGURATION "SC"
#define SEND_ERROR "ERR:"

#define ACKNOWLEDGE "ACK"
#define FAILURE "FAIL"

#define START_CHAR "$"
#define NUMBER_OF_ATTEMPTS 5
#define COMPUTER_COMMUNICATION_TIMEOUT 5000
#define COMMS_FAILURE -57

//example: SF:0 //send file 0

//funciton prototypes
//Serial and File communications
int reportOpenedFileContents();
int encodeRawFlightDataForDataStorage(byte *target, ulong timeStamp, float pressureAlt, float acceleration);
int encodeRawFlightDataForDataStorage(byte *target, Raw_Flight_Data *data);
int decodeRawFlightDataFromDataStorage(byte *toDecode, Raw_Flight_Data *target);
int decodeFlightEstimations(byte *toDecode, Flight_Estimations *target);
int encodeFlightEstimations(byte *target, ulong timeStamp, float estimatedVelocity, float estimatedApogee);
int encodeFlightEstimations(byte *target, Flight_Estimations *data);
int encodeGpsData(byte *target, ulong timeStamp, String latitude, String longitude);
int encodeGpsData(byte *target, GPS_data *data);
int computeChecksum(String message);
int handleComputerSend(String *message, bool requestAck = true);
void sendErrorToComputer(String message);
int handleFileSendRequest(String message);
String encodeRawFlightDataForComputer(Raw_Flight_Data *target);
int reportOpenedFileContents();
void updateFileStructure();
void handleFatRequest(String message);
int initFileSystem();

//flight time communications

int encodeRawFlightDataForDataStorage(byte *target, ulong timeStamp, float pressureAlt, float acceleration){
  //first, scale the data
  ulong scaledAlt = (ulong)(fabs(pressureAlt)*ALTITUDE_SCALE_FACTOR + .5);
  ulong scaledAccel = (ulong)(fabs(acceleration)*ACCELERATION_SCALE_FACTOR + .5);
  target[0] = RAW_FLIGHT_DATA_DATA_DESCRIPTOR;
  target[1] = (timeStamp >> 16) & 0xFF;
  target[2] = (timeStamp >> 8) & 0xFF;
  target[3] = (timeStamp >> 0) & 0xFF;
  target[4] = (scaledAlt >> 16) & 0xFF;
  target[5] = (scaledAlt >> 8) & 0xFF;
  target[6] = (scaledAlt >> 0) & 0xFF;
  target[7] = (scaledAccel >> 8) & 0xFF;
  target[8] = (scaledAccel >> 0) & 0xFF;
  return 0;
}

int encodeRawFlightDataForDataStorage(byte *target, Raw_Flight_Data *data){
  return encodeRawFlightDataForDataStorage(target, data->timeStamp, data->pressureAlt, data->acceleration);
}

int decodeRawFlightDataFromDataStorage(byte *toDecode, Raw_Flight_Data *target){
  target->timeStamp = (ulong)(toDecode[1] << 16) | (toDecode[2] << 8) | (toDecode[3]);
  target->pressureAlt = (float)(((toDecode[4] << 16) | (toDecode[5] << 8) | (toDecode[6]))/ALTITUDE_SCALE_FACTOR);
  target->acceleration = (float)(((toDecode[7] << 8) | (toDecode[8]))/ACCELERATION_SCALE_FACTOR);
  return 0;
}

int decodeFlightEstimations(byte *toDecode, Flight_Estimations *target){
  target->timeStamp = (ulong)(toDecode[1] << 16) | (toDecode[2] << 8) | (toDecode[3]);
  target->estimatedApogee = (float)(((toDecode[4] << 16) | (toDecode[5] << 8) | (toDecode[6]))/ALTITUDE_SCALE_FACTOR);
  target->estimatedVelocity = (float)(((toDecode[7] << 8) | (toDecode[8]))/VELOCITY_SCALE_FACTOR);
  return 0;
}

int encodeFlightEstimations(byte *target, ulong timeStamp, float estimatedVelocity, float estimatedApogee){
  ulong scaledVelocity = (ulong)(fabs(estimatedVelocity)*VELOCITY_SCALE_FACTOR + .5);
  ulong scaledApogee = (ulong)(fabs(estimatedApogee)*ALTITUDE_SCALE_FACTOR + .5);
  target[0] = FLIGHT_ESTIMATION_DATA_TAG;
  target[1] = (scaledApogee >> 16) & 0xFF;
  target[2] = (scaledApogee >> 8) & 0xFF;
  target[3] = (scaledApogee >> 0) & 0xFF;
  target[4] = (scaledVelocity >> 16) & 0xFF;
  target[4] = (scaledVelocity >> 8) & 0xFF;
  target[4] = (scaledVelocity >> 0) & 0xFF;
  return 0;
}

int encodeFlightEstimations(byte *target, Flight_Estimations *data){
  return encodeFlightEstimations(target, data->timeStamp, data->estimatedVelocity, data->estimatedApogee);
}

int encodeGpsData(byte *target, ulong timeStamp, String latitude, String longitude){
  //
  return 0;
}

int encodeGpsData(byte *target, GPS_data *data){
  return encodeGpsData(target, data->timeStamp, data->latitude, data->longitude);
}

SerialBuffer computerBuffer;

int computeChecksum(String message){
  int check = 0;
  for(uint i = 0; i < message.length(); i ++){
    check = check ^ message.charAt(i);
  }
  return check;
}

int handleComputerSend(String *message, bool requestAck){
  //encode in a standard start char, checkSum, endChar
  String temp;
  while(computerBuffer.available()) computerBuffer.readLine(&temp);
  String newMessage = START_CHAR + *message + "*" + String(computeChecksum(*message)) + "\n";
  //go and send the message and wait for an ack
  int attempt = 0;
  long timeStart = millis();
  while((attempt < NUMBER_OF_ATTEMPTS) & (millis()-timeStart < COMPUTER_COMMUNICATION_TIMEOUT)){
    Serial.print(newMessage);
    while(computerBuffer.available() <= 0 && (millis() - timeStart < COMPUTER_COMMUNICATION_TIMEOUT));
    if(computerBuffer.available() <= 0) break;
    computerBuffer.readLine(&temp);
    if(temp.indexOf(ACKNOWLEDGE) >= 0) return 0;
  }
  return COMMS_FAILURE;
}

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

String encodeRawFlightDataForComputer(Raw_Flight_Data *target){
  //go encode in a standard
  return String(RAW_FLIGHT_DATA_DATA_DESCRIPTOR) + ":" + String(target->timeStamp) + "," + String(target->pressureAlt) + "," + String(target->acceleration);
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
        Serial.println("RAW FLIGHT DATA");
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
  /*
  index,size;index,size;index,size
  */
  updateFileStructure();
  String totalMessage;
  for(int i = 0; i < fileStructure.numberOfFiles; i ++){
    totalMessage+=String(i) + ",";
    totalMessage += String(fileStructure.files[i].size) + ";";
  }
  //go and send it
  status = handleComputerSend(&totalMessage);
  if(status != 0){
    Serial.println("FAT Send to the user failed: " + String(status));
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
    Serial.println("Sending file!");
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

class CircularBufferIndexer{
public:
  CircularBufferIndexer(uint size);
  CircularBufferIndexer(){}
  void init(uint size);
  int add(); //returns the index to place the member in
  void removeLast();
  int readFromStart(uint *buffer);
  int readFromEnd(uint *buffer);
  void clear();
private:
  uint counter; //points at the next member to fill
  uint minSizeCounter; //this is used if the available buffer is less than size
  uint size = 0;
};

CircularBufferIndexer::CircularBufferIndexer(uint size){
  this->size = size;
  counter = 0;
  minSizeCounter = 0;
}

void CircularBufferIndexer::init(uint size){
  this->size = size;
  counter = 0;
  minSizeCounter = 0;
}

int CircularBufferIndexer::readFromStart(uint *buffer){
  //read from the start index
  if(minSizeCounter >= size){
    for(uint i = 0; i < size; i ++){
      uint actual = counter + i;
      if(actual >= size) actual -= size;
      buffer[i] = actual;
    }
    return size; //tell them we gots a full buffer
  }
  else{
    //its going to read from 0 to the minSize
    for(uint i = 0; i < minSizeCounter; i ++){
      buffer[i] = i;
    }
    return minSizeCounter;
  }
}

int CircularBufferIndexer::readFromEnd(uint *buffer){
  //read from the start index
  if(minSizeCounter >= size){
    for(uint i = 0; i < size; i ++){
      uint actual = counter - i;
      if(actual < 0) actual += size;
      buffer[i] = actual;
    }
    return size; //tell them we gots a full buffer
  }
  else{
    //its going to read from 0 to the minSize
    for(uint i = minSizeCounter - 1; i >= 0; i --){
      buffer[minSizeCounter-1 - i] = i;
    }
    return minSizeCounter;
  }
}

void CircularBufferIndexer::removeLast(){
  //check bounds
  if(counter - 1 < 0){
    counter = size-1;
  }
  else{
    counter --;
  }
}

int CircularBufferIndexer::add(){
  //do 2* just in case removes are called frequently
  if(minSizeCounter<2*size) minSizeCounter ++;
  //check bounds
  if(counter + 1 > size){
    counter = 0;
    return size;
  }
  else{
    counter ++;
    return counter - 1;
  }
  return -1;
}

void CircularBufferIndexer::clear(){
  counter = 0;
  minSizeCounter = 0;
}



/*
  recordFlight
  main function for flight routines
  records all available data and controls data storage
  six distinct flight phases: waitingForLaunch, Boost, Coast, Apogee, Recovery, Landed
  serial interupt pauses all flight systems as this is a sign we are not actually in flight, or that recovery is happening

  the waitForLaunch uses a circular buffer of ~5 seconds
*/

//these are flags that can be commented out as needed
//#define GPS
#define ACCEL
#define GYRO
#define PRESSURE_ALT
//#define RADIO_TELEMETRY
#define SERIAL_FEED_THROUGH

enum FLIGHT_PHASES{
  WAITING_FOR_LAUNCH,BOOST_PHASE,COAST_PHASE,APOGEE,RECOVERY,LANDING,STANDBY
};
//flight
#define FLIGHT_DATA_STORAGE_HERTZ_DURING_FLIGHT 20
#define GPS_DATA_STORAGE_HERTZ_DURING_FLIGHT 1
#define FLIGHT_ESTIMATION_STORAGE_HERTZ_DURING_FLIGHT 2

#define FLIGHT_DATA_TELEMETRY_HERTZ_DURING_FLIGHT 2
#define GPS_DATA_TELEMETRY_HERTZ_DURING_FLIGHT .5
#define FLIGHT_ESTIMATION_TELEMETRY_HERTZ_DURING_FLIGHT .5
//recovery
#define FLIGHT_DATA_STORAGE_HERTZ_DURING_RECOVERY 2
#define GPS_DATA_STORAGE_HERTZ_DURING_RECOVERY 1
#define FLIGHT_ESTIMATION_STORAGE_HERTZ_DURING_RECOVERY 0

#define FLIGHT_DATA_TELEMETRY_HERTZ_DURING_RECOVERY 2
#define GPS_DATA_TELEMETRY_HERTZ_DURING_RECOVERY .5
#define FLIGHT_ESTIMATION_TELEMETRY_HERTZ_DURING_RECOVERY 0


Raw_Flight_Data flightDataPreLaunchBuffer[50];
GPS_data gpsDataPreLaunchBuffer[5];

CircularBufferIndexer flightDataPreLaunchBufferIndexer;
CircularBufferIndexer gpsDataPreLaunchBufferIndexer;


bool serialInterrupt = false;



void recordFlight(){

}

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


void setup() {
  Serial.begin(115200);
  delay(2000);
  //testCircularBuffer();
  //while(true);
  Serial.println("Beginning!");
  String message;
  initFileSystem();
  makeDummyFlightFile();
  //fileSystem.makeFileAllocationTable();
  while(true){
    while(computerBuffer.available() <=0){
      delay(1);
    }
    computerBuffer.readLine(&message);
    Serial.println("Recieved Message: " + message);
    handleComputerCommunication(message);
  }
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
