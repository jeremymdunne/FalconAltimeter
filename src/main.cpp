#include <Arduino.h>
#include <FLASH_FAT.h>
#include <BMP280.h>
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






//these are important for converting floats and larger values into bytes. Do lose resolution, but so what?

#define ALTITUDE_SCALE_FACTOR 550  //~2^24/(30480)
#define TIME_SCALE_FACTOR 1000
#define ACCELERATION_SCALE_FACTOR 300 //~2^16/(20*9.81)
#define VELOCITY_SCALE_FACTOR 90  //~2^16/(343*2)
#define ORIENTATION_SCALE_FACTOR
#define RAW_FLIGHT_DATA_TAG 1
#define FLIGHT_ESTIMATION_DATA_TAG 2
#define GPS_DATA_TAG 3

int encodeRawFlightData(byte *target, ulong timeStamp, float pressureAlt, float acceleration){
  //first, scale the data
  ulong scaledAlt = (ulong)(fabs(pressureAlt)*ALTITUDE_SCALE_FACTOR + .5);
  ulong scaledAccel = (ulong)(fabs(acceleration)*ACCELERATION_SCALE_FACTOR + .5);
  target[0] = RAW_FLIGHT_DATA_TAG;
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

int encodeRawFlightData(byte *target, Raw_Flight_Data *data){
  return encodeRawFlightData(target, data->timeStamp, data->pressureAlt, data->acceleration);
}

int decodeRawFlightData(byte *toDecode, Raw_Flight_Data *target){
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

#define SERIAL_BUFFER_MAX_LENGTH 64



class SerialBuffer{
public:
  int available();
  int readLine(String *target);

private:

  int charBufferToString(char *buf, int n, String *message);
  char serialBuffer[SERIAL_BUFFER_MAX_LENGTH];
  int serialBufferIndex = 0;
  String newestMessage;
  char tempChar;
  bool lineAvailable = false;
};

int SerialBuffer::readLine(String *target){
  if((available() == 1) | lineAvailable){
    *target = newestMessage;
    newestMessage = "";
    lineAvailable = false;
    serialBufferIndex = 0;
  }
  return -1;
}

int SerialBuffer::charBufferToString(char *buf, int n, String *message){
  *message = "";
  for(int i = 0; i < n; i ++){
    *message += buf[i];
  }
  return 0;
}

int SerialBuffer::available(){
  //go and read the entire buffer, look for newline
  if(lineAvailable) return 1;
  while(Serial.available()){
    if(Serial.available()){
      tempChar=Serial.read();
      if(tempChar == '\n' || tempChar == '\r'){
        //end of line
        if(serialBufferIndex > 0){
          lineAvailable = true;
          charBufferToString(&serialBuffer[0], serialBufferIndex, &newestMessage);
          return 1;
        }
      }
      else{
        serialBuffer[serialBufferIndex] = tempChar;
        serialBufferIndex ++;
      }
    }
  }
  return 0;
}

#define SEND_FILE "SF"
#define SEND_FAT_TABLE "SF"
#define WRITE_CONFIGURATION "WC"
#define ERASE_FILE "FE"
#define SEND_CONFIGURATION "SC"

#define ACKNOWLEDGE "ACK"
#define FAILURE "FAIL"

#define START_CHAR "$"
#define NUMBER_OF_ATTEMPTS 5
#define COMPUTER_COMMUNICATION_TIMEOUT 5000
#define COMMS_FAILURE -57


SerialBuffer computerBuffer;

int computeChecksum(String message){
  int check = 0;
  for(uint i = 0; i < message.length(); i ++){
    check = check ^ message.charAt(i);
  }
  return check;
}

int handleComputerSend(String *message, bool requestAck = true){
  //encode in a standard start char, checkSum, endChar
  String temp;
  while(computerBuffer.available()) computerBuffer.readLine(&temp);
  String newMessage = START_CHAR + newMessage + "*" + String(computeChecksum(*message)) + "\n";
  //go and send the message and wait for an ack
  int attempt = 0;
  long timeStart = millis();
  while((attempt < NUMBER_OF_ATTEMPTS) & (millis()-timeStart < COMPUTER_COMMUNICATION_TIMEOUT)){
    Serial.print(newMessage);
    while(computerBuffer.available() <= 0);
    computerBuffer.readLine(&temp);
    if(temp.indexOf(ACKNOWLEDGE)) return 0;
  }
  return COMMS_FAILURE;
}

void handleFatRequest(String message){
  //send over the fat table
  //send each index and length.
  /*
  index,size;index,size;index,size
  */
  FILE_ALLOCATION_TABLE_STRUCTURE myFiles;
  fileSystem.readLookupTable(&myFiles);
  String totalMessage;
  for(int i = 0; i < myFiles.numberOfFiles; i ++){
    totalMessage+=String(i) + ",";
    totalMessage += String(myFiles.files[i].size) + ";";
  }
  //go and send it
  status = handleComputerSend(&totalMessage);
  if(status != 0){
    Serial.println("FAT Send to the user failed: " + String(status));
  }
}

//this is how we communicate with the computer
int handleComputerCommunication(String message){
  //main handler
  //check what the message is, handle appropriately
  if(message.indexOf(SEND_FILE)){
    //handleSendRequest(message);
  }
  else if(message.indexOf(SEND_FAT_TABLE)){
    //handleFatRequest(message);
  }
  else if(message.indexOf(ERASE_FILE)){
    //handleEraseRequest(message);
  }
  else{

  }

}

int sendFileToComputer(int fileIndex){
  //go and send data to the computer
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



void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Beginning!");
  String message;
  while(true){
    while(computerBuffer.available() <=0){
      delay(1);
    }
    computerBuffer.readLine(&message);
    Serial.println("Recieved Message: " + message);
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
