#ifndef _FLIGHT_RECORDER_H_
#define _FLIGHT_RECORDER_H_

//general write request to the storage controller
#define FLIGHT_RECORDER_REQUEST_WRITE 60
//emergency, only used when buffers are about to be filled or a unique event ocurred
//ideally, this would be used just before a catastrophic event (i.e. a crash)
#define FLIGHT_RECORDER_REQUEST_EMERGENCY_WRITE 61
#define FLIGHT_RECORDER_MAX_RETURN_BUFFER_SIZE 256

#include <Arduino.h>
#include <DataConfig.h>

class FlightRecorder{
public:
  int init();
  //returns any requests on data writing
  int update(SensorData newData);
  int update(FlightEvent newEvent);
  //returns up to FLIGHT_RECORDER_MAX_RETURN_BUFFER_SIZE buffer to be written
  int getRequestedStorageBuffer(byte *buf, int n);

private:
  ulong lastRawDataWriteTime = 0;
  ulong lastGPSDataWriteTime = 0;
  ulong lastFlightApproximationDataWriteTime = 0;
  SensorData accelData, pressureAltData, gyroData, magData, attitudeData, gpsData;

  byte bufferRequestingToBeWritten[FLIGHT_RECORDER_MAX_RETURN_BUFFER_SIZE];
  int requestedWriteBufferIndex = 0;
  int encodeSensorDataToRawFlightData(Raw_Flight_Data *target);
  int encodeRawFlightDataToBuffer(Raw_Flight_Data *data, byte *buffer, int offset = 0);

};

//take the available sensor data and convert it to Raw_Flight_Data
int FlightRecorder::encodeSensorDataToRawFlightData(Raw_Flight_Data *target){
  target->timeStamp = pressureAltData.sysTimeStamp;
  target->pressureAlt = pressureAltData.floatData[0];
  target->linearAcceleration = pow(pow(accelData.floatData[0],2) + pow(accelData.floatData[1],2) + pow(accelData.floatData[2],2),.5);
  return 0;
}

int FlightRecorder::encodeRawFlightDataToBuffer(Raw_Flight_Data *data, byte *buffer, int offset){
  //first, scale the data
  ulong scaledAlt = (ulong)(fabs(data->pressureAlt)*ALTITUDE_SCALE_FACTOR + .5);
  ulong scaledAccel = (ulong)(fabs(data->linearAcceleration)*ACCELERATION_SCALE_FACTOR + .5);
  buffer[0 + offset] = RAW_FLIGHT_DATA_DATA_DESCRIPTOR;
  buffer[1 + offset] = (data->timeStamp >> 16) & 0xFF;
  buffer[2 + offset]= (data->timeStamp >> 8) & 0xFF;
  buffer[3 + offset] = (data->timeStamp >> 0) & 0xFF;
  buffer[4 + offset] = (scaledAlt >> 16) & 0xFF;
  buffer[5 + offset] = (scaledAlt >> 8) & 0xFF;
  buffer[6 + offset] = (scaledAlt >> 0) & 0xFF;
  buffer[7 + offset] = (scaledAccel >> 8) & 0xFF;
  buffer[8 + offset] = (scaledAccel >> 0) & 0xFF;
  return 0;
}

int FlightRecorder::readDataType()

int FlightRecorder::init(){
  return 0;
}

int FlightRecorder::update(SensorData newData){

}

#endif
