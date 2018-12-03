#ifndef _FLIGHT_RECORDER_H_
#define _FLIGHT_RECORDER_H_

//general write request to the storage controller
#define FLIGHT_RECORDER_REQUEST_WRITE 60
//emergency, only used when buffers are about to be filled or a unique event ocurred
//ideally, this would be used just before a catastrophic event (i.e. a crash)
#define FLIGHT_RECORDER_REQUEST_EMERGENCY_WRITE 61
#define FLIGHT_RECORDER_MAX_RETURN_BUFFER_SIZE 256

#define FLIGHT_RECORDER_CIRCULAR_BUFFER_ROWS          8
#define FLIGHT_RECORDER_CIRCULAR_BUFFER_MAX_COLUMNS   32

#define DECODE_NOT_VALID_TAG -2
#define DECODE_REQUEST_MODE_DATA 1

#include <Arduino.h>
#include <DataConfig.h>
#include <UpdateScheduler.h>


/*
The flight recorder is responsible for saving events and raw data at discrete time intervales
all sensor data is saved at a certain rate predefined else where
Instead of doing a flight 'frame' reference, as some data will desire a faster update rate, such as acceleration
May change later
*/
class FlightRecorder{
public:
  int init();
  //returns any requests on data writing
  int update(RocketData newData);

  //returns up to FLIGHT_RECORDER_MAX_RETURN_BUFFER_SIZE buffer to be written
  int encodeData(RocketData data);
  int scaleAndEncodeData(float data, uint numBytesToFill, int scale, byte*toFille);
  int getNumberRequestedMessages();
  int getRequestedBufferToStore(byte *buffer, uint maxSize);
  int determineEncodingByteSize(int dataFlag);
  int determineEncodingByteSize(RocketData *data);
  int decodeBuffer(byte *buff, uint maxLength, RocketData *target);
  float decodeAndScaleData(byte *buff, int byteLength, int scale);
private:
  ulong lastRawDataWriteTime = 0;
  ulong lastGPSDataWriteTime = 0;
  ulong lastFlightApproximationDataWriteTime = 0;
  RocketData accelData, pressureAltData, gyroData, magData, attitudeData, gpsData;
  UpdateScheduler accelScheduler, gyroScheduler, magScheduler, gpsScheduler, atltitudeScheduler;
  //this is going to be a circular buffer
  //the amount of data available is the difference between the start read and write indexes
  byte encodedData[FLIGHT_RECORDER_CIRCULAR_BUFFER_ROWS][FLIGHT_RECORDER_CIRCULAR_BUFFER_MAX_COLUMNS];
  uint8 encodedDataSize[FLIGHT_RECORDER_CIRCULAR_BUFFER_ROWS];
  int encodedDataStartReadIndex = 0;
  int encodedDataWriteIndex = 0;
  uint8 tempSize = 0;

  int encodeRocketData(RocketData *data, byte *target);
  int checkSchedulersForUpdates();
  int addUpdateToBuffer(RocketData *toUpdate);

  int copyBuffer(byte *target, byte *source, uint size, uint targetOffset = 0, uint sourceOffset = 0);

};


#endif
