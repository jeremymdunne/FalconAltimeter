#include <FlightRecorder.h>

int FlightRecorder::copyBuffer(byte *target, byte *source, uint size, uint targetOffset, uint sourceOffset){
  for(uint i = 0; i < size; i ++){
    target[i+targetOffset] = source[i+sourceOffset];
  }
  return 0;
}

int FlightRecorder::getRequestedBufferToStore(byte *buffer, uint maxSize){
  //the idea is to stuff as many requests into the available buffer as possible
  tempSize = 0;
  int maxEntries = getNumberRequestedMessages();
  if(maxEntries == 0) return -1;
  int entriesRecorded = 0;

  while((tempSize + encodedDataSize[encodedDataStartReadIndex] < maxSize) & (maxEntries > entriesRecorded)){
    //Serial.println("Size: " + String(encodedDataSize[encodedDataStartReadIndex]));
    copyBuffer(buffer, encodedData[encodedDataStartReadIndex], encodedDataSize[encodedDataStartReadIndex], tempSize);
    tempSize += encodedDataSize[encodedDataStartReadIndex];
    encodedDataStartReadIndex ++;
    if(encodedDataStartReadIndex > FLIGHT_RECORDER_CIRCULAR_BUFFER_ROWS) encodedDataStartReadIndex = 0;
    entriesRecorded ++;
  }
  return tempSize;
}

int FlightRecorder::init(){
  accelScheduler.setUpdateFrequency(FLIGHT_RECORDER_ACCEL_DATA_RECORDING_HERTZ);
  gyroScheduler.setUpdateFrequency(FLIGHT_RECORDER_GYRO_DATA_RECORDING_HERTZ);
  magScheduler.setUpdateFrequency(FLIGHT_RECORDER_MAG_DATA_RECORDING_HERTZ);
  gpsScheduler.setUpdateFrequency(FLIGHT_RECORDER_GPS_DATA_RECORDING_HERTZ);
  atltitudeScheduler.setUpdateFrequency(FLIGHT_RECORDER_PRESSURE_ALTITUDE_DATA_RECORDING_HERTZ);
  return 0;
}

int FlightRecorder::determineEncodingByteSize(RocketData *data){
  uint base = FLIGHT_RECORDER_TIME_STAMP_BYTE_SIZE + FLIGHT_RECORDER_TAG_BYTE_SIZE;
  switch(data->tag){
    case(FLIGHT_GYRO_DATA_TAG):
      base += 3 * FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(FLIGHT_ACCEL_DATA_TAG):
      base += 3 * FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(FLIGHT_MAG_DATA_TAG):
      base += 3 * FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(FLIGHT_GPS_DATA_TAG):
      base += 2 * FLIGHT_RECORDER_GPS_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
      base += 3 * FLIGHT_RECORDER_PRESSURE_ALTITUDE_BYTE_SIZE;
      break;
    case(FLIGHT_LAUNCH_DETECTION_EVENT_TAG):
      break;
    case(FLIGHT_COAST_DETECTION_EVENT_TAG):
      break;
    case(FLIGHT_APOGEE_DETECTION_EVENT_TAG):
      break;
    case(FLIGHT_MAIN_PARACHUTE_DEPLOYED_EVENT_TAG):
      break;
    case(FLIGHT_DROGUE_PARACHUTE_DEPLOYED_EVENT_TAG):
      break;
    case(FLIGHT_LANDING_DETECTION_EVENT_TAG):
      break;
    case(FLIGHT_AIR_BRAKE_DEPLOYMENT_EVENT_TAG):
      break;
    case(FLIGHT_AIR_BRAKE_RETRACTED_EVENT_TAG):
      break;
    default:
      return -1;
  }
  return base;
}

int FlightRecorder::scaleAndEncodeData(float data, uint numBytesToFill, int scale, byte *toFill){
  long tempData = (long)(data*scale+.5);
  for(uint i = 0; i < numBytesToFill; i ++){
    //Serial.println(((numBytesToFill - i - 1)*8));
    toFill[i] = (tempData >> ((numBytesToFill - i - 1)*8))&0xFF;
  }
  /*
  for(int i = numBytesToFill; i > 0; i --){
    toFill[i] = (tempData >> ((numBytesToFill - i)*8)) & 0xFF;
  }
  */
  return 0;
};

int FlightRecorder::encodeRocketData(RocketData *data, byte *target){
  tempSize = FLIGHT_RECORDER_TIME_STAMP_BYTE_SIZE + FLIGHT_RECORDER_TAG_BYTE_SIZE;
  scaleAndEncodeData(data->tag, FLIGHT_RECORDER_TAG_BYTE_SIZE, FLIGHT_RECORDER_TAG_SCALE_FACTOR, &target[0]);
  scaleAndEncodeData(data->timeStamp, FLIGHT_RECORDER_TIME_STAMP_BYTE_SIZE, FLIGHT_RECORDER_TIME_STAMP_SCALE_FACTOR, &target[1]);
  switch(data->tag){
    case(FLIGHT_ACCEL_DATA_TAG):
      scaleAndEncodeData(data->data[0],FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_ACCEL_DATA_SCALE_FACTOR, &target[4]);
      scaleAndEncodeData(data->data[1],FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_ACCEL_DATA_SCALE_FACTOR, &target[7]);
      scaleAndEncodeData(data->data[2],FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_ACCEL_DATA_SCALE_FACTOR, &target[10]);
      tempSize += 3*FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(FLIGHT_GYRO_DATA_TAG):
      scaleAndEncodeData(data->data[0],FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_GYRO_DATA_SCALE_FACTOR, &target[4]);
      scaleAndEncodeData(data->data[1],FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_GYRO_DATA_SCALE_FACTOR, &target[7]);
      scaleAndEncodeData(data->data[2],FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_GYRO_DATA_SCALE_FACTOR, &target[10]);
      tempSize += 3*FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(FLIGHT_MAG_DATA_TAG):
      scaleAndEncodeData(data->data[0],FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_MAG_DATA_SCALE_FACTOR, &target[4]);
      scaleAndEncodeData(data->data[1],FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_MAG_DATA_SCALE_FACTOR, &target[7]);
      scaleAndEncodeData(data->data[2],FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS, FLIGHT_RECORDER_MAG_DATA_SCALE_FACTOR, &target[10]);
      tempSize += 3*FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
      scaleAndEncodeData(data->data[0],FLIGHT_RECORDER_PRESSURE_ALTITUDE_BYTE_SIZE, FLIGHT_RECORDER_PRESSURE_ALTITUDE_SCALE_FACTOR, &target[4]);
      tempSize += FLIGHT_RECORDER_PRESSURE_ALTITUDE_BYTE_SIZE;
      break;
  }
  return tempSize;
}

int FlightRecorder::addUpdateToBuffer(RocketData *toUpdate){
  //check the byte buffer
  //Serial.println(toUpdate->data[0]);
  if(toUpdate->data != NULL){
    encodedDataSize[encodedDataWriteIndex] = encodeRocketData(toUpdate, &encodedData[encodedDataWriteIndex][0]);
    //Serial.println("Size of encode:" + String(encodedDataSize[encodedDataWriteIndex]));
    encodedDataWriteIndex ++;
    //Serial.println("Write Index: " + String(encodedDataWriteIndex));
    if(encodedDataWriteIndex >= FLIGHT_RECORDER_CIRCULAR_BUFFER_ROWS) encodedDataWriteIndex = 0;
    //Serial.println("Write Index: " + String(encodedDataWriteIndex));
    return 0;
  }
  return -1;
}

int FlightRecorder::checkSchedulersForUpdates(){
  if(accelScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    addUpdateToBuffer(&accelData);
  }
  if(gyroScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    addUpdateToBuffer(&gyroData);
  }
  if(magScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    addUpdateToBuffer(&magData);
  }
  if(atltitudeScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    addUpdateToBuffer(&pressureAltData);
  }
  if(gpsScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    addUpdateToBuffer(&gpsData);
  }
  return 0;
}

int FlightRecorder::getNumberRequestedMessages(){
  if(encodedDataWriteIndex < encodedDataStartReadIndex){
    return FLIGHT_RECORDER_CIRCULAR_BUFFER_ROWS + (encodedDataWriteIndex - encodedDataStartReadIndex);
  }
  return encodedDataWriteIndex - encodedDataStartReadIndex;
}

int FlightRecorder::update(RocketData newData){
  //Serial.println("Start: " + String(encodedDataWriteIndex) +" Read:" + String(encodedDataStartReadIndex));
  switch(newData.tag){
    case(FLIGHT_ACCEL_DATA_TAG):
      accelData = newData;
      break;
    case(FLIGHT_GPS_DATA_TAG):
      gpsData = newData;
      break;
    case(FLIGHT_GYRO_DATA_TAG):
      gyroData = newData;
      break;
    case(FLIGHT_MAG_DATA_TAG):
      magData = newData;
      break;
    case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
      pressureAltData = newData;
      break;
  }
  //update time handler
  checkSchedulersForUpdates();
  return getNumberRequestedMessages();
}
