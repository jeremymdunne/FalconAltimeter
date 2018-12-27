#include <FlightRecorder.h>

int FlightRecorder::copyBuffer(byte *target, byte *source, uint size, uint targetOffset, uint sourceOffset){
  for(uint i = 0; i < size; i ++){
    target[i+targetOffset] = source[i+sourceOffset];
  }
  return 0;
}

int FlightRecorder::getRequestedDataToStore(RocketData *data, uint maxSize){
  uint counter = 0;
  RocketData tempRocketData;
  while(true){
    if(newAccelData){
      tempRocketData = accelData;
      //Serial.println("\nAccel tag requesting storage: Time:" + String(tempRocketData.timeStamp) + " X:" + String(tempRocketData.data[0]) + " Y:" + String(tempRocketData.data[1]) + " Z:" + String(tempRocketData.data[2]));
    }
    else if(newGyroData) {
       tempRocketData = gyroData;
       //Serial.println("\nGyro tag requesting storage: Time:" + String(tempRocketData.timeStamp) + " X:" + String(tempRocketData.data[0]) + " Y:" + String(tempRocketData.data[1]) + " Z:" + String(tempRocketData.data[2]));
    }
    else if(newMagData) tempRocketData = magData;
    else if(newGpsData) tempRocketData = gpsData;
    else if(newPressureAltitudeData){
      tempRocketData = pressureAltData;
      //Serial.println("\nPressure tag requesting storage: Time:" + String(tempRocketData.timeStamp) + " Alt:" + String(tempRocketData.data[0]));
    }
    else if(newEvent){
      //Serial.println("Encoding newEvent!");
      tempRocketData = eventData;
    }
    else return counter;
    if(counter < maxSize){
      data[counter] = tempRocketData;
      counter ++;
      //Serial.println("Tags written");
      switch(tempRocketData.tag){
        case(FLIGHT_ACCEL_DATA_TAG):
          newAccelData = false;
          break;
        case(FLIGHT_GYRO_DATA_TAG):
          newGyroData = false;
          break;
        case(FLIGHT_MAG_DATA_TAG):
          newMagData = false;
          break;
        case(FLIGHT_GPS_DATA_TAG):
          newGpsData = false;
          break;
        case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
          newPressureAltitudeData = false;
          break;
        case(FLIGHT_LAUNCH_DETECTION_EVENT_TAG):
          newEvent = false;
          break;
      }
    }
    else{
      return counter;
    }
  }
}

int FlightRecorder::getRequestedBufferToStore(byte *buffer, uint maxSize){
  uint counter = 0;
  RocketData tempRocketData;
  while(true){
    if(newAccelData){
      tempRocketData = accelData;
      Serial.println("\nAccel tag requesting storage: Time:" + String(tempRocketData.timeStamp) + " X:" + String(tempRocketData.data[0]) + " Y:" + String(tempRocketData.data[1]) + " Z:" + String(tempRocketData.data[2]));
    }
    else if(newGyroData) {
       tempRocketData = gyroData;
       Serial.println("\nGyro tag requesting storage: Time:" + String(tempRocketData.timeStamp) + " X:" + String(tempRocketData.data[0]) + " Y:" + String(tempRocketData.data[1]) + " Z:" + String(tempRocketData.data[2]));
    }
    else if(newMagData) tempRocketData = magData;
    else if(newGpsData) tempRocketData = gpsData;
    else if(newEvent){
      Serial.println("Encoding newEvent!");
      tempRocketData = eventData;
    }
    else return counter;
    tempSize = determineEncodingByteSize(tempRocketData.tag);
    if(counter + tempSize < maxSize){
      counter += encodeRocketData(&tempRocketData, &buffer[counter]);
      switch(tempRocketData.tag){
        case(FLIGHT_ACCEL_DATA_TAG):
          newAccelData = false;
          break;
        case(FLIGHT_GYRO_DATA_TAG):
          newGyroData = false;
          break;
        case(FLIGHT_MAG_DATA_TAG):
          newMagData = false;
          break;
        case(FLIGHT_GPS_DATA_TAG):
          newGpsData = false;
          break;
        case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
          newPressureAltitudeData = false;
          break;
        case(FLIGHT_LAUNCH_DETECTION_EVENT_TAG):
          newEvent = false;
          break;
      }
    }
    else{
      return counter;
    }
  }
/*
  //the idea is to stuff as many requests into the available buffer as possible
  tempSize = 0;
  //int maxEntries = getNumberRequestedMessages();
  if(maxEntries == 0) return -1;
  int entriesRecorded = 0;




  while((tempSize + encodedDataSize[encodedDataStartReadIndex] < maxSize) & (maxEntries > entriesRecorded)){
    //Serial.println("Size: " + String(encodedDataSize[encodedDataStartReadIndex]));
    copyBuffer(buffer, encodedData[encodedDataStartReadIndex], encodedDataSize[encodedDataStartReadIndex], tempSize);
    tempSize += encodedDataSize[encodedDataStartReadIndex];
    encodedDataStartReadIndex ++;
    if(encodedDataStartReadIndex >= FLIGHT_RECORDER_CIRCULAR_BUFFER_ROWS) encodedDataStartReadIndex = 0;
    entriesRecorded ++;
  }
  return tempSize;
  */
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
  return determineEncodingByteSize(data->tag);
}

int FlightRecorder::determineEncodingByteSize(int tag){
  uint base = FLIGHT_RECORDER_TIME_STAMP_BYTE_SIZE + FLIGHT_RECORDER_TAG_BYTE_SIZE;
  switch(tag){
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
      base += FLIGHT_RECORDER_PRESSURE_ALTITUDE_BYTE_SIZE;
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
  ulong tempData = (ulong)(long)(data*scale+.5);
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

//combines the necessary bytes and scales
float FlightRecorder::decodeAndScaleData(byte *buff, int byteLength, int scale, bool dataSigned){
  ulong uvalue = 0;

  for(int i = 0; i < byteLength; i ++){
    uvalue += buff[i] << ((byteLength - i - 1)*8);
    //Serial.println("Raw Value: " + String(buff[i]) + " shifted: " + String((byteLength - i - 1)*8));
  }
  long value = (long)uvalue;
  /*
  okay, so the problem is we can't do this blind conversion due to negatives and non
  standard data sizes
  solution:
  */
  if(dataSigned){
    if(value > pow(2,byteLength*8)/2){
      value = value - pow(2,byteLength*8);
    }
  }

  //Serial.println("Combined value:" + String(value));
  return (float)value/(float)scale;
}

int FlightRecorder::decodeBuffer(byte *buff, uint maxLength, RocketData *target){
  target->tag = decodeAndScaleData(&buff[0], FLIGHT_RECORDER_TAG_BYTE_SIZE, FLIGHT_RECORDER_TAG_SCALE_FACTOR,false);
  target->timeStamp = decodeAndScaleData(&buff[FLIGHT_RECORDER_TAG_BYTE_SIZE], FLIGHT_RECORDER_TIME_STAMP_BYTE_SIZE, FLIGHT_RECORDER_TIME_STAMP_SCALE_FACTOR,false);
  if(target->tag < 0 || target->tag > 10){
    //bad peice of data
    return DECODE_NOT_VALID_TAG;
  }
  uint requiredLength = determineEncodingByteSize(target->tag);
  //Serial.println("Length: " + String(requiredLength));
  if(maxLength < requiredLength){
    //uh oh, request more datas to complete
    return DECODE_REQUEST_MODE_DATA;
  }
  int base = FLIGHT_RECORDER_TAG_BYTE_SIZE+FLIGHT_RECORDER_TIME_STAMP_BYTE_SIZE;

  switch(buff[0]){
    case(FLIGHT_ACCEL_DATA_TAG):
      target->data[0] = decodeAndScaleData(&buff[base],FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_ACCEL_DATA_SCALE_FACTOR);
      target->data[1] = decodeAndScaleData(&buff[base + FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS],FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_ACCEL_DATA_SCALE_FACTOR);
      target->data[2] = decodeAndScaleData(&buff[base + 2*FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS],FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_ACCEL_DATA_SCALE_FACTOR);
      break;
    case(FLIGHT_GYRO_DATA_TAG):
      target->data[0] = decodeAndScaleData(&buff[base],FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_GYRO_DATA_SCALE_FACTOR);
      target->data[1] = decodeAndScaleData(&buff[base + FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS],FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_GYRO_DATA_SCALE_FACTOR);
      target->data[2] = decodeAndScaleData(&buff[base + 2*FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS],FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_GYRO_DATA_SCALE_FACTOR);
      break;
    case(FLIGHT_MAG_DATA_TAG):
      target->data[0] = decodeAndScaleData(&buff[base],FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_MAG_DATA_SCALE_FACTOR);
      target->data[1] = decodeAndScaleData(&buff[base + FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS],FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_MAG_DATA_SCALE_FACTOR);
      target->data[2] = decodeAndScaleData(&buff[base + 2*FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS],FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS,FLIGHT_RECORDER_MAG_DATA_SCALE_FACTOR);
      break;
    case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
      //Serial.println("Pressure tag detected!");
      target->data[0] = decodeAndScaleData(&buff[base],FLIGHT_RECORDER_PRESSURE_ALTITUDE_BYTE_SIZE,FLIGHT_RECORDER_PRESSURE_ALTITUDE_SCALE_FACTOR);
      //Serial.println("Done");
      break;
    case(FLIGHT_LAUNCH_DETECTION_EVENT_TAG):
      break;
    default:
      return -1;
      break;
  }
  return requiredLength;
}

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
    case(FLIGHT_LAUNCH_DETECTION_EVENT_TAG):
      //as of now, no extra data to encode
      break;
  }
  return tempSize;
}

int FlightRecorder::checkSchedulersForUpdates(){
  uint numUpdates = 0;
  if(accelScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    newAccelData = true;
    numUpdates ++;
  }
  if(gyroScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    newGyroData = true;
    numUpdates ++;
  }
  if(magScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    //newMagData = true;
    //numUpdates ++;
  }
  if(atltitudeScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    newPressureAltitudeData = true;
    numUpdates ++;
  }
  if(gpsScheduler.update() == TIME_UPDATER_UPDATE_REQUIRED){
    //newGpsData = true;
    //numUpdates ++;
  }
  if(newEvent){
    numUpdates ++;
  }
  return numUpdates;
}

int FlightRecorder::update(RocketData *newData, uint n){
  //Serial.println("Start: " + String(encodedDataWriteIndex) +" Read:" + String(encodedDataStartReadIndex));
  for(uint i = 0; i < n; i ++){
    switch(newData[i].tag){
      case(FLIGHT_ACCEL_DATA_TAG):
        accelData.timeStamp = newData[i].timeStamp;
        accelData.data[0] = newData[i].data[0];
        accelData.data[1] = newData[i].data[1];
        accelData.data[2] = newData[i].data[2];
        break;
      case(FLIGHT_GPS_DATA_TAG):
        gpsData.timeStamp = newData[i].timeStamp;
        gpsData.data[0] = newData[i].data[0];
        gpsData.data[1] = newData[i].data[1];
        break;
      case(FLIGHT_GYRO_DATA_TAG):
        gyroData.timeStamp = newData[i].timeStamp;
        gyroData.data[0] = newData[i].data[0];
        gyroData.data[1] = newData[i].data[1];
        gyroData.data[2] = newData[i].data[2];
        break;
      case(FLIGHT_MAG_DATA_TAG):
        magData.timeStamp = newData[i].timeStamp;
        magData.data[0] = newData[i].data[0];
        magData.data[1] = newData[i].data[1];
        magData.data[2] = newData[i].data[2];
        break;
      case(FLIGHT_PRESSURE_ALTITUDE_DATA_TAG):
        pressureAltData.timeStamp = newData[i].timeStamp;
        pressureAltData.data[0] = newData[i].data[0];
        break;
      case(FLIGHT_LAUNCH_DETECTION_EVENT_TAG):
        eventData.tag = newData[i].tag;
        eventData.timeStamp = newData[i].timeStamp;
        newEvent = true;
        break;
    }
  }
  //update time handler
  return checkSchedulersForUpdates();
}
