#include <FlightRecorder.h>


int FlightRecorder::init(){
  //set default update states on all sensors
  imuAccelUpdater.setUpdateFrequency(FLIGHT_RECORDER_DEFAULT_ACCEL_RECORD_FREQUENCY);
  imuGyroUpdater.setUpdateFrequency(FLIGHT_RECORDER_DEFUALT_ANGULAR_RATE_RECORD_FREQUENCY);
  imuMagneticFieldUpdater.setUpdateFrequency(FLIGHT_RECORDER_DEFAULT_MAGNETIC_FIELD_RECORD_FREQUENCY);
  imuEulerUpdater.setUpdateFrequency(FLIGHT_RECORDER_DEFAULT_EULER_RECORD_FREQUENCY);
  pressureUpdater.setUpdateFrequency(FLIGHT_RECORDER_DEFAULT_PRESSURE_RECORD_FREQUENCY);
  pressureAltitudeUpdater.setUpdateFrequency(FLIGHT_RECORDER_DEFUALT_PRESSURE_ALTITUDE_RECORD_FREQUENCY);
  temperatureUpdater.setUpdateFrequency(FLIGHT_RECORDER_DEFAULT_TEMPERATURE_RECORD_FREQUENCY);
  //gpsUpdater.setUpdateFrequency(FLIGHT_RECORDER_DEFAULT_GPS_RECORD_FREQUENCY);

  return 0;
}

int FlightRecorder::update(Rocket_State *updatedState){
    //check the flags
    if(updatedState->currentFlightPhase != runningState.currentFlightPhase){
      //flight phase change!
      phaseChange = true;
    }
    //copy the data
    copyRocketState(&runningState,updatedState);
    //check the schedulers
    uint8_t numUpdates = 0;
    if(imuAccelUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newIMUAccelData = true;
      numUpdates ++;
    }
    if(imuGyroUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newIMUAngularRateData = true;
      numUpdates ++;
    }
    if(imuMagneticFieldUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newIMUMagneticFieldData = true;
      numUpdates ++;
    }
    if(imuEulerUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newEulerData = true;
      numUpdates ++;
    }
    if(gpsUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newGPSData = true;
      numUpdates ++;
    }
    if(pressureUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newPressureData = true;
      numUpdates ++;
    }
    if(pressureAltitudeUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newPressureAltitudeData = true;
      numUpdates ++;
    }
    if(temperatureUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newTemperatureData = true;
      numUpdates ++;
    }
    if(filteredAltitudeUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newFilteredAltitudeData = true;
      numUpdates ++;
    }
    if(filteredVelocityUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newFilteredVelocityData = true;
      numUpdates ++;
    }
    if(filteredAccelerationUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      newFilteredAccelerationsData = true;
      numUpdates ++;
    }
    return numUpdates;
}

int FlightRecorder::hardSetState(Rocket_State *hardState){
  //hard set everything
  copyRocketState(&runningState, hardState);
  return 0;
}

int FlightRecorder::getRequestedDataToStore(uint8_t *targetBuffer, uint maxSize){
  //go through each new data line, add it to the buffer, flip flag, stop when exceeding size
  uint usedSize = 0;
  if(newIMUAccelData){
    if(usedSize + determineEncodingByteSize(RAW_ACCEL_DATA_TAG) < maxSize){
      usedSize += encodeData(RAW_ACCEL_DATA_TAG, runningState.primaryIMUdata.timestamp,  &runningState.primaryIMUdata.acceleration[0], &targetBuffer[usedSize]);
      newIMUAccelData = false;
    }
  }
  if(newIMUAngularRateData){
    if(usedSize + determineEncodingByteSize(RAW_ANGULAR_RATE_DATA_TAG) < maxSize){
      usedSize += encodeData(RAW_ANGULAR_RATE_DATA_TAG, runningState.primaryIMUdata.timestamp,  &runningState.primaryIMUdata.angularRate[0], &targetBuffer[usedSize]);
      newIMUAngularRateData = false;
    }
  }
  if(newIMUMagneticFieldData){
    if(usedSize + determineEncodingByteSize(RAW_MAGNETIC_FIELD_DATA_TAG) < maxSize){
      usedSize += encodeData(RAW_MAGNETIC_FIELD_DATA_TAG, runningState.primaryIMUdata.timestamp,  &runningState.primaryIMUdata.magneticField[0], &targetBuffer[usedSize]);
      newIMUMagneticFieldData = false;
    }
  }
  //TODO: GPS
  /*
  if(newGPSData){
    if(usedSize + determineEncodingByteSize(RAW_GPS_DATA_TAG) < maxSize){
      usedSize += encodeData(RAW_GPS_DATA_TAG, runningState.primaryGPSdata.timestamp,  &runningState.primaryGPSdata.acceleration, &targetBuffer[usedSize]);
      newGPSData = false;
    }
  }
  */
  if(newTemperatureData){
    if(usedSize + determineEncodingByteSize(RAW_TEMPERATURE_DATA_TAG) < maxSize){
      usedSize += encodeData(RAW_TEMPERATURE_DATA_TAG, runningState.primaryBarometricData.timestamp,  &runningState.primaryBarometricData.temperature, &targetBuffer[usedSize]);
      newTemperatureData = false;
    }
  }
  if(newPressureAltitudeData){
    if(usedSize + determineEncodingByteSize(RAW_PRESSURE_ALTITUDE_TAG) < maxSize){
      usedSize += encodeData(RAW_PRESSURE_ALTITUDE_TAG, runningState.primaryBarometricData.timestamp,  &runningState.primaryBarometricData.altitude, &targetBuffer[usedSize]);
      newPressureAltitudeData = false;
    }
  }
  if(newPressureData){
    if(usedSize + determineEncodingByteSize(RAW_PRESSURE_TAG) < maxSize){
      usedSize += encodeData(RAW_PRESSURE_TAG, runningState.primaryBarometricData.timestamp,  &runningState.primaryBarometricData.pressure, &targetBuffer[usedSize]);
      newPressureData = false;
    }
  }
  return usedSize;
}

int FlightRecorder::determineEncodingByteSize(int dataFlag){
  uint tempSize = DATA_STORAGE_TIME_STAMP_BYTE_SIZE + DATA_STORAGE_TAG_BYTE_SIZE;
  switch(dataFlag){
    case(RAW_ACCEL_DATA_TAG):
      tempSize += 3* DATA_STORAGE_ACCEL_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(RAW_ANGULAR_RATE_DATA_TAG):
      tempSize += 3* DATA_STORAGE_GYRO_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(RAW_MAGNETIC_FIELD_DATA_TAG):
      tempSize += 3* DATA_STORAGE_MAG_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(RAW_PRESSURE_ALTITUDE_TAG):
      tempSize += DATA_STORAGE_PRESSURE_ALTITUDE_BYTE_SIZE;
      break;
  }
  return tempSize;
}

int FlightRecorder::encodeData(int dataFlag, long timeStamp, float *data, uint8_t *target){
  uint tempSize = DATA_STORAGE_TIME_STAMP_BYTE_SIZE + DATA_STORAGE_TAG_BYTE_SIZE;
  scaleAndEncodeData(dataFlag, DATA_STORAGE_TAG_BYTE_SIZE, DATA_STORAGE_TAG_SCALE_FACTOR, &target[0]);
  scaleAndEncodeData(timeStamp, DATA_STORAGE_TIME_STAMP_BYTE_SIZE, DATA_STORAGE_TIME_STAMP_SCALE_FACTOR, &target[DATA_STORAGE_TAG_BYTE_SIZE]);
  switch(dataFlag){
    case(RAW_ACCEL_DATA_TAG):
      scaleAndEncodeData(data[0],DATA_STORAGE_ACCEL_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_ACCEL_DATA_SCALE_FACTOR, &target[tempSize]);
      scaleAndEncodeData(data[1],DATA_STORAGE_ACCEL_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_ACCEL_DATA_SCALE_FACTOR, &target[tempSize + DATA_STORAGE_ACCEL_DATA_BYTE_SIZE_PER_AXIS]);
      scaleAndEncodeData(data[2],DATA_STORAGE_ACCEL_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_ACCEL_DATA_SCALE_FACTOR, &target[tempSize + 2*DATA_STORAGE_ACCEL_DATA_BYTE_SIZE_PER_AXIS]);
      tempSize += 3* DATA_STORAGE_ACCEL_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(RAW_ANGULAR_RATE_DATA_TAG):
      scaleAndEncodeData(data[0],DATA_STORAGE_GYRO_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_GYRO_DATA_SCALE_FACTOR, &target[tempSize]);
      scaleAndEncodeData(data[1],DATA_STORAGE_GYRO_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_GYRO_DATA_SCALE_FACTOR, &target[tempSize + DATA_STORAGE_GYRO_DATA_BYTE_SIZE_PER_AXIS]);
      scaleAndEncodeData(data[2],DATA_STORAGE_GYRO_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_GYRO_DATA_SCALE_FACTOR, &target[tempSize + 2*DATA_STORAGE_GYRO_DATA_BYTE_SIZE_PER_AXIS]);
      tempSize += 3* DATA_STORAGE_GYRO_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(RAW_MAGNETIC_FIELD_DATA_TAG):
      scaleAndEncodeData(data[0],DATA_STORAGE_MAG_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_MAG_DATA_SCALE_FACTOR, &target[tempSize]);
      scaleAndEncodeData(data[1],DATA_STORAGE_MAG_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_MAG_DATA_SCALE_FACTOR, &target[tempSize + DATA_STORAGE_MAG_DATA_BYTE_SIZE_PER_AXIS]);
      scaleAndEncodeData(data[2],DATA_STORAGE_MAG_DATA_BYTE_SIZE_PER_AXIS, DATA_STORAGE_MAG_DATA_SCALE_FACTOR, &target[tempSize + 2*DATA_STORAGE_MAG_DATA_BYTE_SIZE_PER_AXIS]);
      tempSize += 3* DATA_STORAGE_MAG_DATA_BYTE_SIZE_PER_AXIS;
      break;
    case(RAW_PRESSURE_ALTITUDE_TAG):
      scaleAndEncodeData(data[2],DATA_STORAGE_PRESSURE_ALTITUDE_BYTE_SIZE, DATA_STORAGE_PRESSURE_ALTITUDE_SCALE_FACTOR, &target[tempSize]);
      tempSize += DATA_STORAGE_PRESSURE_ALTITUDE_BYTE_SIZE;
      break;
  }
  return tempSize;
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
}

int FlightRecorder::translateNext(uint8_t *dataToRead, uint length, float *target, uint targetLen){
  //assume the first part is the data tag
  int flag = decodeAndScaleData(&dataToRead[0], DATA_STORAGE_TAG_BYTE_SIZE, DATA_STORAGE_TAG_SCALE_FACTOR, false);
  //determine the byte size of the message
  int size = determineEncodingByteSize(flag);
  if(size < 0) //some sort of error
  return -1; 
}

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
