#include <MPU9250_IMU.h>

void MPU9250_IMU::reportToUser(String what){
  if(useDifferentCallback){
      (*userCallback)(what);
  }
  else{
    reportToUser(what);
  }
}

void MPU9250_IMU::reportToUser(float what){
  reportToUser(String(what));
}

void MPU9250_IMU::setUserCallbackFunction(void (*messageToUserCallback)(String)){
  userCallback = (*messageToUserCallback);
  useDifferentCallback = true;
}

void MPU9250_IMU::loadIMUCalibrationDataFromEEPROM(int startAddress){
  uint16_t gyroOffsets[3];
  uint16_t accelOffsets[3];
  uint16_t magOffsets[3];
  uint16_t magScales[3];
  uint16_t orientationOffsets[3];
  float scaledGyroOffsets[3];
  float scaledAccelOffsets[3];
  float scaledMagOffsets[3];
  float scaledMagScales[3];
  float scaledOrientationOffsets[3];
  //go and grab the values
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_GYRO_X_OFFSET_EEPROM_OFFSET), &gyroOffsets[0]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_GYRO_Y_OFFSET_EEPROM_OFFSET), &gyroOffsets[1]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_GYRO_Z_OFFSET_EEPROM_OFFSET), &gyroOffsets[2]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_ACCEL_X_OFFSET_EEPROM_OFFSET), &accelOffsets[0]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_ACCEL_Y_OFFSET_EEPROM_OFFSET), &accelOffsets[1]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_ACCEL_Z_OFFSET_EEPROM_OFFSET), &accelOffsets[2]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_MAG_X_OFFSET_EEPROM_OFFSET), &magOffsets[0]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_MAG_Y_OFFSET_EEPROM_OFFSET), &magOffsets[1]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_MAG_Z_OFFSET_EEPROM_OFFSET), &magOffsets[2]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_MAG_X_SCALE_EEPROM_OFFSET), &magScales[0]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_MAG_Y_SCALE_EEPROM_OFFSET), &magScales[1]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_MAG_Z_SCALE_EEPROM_OFFSET), &magScales[2]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_ORIENTATION_X_OFFSET_EEPROM_OFFSET), &orientationOffsets[0]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_ORIENTATION_Y_OFFSET_EEPROM_OFFSET), &orientationOffsets[1]);
  EEPROM.read((uint16_t)(startAddress + MPU9250_IMU_ORIENTATION_Z_OFFSET_EEPROM_OFFSET), &orientationOffsets[2]);
  for(int i = 0; i < 3; i ++){
    scaledGyroOffsets[i] = (int16_t)gyroOffsets[i];
    scaledAccelOffsets[i] = (int16_t)accelOffsets[i];
    scaledMagOffsets[i] = (int16_t)magOffsets[i];
    scaledMagScales[i] = (int16_t)magScales[i];
    scaledOrientationOffsets[i] = (int16_t)orientationOffsets[i];
    scaledGyroOffsets[i] /= MPU9250_IMU_GYRO_OFFSET_SCALE_EEPROM;
    scaledAccelOffsets[i] /= MPU9250_IMU_ACCEL_OFFSET_SCALE_EEPROM;
    scaledMagOffsets[i] /= MPU9250_IMU_MAG_OFFSET_SCALE_EEPROM;
    scaledMagScales[i] /= MPU9250_IMU_MAG_SCALE_SCALE_EEPROM;
    scaledOrientationOffsets[i] /= MPU9250_IMU_ORIENTATION_OFFSET_SCALE;
    //reportToUser(scaledGyroOffsets[i]);
    //reportToUser(scaledAccelOffsets[i]);
    //reportToUser(scaledMagOffsets[i]);
    //reportToUser(scaledMagScales[i]);
    //reportToUser(scaledOrientationOffsets[i]);
  }
  //go ahead and write them
  setCalibrationOffsets(&scaledGyroOffsets[0], &scaledAccelOffsets[0], &scaledMagOffsets[0], &scaledMagScales[0], &scaledOrientationOffsets[0]);
}

void MPU9250_IMU::saveIMUCalibrationDataToEEPROM(int startAddress, float *gyro, float *accel, float *mag, float *magScale, float *origOffset){
  //TODO implement size check on signed 16 bit numbers
  uint16_t gyroOffsets[3];
  uint16_t accelOffsets[3];
  uint16_t magOffsets[3];
  uint16_t magScales[3];
  uint16_t orientationOffsets[3];
  reportToUser(mag[0]);
  reportToUser(mag[1]);
  reportToUser(mag[2]);
  for(int i = 0; i < 3; i ++){
    //cast to int16 and then unsigned
    gyroOffsets[i] = (uint16_t)(int16_t)(gyro[i]*MPU9250_IMU_GYRO_OFFSET_SCALE_EEPROM);
    accelOffsets[i] = (uint16_t)(int16_t)(accel[i]*MPU9250_IMU_ACCEL_OFFSET_SCALE_EEPROM);
    magOffsets[i] = (uint16_t)(int16_t)(mag[i]*MPU9250_IMU_MAG_OFFSET_SCALE_EEPROM);
    magScales[i] = (uint16_t)(int16_t)(magScale[i]*MPU9250_IMU_MAG_SCALE_SCALE_EEPROM);
    orientationOffsets[i] = (uint16_t)(int16_t)(origOffset[i] * MPU9250_IMU_ORIENTATION_OFFSET_SCALE);
  }
  EEPROM.write(startAddress + MPU9250_IMU_GYRO_X_OFFSET_EEPROM_OFFSET, gyroOffsets[0]);
  EEPROM.write(startAddress + MPU9250_IMU_GYRO_Y_OFFSET_EEPROM_OFFSET, gyroOffsets[1]);
  EEPROM.write(startAddress + MPU9250_IMU_GYRO_Z_OFFSET_EEPROM_OFFSET, gyroOffsets[2]);
  EEPROM.write(startAddress + MPU9250_IMU_ACCEL_X_OFFSET_EEPROM_OFFSET, accelOffsets[0]);
  EEPROM.write(startAddress + MPU9250_IMU_ACCEL_Y_OFFSET_EEPROM_OFFSET, accelOffsets[1]);
  EEPROM.write(startAddress + MPU9250_IMU_ACCEL_Z_OFFSET_EEPROM_OFFSET, accelOffsets[2]);
  EEPROM.write(startAddress + MPU9250_IMU_MAG_X_OFFSET_EEPROM_OFFSET, magOffsets[0]);
  EEPROM.write(startAddress + MPU9250_IMU_MAG_Y_OFFSET_EEPROM_OFFSET, magOffsets[1]);
  EEPROM.write(startAddress + MPU9250_IMU_MAG_Z_OFFSET_EEPROM_OFFSET, magOffsets[2]);
  EEPROM.write(startAddress + MPU9250_IMU_MAG_X_SCALE_EEPROM_OFFSET, magScales[0]);
  EEPROM.write(startAddress + MPU9250_IMU_MAG_Y_SCALE_EEPROM_OFFSET, magScales[1]);
  EEPROM.write(startAddress + MPU9250_IMU_MAG_Z_SCALE_EEPROM_OFFSET, magScales[2]);
  //reportToUser(magOffsets[0]);
  //reportToUser(magOffsets[1]);
  //reportToUser(magOffsets[2]);
  EEPROM.write(startAddress + MPU9250_IMU_ORIENTATION_X_OFFSET_EEPROM_OFFSET, orientationOffsets[0]);
  EEPROM.write(startAddress + MPU9250_IMU_ORIENTATION_Y_OFFSET_EEPROM_OFFSET, orientationOffsets[1]);
  EEPROM.write(startAddress + MPU9250_IMU_ORIENTATION_Z_OFFSET_EEPROM_OFFSET, orientationOffsets[2]);
}

void MPU9250_IMU::setCalibrationOffsets(float *gyroOffset, float *accelOffset, float *magOffset, float *magScale, float *orientationOffsets){
  mpu.setAccelCalibration(&accelOffset[0]);
  mpu.setGyroCalibration(&gyroOffset[0]);
  mpu.setMagCalibration(&magScale[0], &magOffset[0]);
  zeroOriginOffsets[0] = orientationOffsets[0];
  zeroOriginOffsets[1] = orientationOffsets[1];
  zeroOriginOffsets[2] = 0;
}

void MPU9250_IMU::beginRelativeZ(){
  //reset the Z to the current offset
  MPU_IMU_DATA tempData;
  getData(&tempData);
  zeroOriginOffsets[2] += tempData.euler[2];
}

void MPU9250_IMU::zeroAll(){
    //zero so that the new 'origin' is the current orientation
  MPU_IMU_DATA tempData;
  getData(&tempData);
  zeroOriginOffsets[0] += tempData.euler[0];
  zeroOriginOffsets[1] += tempData.euler[1];
  zeroOriginOffsets[2] += tempData.euler[2];
}

void MPU9250_IMU::calibrateAll(){
  //zero goes a couple of ways
  //first, grab the delta gyro degrees
  //then grab the accel offsets and scale
  //then begin grabbing the mag offsets
  //first clear all calibration stuffs
  //prompt the user and make sure they have a serial connection
  reportToUser("Entering MPU Calibration Mode\n");
  delay(2000);
  float gyroOffsets[3] = {0,0,0};
  float accelOffsets[3] = {0,0,0};
  float magOffsets[3] = {0,0,0};
  float magScales[3] = {0,0,0};
  float zeroOffsets[3] = {0,0,0};
  //clear orientationOffset
  zeroOriginOffsets[0] = 0;
  zeroOriginOffsets[1] = 0;
  zeroOriginOffsets[2] = 0;
  mpu.clearCalibration();
  reportToUser("Gyro Calibration:\n");
  zeroGyro(&gyroOffsets[0]);
  //reportToUser("Gyro Configuration: X:" + String(gyroOffsets[0]) + " Y:" + String(gyroOffsets[1]) + " Z:" + String(gyroOffsets[2]));
  //zeroAccel(&accelOffsets[0]);
  reportToUser("Skipping Accel Configuration, not yet implemented!\n");
  //reportToUser("Accel Configuration: X:" + String(accelOffsets[0]) + " Z:" + String(accelOffsets[1]) + " Z:" + String(accelOffsets[2]));
  reportToUser("Magnetometer Calibration:\n");
  zeroMag(&magOffsets[0], &magScales[0]);
  setCalibrationOffsets(&gyroOffsets[0], &accelOffsets[0], &magOffsets[0], &magScales[0], &zeroOffsets[0]);
  zeroOrigin(&zeroOffsets[0]);
  //save to EEPROM
  setCalibrationOffsets(&gyroOffsets[0], &accelOffsets[0], &magOffsets[0], &magScales[0], &zeroOffsets[0]);
  saveIMUCalibrationDataToEEPROM(eepromCalibrationOffset, &gyroOffsets[0], &accelOffsets[0], &magOffsets[0], &magScales[0], &zeroOffsets[0]);
  reportToUser("Done!\n");
  //go save to eeprom

}

void MPU9250_IMU::zeroOrigin(float *origin){
  //ask user to put the craft on level ground
  reportToUser("Please place the craft on level ground!\n");
  for(int i = 0; i < 10; i ++){
    reportToUser(10 - i);
    for(int i = 0 ; i < 3; i ++){
      delay(1000/3);
      reportToUser(".");
    }
  }
  //go grab data for a couple seconds
  long start = millis();
  MPU_IMU_DATA tempData;
  float runningOrientation[3] = {0,0,0};
  int numUpdates = 0;

  while(millis() - start < ORIENTATION_CALIBRATION_MILLIS){
    getData(&tempData);
    //reportToUser(tempData.euler[0]);
    numUpdates ++;
    for(int i = 0; i < 3; i ++){
      runningOrientation[i] += tempData.euler[i];
    }
    delayMicroseconds(microsBetweenUpdates);
  }
  for(int i = 0; i < 3; i ++){
    runningOrientation[i] /= numUpdates;
    origin[i] = -runningOrientation[i];
  }
  reportToUser("Orientation Offsets: X:" + String(origin[0]) + " Y:" + String(origin[1]) + " Z:" + String(origin[2]) + "\n");
}

void MPU9250_IMU::zeroGyro(float *gyroRateOffsets){
  //ask the user to hold the device still
  reportToUser("Hold the device still. Minimize all external influences on the IMU\n");
  //begin a countdown
  reportToUser("Begining calibration in 5");
  //print out some dots
  for(int i = 0; i < 5; i ++){
    reportToUser(5 - i);
    for(int i = 0 ; i < 3; i ++){
      delay(1000/3);
      reportToUser(".");
    }
  }

  reportToUser("Beginning");
  //begin a running tally of gyro rate of rotations for 5 seconds
  float rateOffsets[3]= {0,0,0};
  long lastTimeNotified = millis();
  int numUpdates = 0;
  long startTime = millis();
  while(millis() - startTime < GYRO_CALIBRATION_MILLIS){
    delayMicroseconds(microsBetweenUpdates);
    numUpdates ++;
    if(millis() - lastTimeNotified > 5000){
      lastTimeNotified = millis();
      reportToUser(String(GYRO_CALIBRATION_MILLIS/1000 - (millis() - startTime)/1000) + " Seconds left\n");
    }
    mpu.readAll(&sensorData);

    rateOffsets[0] += sensorData.gyro[0];
    rateOffsets[1] += sensorData.gyro[1];
    rateOffsets[2] += sensorData.gyro[2];
  }
  gyroRateOffsets[0] = (rateOffsets[0]/numUpdates);
  gyroRateOffsets[1] = (rateOffsets[1]/numUpdates);
  gyroRateOffsets[2] = (rateOffsets[2]/numUpdates);
}

void MPU9250_IMU::zeroAccel(float * accelOffsets){
  //ask the user to, one axis at a time, to rotate the imu slowly
  //the idea is to get both a scale factor and a offset
  reportToUser("Accel Calibration\n");
  reportToUser("Please slowly rotate the IMU, one axis at a time, for 40 seconds\n");
  long startTime = millis();
  float minValues[3] = {0,0,0};
  float maxValues[3] = {0,0,0};
  long lastTimeNotified = millis();
  while(millis() - startTime < ACCEL_CALIBRATION_MILLIS){
    delayMicroseconds(microsBetweenUpdates);
    if(millis() - lastTimeNotified > 5000){
      lastTimeNotified = millis();
      reportToUser(String(ACCEL_CALIBRATION_MILLIS/1000 - (millis() - startTime)/1000) + " Seconds left\n");
    }
    mpu.readAll(&sensorData);
    //check for mins
    if(sensorData.accel[0] < minValues[0]){
      minValues[0] = sensorData.accel[0];
    }
    if(sensorData.accel[1] < minValues[1]){
      minValues[1] = sensorData.accel[1];
    }
    if(sensorData.accel[2] < minValues[2]){
      minValues[2] = sensorData.accel[2];
    }

    if(sensorData.accel[0] > maxValues[0]){
      maxValues[0] = sensorData.accel[0];
    }
    if(sensorData.accel[1] > maxValues[1]){
      maxValues[1] = sensorData.accel[1];
    }
    if(sensorData.accel[2] > maxValues[2]){
      maxValues[2] = sensorData.accel[2];
    }
    //print out the min max values
  }

  //find offsets
  float midPoints[3] = {0,0,0};
  for(int i = 0; i < 3; i ++){
    midPoints[i] = (maxValues[i] + minValues[i])/2;
  }
  accelOffsets[0] = midPoints[0];
  accelOffsets[1] = midPoints[1];
  accelOffsets[2] = midPoints[2];
  //print out min max for safety check
  reportToUser("X: " + String(minValues[0]) + " " + String(maxValues[0]) + " Offset: " + String(accelOffsets[0]) + "\n");
  reportToUser("Y: " + String(minValues[1]) + " " + String(maxValues[1]) + " Offset: " + String(accelOffsets[1]) + "\n");
  reportToUser("Z: " + String(minValues[2]) + " " + String(maxValues[2]) + " Offset: " + String(accelOffsets[2]) + "\n");
}

void MPU9250_IMU::zeroMag(float *offsets, float *scale){
  reportToUser("Mag Calibration\n");
  reportToUser("Please slowly rotate the IMU, ideally in a figure 8 pattern, for 60 seconds\n");
  long startTime = millis();
  float minValues[3] = {sensorData.mag[0],sensorData.mag[1],sensorData.mag[2]};
  float maxValues[3] = {sensorData.mag[0],sensorData.mag[1],sensorData.mag[2]};
  long lastTimeNotified = millis();
  while(millis() - startTime < MAG_CALIBRATION_MILLIS){
    if(millis() - lastTimeNotified > 5000){
      lastTimeNotified = millis();
      reportToUser(String(MAG_CALIBRATION_MILLIS/1000 - (millis() - startTime)/1000) + " Seconds left");
    }
    delayMicroseconds(microsBetweenUpdates);
    mpu.readAll(&sensorData);
    //check for mins
    if(sensorData.mag[0] < minValues[0]){
      minValues[0] = sensorData.mag[0];
    }
    if(sensorData.mag[1] < minValues[1]){
      minValues[1] = sensorData.mag[1];
    }
    if(sensorData.mag[2] < minValues[2]){
      minValues[2] = sensorData.mag[2];
    }

    if(sensorData.mag[0] > maxValues[0]){
      maxValues[0] = sensorData.mag[0];
    }
    if(sensorData.mag[1] > maxValues[1]){
      maxValues[1] = sensorData.mag[1];
    }
    if(sensorData.mag[2] > maxValues[2]){
      maxValues[2] = sensorData.mag[2];
    }
  }
  reportToUser("Mag X Min, Max: " + String(maxValues[0]) + " " + String(minValues[0]) + "\n");
  reportToUser("Mag Y Min, Max: " + String(maxValues[1]) + " " + String(minValues[1]) + "\n");
  reportToUser("Mag Z Min, Max: " + String(maxValues[2]) + " " + String(minValues[2]) + "\n");

  //find offsets
  float midPoints[3] = {0,0,0};
  for(int i = 0; i < 3; i ++){
    midPoints[i] = (maxValues[i] + minValues[i])/2;
  }
  offsets[0] = midPoints[0];
  offsets[1] = midPoints[1];
  offsets[2] = midPoints[2];
  //scale everything according to the x axis
  float yAxisScale = (maxValues[0] - midPoints[0])/(maxValues[1] - midPoints[1]);
  float zAxisScale = (maxValues[0] - midPoints[0])/(maxValues[2] - midPoints[2]);
  scale[0] = 1.0;
  scale[1] = yAxisScale;
  scale[2] = zAxisScale;
  reportToUser("Mag X Offset: " + String(offsets[0]) + " Scale: " + String(scale[0]) + "\n");
  reportToUser("Mag Y Offset: " + String(offsets[1]) + " Scale: " + String(scale[1]) + "\n");
  reportToUser("Mag Z Offset: " + String(offsets[2]) + " Scale: " + String(scale[2]) + "\n");
}

int MPU9250_IMU::getData(MPU_IMU_DATA *data){
  update();
  //make a copy
  for(int i = 0; i < 3; i ++){
      data->accel[i] = currentData.accel[i];
      data->rateOfRotation[i] = currentData.rateOfRotation[i];
      data->euler[i] = currentData.euler[i];
      data->mag[0] = currentData.mag[0];
      data->mag[1] = currentData.mag[1];
      data->mag[2] = currentData.mag[2];
  }
  data->euler[0] += zeroOriginOffsets[0];
  data->euler[1] += zeroOriginOffsets[1];
  data->euler[2] += zeroOriginOffsets[2];
  if(data->euler[2] > 360){
    data->euler[2] -= 360;
  }
  else if(data->euler[2] < 0){
    data->euler[2] += 360;
  }
  //data = &currentData;
  //reportToUser(data->accel[2]);
  return 0;
}

int MPU9250_IMU::begin(int updateRate, int eepromOffset){
  //userCallback = reportToUser;
  microsBetweenUpdates = (long) (1000000.0/updateRate +.5);
  Serial.println(microsBetweenUpdates);
  delay(1000);
  int mpuStatusCode = mpu.begin(MPU9250::MPU9250_GYRO_RANGE_2000_DPS,MPU9250::MPU9250_ACCEL_RANGE_8_GPS);
  if(mpuStatusCode != 0){
    return mpuStatusCode;
  }
  eepromCalibrationOffset = eepromOffset;
  //calculate
  //load in configuration
  //TODO
  //get basis
  //use accel to find default orientation
  //get initialization data
  loadIMUCalibrationDataFromEEPROM(eepromCalibrationOffset);
  mpu.readAll(&sensorData);
  //set the current data stuffs
  #ifdef LOW_PASS_ACCEL_FILTER
    for(int i = 0; i < 3; i ++){
        //do this for the low pass accel filter
        currentData.accel[i] = sensorData.accel[i];
        currentData.mag[i] = sensorData.mag[i];
    }
  #endif
  normalizeAccel();
  normalizeMag();
  //computeHeading();
  currentData.euler[0] = normalizedAccelDegrees[0];
  currentData.euler[1] = normalizedAccelDegrees[1];
  currentData.euler[2] = 0;
  //currentData.euler[2] = cu;
  //reportToUser("Initializing Z Heading to: " + currentData)
  //reportToUser(currentData.euler[0]);
  microsAtLastUpdate = micros();
  delay(6);
  update();
  return 0;
}

void MPU9250_IMU::update(){
  //check if elapsed time
  long curMicros = micros();
  if(curMicros - microsAtLastUpdate > microsBetweenUpdates){
    //get new data
    mpu.readAll(&sensorData);
    //reportToUser(sensorData.accel[2]);
    //normalize the gyro
    normalizeAccel();
    normalizeGyro();
    normalizeMag();
    //begin sensor fusion
    sensorFusion();
    currentData.accel[0] = sensorData.accel[0];
    currentData.accel[1] = sensorData.accel[1];
    currentData.accel[2] = sensorData.accel[2];
    currentData.rateOfRotation[0] = sensorData.gyro[0];
    currentData.rateOfRotation[1] = sensorData.gyro[1];
    currentData.rateOfRotation[2] = sensorData.gyro[2];
    currentData.sysTimestamp = millis();
    currentData.mag[0] = normalizedMagDegrees[0];
    currentData.mag[1] = normalizedMagDegrees[1];
    currentData.mag[2] = normalizedMagDegrees[2];
    microsAtLastUpdate = curMicros;
  }

}

void MPU9250_IMU::sensorFusion(){
  //TODO add tilt compensation on mag and z axis

  float linearAcceleration = sqrt(pow(sensorData.accel[0],2) + pow(sensorData.accel[1],2) + pow(sensorData.accel[2],2));
  if(linearAcceleration < 1.1){
    //reportToUser("Mode: Complementary");
    currentData.euler[0] = (currentData.euler[0] + normalizedGyroDegrees[0]) * COMPLEMENTARY_K_VALUE + (1.0 - COMPLEMENTARY_K_VALUE) * normalizedAccelDegrees[0];
    currentData.euler[1] = (currentData.euler[1] + normalizedGyroDegrees[1]) * COMPLEMENTARY_K_VALUE + (1.0 - COMPLEMENTARY_K_VALUE) * normalizedAccelDegrees[1];
  }
  else{
    currentData.euler[0] = (currentData.euler[0] + normalizedGyroDegrees[0]);
    currentData.euler[1] = (currentData.euler[1] + normalizedGyroDegrees[1]);
  }
  //mag
  fuseHeading();
  //add offsets

  //do one last check on heading


}

void MPU9250_IMU::fuseHeading(){
  //TODO tilt compensation!
  /*
  This?
  magComp[0] = (magVector[0]*cosPitch) + (magVector[1]*sinRoll*sinPitch) + (magVector[2]*sinPitch*cosRoll);
  magComp[1] = (magVector[1]*cosRoll) - (magVector[2]*sinRoll);
  magComp[2] = -(magVector[0]*sinPitch) + (magVector[1]*sinRoll*sinPitch) + (magVector[2]* cosRoll*cosPitch);
  */
  float currentOrientation[3] = {currentData.euler[0],currentData.euler[1],currentData.euler[2]};
  float magneticHeading = computeHeading(&currentOrientation[0]);
  //float tiltCorrectedMag[3];
  //tiltCorrectedMag[0] = (normalizedMagDegrees[0] * cos(currentData.euler[1]/RADIANS_TO_DEGREES) + normalizedMagDegrees[0])
  float gyroAddition = currentData.euler[2] + normalizedGyroDegrees[2];
  //check the relation
  if(fabs(gyroAddition - magneticHeading) > 180){
    if(gyroAddition > magneticHeading){
      magneticHeading += 360;
    }
    else{
      magneticHeading -= 360;
    }
  }
  //then add them
  currentData.euler[2] = gyroAddition * MAGNETIC_K_VALUE + (1.0 - MAGNETIC_K_VALUE)*magneticHeading;

}

float MPU9250_IMU::computeHeading(float *orientation){
  //TODO tilt compensation
  //tilt compensation
  orientation[0] = -orientation[0];
  orientation[1] = -orientation[1];
  float num = normalizedMagDegrees[2]*sin(orientation[1] / RADIANS_TO_DEGREES) - normalizedMagDegrees[1]*cos(orientation[1] / RADIANS_TO_DEGREES);
  float den = normalizedMagDegrees[0]*cos(orientation[0]/RADIANS_TO_DEGREES) + orientation[1]*sin(orientation[1]/RADIANS_TO_DEGREES)*sin(orientation[1]/RADIANS_TO_DEGREES) + normalizedMagDegrees[2]*sin(orientation[0]/RADIANS_TO_DEGREES)*cos(orientation[1]/RADIANS_TO_DEGREES);

  return atan2(num,den) * RADIANS_TO_DEGREES;

}

void MPU9250_IMU::normalizeAccel(){
  //transform the accel readings into degrees
  #ifdef LOW_PASS_ACCEL_FILTER
    for(int i = 0; i < 3; i ++){
      sensorData.accel[i] = currentData.accel[i] + MPU9250_ACCEL_LOW_PASS_K * (sensorData.accel[i] - currentData.accel[i]);
    }
  #endif
  normalizedAccelDegrees[0] = atan2f(sensorData.accel[1], sensorData.accel[2]) * RADIANS_TO_DEGREES;
  normalizedAccelDegrees[1] = atan2f(sensorData.accel[0], sensorData.accel[2]) * RADIANS_TO_DEGREES;
}

void MPU9250_IMU::normalizeGyro(){
  //transform the gyro readings into traveled degrees
  float deltaSeconds = sensorData.elapsedMicrosToRead/1000000.0;
  normalizedGyroDegrees[0] = (sensorData.gyro[0]) * deltaSeconds;
  normalizedGyroDegrees[1] = -(sensorData.gyro[1]) * deltaSeconds;
  normalizedGyroDegrees[2] = (sensorData.gyro[2]) * deltaSeconds;
}


//must be called after new orientation is computed!
void MPU9250_IMU::normalizeMag(){
  //TODO: normalize these based on orientation
  #ifdef LOW_PASS_MAG_FILTER
    for(int i = 0; i < 3; i ++){
      sensorData.mag[i] = currentData.mag[i] + MPU9250_MAG_LOW_PASS_K * (sensorData.mag[i] - currentData.mag[i]);
    }
  #endif
  //let's go let's go let's go
  normalizedMagDegrees[0] = sensorData.mag[0]; //*cos((currentData.euler[0])/RADIANS_TO_DEGREES); // + scaleData->mag.y * sin((runningData.orientation.x)/RADIANS_TO_DEGREES) * sin((runningData.orientation.y)/RADIANS_TO_DEGREES) + scaleData->mag.z * cos((runningData.orientation.x)/RADIANS_TO_DEGREES) * sin((runningData.orientation.y)/RADIANS_TO_DEGREES);
  normalizedMagDegrees[1] = sensorData.mag[1]; //*cos((currentData.euler[0])/RADIANS_TO_DEGREES); // - scaleData->mag.z * sin((runningData.orientation.y)/RADIANS_TO_DEGREES);
  normalizedMagDegrees[2] = sensorData.mag[2];
}
