#include <SensorPackage.h>

int SensorPackage::setSensorDataCollectionRate(int dataFlag, float hertz){
  switch(dataFlag){
    case(SENSOR_PRIMARY_IMU_TAG):
      imuUpdater.setUpdateFrequency(hertz);
      break;
    case(SENSOR_PRIMARY_GPS_TAG):
      gpsUpdater.setUpdateFrequency(hertz);
      break;
    case(SENSOR_PRIMARY_BAROMETER_TAG):
      pressureUpdater.setUpdateFrequency(hertz);
      break;
  }
  return 0;
}

int SensorPackage::initIMU(){
  //todo
  return imu.begin();
}

int SensorPackage::initPressureSensor(){
  //todo
  #ifdef BMP_280_PRESSURE_ALT_SENSOR
    //Serial.println("Initing pressure");
    return this->bmp.begin();

  #endif
  return 0;
}

int SensorPackage::initGps(){
  //todo
  return 0;
}

int SensorPackage::init(){
  //Send error if some sensors fail
  #ifdef MEASURE_PRESSURE
    if(initPressureSensor() != 0) return PRESSURE_SENSOR_INIT_FAILURE;
    pressureUpdater.setUpdateFrequency(PRESSURE_SENSOR_UPDATE_HERTZ);
  #endif
  #ifdef MEASURE_GPS
    if(initGps() != 0) return GPS_SENSOR_INIT_FAILURE;
    gpsUpdater.setUpdateFrequency(GPS_UPDATE_HERTZ);
  #endif
  #ifdef MEASURE_IMU
    //Serial.println("Initing IMU!");
    if(initIMU() != 0) return IMU_SENSOR_INIT_FAILURE;
    imuUpdater.setUpdateFrequency(IMU_UPDATE_HERTZ);
  #endif

  //init all the time based updaters
  return 0;
}

int SensorPackage::updateIMU(){
  imu.getData(&imuData);
  //Serial.println("Getting IMU Data!");
  runningData.imuData.timestamp = millis();

  runningData.imuData.acceleration[0] = imuData.accel[0] * 9.81;
  runningData.imuData.acceleration[1] = imuData.accel[1] * 9.81;
  runningData.imuData.acceleration[2] = imuData.accel[2] * 9.81;
  runningData.imuData.angularRate[0] = imuData.rateOfRotation[0];
  runningData.imuData.angularRate[1] = imuData.rateOfRotation[1];
  runningData.imuData.angularRate[2] = imuData.rateOfRotation[2];
  runningData.imuData.magneticField[0] = imuData.mag[0];
  runningData.imuData.magneticField[1] = imuData.mag[1];
  runningData.imuData.magneticField[2] = imuData.mag[2];
  return 0;
}

int SensorPackage::updatePressureAlt(){
  //Serial.println("Updating pressure");
  runningData.barometricData.timestamp = millis();
  runningData.barometricData.altitude = bmp.getAltitudeFromBaselinePressure(baselinePressure);
  runningData.barometricData.pressure = bmp.getPressure();
  runningData.barometricData.temperature = bmp.getTemperature();
  return 0;
}

int SensorPackage::updateGPS(){
  return 0;
}

int SensorPackage::getNewSensorData(RawRocketSensorData *target){
  //copy over the data, don't give them the target
  copyBarometricData(&target->barometricData, &runningData.barometricData);
  copyIMUdata(&target->imuData, &runningData.imuData);
  copyGPSdata(&target->gpsData, &target->gpsData);
  return 0;
}


//return whether new data is available, 1 == true, 0 == false
//all new data is taged in a global array so to know what data is new and not
int SensorPackage::update(){
  //check all the updaters
  int numUpdates = 0;
  #ifdef MEASURE_IMU
    if(imuUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      //handle imu update
      updateIMU();
      newImuData = true;
      numUpdates += 3; //accel + gyro + mag
    }
  #endif
  #ifdef MEASURE_PRESSURE
    //Serial.println("Checking update!");
    if(pressureUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      //Serial.println("Running Pressure Update!");
      updatePressureAlt();
      newPressureData = true;
      numUpdates ++;
      //Serial.println("Update done!");
    }
  #endif
  #ifdef MEASURE_GPS
    if(gpsUpdater.update() == TIME_UPDATER_UPDATE_REQUIRED){
      updateGPS();
      newGpsData = true;
      numUpdates ++;
    }
  #endif
  return numUpdates;
}
