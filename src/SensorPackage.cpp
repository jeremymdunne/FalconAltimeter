#include <SensorPackage.h>

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
  accelData.timeStamp = millis();
  gyroData.timeStamp = millis();
  magData.timeStamp = millis();
  accelData.tag = FLIGHT_ACCEL_DATA_TAG;
  gyroData.tag = FLIGHT_GYRO_DATA_TAG;
  magData.tag = FLIGHT_MAG_DATA_TAG;
  accelData.data[0] = imuData.accel[0];
  accelData.data[1] = imuData.accel[1];
  accelData.data[2] = imuData.accel[2];
  gyroData.data[0] = imuData.rateOfRotation[0];
  gyroData.data[1] = imuData.rateOfRotation[1];
  gyroData.data[2] = imuData.rateOfRotation[2];
  magData.data[0] = imuData.mag[0];
  magData.data[1] = imuData.mag[1];
  magData.data[2] = imuData.mag[2];
  return 0;
}

int SensorPackage::updatePressureAlt(){
  #ifdef BMP_280_PRESSURE_ALT_SENSOR
    //Serial.println("Updating pressure");
    pressureAltData.timeStamp = millis();
    pressureAltData.tag = FLIGHT_PRESSURE_ALTITUDE_DATA_TAG;
    pressureAltData.data[0] = bmp.getAltitudeFromBaselinePressure(baselinePressure);
    return 0;
  #endif
  return -1;
}

int SensorPackage::updateGPS(){
  return 0;
}

int SensorPackage::getNewSensorData(RocketData *targetArray, int maxData){
  int counter = 0;
  #ifdef MEASURE_IMU
    if(newImuData){
      if(maxData - counter >= 3){
        targetArray[counter] = accelData;
        targetArray[counter+1] = gyroData;
        targetArray[counter+2] = magData;
        counter += 3;
        newImuData = false;
      }
    }
  #endif
  #ifdef MEASURE_PRESSURE
    if(newPressureData){
        //Serial.println("Returning new pressure data");
        if(maxData - counter >= 1){
          //Serial.println("Wrote");
          targetArray[counter] = pressureAltData;
          counter +=1;
          newPressureData = false;
        }
    }
  #endif
  return counter;
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
