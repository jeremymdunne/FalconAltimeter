#include <SensorPackage.h>

int SensorPackage::initIMU(){
  //todo
  return 0;
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
    pressureAltData.data = new float[2]{};
  #endif
  #ifdef MEASURE_GPS
    if(initGps() != 0) return GPS_SENSOR_INIT_FAILURE;
    gpsUpdater.setUpdateFrequency(GPS_UPDATE_HERTZ);
  #endif
  #ifdef MEASURE_IMU
    if(initIMU() != 0) return IMU_SENSOR_INIT_FAILURE;
    imuUpdater.setUpdateFrequency(IMU_UPDATE_HERTZ);
  #endif

  //init all the time based updaters
  return 0;
}

int SensorPackage::updateIMU(){
  return 0;
}

int SensorPackage::updatePressureAlt(){
  #ifdef BMP_280_PRESSURE_ALT_SENSOR
    //Serial.println("Updating pressure");
    pressureAltData.timeStamp = millis();
    pressureAltData.tag = FLIGHT_PRESSURE_ALTITUDE_DATA_TAG;
    pressureAltData.data[0] = bmp.getAltitudeFromBaselinePressure(baselinePressure);
    pressureAltData.data[1] = bmp.getPressure();
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
      if(newPressureData){
        //Serial.println("Returning new pressure data");
        if(maxData - counter > 1){
          //Serial.println("Wrote");
          targetArray[counter] = pressureAltData;
          counter +=1;
        }
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
