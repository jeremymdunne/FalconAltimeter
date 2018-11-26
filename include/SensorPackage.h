#ifndef _SENSOR_PACKAGE_H_
#define _SENSOR_PACKAGE_H_

#include "DataConfig.h"
#include <UpdateScheduler.h>
#include <BMP280.h>
//flags for sensors, used in return flags and sensor type flags


// update rates in hertz for major sensors
#define SENSOR_PACKAGE_IMU_UPDATE_FREQUENCY 400
#define SENSOR_PACKAGE_PRESSURE_UPDATE_FREQUENCY 60
#define SENSOR_PACKAGE_GPS_UPDATE_FREQUENCY 1
#define SENSOR_PACKAGE_WANTED_UPDATE_FREQUENCY 400

#define GPS_SENSOR_INIT_FAILURE -70
#define IMU_SENSOR_INIT_FAILURE -71
#define PRESSURE_SENSOR_INIT_FAILURE -72

#define BMP_280_PRESSURE_ALT_SENSOR
#ifdef BMP_280_PRESSURE_ALT_SENSOR
  #define PRESSURE_SENSOR_TIMEOUT_MILLIS 300
  #define PRESSURE_SENSOR_COMMUNICATION_SPEED
  #define PRESSURE_SENSOR_UPDATE_HERTZ 40
#endif

#define UBLOX_GPS_SENSOR
#ifdef UBLOX_GPS_SENSOR
  #define GPS_TIMEOUT_MILLIS 2000
  #define GPS_COMMUNICATION_SPEED 4800
  #define GPS_UPDATE_HERTZ 1
#endif

#define MPU9250_IMU_SENSOR
#ifdef MPU9250_IMU_SENSOR
  #define IMU_TIMEOUT_MILLIS 10
  #define IMU_COMMUNICATION_SPEED
  #define IMU_UPDATE_HERTZ 300
#endif

//sensor dependent flags
#define SENSOR_PACKAGE_GPS_NOT_LOCKED -50

//general sensor error flags
#define SENSOR_PACKAGE_SENSOR_COMMUNICATION_FAILURE -51
#define SENSOR_PACKAGE_SENSOR_LATENCY_REACHED -52

class SensorPackage{
public:
  //returns codes for successfully initialized sensors
  int init();
  //updates sensors if necessary and returns codes for available new sensor data
  int update();
  //returns if a sensor is working, usefull for gps and other dependent sensors
  int getSensorStatus(int sensorFlag);
  //gets the sensor data in the form of
  int getSensorData(int sensorFlag, RocketData *target);
  //disables a certain sensor, removes it from the update queue
  int disableSensor(int sensorFlag);
  //enables a certain sensor
  int enableSensor(int sensorFlag);
private:
  byte sensorEnabledStatus; //made up of the sensor flags to show enabled, disabled means a 0
  UpdateScheduler imuUpdater, pressureUpdater, gpsUpdater;
  RocketData accelData, gyroData, magData, gpsData, pressureAltData;
  int initGps();
  int initIMU();
  int initPressureSensor();
  bool newDataAvail[3] = {0,0,0}; //imu, pressureAlt, gps
  #ifdef BMP_280_PRESSURE_ALT
    BMP280 bmp280;
  #endif
  #ifdef MPU9250_IMU
    MPU9250_IMU imu;
  #endif
  #ifdef UBLOX_GPS
    UBLOX_GPS gps;
  #endif
};

int SensorPackage::initIMU(){
  //todo
  return 0;
}

int SensorPackage::initPressureSensor(){
  //todo
  return 0;
}

int SensorPackage::initGps(){
  //todo
  return 0;
}

int SensorPackage::init(){
  //Send error if some sensors fail
  if(initPressureSensor() != 0) return PRESSURE_SENSOR_INIT_FAILURE;
  if(initGps() != 0) return GPS_SENSOR_INIT_FAILURE;
  if(initIMU() != 0) return IMU_SENSOR_INIT_FAILURE;
  //init all the time based updaters
  imuUpdater.setUpdateFrequency(IMU_UPDATE_HERTZ);
  pressureUpdater.setUpdateFrequency(PRESSURE_SENSOR_UPDATE_HERTZ);
  gpsUpdater.setUpdateFrequency(GPS_UPDATE_HERTZ);
  return 0;
}

//return whether new data is available, 1 == true, 0 == false
//all new data is taged in a global array so to know what data is new and not
int SensorPackage::update(){
  //check all the updaters

  return 0;
}

#endif
