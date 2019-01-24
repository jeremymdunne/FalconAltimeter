#ifndef _SENSOR_PACKAGE_H_
#define _SENSOR_PACKAGE_H_

/*
Handles all sensors on board, updates at pre-defined rates
*/

#include "header.h"
#include <UpdateScheduler.h>
#include <BMP280.h>
#include <MPU9250_IMU.h>
#include <DataTagDeclerations.h>
//flags for sensors, used in return flags and sensor type flags


// update rates in hertz for major sensors
#define SENSOR_PACKAGE_IMU_UPDATE_FREQUENCY 400
#define SENSOR_PACKAGE_PRESSURE_UPDATE_FREQUENCY 60
#define SENSOR_PACKAGE_GPS_UPDATE_FREQUENCY 1
#define SENSOR_PACKAGE_WANTED_UPDATE_FREQUENCY 400

#define GPS_SENSOR_INIT_FAILURE -70
#define IMU_SENSOR_INIT_FAILURE -71
#define PRESSURE_SENSOR_INIT_FAILURE -72



#define BMP_280_PRESSURE_ALT


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

//#define MEASURE_GPS
#define MEASURE_PRESSURE
#define MEASURE_IMU

using namespace RocketHelper;


class SensorPackage{
public:
  //returns codes for successfully initialized sensors
  int init();
  //updates sensors if necessary and returns codes for available new sensor data
  int update();
  int getNewSensorData(RawRocketSensorData *targetArray);
  int setSensorDataCollectionRate(int dataFlag, float hertz);
private:
  RawRocketSensorData runningData;
  byte sensorEnabledStatus; //made up of the sensor flags to show enabled, disabled means a 0
  UpdateScheduler imuUpdater, pressureUpdater, gpsUpdater, secondaryUpdater;

  int initGps();
  int initIMU();
  int initPressureSensor();
  int updateGPS();
  int updateIMU();
  int updatePressureAlt();
  bool newImuData = false, newGpsData = false, newPressureData = false;
  float baselinePressure = 101325;
  #ifdef BMP_280_PRESSURE_ALT
    BMP280 bmp;
  #endif
  MPU9250_IMU imu;
  MPU9250_IMU::MPU_IMU_DATA imuData;
  #ifdef UBLOX_GPS
    UBLOX_GPS gps;
  #endif
};



#endif
