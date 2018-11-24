#ifndef _SENSOR_PACKAGE_H_
#define _SENSOR_PACKAGE_H_
#include <BMP280.h>
#include "DataConfig.h"
//flags for sensors, used in return flags and sensor type flags


// update rates in hertz for major sensors
#define SENSOR_PACKAGE_IMU_UPDATE_FREQUENCY 400
#define SENSOR_PACKAGE_PRESSURE_UPDATE_FREQUENCY 60
#define SENSOR_PACKAGE_GPS_UPDATE_FREQUENCY 1
#define SENSOR_PACKAGE_WANTED_UPDATE_FREQUENCY 400


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
  ulong lastSensorReadMillis[6];
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

#endif
