#ifndef _HEADER_H_
#define _HEADER_H_

/*
This file contains all program-wide definitions, declerations, constants, etc.
*/

#include <Arduino.h> //for types

//in case of axis alteration
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

/*
Generic phases for the rocket to be in
*/
enum Flight_Phase{
  WAITING_FOR_LAUNCH_PHASE, BOOST_PHASE, COAST_PHASE, APOGEE_PHASE, RECOVER_PHASE, LANDING_PHASE, STANDOWN_PHASE, DATA_RECOVERY_PHASE
};

/*
standard barometric data from sensor.
the altitude is not to be trusted by itself, expecially at near-mach velocities
*/
struct Barometric_Data{
  float pressure = 0; //pascals
  float temperature = 0; //celcius
  float altitude = 0; //meters above sea level
  long timestamp = 0; //when data was collected in milliseconds
};


/*
IMU data
standard data collected from a sensor
includes sensor fusion euler data
*/
struct IMU_Data{
  float acceleration[3] = {0,0,0}; //in m/s/s
  float angularRate[3] = {0,0,0}; //in deg/s
  float magneticField[3] = {0,0,0}; //in relative units.... TODO set to actual units
  float euler[3] = {0,0,0}; //in deg
  long timestamp = 0;
};

/*
GPS data
stores lat, lon in String format
*/
struct GPS_Data{
  String latitude; //format to be determined  TODO
  String longitude;
  long timestamp;
};


/*
Overall state of the rocket
Will add extra flags and data when applicable
*/
struct Rocket_State{
  Barometric_Data primaryBarometricData;
  IMU_Data primaryIMUdata;
  GPS_Data primaryGPSdata;

  long updateMicros = 1000; //the average loop time of the program

  Flight_Phase currentFlightPhase;
};

/*
References every individual sensor on the rocket
Is fused into 'global' values when placed in Rocket_State
*/
struct RawRocketSensorData{
  Barometric_Data barometricData;
  IMU_Data imuData;
  GPS_Data gpsData;
};


namespace RocketHelper{
  /*
  Helper function for copying barometric data
  @return int 0 = data copied, 1 = no need to copy data (timestamps same)
  */
  static int copyBarometricData(Barometric_Data *target, Barometric_Data *source){
    //do a sanity check on time stamp, if same, don't bother copying
    if(target->timestamp == source->timestamp){
      return 1;
    }
    target->pressure = source->pressure;
    target->temperature = source->temperature;
    target->altitude = source->altitude;
    target->timestamp = source->timestamp;
    return 0;
  }

  static int copyIMUdata(IMU_Data *target, IMU_Data *source){
    //do a sanity check on time stamp, if same, don't bother copying
    if(target->timestamp == source->timestamp){
      return 1;
    }
    for(uint i = 0; i < 3; i ++){
      target->acceleration[i] = source->acceleration[i];
      target->angularRate[i] = source->angularRate[i];
      target->magneticField[i] = source->magneticField[i];
      target->euler[i] = source->euler[i];
    }
    target->timestamp = source->timestamp;
    return 0;
  }


  static int copyGPSdata(GPS_Data *target, GPS_Data *source){
    //do a sanity check on time stamp, if same, don't bother copying
    if(target->timestamp == source->timestamp){
      return 1;
    }
    target->latitude = source->latitude;
    target->longitude = source->longitude;
    target->timestamp = source->timestamp;
    return 0;
  }
  //helper function to copy rocket state data
  static int copyRocketState(Rocket_State *target, Rocket_State *source){
    copyBarometricData(&target->primaryBarometricData, &source->primaryBarometricData);
    copyIMUdata(&target->primaryIMUdata,&source->primaryIMUdata);
    copyGPSdata(&target->primaryGPSdata, &source->primaryGPSdata);
    target->updateMicros = source->updateMicros;
    return 0;
  }

  static void printRocketState(Rocket_State *state){
    Serial.println("Pressure: " + String(state->primaryBarometricData.pressure));
    Serial.println("Temperature: " + String(state->primaryBarometricData.temperature));
    Serial.println("Pressure Altitude: " + String(state->primaryBarometricData.altitude));

    Serial.println("IMU Acceleration (XYZ): " + String(state->primaryIMUdata.acceleration[0]) + ";" + String(state->primaryIMUdata.acceleration[1]) + ";" + String(state->primaryIMUdata.acceleration[2]));


    Serial.println("Update Micros: " + String(state->updateMicros));

  }

}

#endif
