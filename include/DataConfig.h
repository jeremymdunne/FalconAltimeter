#ifndef _DATA_CONFIG_H_
#define _DATA_CONFIG_H_
#include <Arduino.h>

#define SEND_FILE "SF"

#define SEND_FAT_TABLE "ST"
#define WRITE_CONFIGURATION "WC"
#define ERASE_LAST_FILE "EL"
#define ERASE_FILE "FE"
#define ERASE_ALL_FILES "EA"
#define SEND_CONFIGURATION "SC"
#define SEND_ERROR "ERR:"

#define STORE_DATA
#define PRINT_STORAGE_BUFFERS

#define MEASURE_IMU
#define MEASURE_PRESSURE
//#define MEASURE_GPS

//#define RADIO_TELEMETRY
#define SERIAL_FEED_THROUGH

enum FLIGHT_PHASES{
  WAITING_FOR_LAUNCH,BOOST_PHASE,COAST_PHASE,APOGEE,RECOVERY,LANDING,STANDBY
};
//flight
#define PRE_LAUNCH_BUFFER_MILLIS 5000

#define PRE_LAUNCH_RAW_FLIGHT_BUFFER_SIZE PRE_LAUNCH_BUFFER_MILLIS/1000*FLIGHT_DATA_STORAGE_HERTZ_DURING_FLIGHT
#define PRE_LAUNCH_GPS_BUFFER_SIZE PRE_LAUNCH_BUFFER_MILLIS/1000*GPS_DATA_STORAGE_HERTZ_DURING_FLIGHT



//launch + landing conditions to be determined in the flash
//launch conditions
#define LAUNCH_ACCELERATION_DETECTION_THRESHOLD 2
#define LAUNCH_DETECTION_WAIT_MILLIS 200
#define LAUNCH_DETECTION_ALTITUDE_THRESHOLD_WITH_ACCELERATION_DETECTION .5
#define LAUNCH_DETECTION_ALTITUDE_WITHOUT_ACCELERATION_DETECTION 1.5

#define LANDING_DETECTION_MINUMUM_ELAPSED_FLIGHT_TIME 4000
#define LANDING_DETECTION_MINUMUM_VELOCITY 1


//landing conditions
#define LANDING_MAX_SPEED 1
#define LANDING_DURATION 5

#define PRE_LAUNCH_BUFFER_MILLIS 5000




//Storage methods
/*
All data is to be stored in binary format, ideally broken up into bytes
When storing data in the storage, a tag is used to define the data type and length, and all intertwined data dataMembers
The format is as follows in parsedcode

<tag>(1Byte),<TimeStamp>(3 bytes),<Data1>(Nbytes),<Data2>(XBytes)....

When reading the data, the tag is used in order to determine where and how to break up the data
as well as the meaning of all the data.

*/


//all data members share these parameters
#define FLIGHT_RECORDER_TIME_STAMP_BYTE_SIZE 3
#define FLIGHT_RECORDER_TIME_STAMP_SCALE_FACTOR 1
#define FLIGHT_RECORDER_TAG_BYTE_SIZE 1
#define FLIGHT_RECORDER_TAG_SCALE_FACTOR 1

//event/data specific definitions
#define FLIGHT_RECORDER_ACCEL_DATA_BYTE_SIZE_PER_AXIS 3
#define FLIGHT_RECORDER_ACCEL_DATA_SCALE_FACTOR       20
#define FLIGHT_RECORDER_ACCEL_DATA_RECORDING_HERTZ    20

#define FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS 3
#define FLIGHT_RECORDER_GYRO_DATA_SCALE_FACTOR       17
#define FLIGHT_RECORDER_GYRO_DATA_RECORDING_HERTZ    10

#define FLIGHT_RECORDER_MAG_DATA_BYTE_SIZE_PER_AXIS 3
#define FLIGHT_RECORDER_MAG_DATA_SCALE_FACTOR       1
#define FLIGHT_RECORDER_MAG_DATA_RECORDING_HERTZ    10

#define FLIGHT_RECORDER_GPS_DATA_BYTE_SIZE_PER_AXIS 3
#define FLIGHT_RECORDER_GPS_DATA_SCALE_FACTOR       1
#define FLIGHT_RECORDER_GPS_DATA_RECORDING_HERTZ  1

#define FLIGHT_RECORDER_PRESSURE_ALTITUDE_BYTE_SIZE 3
#define FLIGHT_RECORDER_PRESSURE_ALTITUDE_SCALE_FACTOR 550
#define FLIGHT_RECORDER_PRESSURE_ALTITUDE_DATA_RECORDING_HERTZ    5

#define FLIGHT_ACCEL_DATA_TAG                         1
#define FLIGHT_GYRO_DATA_TAG                          2
#define FLIGHT_MAG_DATA_TAG                           9
#define FLIGHT_PRESSURE_ALTITUDE_DATA_TAG             3
#define FLIGHT_GPS_DATA_TAG                           5
#define FLIGHT_ESTIMATION_DATA_TAG                    6

//tags 32-256 are dedicated to event tags
//general flight tags
#define FLIGHT_LAUNCH_DETECTION_EVENT_TAG             32
#define FLIGHT_COAST_DETECTION_EVENT_TAG              33
#define FLIGHT_APOGEE_DETECTION_EVENT_TAG             34
#define FLIGHT_MAIN_PARACHUTE_DEPLOYED_EVENT_TAG      35
#define FLIGHT_DROGUE_PARACHUTE_DEPLOYED_EVENT_TAG    36
#define FLIGHT_LANDING_DETECTION_EVENT_TAG            37

//modular specifics
#define FLIGHT_AIR_BRAKE_DEPLOYMENT_EVENT_TAG         60
#define FLIGHT_AIR_BRAKE_RETRACTED_EVENT_TAG          61

//supported data and type declarations
struct RocketData{
  ulong timeStamp;
  uint8 tag;
  float *data;
};

struct AccelData : public RocketData{
  AccelData(){
    data = new float[3];
    tag = FLIGHT_ACCEL_DATA_TAG;
  }
};

struct GyroData : public RocketData{
  GyroData(){
    data = new float[3];
    tag = FLIGHT_GYRO_DATA_TAG;
  }
};

struct MagData : public RocketData{
  MagData(){
    data = new float[3];
    tag = FLIGHT_MAG_DATA_TAG;
  }
};

struct PressureAltitudeData : public RocketData{
  PressureAltitudeData(){
    data = new float;
    tag = FLIGHT_PRESSURE_ALTITUDE_DATA_TAG;
  }
};

struct GpsData : public RocketData{
  GpsData(){
    data = new float[2];
    tag = FLIGHT_GPS_DATA_TAG;
  }
};

struct PreFlightAllocation : public RocketData{
  PreFlightAllocation(){
    data = new float[3]{0,0,0};
  }
};

struct EventData : public RocketData{
  EventData(){

  }
};



#endif
