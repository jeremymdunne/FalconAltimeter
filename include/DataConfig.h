#ifndef _DATA_CONFIG_H_
#define _DATA_CONFIG_H_

#define SEND_FILE "SF"
#define SEND_FAT_TABLE "ST"
#define WRITE_CONFIGURATION "WC"
#define ERASE_FILE "FE"
#define SEND_CONFIGURATION "SC"
#define SEND_ERROR "ERR:"

//#define STORE_DATA

#define MEASURE_IMU
#define MEASURE_PRESSURE
//#define MEASURE_GPS


//these are flags that can be commented out as needed
//#define GPS
#define ACCEL
#define GYRO
#define PRESSURE_ALT
//#define RADIO_TELEMETRY
#define SERIAL_FEED_THROUGH

enum FLIGHT_PHASES{
  WAITING_FOR_LAUNCH,BOOST_PHASE,COAST_PHASE,APOGEE,RECOVERY,LANDING,STANDBY
};
//flight
#define FLIGHT_DATA_STORAGE_HERTZ_DURING_FLIGHT 20
#define GPS_DATA_STORAGE_HERTZ_DURING_FLIGHT 1
#define FLIGHT_ESTIMATION_STORAGE_HERTZ_DURING_FLIGHT 2

#define FLIGHT_DATA_TELEMETRY_HERTZ_DURING_FLIGHT 2
#define GPS_DATA_TELEMETRY_HERTZ_DURING_FLIGHT .5
#define FLIGHT_ESTIMATION_TELEMETRY_HERTZ_DURING_FLIGHT .5
//recovery
#define FLIGHT_DATA_STORAGE_HERTZ_DURING_RECOVERY 2
#define GPS_DATA_STORAGE_HERTZ_DURING_RECOVERY 1
#define FLIGHT_ESTIMATION_STORAGE_HERTZ_DURING_RECOVERY 0

#define FLIGHT_DATA_TELEMETRY_HERTZ_DURING_RECOVERY 2
#define GPS_DATA_TELEMETRY_HERTZ_DURING_RECOVERY .5
#define FLIGHT_ESTIMATION_TELEMETRY_HERTZ_DURING_RECOVERY 0

#define PRE_LAUNCH_BUFFER_MILLIS 5000

#define PRE_LAUNCH_RAW_FLIGHT_BUFFER_SIZE PRE_LAUNCH_BUFFER_MILLIS/1000*FLIGHT_DATA_STORAGE_HERTZ_DURING_FLIGHT
#define PRE_LAUNCH_GPS_BUFFER_SIZE PRE_LAUNCH_BUFFER_MILLIS/1000*GPS_DATA_STORAGE_HERTZ_DURING_FLIGHT



//launch + landing conditions to be determined in the flash
//launch conditions
#define LAUNCH_ACCELERATION_DETECTION_THRESHOLD 20
#define LAUNCH_DETECTION_WAIT_MILLIS 200
#define LAUNCH_DETECTION_ALTITUDE_THRESHOLD 2

#define LANDING_DETECTION_MINUMUM_ELAPSED_FLIGHT_TIME 4000
#define LANDING_DETECTION_MINUMUM_VELOCITY 1


//landing conditions
#define LANDING_MAX_SPEED 1
#define LANDING_DURATION 5

#define PRE_LAUNCH_BUFFER_MILLIS 5000

#include <Arduino.h>


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
#define FLIGHT_RECORDER_ACCEL_DATA_SCALE_FACTOR       300
#define FLIGHT_RECORDER_ACCEL_DATA_RECORDING_HERTZ    20

#define FLIGHT_RECORDER_GYRO_DATA_BYTE_SIZE_PER_AXIS 3
#define FLIGHT_RECORDER_GYRO_DATA_SCALE_FACTOR       1
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

struct RocketData{
  uint8 tag;
  ulong timeStamp;
  float *data; //preserve memory if possible, write over this if needed
  uint8 numMembers = 2; //should reflect the number of members in data[]
};

struct GeneralEncodingStructure{
  uint8 tag;
  ulong timeStamp;

};

#endif
