#define ALTITUDE_SCALE_FACTOR 550  //~2^24/(30480)
#define TIME_SCALE_FACTOR 1000
#define ACCELERATION_SCALE_FACTOR 300 //~2^16/(20*9.81)
#define VELOCITY_SCALE_FACTOR 90  //~2^16/(343*2)
#define ORIENTATION_SCALE_FACTOR

#define RAW_FLIGHT_DATA_DATA_DESCRIPTOR 1
#define RAW_FLIGHT_DATA_SIZE 9
#define FLIGHT_ESTIMATION_DATA_TAG 2
#define GPS_DATA_TAG 3


#define SEND_FILE "SF"
#define SEND_FAT_TABLE "ST"
#define WRITE_CONFIGURATION "WC"
#define ERASE_FILE "FE"
#define SEND_CONFIGURATION "SC"
#define SEND_ERROR "ERR:"

#define ACKNOWLEDGE "ACK"
#define FAILURE "FAIL"

#define START_STATEMENT_CHAR "$"
#define START_VERBOSE_CHAR "#"
#define HOST_START_CHAR "@"
#define NUMBER_OF_ATTEMPTS 5
#define COMPUTER_COMMUNICATION_TIMEOUT 5000
#define COMMS_FAILURE -57


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
#define LAUNCH_DETECTION_ACCELERATION_THRESHOLD_G 3
#define LAUNCH_DETECTION_ALTITUDE_THRESHOLD 1
#define LAUNCH_DETECTION_TIME .2

//landing conditions
#define LANDING_MAX_SPEED 1
#define LANDING_DURATION 5

#define PRE_LAUNCH_BUFFER_MILLIS 5000

#include <Arduino.h>

struct Raw_Flight_Data{
  ulong timeStamp;
  float pressureAlt;
  float linearAcceleration;
};

struct Rocket_Attitude_Estimations{
  ulong timeStamp;
  float xOrientation;
  float yOrientation;
  float zOrientation;
};

struct Flight_Estimations{
  ulong timeStamp;
  float estimatedVelocity;
  float estimatedApogee;
};

struct GPS_data{
  ulong timeStamp;
  String latitude;
  String longitude;
  ulong altitude;
  ulong gpsTime;
};

struct Event_Data{
  ulong timeStamp;
  int dataMembers;
  float *data;
};

enum SensorType{
  SENSOR_GPS, SENSOR_ACCEL, SENSOR_MAG, SENSOR_GYRO, SENSOR_PRESSURE_ALT, NOT_VALID_SENSOR_DATA
};

struct SensorData{
  SensorType sensorType = NOT_VALID_SENSOR_DATA;
  ulong sysTimeStamp;
  float *floatData;
  uint numData;
};

enum FlightEventTypes{
  LAUNCH_DETECTED, BOOST_END_DETECTED, APOGEE_DETECTED, DROGUE_CHUTE_DEPLOYMENT,
  MAIN_CHUTE_DEPLOYMENT, LANDING_DETECTED, FLIGHT_RECORDER_END, SENSOR_OVERLOAD,
  UNEXPECTED_EVENT, AIR_BRAKES_DEPLOYED, AIR_BRAKES_RETRACTED, NOT_VALID_EVENT
};

struct FlightEvent{
  FlightEventTypes type = NOT_VALID_EVENT;
  ulong sysTimeStamp;
  float *data;
  uint numData;
};
