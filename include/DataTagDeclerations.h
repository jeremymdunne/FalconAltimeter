#ifndef _DATA_TAG_DECLERATIONS_H_
#define _DATA_TAG_DECLERATIONS_H_

/*
This includes the tags/flags of all data used by the flight controller
Mainly used in the storage and recall of data
*/


//these are used to identify sensors, not data
#define SENSOR_PRIMARY_IMU_TAG                          300
#define SENSOR_PRIMARY_BAROMETER_TAG                    301
#define SENSOR_PRIMARY_GPS_TAG                          302


//Raw sensor data flags
#define RAW_ACCEL_DATA_TAG                              1
#define RAW_ANGULAR_RATE_DATA_TAG                       2
#define RAW_MAGNETIC_FIELD_DATA_TAG                     3
#define RAW_TEMPERATURE_DATA_TAG                        4
#define RAW_PRESSURE_TAG                                5
#define RAW_PRESSURE_ALTITUDE_TAG                       6
#define RAW_GPS_DATA_TAG                                7



//Filtered Data tags
#define FILTERED_ORIENTATION_DATA_TAG                   20
#define FILTERED_ALTITUDE_DATA_TAG                      21
#define FILTERED_VELOCITY_DATA_TAG                      22
#define FILTERED_ACCELERATION_DATA_TAG                  23


//Event tags
#define LAUNCH_PAHSE_DETECTION_EVENT_TAG                50
#define COAST_PHASE_DETECTION_EVENT_TAG                 51
#define APOGEE_PHASE_DETECTION_EVENT_TAG                52
#define LANDING_PHASE_DETECTION_EVENT_TAG               53


//Recovery events
#define DROGUE_CHUTE_DEPLOYMENT_EVENT_TAG               70
#define MAIN_CHUTE_DEPLOYMENT_EVENT_TAG                 71



#endif
