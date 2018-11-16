#ifndef MPU9250_H_
#define MPU9250_H_
#include <Arduino.h>
#include <MPU9250.h>
#include <EEPROM.h>




#define RADIANS_TO_DEGREES 180.0/M_PI

//eprom storage stuffs
#define MPU9250_IMU_GYRO_X_OFFSET_EEPROM_OFFSET 0
#define MPU9250_IMU_GYRO_Y_OFFSET_EEPROM_OFFSET 1
#define MPU9250_IMU_GYRO_Z_OFFSET_EEPROM_OFFSET 2
#define MPU9250_IMU_ACCEL_X_OFFSET_EEPROM_OFFSET 3
#define MPU9250_IMU_ACCEL_Y_OFFSET_EEPROM_OFFSET 4
#define MPU9250_IMU_ACCEL_Z_OFFSET_EEPROM_OFFSET 5
#define MPU9250_IMU_MAG_X_OFFSET_EEPROM_OFFSET 6
#define MPU9250_IMU_MAG_Y_OFFSET_EEPROM_OFFSET 7
#define MPU9250_IMU_MAG_Z_OFFSET_EEPROM_OFFSET 8
#define MPU9250_IMU_MAG_X_SCALE_EEPROM_OFFSET 9
#define MPU9250_IMU_MAG_Y_SCALE_EEPROM_OFFSET 10
#define MPU9250_IMU_MAG_Z_SCALE_EEPROM_OFFSET 11
#define MPU9250_IMU_ORIENTATION_X_OFFSET_EEPROM_OFFSET 12
#define MPU9250_IMU_ORIENTATION_Y_OFFSET_EEPROM_OFFSET 13
#define MPU9250_IMU_ORIENTATION_Z_OFFSET_EEPROM_OFFSET 14

#define MPU9250_IMU_GYRO_OFFSET_SCALE_EEPROM 1000
#define MPU9250_IMU_ACCEL_OFFSET_SCALE_EEPROM 100
#define MPU9250_IMU_MAG_OFFSET_SCALE_EEPROM 10
#define MPU9250_IMU_MAG_SCALE_SCALE_EEPROM 1000
#define MPU9250_IMU_ORIENTATION_OFFSET_SCALE 100


//filters
#define MPU9250_ACCEL_LOW_PASS_K .1
#define MPU9250_MAG_LOW_PASS_K .1
//comment this out if not in a high acceleration environment
#define LOW_PASS_ACCEL_FILTER
#define LOW_PASS_MAG_FILTER

#define COMPLEMENTARY_K_VALUE .98
#define MAGNETIC_K_VALUE .985
#define DEFAULT_UPDATE_HTZ 200

//calibration
#define MAG_CALIBRATION_MILLIS 30000
#define GYRO_CALIBRATION_MILLIS 1000
#define ACCEL_CALIBRATION_MILLIS 40000
#define ORIENTATION_CALIBRATION_MILLIS 2000



/*
class that handles the sensor and all imu algorithms associated with
*/

class MPU9250_IMU{
public:
  struct MPU_IMU_DATA{
    float euler[3] = {0,0,0};
    float magneticHeading;
    float rateOfRotation[3] = {0,0,0};
    float accel[3] = {0,0,0};
    float mag[3] = {0,0,0};
    long sysTimestamp;
  };
  int begin(int updateRate = DEFAULT_UPDATE_HTZ, int eepromOffset = 0);
  void setUserCallbackFunction(void (*messageToUserCallback) (String));
  int getData(MPU_IMU_DATA *data);
  void update();
  void zeroHeading();
  void zeroAll();
  void calibrateAll();
  void loadIMUCalibrationDataFromEEPROM(int startAddress);
  void saveIMUCalibrationDataToEEPROM(int startAddress, float *gyro, float *accel, float *mag, float *magScale, float *origOffset);
  void setCalibrationOffsets(float *gyroOffset, float *accelOffset, float *magOffset, float *magScale, float*orientationOffset);
  void beginRelativeZ();


private:
  void (*userCallback)(String);
  bool useDifferentCallback = false;
  int eepromCalibrationOffset = 0;
  MPU9250 mpu;
  long microsAtLastUpdate = 0;
  long microsBetweenUpdates = 0;
  MPU_IMU_DATA currentData;
  float normalizedGyroDegrees[3] = {0,0,0};
  float normalizedAccelDegrees[3] = {0,0,0};
  float normalizedMagDegrees[3] = {0,0,0};
  float zeroOriginOffsets[3] = {0,0,0};
  MPU9250::MPU9250_Data sensorData;
  void normalizeMag();
  void normalizeGyro();
  void normalizeAccel();
  void sensorFusion();
  float computeHeading(float *orientation);
  void zeroGyro(float *gyroRateOffsets);
  void zeroAccel(float *accelOffset);
  void zeroMag(float *offsets, float *scale);
  void zeroOrigin(float *origin);
  void fuseHeading();
  void reportToUser(String what);
  void reportToUser(float what);
};

#endif
