#ifndef BMP_280_H_
#define BMP_280_H_
#include <Arduino.h>
#include <Wire.h>


#define BMP180_TEMP_XLSB_ADDRESS 0xFC
#define BMP180_TEMP_LSB_ADDRESS 0xFB
#define BMP180_TEMP_MSB_ADDRESS 0xFA
#define BMP180_PRESSURE_XLSB_ADDRESS 0xF9
#define BMP180_PRESSURE_LSB_ADDRESS 0xF8
#define BMP180_PRESSURE_MSB_ADDRESS 0xF7
#define BMP180_CONFIG_ADDRESS 0xF5
#define BMP180_CTRL_MEAS_ADDRESS 0xF4
#define BMP180_STATUS_ADDRESS 0xF3
#define BMP180_RESET_ADDRESS 0xE0
#define BMP180_ID_ADDRESS 0xD0
#define BMP180_CALIBRATION_START_ADDRESS 0xA1
#define BMP180_CALIBRATION_DIG_T1_ADDRESS 0x88
#define BMP180_CALIBRATION_DIG_T2_ADDRESS 0x8A
#define BMP180_CALIBRATION_DIG_T3_ADDRESS 0x8C
#define BMP180_CALIBRATION_DIG_P1_ADDRESS 0x8E
#define BMP180_CALIBRATION_DIG_P2_ADDRESS 0x90
#define BMP180_CALIBRATION_DIG_P3_ADDRESS 0x92
#define BMP180_CALIBRATION_DIG_P4_ADDRESS 0x94
#define BMP180_CALIBRATION_DIG_P5_ADDRESS 0x96
#define BMP180_CALIBRATION_DIG_P6_ADDRESS 0x98
#define BMP180_CALIBRATION_DIG_P7_ADDRESS 0x9A
#define BMP180_CALIBRATION_DIG_P8_ADDRESS 0x9C
#define BMP180_CALIBRATION_DIG_P9_ADDRESS 0x9F


#define BMP180_WHO_AM_I_RESPONSE 0x58
#define BMP180_RESET_WRITE_VALUE 0xB6
#define BMP180_ADDRESS_1 0x76
#define BMP180_ADDRESS_2 0x77
#define BMP180_CONNECTION_ISSUE 101
#define BMP180_BAD_WHO_AM_I_ADDRESS 102
#define BMP180_CALIBRATION_DATA_READ_FAILURE 103
#define BMP180_UPDATE_NOT_REQUIRED 104


class BMP280{
public:
  enum BMP180_Filter_Coefficients{
    Filter_Off = 0x00,
    Filter_2 = 0x01,
    Filter_4 = 0x02,
    Filter_8 = 0x03,
    Filter_16 = 0x04
  };
  enum BMP180_Temperature_Oversampling{
    Temperature_Oversampling_Skip = 0x00,
    Temperature_Oversampling_1X = 0x01,
    Temperature_Oversampling_2X = 0x02,
    Temperature_Oversampling_4X = 0x03,
    Temperature_Oversampling_8X = 0x04,
    Temperature_Oversampling_16X = 0x05
  };

  enum BMP180_Pressure_Oversampling{
    Pressure_Oversampling_Skip = 0x00,
    Pressure_Oversampling_1X = 0x01,
    Pressure_Oversampling_2X = 0x02,
    Pressure_Oversampling_4X = 0x03,
    Pressure_Oversampling_8X = 0x04,
    Pressure_Oversampling_16X = 0x05,
  };

  enum BMP180_Update_Rate{
    Update_Rate_80Hz = 80,
    Update_Rate_50Hz = 50,
    Update_Rate_25Hz = 25
  };

  int begin(int address = BMP180_ADDRESS_1, BMP180_Update_Rate rate = Update_Rate_50Hz, bool initWire = true);
  float getTemperature();
  float getPressure();
  float getAltitudeFromBaselinePressure(float baseLinePressure);
  int reset();
  int setTemperatureOversampling(BMP180_Temperature_Oversampling sample);
  int setPressureOversampling(BMP180_Pressure_Oversampling sample);
  int setMode();
  int setFilter(BMP180_Filter_Coefficients coefficient);

private:
  int bmpAddress;
  struct BMP180_Calibration_Values{
    u_int16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    u_int16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
  };

  int read8(int address);
  int write8(int address, int value);
  int write8(int address);
  int update(bool forceUpdate = false);
  float compensateTemperature(int32_t tempReading);
  float compensatePressure(int32_t pressureReading);
  bool checkForUpdate();
  int getCalibrationValues(BMP180_Calibration_Values *values);
  int checkID(bool shouldReset = true);

  BMP180_Calibration_Values calibrationValues;
  int32_t t_fine;
  int32_t currentPressureData = 0;
  int32_t currentTemperatureData = 0;
  float previousTemperature = 0;
  float previousPressure = 0;
  float previousAltitude = 0;
  bool newPressureReported = false;
  bool newTemperatureReported = false;
  bool newAltitudeReported = false;
  long lastUpdateMillis = 0;
  BMP180_Update_Rate rate;
};

#endif
