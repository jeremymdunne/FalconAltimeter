#include <MPU9250.h>

void MPU9250::clearCalibration(){
  for(int i = 0; i < 3; i ++){
    accelOffset[i] = 0;
    gyroOffset[i] = 0;
    magOffset[i] = 0;
    magScale[i] = 1;
  }
  //reset everything
}

int MPU9250::readAll(MPU9250_Data *dat){
  //Serial.println(mpuRead16(MPU9250_ACCEL_XOUT_H));
  //long readStart = micros();
  mpuWrite8(MPU9250_ACCEL_XOUT_H);
  #ifdef STM32_SERIES_F1
    Wire.requestFrom(mpuAddr,14);
  #else
    Wire.requestFrom(mpuAddr,14,true);
  #endif
  // /Serial.println(accelOffset[0]);
  dat->accel[0]= (int16_t) (Wire.read()<<8|Wire.read()) * accelScale - accelOffset[0];
  dat->accel[1] = (int16_t) (Wire.read() <<8|Wire.read()) * accelScale - accelOffset[1];
  dat->accel[2] = (int16_t) (Wire.read() <<8|Wire.read()) * accelScale - accelOffset[2];
  dat->temperature = (int16_t) (Wire.read() <<8|Wire.read()) * tempScale - tempOffset;
  dat->gyro[0] = (int16_t) (Wire.read()<< 8| Wire.read()) * gyroScale - gyroOffset[0];
  dat->gyro[1] = (int16_t) (Wire.read() <<8|Wire.read()) * gyroScale - gyroOffset[1];
  dat->gyro[2] = (int16_t) (Wire.read() <<8|Wire.read()) * gyroScale - gyroOffset[2];
  dat->sysMicrosAtRead = micros();
  dat->elapsedMicrosToRead = dat->sysMicrosAtRead - microsAtLastRead;
  microsAtLastRead = dat->sysMicrosAtRead;
  //Serial.println("Read time: " + String((micros() - readStart)));
  //check if new mag data avialble
  if(ak8963Read8(AK8963_STATUS_1) & 0x01){
    dat->mag[0] = (ak8963Read16(AK8963_MAG_X_L) - magOffset[0]) * magScale[0];
    dat->mag[1] = (ak8963Read16(AK8963_MAG_Y_L) - magOffset[1]) * magScale[0];
    dat->mag[2] = (ak8963Read16(AK8963_MAG_Z_L) - magOffset[2]) * magScale[0];
    ak8963Read8(AK8963_STATUS_2);
  }
  return 0;
}

int MPU9250::begin(MPU9250_gyro_range gyroRange, MPU9250_accel_range accelRange){
  Wire.begin();
  Wire.setClock(400000);
  mpuAddr = MPU9250_ADDR_0;
  if(!checkMPU()){
    return -1;
  }
  resetDevice();
  setClockSource(MPU9250_CLOCK_SOURCE_X_GYRO);
  delay(100);
  setAccelRange(accelRange);
  setGyroRange(gyroRange);
  enableBypass();
  if(initAK8963() < 0){
    return -2;
  }

  return 0;
}


float MPU9250::calculateAccelScale(int scale){
  switch (scale) {
    case MPU9250_ACCEL_RANGE_2_GPS:
      return 1.0/16384.0;
      break;
    case MPU9250_ACCEL_RANGE_4_GPS:
      return 1.0/8192.0;
      break;
    case MPU9250_ACCEL_RANGE_8_GPS:
      return 1.0/4096.0;
      break;
    case MPU9250_ACCEL_RANGE_16_GPS:
      return 1.0/2048.0;
      break;
  }
  return -1;
}

float MPU9250::calculateMagScale(int scale){
  switch (scale){
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case AK8963_14_BIT:
          return 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case AK8963_16_BIT:
          return 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
    }
}

float MPU9250::calculateGyroScale(int scale){
  switch (scale) {
    case MPU9250_GYRO_RANGE_250_DPS:
      return 1.0/131.0;
      break;
    case MPU9250_GYRO_RANGE_500_DPS:
      return 1.0/65.5;
      break;
    case MPU9250_GYRO_RANGE_1000_DPS:
      return 1.0/32.8;
      break;
    case MPU9250_GYRO_RANGE_2000_DPS:
      return 1.0/16.4;
      break;
  }
  return -1;
}

void MPU9250::setAccelCalibration(float * offsets){
  accelOffset[0] = offsets[0];
  accelOffset[1] = offsets[1];
  accelOffset[2] = offsets[2];
}

void MPU9250::setGyroCalibration(float * offsets){
  gyroOffset[0] = offsets[0];
  gyroOffset[1] = offsets[1];
  gyroOffset[2] = offsets[2];
}

void MPU9250::setMagCalibration(float * scaleFactors, float * offsets){
  magScale[0] = scaleFactors[0];
  magScale[1] = scaleFactors[1];
  magScale[2] = scaleFactors[2];
  magOffset[0] = offsets[0];
  magOffset[1] = offsets[1];
  magOffset[2] = offsets[2];
}

void MPU9250::mpuWrite8(int reg, int value){
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void MPU9250::mpuWrite8(int reg){
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.endTransmission();
}

int MPU9250::mpuRead8(int reg){
  mpuWrite8(reg);
  Wire.requestFrom(mpuAddr,1);
  if(Wire.available() >= 1){
    return Wire.read();
  }
  return -1;
}

int MPU9250::mpuRead16(int regLow){
  mpuWrite8(regLow);
  Wire.requestFrom(mpuAddr,2);
  int value = -1;
  if(Wire.available() >= 2){
    value = Wire.read();
    value = value << 8;
    value |= Wire.read();
  }
  return (int)(int16_t)value;
}

int MPU9250::ak8963Read8(int reg){
  ak8963Write8(reg);
  Wire.requestFrom(AK8963_ADDR,1);
  if(Wire.available() >= 1){
    return Wire.read();
  }
  return -1;
}

int MPU9250::ak8963Read16(int reg){
  //int16_t valueL,valueH;
  ak8963Write8(reg);
  Wire.requestFrom(AK8963_ADDR,2);
  if(Wire.available() >= 2){
    akValueL = Wire.read();
    akValueH = Wire.read();
    return (int)(int16_t)(akValueH << 8 | akValueL);
  }
  return -1;
}

void MPU9250::ak8963Write8(int reg, int value){
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void MPU9250::ak8963Write8(int value){
  Wire.beginTransmission(AK8963_ADDR);
  //Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void MPU9250::enableBypass(){
  i2cReadValue = mpuRead8(MPU9250_INT_PIN_CFG);
  i2cReadValue &= 0b11111101;
  i2cReadValue |= 1 << 1;
  mpuWrite8(MPU9250_INT_PIN_CFG,i2cReadValue);
}

int MPU9250::initAK8963(){
  if(!checkAK8963()){
    return -1;
  }

  ak8963Write8(AK8963_CONTROL_1,0x00);
  delay(10);
  ak8963Write8(AK8963_CONTROL_1,AK8963_16_BIT << 4 | AK8963_CONTINUOUS_MODE_2);
  delay(10);
  return 0;
}

bool MPU9250::checkAK8963(){
  Serial.println("Mag init: Who am I: " + String(ak8963Read8(AK8963_WHO_AM_I)));
  if(ak8963Read8(AK8963_WHO_AM_I) != AK8963_WHO_AM_I_RESPONSE){
    return false;
  }
  return true;
}

void MPU9250::setClockSource(MPU9250_clock_source source){
  mpuWrite8(MPU9250_PWR_MGMT_1,source);
}

void MPU9250::setAccelRange(MPU9250_accel_range range){
    i2cReadValue = mpuRead8(MPU9250_ACCEL_CONFIG);
    //mpuWrite8(MPU9250_ACCEL_CONFIG, value &=0b00011111);
    //mpuWrite8(MPU9250_ACCEL_CONFIG, value &=0b11100111);
    i2cReadValue &= 0b11100111;
    i2cReadValue |= range << 3;
    //Serial.println(value|(range  << 3),BIN);
    mpuWrite8(MPU9250_ACCEL_CONFIG, i2cReadValue);
    accelScale = calculateAccelScale(range);
}

void MPU9250::setGyroRange(MPU9250_gyro_range range){
  i2cReadValue = mpuRead8(MPU9250_GYRO_CONFIG);
  i2cReadValue &= 0b11100111;
  i2cReadValue |= range << 3;
  mpuWrite8(MPU9250_GYRO_CONFIG,i2cReadValue);
  gyroScale = calculateGyroScale(range);
}



void MPU9250::resetDevice(){
  i2cReadValue = mpuRead8(MPU9250_PWR_MGMT_1);
  //Serial.println(value, BIN);
  i2cReadValue &= 0b01111111;
  i2cReadValue |= 1 << 7;
  mpuWrite8(MPU9250_PWR_MGMT_1,i2cReadValue);
  delay(10);
  while(mpuRead8(MPU9250_PWR_MGMT_1) >> 7 != 0) delay(10);
  i2cReadValue = mpuRead8(MPU9250_PWR_MGMT_1);
  //Serial.println(value, BIN);
}


bool MPU9250::checkMPU(){
  if(mpuRead8(MPU9250_WHO_AM_I) != MPU9250_WHO_AM_I_RESPONSE && mpuRead8(MPU9250_WHO_AM_I) != MPU9250_WHO_AM_I_RESPONSE_2){
    while(true){
      Serial.println(mpuRead8(MPU9250_WHO_AM_I),HEX);
    }

    return false;
  }
  return true;
}
