#include "MPU_6050.h"

MPU_6050::MPU_6050() {}

int16_t MPU_6050::GetData(uint16_t reg_address, uint8_t num_bytes){
  Wire.beginTransmission(MPU_SENSOR_I2C_ADDRESS);
  Wire.write(reg_address);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_SENSOR_I2C_ADDRESS, num_bytes, true);
  int16_t value = Wire.read() << 8 | Wire.read();
  return value;
}

float MPU_6050::AccelCalculator(int16_t raw_accel){

  float rangePerUnit = .000061f;

  float no_accel = raw_accel * rangePerUnit * 9.80665;

  return no_accel;

}

void MPU_6050::initiate(){
  
  Wire.begin();
  Wire.beginTransmission(MPU_SENSOR_I2C_ADDRESS);
  Wire.write(POWER_MANAGEMENT);
  Wire.write(POWER_MANAGEMENT_CFG);
  Wire.endTransmission(true);
  
}    

   
void MPU_6050::ConfigGyroRange(int gyro_range){

	Wire.beginTransmission(MPU_SENSOR_I2C_ADDRESS);
	Wire.write(GYRO_CONFIG);
	Wire.write(GYRO_FS_SEL_CFG);
	Wire.endTransmission();
}

int MPU_6050::GyroX(){

	int16_t result = GetData(GYRO_XOUT_H, 2);

}

int MPU_6050::GyroY(){
	
	int16_t result = GetData(GYRO_YOUT_H, 2);

}   

int MPU_6050::GyroZ(){        

  int16_t result = GetData(GYRO_ZOUT_H, 2);

}
double MPU_6050::Temp(){

  double result;
  int16_t value = GetData(0x41, 2);
  result = double(value / 340.00) + 36.53;
  return result;

}

float MPU_6050::AccelX(){

  int16_t value = GetData(ACCEL_XOUT_H,2);
  return AccelCalculator(value);

}

float MPU_6050::AccelY(){
  
  int16_t value = GetData(ACCEL_YOUT_H,2);
  return AccelCalculator(value);

}

float MPU_6050::AccelZ(){
  
  int16_t value = GetData(ACCEL_ZOUT_H,2);
  return AccelCalculator(value);

}
