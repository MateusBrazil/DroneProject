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

bool MPU_6050::CheckI2CConnection(int8_t i2c_sda, int8_t i2c_scl, int8_t led_pin){

  if(i2c_sda != NULL && i2c_scl !=NULL) Wire.begin(i2c_sda, i2c_scl);
  else Wire.begin();

  int8_t transmission_code = Wire.endTransmission(); 

  if (led_pin != NULL){

    if(transmission_code != 0){

      for(int i = 0; i <= transmission_code; i++){
        digitalWrite(led_pin, HIGH);
        delay(500);
        digitalWrite(led_pin, LOW);
        delay(100);
      }
    }
    else return true;

  }

  else{
    
    Serial.begin(115200);
    switch (transmission_code){        
        case 1:
          Serial.println("data too long to fit in transmit buffer");
          break;
        case 2:
          Serial.println("received NACK on transmit of address");
          break;
        case 3:
          Serial.println("received NACK on transmit of data");
          break;
        case 4:
          Serial.println("other error");          
          break;
        default:        
          Serial.println("Error out of table. Code -->");Serial.print(transmission_code);
    }

  }  

}

void MPU_6050::Begin(){  

  while(!CheckI2CConnection(NULL, NULL, NULL));
  GyroCheck();
  AccellCheck();

}

void MPU_6050::Begin(int8_t led_pin){
  
  while(!CheckI2CConnection(NULL, NULL, led_pin));  
  //GyroCheck();
  //AccellCheck();
  
}    

void MPU_6050::Begin(int8_t scl_pin, int8_t sda_pin){

  while(!CheckI2CConnection(sda_pin, scl_pin, NULL));
  //GyroCheck();
  //AccellCheck();
  
}

void MPU_6050::Begin(int8_t scl_pin, int8_t sda_pin, int8_t led_pin){

  while(!CheckI2CConnection(sda_pin, scl_pin, led_pin));
  //GyroCheck();
  //AccellCheck();
  
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
