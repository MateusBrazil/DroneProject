#include "MPU_6050.h"

MPU_6050::MPU_6050() {}

bool MPU_6050::checkI2CConnection(int8_t i2c_sda, int8_t i2c_scl){

  if(i2c_sda != NULL && i2c_scl !=NULL) Wire.begin(i2c_sda, i2c_scl);
  else Wire.begin();

  int8_t transmission_code = Wire.endTransmission(); 

  if (default_led_pin != NULL){

    if(transmission_code != 0){

      for(int i = 0; i <= transmission_code; i++){
        digitalWrite(default_led_pin, HIGH);
        delay(500);
        digitalWrite(default_led_pin, LOW);
        delay(100);
      }
    }

    else return true;

  }

  else{
    
    Serial.begin(115200);
    switch (transmission_code){  

        case 0:
          return true;
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

    delay(500);

  }  

  delay(500);

}

void MPU_6050::begin(){  

  while(!checkI2CConnection(NULL, NULL));
  //gyroCheck(NULL);
  //accellCheck(NULL);

}

void MPU_6050::begin(int8_t led_pin){

  default_led_pin = led_pin;

  while(!checkI2CConnection(NULL, NULL));  
  //gyroCheck();
  //accellCheck();
  
}    

void MPU_6050::begin(int8_t scl_pin, int8_t sda_pin){

  while(!checkI2CConnection(sda_pin, scl_pin));
  //gyroCheck();
  //accellCheck();
  
}

void MPU_6050::begin(int8_t scl_pin, int8_t sda_pin, int8_t led_pin){

  default_led_pin = led_pin;

  while(!checkI2CConnection(sda_pin, scl_pin));
  //gyroCheck();
  //accellCheck();
  
}

int16_t MPU_6050::getData(uint16_t reg_address, uint8_t num_bytes){
  Wire.beginTransmission(MPU_SENSOR_I2C_ADDRESS);
  Wire.write(reg_address);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_SENSOR_I2C_ADDRESS, num_bytes, true);
  int16_t value = Wire.read() << 8 | Wire.read();
  return value;
}

   
void MPU_6050::configGyro(int gyro_range){

	Wire.beginTransmission(MPU_SENSOR_I2C_ADDRESS);
	Wire.write(GYRO_CONFIG);
	Wire.write(GYRO_FS_SEL_CFG);
	Wire.endTransmission();

  if(default_led_pin != NULL) digitalWrite(default_led_pin, HIGH);
  else Serial.println("DON'T MOVE MPU. CALIBRATING GYROSCOPE.");

  int16_t xdata = getData(GYRO_XOUT_H, 2);
  int16_t ydata = getData(GYRO_YOUT_H, 2);
  int16_t zdata = getData(GYRO_ZOUT_H, 2);

  float x = ((float)xdata) / 65.5;
  float y = ((float)ydata) / 65.5;
  float z = ((float)zdata) / 65.5;

  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;

  if(default_led_pin != NULL) digitalWrite(default_led_pin, LOW);
  else Serial.println("CALIBRATION COMPLETE.");

}



float MPU_6050::accelCalculator(int16_t raw_accel){

  float rangePerUnit = .000061f;

  float no_accel = raw_accel * rangePerUnit * 9.80665;

  return no_accel;
}

float MPU_6050::gyroCalculator(int choice){

  int16_t rawGyroX = getData(GYRO_XOUT_H, 2);
  int16_t rawGyroY = getData(GYRO_YOUT_H, 2);
  int16_t rawGyroZ = getData(GYRO_ZOUT_H, 2);

  float rawXacc = getData(ACCEL_XOUT_H, 2);
  float rawYacc = getData(ACCEL_YOUT_H, 2);
  float rawZacc = getData(ACCEL_ZOUT_H, 2);

  float accX = ((float)rawXacc) / 16384.0;
  float accY = ((float)rawYacc) / 16384.0;
  float accZ = ((float)rawZacc) / 16384.0;

  float angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * 360 / 2.0 / PI;
  float angleAccY = atan2(accX, sqrt(accZ * accZ + accY * accY)) * 360 / -2.0 / PI;

  float gyroX = ((float)rawGyroX) / 65.5;
  float gyroY = ((float)rawGyroY) / 65.5;
  float gyroZ = ((float)rawGyroZ) / 65.5;

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  float interval = (millis() - preInterval) * 0.001;

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

  float angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  float angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  float angleZ = angleGyroZ;
  
  preInterval = millis();

  switch(choice){
    case 1:
      return angleX;
    
    case 2:
      return angleY;

    case 3: 
      return angleZ;
  }

  return 0;

}

double MPU_6050::temp(){

  double result;
  int16_t value = getData(0x41, 2);
  result = double(value / 340.00) + 36.53;
  return result;

}

float MPU_6050::accelX(){

  int16_t value = getData(ACCEL_XOUT_H,2);
  return accelCalculator(value);

}

float MPU_6050::accelY(){
  
  int16_t value = getData(ACCEL_YOUT_H,2);
  return accelCalculator(value);

}

float MPU_6050::accelZ(){
  
  int16_t value = getData(ACCEL_ZOUT_H,2);
  return accelCalculator(value);

}
