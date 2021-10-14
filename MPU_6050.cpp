#include "MPU_6050.h"


MPU_6050::MPU_6050(){}


int16_t MPU_6050::GetData(uint16_t reg_address, uint8_t num_bytes){
  int16_t value;
  Wire.beginTransmission(MPU);
  Wire.write(reg_address);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, num_bytes, true);

  value = Wire.read() << 8 | Wire.read();
  return value;
}

float MPU_6050::AccelCalculator(int16_t raw_accel){

  float no_accel = 0.0;

  float rangePerUnit = .000061f;

  no_accel = raw_accel * rangePerUnit * 9.80665;

  return no_accel;

}

void MPU_6050::begin(){
  
  const short int FS_SEL = 0;  

  Serial.println("ESTOU NA BEGIN");
  
  Wire.begin();
  Wire.beginTransmission(MPU);
  int8_t mpu_status = Wire.endTransmission();

  //if(mpu_status != 0) return mpu_status;

  //Configuração da escala dos registradores do giroscopio
  Serial.println("########################################");
  Serial.println("Configurando registradores do giroscopio");
  Wire.beginTransmission(MPU);
  Wire.write(GYRO_CONFIG);
  Wire.write(GYRO_FS_SEL_CFG);
  Wire.endTransmission();
  Serial.println("##########Configuração finalizada#######");
  Serial.println("########################################");        
  //Fim da configuração dos registradores do giroscopio

  Serial.println("#######################Configurações#######################");
  Serial.println("## Registrador FS_SEL = " + String(FS_SEL));
  Serial.println("###########################################################");

  Wire.beginTransmission(MPU);
  Wire.write(POWER_MANAGEMENT);
  Wire.write(POWER_MANAGEMENT_CFG);
  Wire.endTransmission(true);
  
}    

   
   
int MPU_6050::GyroX(){

  int16_t result = GetData(GYRO_XOUT_H, 2);
  double radians = double(result/32.8);  
  return radians;
}

int MPU_6050::GyroY(){
 
  int16_t result = GetData(GYRO_YOUT_H, 2);
  double radians = double(result/32.8);
  
  return radians;
}   

int MPU_6050::GyroZ(){        

  int16_t result = GetData(GYRO_ZOUT_H, 2);
  double radians = double(result/32.8);        
  return radians;
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
