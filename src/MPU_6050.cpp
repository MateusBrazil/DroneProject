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


void MPU_6050::gyroOffSet(){

  float median_value = 0.0, scale = .007633f;
  int address = 0x00;
  short int count = 0;
  int16_t data = 0;


  
  Serial.println("CALIBRANDO O OFFSET DO GYROSCOPIO NÃO MOVA O SENSOR");

  while(count < 3){
   
    if(count == 0){
      Serial.println("Calculando offset eixo X...");
      address = 0x43;   //Gyro X
    }
    else if(count == 1){
      Serial.println("Calculando offset eixo Y...");
      address = 0x45;   //Gyro Y
    }
    else if(count == 2){
      Serial.println("Calculando offset eixo Z...");
      address = 0x47;   //Gyro Z
    }else{
      Serial.println("Não deveria vir até aqui");
    }
        
    for(int i = 0; i <= 9; i++){

      data = GetData(address,2);
      Serial.print("Data :");Serial.println(data);
      median_value += data;
      
      delay(200);

    }    

    Serial.print("Median: ");Serial.println(median_value);
    median_value = median_value*scale;

    if(count == 0){      
      gyro_offset_x = (float)median_value/10.0;
    }
    else if(count == 1){
      gyro_offset_y = (float)median_value/10.0;
    }
    else if(count == 2){
      gyro_offset_z = (float)median_value/10.0;
    }
    count += 1;
  }

  Serial.println("OFFSET OBTIDO");
  Serial.println();
  Serial.print("GYRO X OFFSET: ");Serial.print(gyro_offset_x);Serial.print("GYRO Y OFFSET: ");Serial.print("GYRO OFFSET Y: ");Serial.print(gyro_offset_y);Serial.print("GYRO Z OFFSET: ");Serial.print(gyro_offset_z);
  Serial.println();

  delay(2000);
}


float MPU_6050::AccelCalculator(int16_t raw_accel){

  float no_accel = 0.0;

  float rangePerUnit = .000061f;

  no_accel = raw_accel * rangePerUnit * 9.80665;

  return no_accel;

}

float MPU_6050::GyroCalculator(int axis)
{
  float rangePerUnit = .007633f;

  switch (axis){
    
    default:
      return 0;
      
    case 0:
    {
      float no_gyro_x = 0;
      float tini = millis();
      int16_t raw_gyro_x = GetData(GYRO_XOUT_H, 2);
      float tfin = millis();
      no_gyro_x *= rangePerUnit;
      float gyro_x_value = no_gyro_x - gyro_offset_x;
      float interval = (tfin-tini) / 1000;     
      float dangle = gyro_x_value * interval;
      if (gyro_x_value > 0.5f | gyro_x_value < -0.5f) gyro_angle_x += dangle * 57.2958f;
      return gyro_angle_x;
    }

    case 1:
    {
      float no_gyro_y = 0;
      float tini = millis();
      int16_t raw_gyro_y = GetData(GYRO_YOUT_H, 2);
      float tfin = millis();
      float interval = (tfin-tini) / 1000;
      no_gyro_y *= rangePerUnit; 
      float gyro_y_value = no_gyro_y - gyro_offset_y;
      float dangle = gyro_y_value * interval;
      if (gyro_y_value > 0.5f | gyro_y_value < -0.5f) gyro_angle_y += dangle * 57.2958f;
      return gyro_angle_y;
    }

    case 2:
    {
      float no_gyro_z = 0;
      float tini = millis();
      int16_t raw_gyro_z = GetData(GYRO_ZOUT_H, 2);
      float tfin = millis();
      float interval = (tfin-tini) / 1000;
      no_gyro_z *= rangePerUnit;
      float gyro_z_value = no_gyro_z - gyro_offset_z;
      float dangle = gyro_z_value * interval;
      if (gyro_z_value > 0.5f | gyro_z_value < -0.5f) gyro_angle_z += dangle * 57.2958f;
      return gyro_angle_z;
    }
      
  }

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

  gyroOffSet();
  
}       

double MPU_6050::Temp(){

  double result;
  int16_t value = GetData(0x41, 2);
  result = double(value / 340.00) + 36.53;
  return result;
}



float MPU_6050::AccelX(){

  int16_t value = GetData(ACCEL_XOUT_H,2);
  return AccelCalculator(value) ;

}

float MPU_6050::AccelY(){
  
  int16_t value = GetData(ACCEL_YOUT_H,2);
  return AccelCalculator(value);

}

float MPU_6050::AccelZ(){
  
  int16_t value = GetData(ACCEL_ZOUT_H,2);
  return AccelCalculator(value);

}