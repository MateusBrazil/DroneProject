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

float MPU_6050::GyroCalculator(int16_t raw_gyro, int axis)
{

  float no_gyro = 0;
    //.007633f
  float rangePerUnit = .007633f;

  no_gyro = raw_gyro * rangePerUnit;

  float gyro_x_value = no_gyro - gyro_offset_x;
  float gyro_y_value = no_gyro - gyro_offset_y;
  float gyro_z_value = no_gyro - gyro_offset_z;


  switch (axis){
    case 0:
        if (gyro_x_value > 1 | gyro_x_value <1)
        {
            gyro_angle_x += (gyro_x_value * 57.2958f) / 360;
        }
        return gyro_angle_x;
    case 1:
        if (gyro_y_value > 1 | gyro_y_value <1)
        {
            gyro_angle_y += (gyro_y_value * 57.2958f) / 360;
        }
        return gyro_angle_y;
    case 2:
        if (gyro_z_value > 1 | gyro_z_value < 1)
        {
            gyro_angle_z += (gyro_z_value * 57.2958f) / 360;
        }
        return gyro_angle_z;
  
    default:
        return 0;
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

   
   
float MPU_6050::GyroX(){

  int16_t result = GetData(GYRO_XOUT_H, 2);
  return (GyroCalculator(result)); ;

}

float MPU_6050::GyroY(){
 
  int16_t result = GetData(GYRO_YOUT_H, 2);
  return (GyroCalculator(result);
 
}   

float MPU_6050::GyroZ(){        

  int16_t result = GetData(GYRO_ZOUT_H, 2);
  return (GyroCalculator(result);
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
