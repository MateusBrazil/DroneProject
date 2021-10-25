#ifndef __MPU_6050__ 
#define __MPU_6050__

#include "arduino.h"
#include <stdint.h>
#include <Wire.h>
#include <time.h>

//Registers
#define MPU_SENSOR_I2C_ADDRESS 0x68     //Address of MPU6050 in I2C bus
#define GYRO_CONFIG 0x1B                //The configuration register of Gyroscope
#define ACCEL_CONFIG 0x1C               //The configuration register of Accelerometer 
#define ACCEL_XOUT_H 0x3B               //Register that contains the most valuable bits of the X axis of accelerometer --> this returns 8 bits       
#define ACCEL_XOUT_L 0x3C               //Register that contains the least valuable bits of the X axis of accelerometer --> this returns 8 bits
#define ACCEL_YOUT_H 0x3D               //
#define ACCEL_YOUT_L 0x3E               //  
#define ACCEL_ZOUT_H 0x3F               //  
#define ACCEL_ZOUT_L 0x40               //  
#define TEMP_OUT_H 0x41                 //
#define TEMP_OUT_L 0x42                 //
#define GYRO_XOUT_H 0x43                //
#define GYRO_XOUT_L 0x44                //
#define GYRO_YOUT_H 0x45                //                    
#define GYRO_YOUT_L 0x46                //
#define GYRO_ZOUT_H 0x47                //
#define GYRO_ZOUT_L 0x48                //
#define POWER_MANAGEMENT 0x6B           //
//end of registers

//Register Configuration
//The configuration values are on the datasheet of registers
#define POWER_MANAGEMENT_CFG 0x00          
#define ACCEL_FS_SEL_CFG 0x00                 
#define GYRO_FS_SEL_CFG 0x00  
//end of register configuration                

/*!
 * @brief Main MPU6050 class
 */
class MPU_6050
{

  private:
    const uint16_t MPU = MPU_SENSOR_I2C_ADDRESS;
    int mpu_status = 404;
    float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
    float gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
    bool control = true;

    /*!
     * @brief Get data from registers over I2C from the MPU6050
     * @param reg_address The address in hexadecimal of register 
     * @param num_bytes The number of bytes requested from register
     * @return Returns readed value from register
     */
    int16_t GetData(uint16_t reg_address, uint8_t num_bytes);

    /*!
     * @brief Transform the bytes coming from register in readable information like m/2²
     * @param raw_accell Raw data readed by the sensor
     * @return returns the acceleration in m/s²
     */
    float AccelCalculator(int16_t raw_accel);

    /*!
     * @brief Transform the bytes coming from register in readable information like °;
     * @param axis Axis that function should return
     * @return returns the rotation in degrees
     */
    float GyroCalculator(int axis);

    void gyroOffSet();


  
  
  public:
    
    MPU_6050();    

     /*!
     * @brief Starts the I2C communication with the MPU6050
     */
    void begin();    

    /*!
     * @brief Gets the X value of Gyroscope
     * @return Returns X value in radians    
     */      
    float GyroX() return GyroCalculator(0);

     /*!
     * @brief Gets the Y value of Gyroscope
     * @return Returns Y value in radians    
     */  
    float GyroY() return GyroCalculator(1);
    
     /*!
     * @brief Gets the Z value of Gyroscope
     * @return Returns Z value in radians    
     */  
    float GyroZ() return GyroCalculator(2);

    /*!
     * @brief Gets the temperature over I2C from the MPU6050 
     * @return Returns the temperature in Celsius  
     */  
    double Temp();

    /*!
     * @brief Get the X value of Acelerometer 
     * @return Returns X value in m/s²
     */
    float AccelX();

     /*!
     * @brief Get the X value of Acelerometer 
     * @return Returns X value in m/s²
     */
    float AccelY();

     /*!
     * @brief Get the X value of Acelerometer 
     * @return Returns X value in m/s²
     */
    float AccelZ();

};

#endif
