#ifndef __MPU_6050__ 
#define __MPU_6050__

#include "arduino.h"
#include <stdint.h>
#include <Wire.h>

// BEGIN of registers
#define MPU_SENSOR_I2C_ADDRESS 0x68      //Address of MPU6050 in I2C bus.
#define GYRO_CONFIG 0x1B                 // The configuration register of Gyroscope.
#define ACCEL_CONFIG 0x1C                // The configuration register of Accelerometer.
#define ACCEL_XOUT_H 0x3B                // Register that contains the most valuable bits of the X axis of accelerometer --> this returns 8 bits.
#define ACCEL_XOUT_L 0x3C                // Register that contains the least valuable bits of the X axis of accelerometer --> this returns 8 bits.
#define ACCEL_YOUT_H 0x3D                // Register that contains the most valuable bits of the Y axis of accelerometer --> this returns 8 bits.
#define ACCEL_YOUT_L 0x3E                // Register that contains the least valuable bits of the Y axis of accelerometer --> this returns 8 bits.
#define ACCEL_ZOUT_H 0x3F                // Register that contains the most valuable bits of the Z axis of accelerometer --> this returns 8 bits.
#define ACCEL_ZOUT_L 0x40                // Register that contains the least valuable bits of the Z axis of accelerometer --> this returns 8 bits.
#define TEMP_OUT_H 0x41                  // Register that contains the most valuable bits of the termometer --> this returns 8 bits.
#define TEMP_OUT_L 0x42                  // Register that contains the least valuable bits of the termometer --> this returns 8 bits.
#define GYRO_XOUT_H 0x43                 // Register that contains the most valuable bits of the X axis of Gyroscope --> this returns 8 bits.
#define GYRO_XOUT_L 0x44                 // Register that contains the least valuable bits of the X axis of Gyroscope --> this returns 8 bits.
#define GYRO_YOUT_H 0x45                 // Register that contains the most valuable bits of the Y axis of Gyroscope --> this returns 8 bits.
#define GYRO_YOUT_L 0x46                 // Register that contains the least valuable bits of the Y axis of Gyroscope --> this returns 8 bits.
#define GYRO_ZOUT_H 0x47                 // Register that contains the most valuable bits of the Z axis of Gyroscope --> this returns 8 bits.
#define GYRO_ZOUT_L 0x48                 // Register that contains the least valuable bits of the Z axis of Gyroscope --> this returns 8 bits.
#define POWER_MANAGEMENT 0x6B            // Read/Write register that configurate power management of MPU_6050.
// END of registers

// Register Configuration
// The configuration values are on the datasheet of registers.
#define POWER_MANAGEMENT_CFG 0x00          
#define ACCEL_FS_SEL_CFG 0x00                 
#define GYRO_FS_SEL_CFG 0x10  
//end of register configuration                

/*!
 * @brief MPU_6050 Main Class
 */
class MPU_6050
{

  private:

    MPU_6050();

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
     * @return returns the acceleration in m/s²     * 
     */
    float AccelCalculator(int16_t raw_accel);

    
    int16_t GyroCalculator(int16_t Gx, int16_t Gy, int16_t Gz);

    /*!
     * @brief Converts 2's complement binary into decimal
     * @param twocomplement binary 2's complement value to convert
     * @return Returns converted value in decimal
     */
    int TwosComplementToDecimal(int16_t twoscomplement);

  
  
  public:

     /*!
     * @brief Starts the I2C communication with the MPU6050
     */
    void initiate();    

    /*!
     * @brief Gets the X value of Gyroscope
     * @return Returns X value in radians    
     */      
    int GyroX();

     /*!
     * @brief Gets the Y value of Gyroscope
     * @return Returns Y value in radians    
     */  
    int GyroY();
    
     /*!
     * @brief Gets the Z value of Gyroscope
     * @return Returns Z value in radians    
     */  
    int GyroZ();

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

    /*!
     * @brief Configure the Gyroscope range. 
     * 0 = 250 rads/s;
     * 1 = 500 rads/s;
     * 2 = 1000 rads/s;
     * 3 = 2000 rads/s;
     * @param gyro_range The Gyroscope sensitivity in a scale 0-3
     */
    void ConfigGyroRange(int gyro_range);

};

#endif
