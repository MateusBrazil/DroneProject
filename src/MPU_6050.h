#ifndef __MPU_6050__ 
#define __MPU_6050__

#include "arduino.h"
#include <stdint.h>
#include <Wire.h>


#define ESP32_BUILTIN_LED 2

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
#define GYRO_FS_SEL_CFG 0x00
//end of register configuration     

//CONSTANTS
#define PI 3.1415
// END of CONSTANTS

/*!
 * @brief MPU_6050 Main Class
 */
class MPU_6050
{

  private:

    const uint16_t MPU = MPU_SENSOR_I2C_ADDRESS;

    /*!
    * @brief Check if MPU is working well.
    * @param i2c_sda SDA PIN number.
    * @param i2c_scl SCL PIN number.
    * @return returns true if MPU are working well and an ERROR if dont (1 - data too long to fit in transmit buffer, 2 - received NACK on transmit of address, 3 - received NACK on transmit of data, 4 - other error).
    */
    bool checkI2CConnection(int8_t i2c_sda, int8_t i2c_scl);

    /*!
     *  @brief Realize the autotest of accelerometer   
     */    
    void accellCheck();

    /*!
     *  @brief Realize the autotest of gyroscope     
     */    
    void gyroCheck();

    /*!
     * @brief Get data from registers over I2C from the MPU6050
     * @param reg_address The address in hexadecimal of register 
     * @param num_bytes The number of bytes requested from register
     * @return Returns readed value from register
     */
    int16_t getData(uint16_t reg_address, uint8_t num_bytes);

    /*!
     * @brief Transform the bytes coming from register in readable information like m/2²
     * @param raw_accell Raw data readed by the sensor
     * @return returns the acceleration in m/s²
     */
    float accelCalculator(int16_t raw_accel);

    /*!
    * @brief Transform the bytes coming from register in readable information
    * @return returns the rotation in degrees °
    */
    float gyroCalculator();

  public:

    int8_t default_led_pin = NULL;
    const float accCoef = 0.02f, gyroCoef = 0.98f;
    float angleGyroX = 0, angleGyroY = 0, angleGyroZ;
    float gyroXoffset = 0.0, gyroYoffset = 0.0, gyroZoffset = 0.0;
    long preInterval;

    MPU_6050();

    /*!
     * @brief Starts the I2C communication with the MPU6050 (Default PIN SCL: A5 | SDA: A4 [Arduino] / [ESP32] SCL: 22 SDA: 21)
     */
    void begin();

     /*!
     * @brief Starts the I2C communication with the MPU6050 (Default PIN SCL: A5 | SDA: A4 [Arduino] / [ESP32] SCL: 22 SDA: 21)
     * Don't move MPU while LED are on.
     * @param led_pin Log LED PIN number
     */
    void begin(int8_t led_pin);
    
    /*!
     * @brief Starts the I2C communication with the MPU6050
     * @param scl_pin SCL PIN number
     * @param sda_pin SDA PIN number
     */
    void begin(int8_t scl_pin, int8_t sda_pin);

    /*!
     * @brief Starts the I2C communication with the MPU6050
     * @param scl_pin SCL PIN number
     * @param sda_pin SDA PIN number
     * @param led_pin Log LED PIN number
     */
    void begin(int8_t scl_pin, int8_t sda_pin, int8_t led_pin);

    /*!
     * @brief Configure the Gyroscope range. (0 = 250 rads/s | 1 = 500 rads/s | 2 = 1000 rads/s | 3 = 2000 rads/s)
     * @param gyro_range The Gyroscope sensitivity in a scale 0-3
     */
    void configGyro(int gyro_range);

    /*!
     * @brief Gets the X value of Gyroscope
     * @return Returns X value in radians    
     */      
    int gyroX(){return gyroCalculator(1);}

     /*!
     * @brief Gets the Y value of Gyroscope
     * @return Returns Y value in radians    
     */  
    int gyroY(){return gyroCalculator(2);}
    
     /*!
     * @brief Gets the Z value of Gyroscope
     * @return Returns Z value in radians    
     */  
    int gyroZ(){return gyroCalculator(3);}

    /*!
     * @brief Gets the temperature over I2C from the MPU6050 
     * @return Returns the temperature in Celsius  
     */  
    double temp();

    /*!
     * @brief Get the X value of Acelerometer 
     * @return Returns X value in m/s²
     */
    float accelX();

     /*!
     * @brief Get the X value of Acelerometer 
     * @return Returns X value in m/s²
     */
    float accelY();

     /*!
     * @brief Get the X value of Acelerometer 
     * @return Returns X value in m/s²
     */
    float accelZ();

};

#endif
