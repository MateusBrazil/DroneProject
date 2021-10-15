#include <Wire.h>
#include "MPU_6050.h"

#define BMP085_DEBUG 1

MPU_6050 mpu;

void setup() {

  Serial.begin(115200);

  mpu.begin(ESP32_BUILTIN_LED);
  mpu.configGyro(0);

}

void loop() {

  Serial.println();
  Serial.println(mpu.temp());
  Serial.println(); Serial.print(mpu.accelX()); Serial.print(" | "); Serial.print(mpu.accelY()); Serial.print(" | "); Serial.print(mpu.accelZ());
  Serial.println(); Serial.print(mpu.gyroX()); Serial.print(" | "); Serial.print(mpu.gyroY()); Serial.print(" | "); Serial.print(mpu.gyroZ());
  delay(500);
}
