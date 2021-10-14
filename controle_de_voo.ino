#include <Wire.h>
#include "MPU_6050.h"

#define BMP085_DEBUG 1


MPU_6050 mpu;

void setup() {
  Serial.begin(115200);
  mpu.begin();
}

void loop() {

  Serial.println();
  Serial.println(mpu.Temp());
  Serial.println(); Serial.print(mpu.AccelX()); Serial.print(" | "); Serial.print(mpu.AccelY()); Serial.print(" | "); Serial.print(mpu.AccelZ());
  delay(500);
}
