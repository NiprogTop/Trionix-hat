//////////////////     MPU-6050   ////////////////////////

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include "Wire.h"
#include <MPU6050_light.h>
#include <GyverOS.h>
#include <SerialParser.h>
#include <Servo.h>
#include "MS5837.h"

MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();

//  if(millis() - timer > 1000){ // print data every second
//    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
//    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
//    Serial.print("\tY: ");Serial.print(mpu.getAccY());
//    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
//  
//    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
//    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
//    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
//  
//    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
//    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.print(mpu.getAngleZ());
    Serial.print("\tT: ");Serial.println(timer);
    Serial.println(F("=====================================================\n"));
    timer = millis();
//  }

}