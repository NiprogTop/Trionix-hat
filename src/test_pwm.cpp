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


//  #######   PWM_TEST   #########



#include <Servo.h>

// создаём объекты для управления сервоприводами
Servo myservo5;
Servo myservo1;
Servo myservo4;
Servo myservo3;
Servo myservo2;

void setup(){
//    Serial.begin(9600);
   myservo1.attach(6);
   myservo1.write(1500);
   myservo2.attach(3);
   myservo2.write(1500);
   myservo3.attach(9);
   myservo3.write(1500);
   myservo4.attach(11);
   myservo4.write(1500);
   myservo5.attach(10);
   myservo5.write(1500);
   delay(7000);
   
} 

void loop(){
 myservo1.write(1600);
 myservo2.write(1600);
 myservo3.write(1600);
 myservo4.write(1600);
 myservo5.write(1600);
}







///////////////////     ALL_CHECK     /////////////////////////
//
//#include "Wire.h"
////#include <basicMPU6050.h>
//
//#include <Servo.h>
//
//#include "MPU6050.h"
//MPU6050 mpu;
//
//long timer = 0;
//
////#include <Wire.h>
//#include "MS5837.h"
//
//MS5837 sensor;
//
//Servo myservo5;
//Servo myservo;
//Servo myservo4;
//Servo myservo3;
//Servo myservo2;
//
//void setup_PWM(){
//    myservo.attach(6);
//    myservo.write(1500);
//    myservo2.attach(3);
//    myservo2.write(1500);
//    myservo3.attach(9);
//    myservo3.write(1500);
//    myservo4.attach(11);
//    myservo4.write(1500);
//    myservo5.attach(10);
//    myservo5.write(1500);
//    delay(7000);
//    
//} 
//
//
//void PWM_start(){
//  myservo.write(1600);
//  myservo2.write(1600);
//  myservo3.write(1600);
//  myservo4.write(1600);
//  myservo5.write(1600);
//}
//
//void setup_I2C() {
//  pinMode(2, OUTPUT);
//  digitalWrite(2, HIGH);
//  Serial.begin(115200);
//
//  Serial.println("Starting");
//
//  Wire.begin();
//
//  // Initialize pressure sensor
//  // Returns true if initialization was successful
//  // We can't continue with the rest of the program unless we can initialize the sensor
//  while (!sensor.init()) {
//    Serial.println("Init failed!");
//    Serial.println("Are SDA/SCL connected correctly?");
//    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
//    Serial.println("\n\n\n");
//    delay(5000);
//  }
//
//  // .init sets the sensor model for us but we can override it if required.
//  // Uncomment the next line to force the sensor model to the MS5837_30BA.
//  //sensor.setModel(MS5837::MS5837_30BA);
//
//  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
//}
//
//void start_I2C() {
//  // Update pressure and temperature readings
//  sensor.read();
//
//  Serial.print("Pressure: ");
//  Serial.print(sensor.pressure());
//  Serial.println(" mbar");
//
//  Serial.print("Temperature: ");
//  Serial.print(sensor.temperature());
//  Serial.println(" deg C");
//
//  Serial.print("Depth: ");
//  Serial.print(sensor.depth());
//  Serial.println(" m");
//
//  Serial.print("Altitude: ");
//  Serial.print(sensor.altitude());
//  Serial.println(" m above mean sea level");
//
//  delay(1000);
//}
//
//void setup_MPU() {
//  
//  pinMode(2, OUTPUT);
//  digitalWrite(2, HIGH);
////  Serial.begin(9600);
//  Wire.begin();
//  
//  
////  Serial.print(F("MPU6050 status: "));
////  Serial.println(status);
//  while(status!=0){ } // stop everything if could not connect to MPU6050
//  
//  Serial.println(F("Calculating offsets, do not move MPU6050"));
//  delay(1000);
//  mpu.calcOffsets(true,true); // gyro and accelero
//  Serial.println("Done!\n");
//  
//}
//
//void start_MPU() {
//  mpu.update();
//
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
//    
//    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
//    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
//    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
//    Serial.println(F("=====================================================\n"));
//    timer = millis();
//   }
// }
//
//void setup(){
//  Serial.begin(9600);
//  setup_PWM();
//  setup_MPU();
//  setup_I2C();
//}
//
//
//void loop(){
//  start_MPU();
//  start_I2C();
//  PWM_start();
//}