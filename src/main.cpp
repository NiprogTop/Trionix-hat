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

Servo dr1, dr2, dr3, dr4, dr5, dr6;

//front
#define pin1 3
//right
#define pin2 9
//back
#define pin3 11
//left
#define pin4 10
//back_l
#define pin5 5 
//back_w
#define pin6 6 

#define PARSE_AMOUNT 6

GyverOS<3> OS; 
SerialParser parser(PARSE_AMOUNT);

void initMotors(){
  dr1.attach(pin1);
  dr1.writeMicroseconds(1500);
  dr2.attach(pin2);
  dr2.writeMicroseconds(1500);
  dr3.attach(pin3);
  dr3.writeMicroseconds(1500);
  dr4.attach(pin4);
  dr4.writeMicroseconds(1500);
  dr5.attach(pin5);
  dr5.writeMicroseconds(1500);
  dr6.attach(pin6);
  dr6.writeMicroseconds(1500);
  delay(7000);
}

void start(){
  dr1.writeMicroseconds(1550);
  dr2.writeMicroseconds(1550);
  dr3.writeMicroseconds(1550);
  dr4.writeMicroseconds(1550);
  dr5.writeMicroseconds(1550);
  dr6.writeMicroseconds(1550);
}

void setup() {
  pinMode(2, OUTPUT); // activate level comunication
  digitalWrite(2, HIGH);

  Serial.begin(115200);
  initMotors();

}

void loop() {
  // Serial.println(" hi! ");
  parser.update();
  if (parser.received()){
    int comand = parser.getData()[1];
    Serial.println(comand);
    
    // if (comand == 3 && mode == 2){straeming();}

    // setMotors();
  }
  

  // updateIMU();

  // OS.tick();
}