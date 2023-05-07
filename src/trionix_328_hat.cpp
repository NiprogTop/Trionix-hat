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

Servo dr1, dr2, dr3, dr4;
Servo LEDs;

//front
#define pin1 3
//right
#define pin2 9
//back
#define pin3 11
//left
#define pin4 10

#define ledPin 5 // 6

#define PARSE_AMOUNT 6

GyverOS<3> OS; 
SerialParser parser(PARSE_AMOUNT);

int16_t dr1_val, dr2_val, dr3_val, dr4_val;
int16_t dr1_val_new, dr2_val_new, dr3_val_new, dr4_val_new;
int intData = 0;

float depth_cal = 0; //калибровочное значение глубины
// int *intData;

int ledValue = 0;

int timr = 0;

int mode = 1; // 1 - streaming,  2 - calibration

MPU6050 mpu(Wire);
MS5837 sensor;

void initMotors(){
  dr1.attach(pin1);
  dr1.writeMicroseconds(1500);
  dr2.attach(pin2);
  dr2.writeMicroseconds(1500);
  dr3.attach(pin3);
  dr3.writeMicroseconds(1500);
  dr4.attach(pin4);
  dr4.writeMicroseconds(1500);
  // delay(7000);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 1);
  delay(7000);
  digitalWrite(ledPin, 0);
}


void printServiceMsg(String msg)
{
    Serial.print("#0 " + msg + ";");    
}

void stopStreaming()
{
    OS.stop(0);
    OS.stop(1);
}

void straeming()
{
    mode = 1;
    OS.start(0);
    OS.start(1);
}

void printData()
{
        String answer = "#3 "
                    // pitch
                    + String(mpu.getAngleY()) + " "
                    // roll
                    + String(mpu.getAngleX()) + " "
                    // heading
                    + String(mpu.getAngleZ()) + " "
                    // + String(mpu.getRoll()) + " "
                    // + String(dr4_val_new) + " "
                    // depth
                    + String(sensor.depth() - depth_cal) + " "
                    // temp
                    + String(sensor.temperature()) + ";";
                    // end
                    // + ";";

        Serial.println(answer);
    
}

void updateDepth()
{
    sensor.read();
}

void updateIMU()
{
    mpu.update();
}




void setup() {
  pinMode(2, OUTPUT); // activate level comunication
  digitalWrite(2, HIGH);

  Serial.begin(115200);
  Wire.begin();

  initMotors();

  // MPU sensor activation

  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050

  delay(1000);
  mpu.calcOffsets(true,true);

  // Depth sensor activation

  while (!sensor.init()) {
    delay(1000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);

  updateDepth();
  depth_cal = sensor.depth(); //калибровка глубины в самом начале работы

  OS.attach(0, updateDepth, 120);
  OS.attach(1, printData, 50);
}


void setMotors()
{
    int *intData = parser.getData();
    dr1_val_new = map(intData[1], -100, 100, 1100, 1900);
    dr2_val_new = map(intData[2], -100, 100, 1100, 1900);
    dr3_val_new = map(intData[3], -100, 100, 1100, 1900);
    dr4_val_new = map(intData[4], -100, 100, 1100, 1900);

    dr1.writeMicroseconds(dr1_val_new);

    dr2.writeMicroseconds(dr2_val_new);

    dr3.writeMicroseconds(dr3_val_new);

    dr4.writeMicroseconds(dr4_val_new);
}

void setLight(){
  int *intData = parser.getData();
  if (intData[5] > 0) {
    ledValue = intData[5];
    // analogWrite(ledPin, ledValue);
    digitalWrite(ledPin, 1);
  } else{ digitalWrite(ledPin, 0);}

}



void loop() {
  parser.update();
  if (parser.received()){
    int comand = parser.getData()[0];
    
    if (comand == 3 && mode == 2){straeming();}

    setMotors();
    setLight();
  }
  

  updateIMU();

  OS.tick();

}