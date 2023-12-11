#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#if FMT_EXCEPTIONS
# define FMT_TRY try
# define FMT_CATCH(x) catch (x)
#else
# define FMT_TRY if (true)
# define FMT_CATCH(x) if (false)
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
//front
#define pin2 9
//back
#define pin3 11
//left
#define pin4 10
//right 
#define pin5 5
//back
#define pin6 6

#define PARSE_AMOUNT 7

GyverOS<3> OS; 
SerialParser parser(PARSE_AMOUNT);

int16_t dr1_val, dr2_val, dr3_val, dr4_val, dr5_val, dr6_val;
int16_t dr1_val_new, dr2_val_new, dr3_val_new, dr4_val_new, dr5_val_new, dr6_val_new;
int intData = 0;

float depth_cal = 0; //калибровочное значение глубины

int timr = 0;

int mode = 1; // 1 - streaming

MPU6050 mpu(Wire);
MS5837 sensor;

void initMotors(){
  dr1.attach(pin1);
  dr1.writeMicroseconds(1480);
  dr2.attach(pin2);
  dr2.writeMicroseconds(1480);
  dr3.attach(pin3);
  dr3.writeMicroseconds(1480);
  dr4.attach(pin4);
  dr4.writeMicroseconds(1480);
  dr5.attach(pin5);
  dr5.writeMicroseconds(1480);
  dr6.attach(pin6);
  dr6.writeMicroseconds(1480);
  delay(7000);
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
                    // roll
                    + String(mpu.getAngleY()) + " "
                    // pitch
                    + String(mpu.getAngleX()) + " "
                    // heading
                    + String(mpu.getAngleZ()) + " "
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

  straeming();
}


void setMotors()
{
    int *intData = parser.getData();
    dr1_val_new = map(intData[1], -100, 100, 1080, 1880);
    dr2_val_new = map(intData[2], -100, 100, 1080, 1880);
    dr3_val_new = map(intData[3], -100, 100, 1080, 1880);
    dr4_val_new = map(intData[4], -100, 100, 1080, 1880);
    dr5_val_new = map(intData[5], -100, 100, 1080, 1880);
    dr6_val_new = map(intData[6], -100, 100, 1080, 1880);

    dr1.writeMicroseconds(dr1_val_new);

    dr2.writeMicroseconds(dr2_val_new);

    dr3.writeMicroseconds(dr3_val_new);

    dr4.writeMicroseconds(dr4_val_new);

    dr5.writeMicroseconds(dr5_val_new);

    dr6.writeMicroseconds(dr6_val_new);

}




void loop() {  
  parser.update();
  if (parser.received()){
    setMotors();
  }

  updateIMU();

  OS.tick();
}