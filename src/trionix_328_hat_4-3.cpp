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
#include <DFRobot_QMC5883.h>

Servo dr1, dr2, dr3, dr4, Manipul;


#define pin1 6

#define pin2 5

#define pin3 10

#define pin4 11

#define ledPin 3 // 5

// #define ServoPin 5

#define PARSE_AMOUNT 7


GyverOS<3> OS; 
SerialParser parser(PARSE_AMOUNT);

DFRobot_QMC5883 compass_2(&Wire, QMC5883_ADDRESS); /* I2C addr */
DFRobot_QMC5883 compass_1(&Wire, HMC5883L_ADDRESS); /* I2C addr */
DFRobot_QMC5883 compass;
int compass_type = 0;
int heading = 0;
float declinationAngle = (12.0 + (11.0 / 60.0)) / (180 / PI);


int16_t dr1_val, dr2_val, dr3_val, dr4_val, manip_val;
int16_t dr1_val_new, dr2_val_new, dr3_val_new, dr4_val_new, manip_val_new;
int intData = 0;

float depth_cal = 0; //калибровочное значение глубины

int ledValue = 0;

int timr = 0;

int mode = 1; // 1 - streaming

MPU6050 mpu(Wire);
MS5837 sensor;
sVector_t mag;

void initMotors(){
  dr1.attach(pin1);
  dr1.writeMicroseconds(1480);
  dr2.attach(pin2);
  dr2.writeMicroseconds(1480);
  dr3.attach(pin3);
  dr3.writeMicroseconds(1480);
  dr4.attach(pin4);
  dr4.writeMicroseconds(1480);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 1);
  delay(7000);
  digitalWrite(ledPin, 0);
}

void Check_i2c(void){
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      // Serial.print("I2C device found at address 0x");
      // if (address<16) 
      //   Serial.print("0");
      if (String(address,HEX) == String(13,HEX) && compass_type == 0){
        compass_type = 1;
      }
      // Serial.println((address,HEX));
      // Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void printServiceMsg(String msg)
{
    Serial.print("#0 " + msg + ";");    
}

void stopStreaming()
{
    OS.stop(0);
    OS.stop(1);
    OS.stop(2);
}

void updateCompass()
{    
    compass.setDeclinationAngle(declinationAngle);
    mag = compass.readRaw();
    compass.getHeadingDegrees();
    heading = mag.HeadingDegress;
}

void straeming()
{
    mode = 1;
    OS.start(0);
    OS.start(1);
    OS.start(2);
}

void printData()
{
        String answer = "#3 "
                    // roll
                    + String(mpu.getAngleY()) + " "
                    // pitch
                    + String(mpu.getAngleX()) + " "
                    // heading
                    // + String(mpu.getAngleZ()) + " "
                    + String(heading) + " "
                    // depth
                    + String(sensor.depth() - depth_cal) + " "
                    // temp
                    + String(sensor.temperature()) + ";";
                    // end

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
  // pinMode(2, OUTPUT); // activate level comunication
  // digitalWrite(2, HIGH);

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

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997);

  updateDepth();
  depth_cal = sensor.depth(); //калибровка глубины в самом начале работы

  

  // #####   Compas sensor activation  #####

  Check_i2c();

  if (compass_type == 1){compass = compass_2;}
  else {compass = compass_1;}

  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }


  OS.attach(0, updateDepth, 120);
  OS.attach(1, printData, 50);
  OS.attach(2, updateCompass, 300);

  straeming();
}



void setMotors()
{
    int *intData = parser.getData();
    dr1_val_new = map(intData[1], -100, 100, 1080, 1880);
    dr2_val_new = map(intData[2], -100, 100, 1080, 1880);
    dr3_val_new = map(intData[3], -100, 100, 1080, 1880);
    dr4_val_new = map(intData[4], -100, 100, 1080, 1880);
    // manip_val_new = map(intData[6], 0, 100, 1800, 2500);

    dr1.writeMicroseconds(dr1_val_new);

    dr2.writeMicroseconds(dr2_val_new);

    dr3.writeMicroseconds(dr3_val_new);

    dr4.writeMicroseconds(dr4_val_new);
    
    // Manipul.writeMicroseconds(manip_val_new);

    if (intData[5] != ledValue) {
        ledValue = intData[5];
        analogWrite(ledPin, ledValue);
    }
}


void loop() {
  parser.update();
  if (parser.received()){
    setMotors();
  }

  updateIMU();

  OS.tick();

}