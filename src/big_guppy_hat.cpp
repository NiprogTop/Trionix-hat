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
#include <FastLED.h>
#include <DFRobot_QMC5883.h>
#include "GyverFilters.h"

GMedian3<int> Filter; 
// GKalman Filter(4, 0.01);

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

#define LED_PIN 4
#define BRIGHTNESS 200
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];
int led_mode = 1;
int led_status = 0;

int targetHeading = 340;

GyverOS<4> OS; 
SerialParser parser(PARSE_AMOUNT);
DFRobot_QMC5883 compass_2(&Wire, QMC5883_ADDRESS); /* I2C addr */
DFRobot_QMC5883 compass_1(&Wire, QMC5883_ADDRESS); /* I2C addr */
DFRobot_QMC5883 compass;
int compass_type = 0;


int16_t dr1_val, dr2_val, dr3_val, dr4_val, dr5_val, dr6_val;
int16_t dr1_val_new, dr2_val_new, dr3_val_new, dr4_val_new, dr5_val_new, dr6_val_new;
int intData = 0;

float depth_cal = 0; //калибровочное значение глубины
float depth = 0.0;
float depth_t = 0.0;

int timr = 0;
int heading = 0;
int heading_p = 0;
float declinationAngle = (12.0 + (11.0 / 60.0)) / (180 / PI);

int mode = 1; // 1 - streaming

MPU6050 mpu(Wire);
MS5837 sensor;
sVector_t mag;


void Check_i2c(void){
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      if (String(address,HEX) == String(13,HEX) && compass_type == 0){
        compass_type = 1;
      }

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
    OS.stop(2);
    OS.stop(3);
}

void straeming()
{
    mode = 1;
    OS.start(0);
    OS.start(1);
    OS.start(2);
    OS.start(3);
}

void printData()
{
        String answer = "#3 "
                    // roll
                    + String(mpu.getAngleY()) + " "
                    // pitch
                    + String(mpu.getAngleX()) + " "
                    // heading
                    // + String(mpu.getAccZ()) + " "
                    // + String(0.9 * (heading + mpu.getAccZ() * 0.3) + 0.1 * mpu.getAccZ()) + " "
                    + String(heading) + " "
                    // + String(Filter.filtered(heading)) + " "
                    // depth
                    + String((Filter.filtered(depth * 100))* 0.01 - depth_cal) + " "
                    // temp
                    + String(sensor.temperature()) + ";";
                    // BatLevel
                    // + String(sensor.temperature()) + ";";
                    // end
                    // + ";";

        Serial.println(answer);
    
}

void updateDepth()
{
    sensor.read();
    depth_t = sensor.depth();
    if (-1.0 <= depth_t && depth_t <= 30.0){
      depth = depth_t;}
}

void updateIMU()
{
    mpu.update();
}


// double calculateTargetHeading(float currentHead) {
//   if(targetHeading < 90 && currentHead > 180){
//       return targetHeading + 360;
//   }
//   else if(targetHeading > 270 && currentHead < 180){
//       return targetHeading - 360;
//   }
//   else {
//       return targetHeading;
//   }      
// } 

// double targetHeading;
// double manualTurnMultiplier;

// double calculateTargetHeading(double currentHead, int controlMsg) {
//   if (controlMsg > 0.05 || controlMsg < -0.05) {
//       targetHeading += manualTurnMultiplier * controlMsg;

//       if (targetHeading > 360) {
//           targetHeading -= 360;
//       } else if (targetHeading < 0) {
//           targetHeading += 360;
//       }

//       if ((targetHeading < 90 && targetHeading > 0) && (currentHead > 270 && currentHead < 360)) {
//           targetHeading += 360;
//       }
//       if ((targetHeading > 270 && targetHeading < 360) && (currentHead < 90 && currentHead > 0)) {
//           targetHeading -= 360;
//       }

//       return targetHeading;
//   } else {
//       if (targetHeading < 90 && currentHead > 180) {
//           return targetHeading + 360;
//       } else if (targetHeading > 270 && currentHead < 180) {
//           return targetHeading - 360;
//       } else {
//           return targetHeading;
//       }
//   }
// }


void updateCompass()
{
    // mpu.update();

    // mag = compass.readRaw();
    // sVector_t mag = compass.readRaw();
    // compass.getHeadingDegrees();
    // heading = mag.HeadingDegress;
    
    compass.setDeclinationAngle(declinationAngle);
    mag = compass.readRaw();
    compass.getHeadingDegrees();
    heading = int(mag.HeadingDegress) * 5 % 360;
    // heading = mag.HeadingDegress;
    // heading = 55;


    // float acsZ = mpu.getAccZ();
    // if (acsZ - 1 != 0)
    // {
    //   // float head_1 = ( (heading - mag.HeadingDegress * (mpu.getAccZ() - 1) * 10);
    //   if (acsZ - 1.00 > 0){
    //     heading = heading + mag.HeadingDegress;
    //   }
    //   else {
    //     heading = heading - mag.HeadingDegress;
    //   }
    //   // heading = heading * (mpu.getAccZ() - 1) * 10;
    //   // heading = heading - (mag.HeadingDegress - heading * (mpu.getAccZ() - 1) * 10);
    // }
    // // else{
    // //   heading = 0;
    // // }

    // heading = acsZ - 1.00;
    // heading = mag.HeadingDegress;
    // heading = heading + (mag.HeadingDegress - heading * (mpu.getAccZ() - 1));
    // heading = mag.AngleXZ;

    // heading_p = calculateTargetHeading(heading);

}


   

void LED_update(){
  if (led_mode == 0){
    if (led_status == 0){
      leds[0] = CRGB::Black;
      FastLED.show();
      led_status = 1;
    }
    else{
      leds[0] = CRGB::Blue;
      FastLED.show();
      led_status = 0;
    }
  }
  else if (led_mode == 1){
    leds[0] = CRGB::Green;
      FastLED.show();
  }
  // leds[0] = CRGB::Black;
  // FastLED.show();
}

double normalizeCompassData(double rawCourse) {
    return rawCourse;
}


void setup() {  
  pinMode(2, OUTPUT); // activate level comunication
  digitalWrite(2, HIGH);

  Serial.begin(115200);
  while (!Serial)
    delay(10);

  
  Wire.begin();

  

  //  #### Light_data #####

  FastLED.addLeds<WS2811, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );

  FastLED.clear();

  leds[0] = CRGB::Yellow;
  FastLED.show();


  initMotors();
  
  // #####     MPU sensor activation   #####

  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050

  delay(1000);
  mpu.calcOffsets(true,true);

  // #####   Depth sensor activation  #####

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
  // compass.begin();

  // float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  // compass.setDeclinationAngle(declinationAngle);


  OS.attach(0, updateDepth, 120);
  OS.attach(1, printData, 120);
  OS.attach(2, updateCompass, 300);
  OS.attach(3, LED_update, 800);

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