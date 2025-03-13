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
#include <FastLED.h>
#include <DFRobot_QMC5883.h>
#include "GyverFilters.h"
#include <iarduino_I2C_Motor.h>
#include <GY_85.h>

#define NUM_OF_LEDS 14
#define LED_1 11
#define LED_2 12
#define BRIGHTNESS 250

#define PARSE_AMOUNT 5
#define TORCH_PIN A1

CRGB leds1[NUM_OF_LEDS];
// CRGB leds2[NUM_OF_LEDS];

GyverOS<3> OS;
SerialParser parser(PARSE_AMOUNT);

// GY_85 GY85;
sVector_t mag;
DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);
int heading = 0;
float declinationAngle = (12.0 + (11.0 / 60.0)) / (180 / PI);

int ledValue = 0;
iarduino_I2C_Motor m1(0x09);                   //   Объявляем объект mot_L для работы с функциями и методами библиотеки iarduino_I2C_Motor, указывая адрес модуля на шине I2C.
iarduino_I2C_Motor m2(0x0A); 
int yaw;



void fillAllLed(int n, CRGB* leds, CRGB color){
    for(int i = 0; i <= n; i++){
      leds[i] = color;      
    }  
}

bool blink_stat = 0;

void stopStreaming()
{
    OS.stop(0);
    OS.stop(1);
    // OS.stop(2);
}

void streaming()
{
    OS.start(0);
    OS.start(1);
    // OS.start(2);
}


void blink(){
    fillAllLed(NUM_OF_LEDS, leds1, CRGB(0, 0, 0));
    // fillAllLed(NUM_OF_LEDS, leds2, CRGB(0, 0, 0));
    FastLED.show();
  if (blink_stat){
    fillAllLed(NUM_OF_LEDS, leds1, CRGB( 255, 140, 0));
    // fillAllLed(NUM_OF_LEDS, leds2, CRGB( 0, 0, 0));
    FastLED.show();
    blink_stat = 0;
  } else {
    // fillAllLed(NUM_OF_LEDS, leds2, CRGB( 255, 140, 0));
    fillAllLed(NUM_OF_LEDS, leds1, CRGB( 0, 0, 0));
    FastLED.show();
    blink_stat = 1;
  }
}

int levels[6]={50, 120, 90, 240, 150, 50};
bool status = true;

void policeLight(){
  for(int i = 0; i < levels; i++){
    FastLED.setBrightness(levels[i]);
    if(i == 5){
      status = !status;
    }
    if(status){
      fillAllLed(NUM_OF_LEDS, leds1, CRGB(0, 0, 255));
    }else{
      // fillAllLed(NUM_OF_LEDS, leds2, CRGB(255, 0, 0));
    }
    FastLED.show();
  }
  FastLED.clear();
  FastLED.show();
}

void updateCompass()
{
    compass.setDeclinationAngle(declinationAngle);
    mag = compass.readRaw();
    compass.getHeadingDegrees();
    heading = mag.HeadingDegress;
}

// int updateCourse()
// {
//   // int* compassReadings = GY85.*readFromCompass();
//   int yaw = GY85.compass_z(GY85.readFromCompass());
//   // yaw = 0;
//   return yaw;
// }


void printData()
{
        String answer = "#3 "
                    + String(m1.getSum(MOT_MET)) + " "
                    + String(m2.getSum(MOT_MET)) + " "
                    // + String(updateCourse()) + " "
                    + String(heading) + " "
                    + String(ledValue) + ";";
                  
        Serial.println(answer);
    
}







void ledAnimation()
{
  if(ledValue == 0){
      FastLED.clear();
      FastLED.show();
  }
  if(ledValue == 1){
    fillAllLed(NUM_OF_LEDS, leds1, CRGB( 0, 255, 0));
    // fillAllLed(NUM_OF_LEDS, leds2, CRGB( 0, 255, 0));
    FastLED.show();
  }
  if(ledValue == 2){
      policeLight();
  }
  if(ledValue == 3){
      blink();
      delay(200);
      blink();
      delay(200);
      blink();
      delay(200);
      blink();
  }  
  
}

void setMotors(){
  int *intData = parser.getData();
  m1.setSpeed(intData[1], MOT_RPM);
  m2.setSpeed(intData[2], MOT_RPM);
  // analogWrite(TORCH_PIN, intData[3]);
  ledValue = intData[4];
  ledAnimation();
  // if (intData[3] != ledValue) {
  //     ledValue = intData[4];
  //     // analogWrite(ledPin, ledValue);
  // }
}

void setup(){
  Serial.begin(115200);

  analogWrite(TORCH_PIN, 250);
  delay(3000);
  // analogWrite(TORCH_PIN, 0);

  // #################  LED  ##############################

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  FastLED.addLeds<WS2811, LED_1, GRB>(leds1, NUM_OF_LEDS).setCorrection( TypicalLEDStrip );
  // FastLED.addLeds<WS2811, LED_2, GRB>(leds2, NUM_OF_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(BRIGHTNESS);

  FastLED.clear();
  FastLED.show();

  // #############  MOTORS  ################

  m1.begin(&Wire);
  m2.begin(&Wire);
  m1.setDirection(false);
  m2.setDirection(true);

  // ##############  GY85  #####################

  // GY85.init();
  compass.begin();

  // ##############  OS  #########################

  OS.attach(0, updateCompass, 400);
  OS.attach(1, printData, 100);
  // OS.attach(2, blink, 1200); 
  
  streaming();

}


void loop(){
  parser.update();
  if (parser.received()){
    setMotors();
  }
  
  OS.tick();
}

















// void setup() {
//   pinMode(2, OUTPUT);
//   digitalWrite(2, LOW);
//   pinMode(4, OUTPUT);
//   digitalWrite(4, LOW);
//   delay(2000);
// }

// void loop() {
//   digitalWrite(2, HIGH);
//   digitalWrite(4, LOW);
//   delay(2000);
  
//   digitalWrite(2, LOW);
//   digitalWrite(4, HIGH);
//   delay(2000);
  
//   digitalWrite(2, HIGH);
//   digitalWrite(4, HIGH);
//   delay(2000);

  
//   digitalWrite(2, LOW);
//   digitalWrite(4, LOW);
//   delay(2000);
// }