#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <FastLED.h>

#include <GY_85.h>

#include <SerialParser.h>

#include <GyverOS.h>

#include <Wire.h>
#include <iarduino_I2C_Motor.h>   

#define PARSE_AMOUNT 5
#define TORCH_PIN A1
#define NUM_OF_LEDS 14
#define LED_1 11
#define LED_2 12
#define BRIGHTNESS 250

GyverOS<3> OS; 
SerialParser parser(PARSE_AMOUNT);
GY_85 GY85;
CRGB leds1[NUM_OF_LEDS];
CRGB leds2[NUM_OF_LEDS];

iarduino_I2C_Motor m1(0x09);                   //   Объявляем объект mot_L для работы с функциями и методами библиотеки iarduino_I2C_Motor, указывая адрес модуля на шине I2C.
iarduino_I2C_Motor m2(0x0A); 
int* yaw = 0;
int LED_stat = 0;

bool policeStatus = false;

int levels[6]={50, 120, 90, 240, 150, 50};
bool status = true;

void updateCourse()
{
 yaw = GY85.readFromCompass();
}
void stopStreaming()
{
    OS.stop(0);
    OS.stop(1);
    OS.stop(2);

}

void streaming()
{
    OS.start(0);
    OS.start(1);
    OS.start(2);
}

void printServiceMsg(String msg)
{
    Serial.print("#0 " + msg + ";");    
}

void printData()
{
        String answer = "#3 "
                    + String(m1.getSum(MOT_MET)) + " "
                    + String(m2.getSum(MOT_MET)) + " "
                    + String(int(yaw)) + " ;"; 
                    // + String(LED_stat) + ";";
                  
                    

        Serial.println(answer);
    
}

void fillAllLed(int n, CRGB* leds, CRGB color){
    for(int i = 0; i <= n; i++){
      leds[i] = color;      
    }  
}

bool blink_stat = 0;

void blinkk(){
    fillAllLed(NUM_OF_LEDS, leds1, CRGB(0, 0, 0));
    fillAllLed(NUM_OF_LEDS, leds2, CRGB(0, 0, 0));
    FastLED.show();
  if (blink_stat){
    fillAllLed(NUM_OF_LEDS, leds1, CRGB( 255, 140, 0));
    fillAllLed(NUM_OF_LEDS, leds2, CRGB( 255, 140, 0));
    FastLED.show();
    blink_stat = 0;
  } else {
    fillAllLed(NUM_OF_LEDS, leds2, CRGB( 255, 140, 0));
    fillAllLed(NUM_OF_LEDS, leds1, CRGB( 255, 140, 0));
    FastLED.show();
    blink_stat = 1;
  }
//   fillAllLed(NUM_OF_LEDS, leds1, CRGB( 255, 140, 0));
//   fillAllLed(NUM_OF_LEDS, leds2, CRGB( 255, 140, 0));
//   FastLED.show();
//   delay(200);
//   fillAllLed(NUM_OF_LEDS, leds1, CRGB( 0, 0, 0));
//   fillAllLed(NUM_OF_LEDS, leds2, CRGB( 0, 0, 0));
//   FastLED.show();
}


void policeLight(){
      for(int i = 0; i < levels; i++){
        FastLED.setBrightness(levels[i]);
        if(i == 5){
          status = !status;
        }
        if(status){
          fillAllLed(NUM_OF_LEDS, leds1, CRGB(0, 0, 255));
        }else{
          fillAllLed(NUM_OF_LEDS, leds2, CRGB(255, 0, 0));
        }
        FastLED.show();
  }
}

void setMotors(){
   int *intData = parser.getData();
   m1.setSpeed(intData[1], MOT_RPM);
   m2.setSpeed(intData[2], MOT_RPM);
   analogWrite(TORCH_PIN, intData[3]);
   LED_stat = intData[4];
}

void ledAnimation()
{
  if(LED_stat == 0){
      FastLED.clear();
      FastLED.show();
  }
  if(LED_stat == 1){
    fillAllLed(NUM_OF_LEDS, leds1, CRGB( 0, 255, 0));
    fillAllLed(NUM_OF_LEDS, leds2, CRGB( 0, 255, 0));
    FastLED.show();
  } 
  if(LED_stat == 2){
      policeLight();
  }
  if(LED_stat == 3){
      blinkk();
  }

 
}

void setup() {
  m1.begin(&Wire);
  m2.begin(&Wire);
  m1.setDirection(false);
  m2.setDirection(true);
  Serial.begin(115200);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(TORCH_PIN, OUTPUT);


  FastLED.addLeds<WS2811, LED_1, GRB>(leds1, NUM_OF_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<WS2811, LED_2, GRB>(leds2, NUM_OF_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(BRIGHTNESS);

  FastLED.clear();

  GY85.init();


  



  policeLight();
  
  OS.attach(0, updateCourse, 120);
  OS.attach(1, printData, 100);
//   OS.attach(2, blinkk, 200); 
  
  streaming();
}
void loop() {
//   parser.update();
//   if (parser.received()){
//     setMotors();
//   }
  blinkk();
  delay(500);
  
  OS.tick();

}
