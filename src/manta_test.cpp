#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <GyverOS.h>
#include <SerialParser.h>
#include <Servo.h>

Servo dr1, dr2;


#define pin1 3

#define pin2 5

#define PARSE_AMOUNT 6
// #define PARSE_AMOUNT 7


SerialParser parser(PARSE_AMOUNT);

int16_t dr1_val, dr2_val;
int16_t dr1_val_new, dr2_val_new;
int intData = 0;

// int mode = 1; // 1 - streaming


// float sampleRate = 25;

void initMotors(){
  dr1.attach(pin1);
  dr1.writeMicroseconds(1420);
  dr2.attach(pin2);
  dr2.writeMicroseconds(1460);
  delay(1000);
  // digitalWrite(ledPin, 0);
}

int value = 30;

void setup() {

  Serial.begin(115200);

  initMotors();
}


void setMotors()
{
    int *intData = parser.getData();
    dr1_val_new = map(intData[0], 0, 100, 420, 2420);
    dr2_val_new = map(intData[1], 0, 100, 460, 2460);
    // manip_val_new = map(intData[6], 0, 100, 1800, 2500);

    dr1.writeMicroseconds(dr1_val_new);

    dr2.writeMicroseconds(dr2_val_new);
    
}


void loop() {
  parser.update();
  if (parser.received()){
    setMotors();
//    Serial.println("go!");
  }

//  for (int i = value; i <= 65; i++) {
//    value = i;
//    Serial.println(value);
//    dr1.writeMicroseconds(map(value, 0, 100, 1500, 2500));
//    dr2.writeMicroseconds(map(value, 0, 100, 1000, 2000));
//    delay(15); // Задержка в полсекунды
//  }
//
//  // Уменьшаем значение переменной на 1, пока не достигнем 30
//  for (int i = value; i >= 25; i--) {
//    value = i;
//    Serial.println(value);
//    dr1.writeMicroseconds(map(value, 0, 100, 500, 2500));
//    dr2.writeMicroseconds(map(value, 0, 100, 1000, 2000));
//    delay(15); // Задержка в полсекунды
//  }
}