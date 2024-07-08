#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif


#include <GyverOS.h>
#include <SerialParser.h>
#include <Servo.h>

Servo dr1, dr2, dr3, dr4, dr5;


#define pin1 6 // 4

#define pin2 5 // 3

#define pin3 3 // 5

#define pin4 9 // 1

#define pin5 10 // 2

#define PARSE_AMOUNT 6
// #define PARSE_AMOUNT 7


SerialParser parser(PARSE_AMOUNT);

int16_t dr1_val, dr2_val, dr3_val, dr4_val, dr5_val;
int16_t dr1_val_new, dr2_val_new, dr3_val_new, dr4_val_new, dr5_val_new;
int intData = 0;

int cen_ang_1 = 97;
int cen_ang_2 = 122;
int cen_ang_3 = 50;
int cen_ang_4 = 72;
int cen_ang_5 = 122;

// int mode = 1; // 1 - streaming


// float sampleRate = 25;

void initMotors(){
  dr1.attach(pin1);
  // dr1.writeMicroseconds(1500);
  dr1.write(cen_ang_1);
  dr2.attach(pin2);
  dr2.write(cen_ang_2);
  dr3.attach(pin3);
  dr3.write(cen_ang_3);
  dr4.attach(pin4);
  dr4.write(cen_ang_4);
  dr5.attach(pin5);
  dr5.write(cen_ang_5);

  // dr1.writeMicroseconds(1500);
  // dr2.writeMicroseconds(1500);
  // dr3.writeMicroseconds(1500);
  // dr4.writeMicroseconds(1500);
  // dr5.writeMicroseconds(1500);

  delay(1000);
  // digitalWrite(ledPin, 0);
}

int value = 30;

void setup() {

  Serial.begin(115200);

  initMotors();
}

void spinn_m(){
  int s2[25] {0,  0, 0,  0,  4,  7,  11, 14, 18, 22, 14, 7,  0,  0,  0,  0,  -3, -7, -10,  -14,  -17,  -20,  -14,  -7, 0};
  int s1[25] {0,  3,  6,  9,  9,  9,  9,  9,  9,  9,  6,  3,  0,  -3, -6, -9, -9, -9, -9, -9, -9, -9, -6, -3, 0};

  for (size_t i = 0; i < 25; i++)
  {
    Serial.println(s1[i]);
    dr1_val_new = s1[i] * -1 + cen_ang_1;
    dr2_val_new = s2[i] * -1 + cen_ang_2;

    dr1.write(dr1_val_new);

    dr2.write(dr2_val_new);
    delay(100);
  }
  
}


void setMotors()
{
//    int *intData = parser.getData();
//    dr1_val_new = map(intData[0], 0, 100, 420, 2420);
//    dr2_val_new = map(intData[1], 0, 100, 460, 2460);
//    // manip_val_new = map(intData[6], 0, 100, 1800, 2500);
//
//    dr1.writeMicroseconds(dr1_val_new);
//
//    dr2.writeMicroseconds(dr2_val_new);
    

     int *intData = parser.getData();
//     dr1_val_new = map(intData[0], 0, 100, 45, );
//     dr2_val_new = map(intData[1], 0, 100, 460, 2460);
     dr1_val_new = intData[3] * -1 + cen_ang_1;
     dr2_val_new = intData[2] * -1 + cen_ang_2;
     dr3_val_new = intData[4] * -1 + cen_ang_3;
     dr4_val_new = intData[1] * -1 + cen_ang_4;
     dr5_val_new = intData[0] * -1 + cen_ang_5;
     // manip_val_new = map(intData[6], 0, 100, 1800, 2500);

     dr1.write(dr1_val_new);

     dr2.write(dr2_val_new);

     dr3.write(dr3_val_new);

     dr4.write(dr4_val_new);

     dr5.write(dr5_val_new);

    // Serial.println(dr2_val_new);
    // delay(200);
}


void loop() {
  parser.update();
  if (parser.received()){
    setMotors();
    // Serial.println("go!");
    // delay(20);
  }
  // spinn_m();
//  delay(200);



//  Serial.println("go-1!");

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