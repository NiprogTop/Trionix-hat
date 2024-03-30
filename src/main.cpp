#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <Servo.h>
Servo Manipul;

void setup() {
  Manipul.attach(3);
  // Manipul.writeMicroseconds(2500);
}

void loop() {
  Manipul.writeMicroseconds(2200);
}