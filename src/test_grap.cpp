#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

void setup() {
    pinMode(8, OUTPUT);
    pinMode(7, OUTPUT);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
}

void loop(){
    digitalWrite(8, HIGH);
    delay(5);
    digitalWrite(7, LOW);
    delay(2000);
    digitalWrite(8, LOW);
    delay(5);
    digitalWrite(7, HIGH);
    delay(2000);
    digitalWrite(7, LOW);
    delay(5);
}


