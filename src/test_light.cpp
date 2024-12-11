//////////////////     MPU-6050   ////////////////////////

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  delay(2000);
}

void loop() {
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  delay(2000);
  
  digitalWrite(2, LOW);
  digitalWrite(4, HIGH);
  delay(2000);
  
  digitalWrite(2, HIGH);
  digitalWrite(4, HIGH);
  delay(2000);

  
  digitalWrite(2, LOW);
  digitalWrite(4, LOW);
  delay(2000);
}