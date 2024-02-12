#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

void setup(){
  Serial.begin(115200);
}

void loop(){
  Serial.println("/n lol /n");
  delay(200);
}