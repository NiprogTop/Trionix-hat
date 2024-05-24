#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif


#include <SoftwareSerial.h>

#include <ModbusMaster.h>
#define RX_PIN 12
#define TX_PIN 11
#define RTS_PIN 13
SoftwareSerial softSerial = SoftwareSerial(RX_PIN, TX_PIN);
int  depth = 0;
ModbusMaster modbusMaster;

void preTransmission(){
  digitalWrite(RTS_PIN , 1);
}
void postTransmission(){
  digitalWrite(RTS_PIN , 0);
}
void setup() {
  Serial.begin(115200);
  softSerial.begin(115200);
  pinMode(RTS_PIN, OUTPUT);
  digitalWrite(RTS_PIN, 0);
  modbusMaster.begin(1, softSerial);
  modbusMaster.preTransmission(preTransmission);
  modbusMaster.postTransmission(postTransmission);
  // put your setup code here, to run once:
}

void loop() {
  depth = modbusMaster.readHoldingRegisters(101, 1);
  // if(depth == modbusMaster.ku8MBSuccess){
    Serial.println(depth);
  // }
  delay(500);
}