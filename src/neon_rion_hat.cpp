#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif


#define RX_PIN 2  // Пин Arduino для приёма данных (DI на преобразователе RS485)
#define TX_PIN A0
#define DE_PIN A1
#include <SoftwareSerial.h>
SoftwareSerial mySerial(RX_PIN, TX_PIN);
int index = 0;
uint16_t buffer[14];

uint16_t heading = 0;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(DE_PIN, OUTPUT);
  
  // Изначально устанавливаем режим приема данных
  
  digitalWrite(DE_PIN, LOW);
  mySerial.begin(9600);
  Serial.begin(9600);

}

float ConvertFloatValue(uint16_t *buffer)
{
	float wholePart = (buffer[0] & 0x0F) * 100 + (buffer[1] >> 4) * 10 + (buffer[1] & 0x0F) * 1,
		  fractionalPart = (buffer[2] >> 4) * 10 + (buffer[2] & 0x0F) * 1;

	float result = wholePart + fractionalPart / 100;

	return (buffer[0] >> 4) ? -result : result;
}


void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(DE_PIN, HIGH);
  byte cmd[] = {0x68, 0x04, 0x00, 0x04, 0x08};
  mySerial.write(cmd, 5);
  digitalWrite(DE_PIN, LOW);
  delay(100);
  while(index < 14){
    if (mySerial.available()) {
        buffer[index] = mySerial.read();
    }
    index ++;
  }
  for (int i = 0; i < 14; i++) {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  heading = ConvertFloatValue(&buffer[10]);
  Serial.println(heading);

  index = 0;
  
}