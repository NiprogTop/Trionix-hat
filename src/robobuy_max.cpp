#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include "Wire.h"
#include <GyverOS.h>
#include <SerialParser.h>
#include <Servo.h>
#include <TroykaGPS.h>
#include <GY_85.h>
#include <DFRobot_QMC5883.h>

Servo dr1, dr2, Manipul;


#define pin1 3

#define pin2 2

#define ledPin 7 // 5

#define GPS_SERIAL Serial3

#define PARSE_AMOUNT 5
// #define PARSE_AMOUNT 7

String depthMeters;

GY_85 GY85;
// int ax=0;
// int ay=0;
// int az =0;
int gx;
int gy;
int gz;

GPS gps(GPS_SERIAL);
// задаём размер массива для времени, даты, широты и долготы
#define MAX_SIZE_MASS 16
// массив для хранения текущего времени
char strTime[MAX_SIZE_MASS];
// массив для хранения текущей даты
char strDate[MAX_SIZE_MASS];
// массив для хранения широты в градусах, минутах и секундах
char latitudeBase60[MAX_SIZE_MASS];
// массив для хранения долготы в градусах, минутах и секундах
char longitudeBase60[MAX_SIZE_MASS];


DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS); /* I2C addr */
int heading = 0;
float declinationAngle = (12.0 + (11.0 / 60.0)) / (180 / PI);
sVector_t mag;


GyverOS<3> OS; 
SerialParser parser(PARSE_AMOUNT);

int16_t dr1_val, dr2_val, manip_val;
int16_t dr1_val_new, dr2_val_new, manip_val_new;
int intData = 0;

int ledValue = 0;

int timr = 0;

int mode = 1; // 1 - streaming

void initMotors(){
  dr1.attach(pin1);
  dr1.writeMicroseconds(1480);
  dr2.attach(pin2);
  dr2.writeMicroseconds(1480);
  // Manipul.attach(ServoPin);
  // Manipul.writeMicroseconds(2500);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 1);
  delay(7000);
  digitalWrite(ledPin, 0);
}


void printServiceMsg(String msg)
{
    Serial.print("#0 " + msg + ";");    
}

void stopStreaming()
{
    // OS.stop(0);
    OS.stop(1);
    OS.stop(2);
}

void straeming()
{
    mode = 1;
    // OS.start(0);
    OS.start(1);
    OS.start(2);
}

void printData()
{
        String answer = "#3 "
                    // // roll
                    + String(gy) + " "
                    // pitch
                    + String(gx) + " "
                    // heading
                    // + String(mpu.getAngleZ()) + ";";
                    + String(heading) + " "
                    //depth under robot
                    + String(depthMeters) + ";";

                    // +String(GPS_data)
                    // temp
                    // + String(sensor.temperature()) + ";";
                    // end

        Serial.println(answer);
}

void updateGPS(){
        // если пришли данные с GPS-модуля
    if (gps.available()) {
        // считываем данные и парсим
        gps.readParsing();
        // проверяем состояние GPS-модуля
        switch (gps.getState()) {
        // всё OK
        case GPS_OK:
            Serial.println("GPS is OK");
            // выводим координаты широты и долготы
            // 1. в градусах, минутах и секундах
            // 2. градусах в виде десятичной дроби
            Serial.println("GPS Coordinates: ");
            gps.getLatitudeBase60(latitudeBase60, MAX_SIZE_MASS);
            gps.getLongitudeBase60(longitudeBase60, MAX_SIZE_MASS);
            Serial.print("Latitude\t");
            Serial.print(latitudeBase60);
            Serial.print("\t\t");
            Serial.println(gps.getLatitudeBase10(), 6);
            Serial.print("Longitude\t");
            Serial.print(longitudeBase60);
            Serial.print("\t\t");
            Serial.println(gps.getLongitudeBase10(), 6);
            // выводим количество видимых спутников
            Serial.print("Sat: ");
            Serial.println(gps.getSat());
            // выводим текущую скорость
            Serial.print("Speed: ");
            Serial.println(gps.getSpeedKm());
            // выводим высоту над уровнем моря
            Serial.print("Altitude: ");
            Serial.println(gps.getAltitude());
            // выводим текущее время
            Serial.print("Time: ");
            gps.getTime(strTime, MAX_SIZE_MASS);
            gps.getDate(strDate, MAX_SIZE_MASS);
            Serial.write(strTime);
            Serial.println();
            // выводим текущую дату
            Serial.print("Date: ");
            Serial.write(strDate);
            Serial.println("\r\n");
            // каждую переменную дату и времени можно выводить отдельно
            /*    Serial.print(gps.getHour());
                  Serial.print(gps.getMinute());
                  Serial.print(gps.getSecond());
                  Serial.print(gps.getDay());
                  Serial.print(gps.getMonth());
                  Serial.print(gps.getYear());
            */
            break;
        // ошибка данных
        case GPS_ERROR_DATA:
            Serial.println("GPS error data");
            break;
        // нет соединения со спутниками
        case GPS_ERROR_SAT:
            Serial.println("GPS is not connected to satellites!!!");
            break;
        }
    }
}

void updateCompass()
{    
    compass.setDeclinationAngle(declinationAngle);
    mag = compass.readRaw();
    compass.getHeadingDegrees();
    heading = mag.HeadingDegress;
}

void updateEcholot(){
  if (Serial2.available()) {
    // Read the incoming data
    String nmeaSentence = Serial2.readStringUntil('\n');
    depthMeters = parseSDDBT(nmeaSentence);
  }
}

String parseSDDBT(String sentence) {
  // Split the sentence into parts based on commas
  int commaIndex1 = sentence.indexOf(',');
  int commaIndex2 = sentence.indexOf(',', commaIndex1 + 1);
  int commaIndex3 = sentence.indexOf(',', commaIndex2 + 1);
  int commaIndex4 = sentence.indexOf(',', commaIndex3 + 1);
  int commaIndex5 = sentence.indexOf(',', commaIndex4 + 1);

  // Extract the depth in meters (between the 4th and 5th commas)
  String depthStr = sentence.substring(commaIndex3+1, commaIndex4);

  // Convert the depth string to a float
  return depthStr;
}


void updateIMU()
{
    int* accelerometerReadings = GY85.readFromAccelerometer();
    gx = GY85.accelerometer_x(accelerometerReadings);
    gy = GY85.accelerometer_y(accelerometerReadings);
    gz = GY85.accelerometer_z(accelerometerReadings);

}




void setup() {
  pinMode(2, OUTPUT); // activate level comunication
  digitalWrite(2, HIGH);

  Serial.begin(115200);
  Wire.begin();

  initMotors();

  // ############################### GPS init ##############################

//   while (!Serial) { }
//     Serial.print("Serial init OK\r\n");
//     // открываем Serial-соединение с GPS-модулем на скорости 115200 бод
//     GPS_SERIAL.begin(115200);
//     // печатаем строку
//     Serial.println("GPS init is OK on speed 115200");
//     // изменяем скорость обещение GPS-модуля с управляющей платой на 9600 бод
//     // используем NMEA-команду «$PMTK251,9600*17\r\n»
//     GPS_SERIAL.write("$PMTK251,9600*17\r\n");
//     // закрываем Serial-соединение с GPS-модулем
//     GPS_SERIAL.end();
//     // открываем Serial-соединение с GPS-модулем на скорости 9600 бод
//     GPS_SERIAL.begin(9600);
//     // печатаем строку
//     Serial.print("GPS init is OK on speed 9600");

  // #####   Compas sensor activation  #####
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

  GY85.init();

  // ################### Echolot init #######################
  Serial2.begin(115200);
  Serial.begin(115200);

  OS.attach(0, updateGPS, 120);
  OS.attach(1, printData, 100);
  OS.attach(2, updateCompass, 300);
}


void setMotors()
{
    int *intData = parser.getData();
    dr1_val_new = map(intData[1], -100, 100, 1080, 1880);
    dr2_val_new = map(intData[2], -100, 100, 1080, 1880);
    // manip_val_new = map(intData[6], 0, 100, 1800, 2500);

    dr1.writeMicroseconds(dr1_val_new);

    dr2.writeMicroseconds(dr2_val_new);
    // Manipul.writeMicroseconds(manip_val_new);

    if (intData[3] != ledValue) {
        ledValue = intData[3];
        analogWrite(ledPin, ledValue);
    }
}


void loop() {
  parser.update();
  if (parser.received()){
    int comand = parser.getData()[0];
    
    if (comand == 3 && mode == 2){straeming();}

    setMotors();
  }
  

  updateIMU();

  OS.tick();

}