#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

/*
HMC5883L_Example.pde - Example sketch for integration with an HMC5883L triple axis magnetomerwe.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// // Reference the I2C Library
// #include <Wire.h>
// // Reference the HMC5883L Compass Library
// #include <HMC5883L.h>

// // Store our compass as a variable.
// HMC5883L compass;
// // Record any errors that may occur in the compass.
// int error = 0;

// // Out setup routine, here we will configure the microcontroller and compass.
// void setup()
// {
//   // Initialize the serial port.
//   Serial.begin(115200);

//   Serial.println("Starting the I2C interface.");
//   Wire.begin(); // Start the I2C interface.

//   Serial.println("Constructing new HMC5883L");
//   compass = HMC5883L(); // Construct a new HMC5883 compass.
    
//   Serial.println("Setting scale to +/- 1.3 Ga");
//   error = compass.SetScale(1.3); // Set the scale of the compass.
//   if(error != 0) // If there is an error, print it out.
//     Serial.println(compass.GetErrorText(error));
  
//   Serial.println("Setting measurement mode to continous.");
//   error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
//   if(error != 0) // If there is an error, print it out.
//     Serial.println(compass.GetErrorText(error));
// }

// // Output the data down the serial port.
// void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
// {
//    Serial.print("Raw:\t");
//    Serial.print(raw.XAxis);
//    Serial.print("   ");   
//    Serial.print(raw.YAxis);
//    Serial.print("   ");   
//    Serial.print(raw.ZAxis);
//    Serial.print("   \tScaled:\t");
   
//    Serial.print(scaled.XAxis);
//    Serial.print("   ");   
//    Serial.print(scaled.YAxis);
//    Serial.print("   ");   
//    Serial.print(scaled.ZAxis);

//    Serial.print("   \tHeading:\t");
//    Serial.print(heading);
//    Serial.print(" Radians   \t");
//    Serial.print(headingDegrees);
//    Serial.println(" Degrees   \t");
// }

// // Our main program loop.
// void loop()
// {
//   // Retrive the raw values from the compass (not scaled).
//   MagnetometerRaw raw = compass.ReadRawAxis();
//   // Retrived the scaled values from the compass (scaled to the configured scale).
//   MagnetometerScaled scaled = compass.ReadScaledAxis();
  
//   // Values are accessed like so:
//   int MilliGauss_OnThe_XAxis = scaled.YAxis;// (or YAxis, or ZAxis)

//   // Calculate heading when the magnetometer is level, then correct for signs of axis.
//   float heading = atan2(scaled.YAxis, scaled.XAxis);
  
//   // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
//   // Find yours here: http://www.magnetic-declination.com/
//   // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
//   // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
//   float declinationAngle = 0.0457;
//   heading += declinationAngle;
  
//   // Correct for when signs are reversed.
//   if(heading < 0)
//     heading += 2*PI;
    
//   // Check for wrap due to addition of declination.
//   if(heading > 2*PI)
//     heading -= 2*PI;
   
//   // Convert radians to degrees for readability.
//   float headingDegrees = heading * 180/M_PI; 

//   // Output the data via the serial port.
//   Output(raw, scaled, heading, headingDegrees);

//   // Normally we would delay the application by 66ms to allow the loop
//   // to run at 15Hz (default bandwidth for the HMC5883L).
//   // However since we have a long serial out (104ms at 9600) we will let
//   // it run at its natural speed.
//   // delay(66);
// }




/*
  HMC5883L Triple Axis Digital Compass. Compass Example.
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;

void setup()
{
  Serial.begin(115200);

  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0, 0);
}

void loop()
{
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();

  delay(100);
}







// #include "Wire.h"
// #include "HMC5883L2.h"

// HMC5883L compass;

// void setupHMC5883L(){
//   // инициализация HMC5883L, и проверка наличия ошибок
//   int error;  
//   error = compass.SetScale(0.8); // чувствительность датчика из диапазона: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1
//   if(error != 0) Serial.println(compass.GetErrorText(error)); // если ошибка, то выводим ее

//   error = compass.SetMeasurementMode(Measurement_Continuous); // установка режима измерений как Continuous (продолжительный)
//   if(error != 0) Serial.println(compass.GetErrorText(error)); // если ошибка, то выводим ее
// }

// float getHeading(){
//   // считываем данные с HMC5883L и рассчитываем  направление
//   MagnetometerScaled scaled = compass.ReadScaledAxis(); // получаем масштабированные элементы с датчика
//   float heading = atan2(scaled.YAxis, scaled.XAxis);    // высчитываем направление

//   // корректируем значения с учетом знаков
//   if(heading < 0) heading += 2*PI;
//   if(heading > 2*PI) heading -= 2*PI;

//   return heading * RAD_TO_DEG; // переводим радианы в градусы
// }

// void setup(){
//   Serial.begin(115200);
//   Wire.begin();
  
//   compass = HMC5883L();  // создаем экземпляр HMC5883L библиотеки
//   setupHMC5883L();       // инициализация HMC5883L
// }

// void loop(){
//   float heading = getHeading();
//   Serial.println(heading);
//   delay(250);
// }
