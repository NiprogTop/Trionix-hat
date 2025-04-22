/*!
 * @file getCompassdata.ino
 * @brief Output the compass data
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      [dexian.huang](952838602@qq.com)
 * @version  V1.0
 * @date  2017-7-3
 * @url https://github.com/DFRobot/DFRobot_QMC5883
 */
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass(&Wire, HMC5883L_ADDRESS); /* I2C addr */

void setup()
{
  Serial.begin(115200);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

  // if(compass.isHMC())
  // {
  //   Serial.println("Initialize HMC5883");
  // }
  // else if(compass.isQMC())
  // {
  //   Serial.println("Initialize QMC5883");
  // }
  // else if(compass.isVCM())
  // {
  //   Serial.println("Initialize VCM5883L");
  // }
  delay(1000);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
}
void loop()
{
  /**
   * @brief  Set declination angle on your location and fix heading
   * @n      You can find your declination on: http://magnetic-declination.com/
   * @n      (+) Positive or (-) for negative
   * @n      For Bytom / Poland declination angle is 4'26E (positive)
   * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
   */
  // float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  // compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  Serial.print("X:");
  Serial.print(mag.XAxis);
  Serial.print(" Y:");
  Serial.print(mag.YAxis);
  Serial.print(" Z:");
  Serial.println(mag.ZAxis);
  Serial.print("Degress = ");
  Serial.println(int(mag.HeadingDegress)* 3 % 360);
  delay(100);
}