// ДАННЫЙ ПРИМЕР ИНФОРМИРУЕТ О ЗНАЧЕНИИ ШИМИ:     // * Строки со звёздочкой являются необязательными.
                                                  //
// Драйвер с мотором 1000rpm 12V с   энкодером:   //   https://iarduino.ru/shop/Mehanika/motor-reduktor-s-enkoderom-n20-1000rpm-12v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  500rpm  6V без энкодера:    //   https://iarduino.ru/shop/Mehanika/N20-500rpm-6V.html
// Драйвер с мотором  300rpm  6V с   энкодером:   //   https://iarduino.ru/shop/Expansion-payments/n20-300rpm-6v-flash-i2c.html
// Драйвер с мотором  300rpm  6V без энкодера:    //   https://iarduino.ru/shop/Expansion-payments/motor-reduktor-bez-enkodera-n20-300rpm-6v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  100rpm 12V с   энкодером:   //   https://iarduino.ru/shop/Mehanika/motor-reduktor-n20-500rpm-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  100rpm  6V без энкодера:    //   https://iarduino.ru/shop/Mehanika/motor-reduktor-bez-enkodera-n20-100rpm-12v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором   50rpm  6V с   энкодером:   //   https://iarduino.ru/shop/Expansion-payments/n20-50rpm-6v-flash-i2c.html
// Драйвер без мотора, под ваш мотор-редуктор:    //   https://iarduino.ru/shop/Expansion-payments/drayver-motora-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Настройка драйвера  под ваш мотор-редуктор:    //   https://wiki.iarduino.ru/page/motor-driver-controller-i2c/
// Информация о подключении модулей к шине I2C:   //   https://wiki.iarduino.ru/page/i2c_connection/
// Информация о модуле и описание библиотеки:     //   https://wiki.iarduino.ru/page/motor-driver-i2c/
                                                  //
#include <Wire.h>                                 //   Подключаем библиотеку для работы с аппаратной шиной I2C, до подключения библиотеки iarduino_I2C_Motor.
#include <iarduino_I2C_Motor.h>                   //   Подключаем библиотеку для работы с мотором I2C-flash.
iarduino_I2C_Motor mot(0x09);                     //   Объявляем объект mot для работы с функциями и методами библиотеки iarduino_I2C_Motor, указывая адрес модуля на шине I2C.
                                                  //   Если объявить объект без указания адреса (iarduino_I2C_Motor mot;), то адрес будет найден автоматически.
void setup(){                                     //
    delay(500);                                   // * Ждём завершение переходных процессов связанных с подачей питания.
    Serial.begin(9600);                           //
    while(!Serial){;}                             // * Ждём завершения инициализации шины UART.
    mot.begin(&Wire); // &Wire1, &Wire2 ...       //   Инициируем работу с мотором, указав ссылку на объект для работы с шиной I2C на которой находится мотор (по умолчанию &Wire).
}                                                 //
                                                  //
void loop(){                                      //
    mot.setSpeed(50.0, MOT_PWM);                  //   Запускаем мотор установив ШИМ = 50% (ШИМ указывается в % 0 до ±100.0).
    Serial.println( mot.getSpeed(MOT_PWM) );      //   Выводим установленное значение ШИМ.
    delay(5000);                                  //   Приостанавливаем выполнение скетча на 5 секунд.
    mot.setSpeed(-50.0, MOT_PWM);                 //   Запускаем мотор установив ШИМ = -50% (ШИМ указывается в % 0 до ±100.0).
    Serial.println( mot.getSpeed(MOT_PWM) );      //   Выводим установленное значение ШИМ.
    delay(5000);                                  //   Приостанавливаем выполнение скетча на 5 секунд.
}                                                 //
