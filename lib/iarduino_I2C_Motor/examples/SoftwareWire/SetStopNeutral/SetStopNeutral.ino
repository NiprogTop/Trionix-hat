// ПРИМЕР ОСТАНАВЛИВАЕТ МОТОР С БЛОКИРОВКОЙ ВАЛА И БЕЗ НЕЁ   // * Строки со звёздочкой являются необязательными.
// Требуется установить библиотеку <iarduino_I2C_Software.h> //   https://iarduino.ru/file/627.html
                                                             //
// Драйвер с мотором 1000rpm 12V с   энкодером:              //   https://iarduino.ru/shop/Mehanika/motor-reduktor-s-enkoderom-n20-1000rpm-12v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  500rpm  6V без энкодера:               //   https://iarduino.ru/shop/Mehanika/N20-500rpm-6V.html
// Драйвер с мотором  300rpm  6V с   энкодером:              //   https://iarduino.ru/shop/Expansion-payments/n20-300rpm-6v-flash-i2c.html
// Драйвер с мотором  300rpm  6V без энкодера:               //   https://iarduino.ru/shop/Expansion-payments/motor-reduktor-bez-enkodera-n20-300rpm-6v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  100rpm 12V с   энкодером:              //   https://iarduino.ru/shop/Mehanika/motor-reduktor-n20-500rpm-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  100rpm  6V без энкодера:               //   https://iarduino.ru/shop/Mehanika/motor-reduktor-bez-enkodera-n20-100rpm-12v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором   50rpm  6V с   энкодером:              //   https://iarduino.ru/shop/Expansion-payments/n20-50rpm-6v-flash-i2c.html
// Драйвер без мотора, под ваш мотор-редуктор:               //   https://iarduino.ru/shop/Expansion-payments/drayver-motora-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Настройка драйвера  под ваш мотор-редуктор:               //   https://wiki.iarduino.ru/page/motor-driver-controller-i2c/
// Информация о подключении модулей к шине I2C:              //   https://wiki.iarduino.ru/page/i2c_connection/
// Информация о модуле и описание библиотеки:                //   https://wiki.iarduino.ru/page/motor-driver-i2c/
                                                             //
#include <iarduino_I2C_Software.h>                           //   Подключаем библиотеку для работы с программной шиной I2C, до подключения библиотеки iarduino_I2C_Motor.
SoftTwoWire sWire(3,4);                                      //   Создаём объект программной шины I2C указав выводы которым будет назначена роль линий: SDA, SCL.
                                                             //
#include <iarduino_I2C_Motor.h>                              //   Подключаем библиотеку для работы с мотором I2C-flash.
iarduino_I2C_Motor mot(0x09);                                //   Объявляем объект mot для работы с функциями и методами библиотеки iarduino_I2C_Motor, указывая адрес модуля на шине I2C.
                                                             //   Если объявить объект без указания адреса (iarduino_I2C_Motor mot;), то адрес будет найден автоматически.
void setup(){                                                //
    delay(500);                                              // * Ждём завершение переходных процессов связанных с подачей питания.
    mot.begin(&sWire);                                       //   Инициируем работу с мотором, указав ссылку на объект для работы с шиной I2C на которой находится мотор (по умолчанию &Wire).
}                                                            //
                                                             //
void loop(){                                                 //
//  Запускаем и плавно останавливаем мотор:                  //
    mot.setSpeed(100.0, MOT_PWM); delay(500);                //   Запускаем мотор на максимальной скорости, указав максимальное значение ШИМ = 100%.
    mot.setStopNeutral(true);                                //   Указываем освободить мотор при его остановке. Ротор остановленного мотора можно вращать.
    mot.setStop();                delay(1000);               //   Останавливаем мотор. Обратите внимание на то, что ротор мотора останавливается плавно.
//  Запускаем и резко останавливаем мотор:                   //
    mot.setSpeed(100.0, MOT_PWM); delay(500);                //   Запускаем мотор на максимальной скорости, указав максимальное значение ШИМ = 100%.
    mot.setStopNeutral(false);                               //   Указываем не освобождать мотор при его остановке. Ротор остановленного мотора будет застопорен.
    mot.setStop();                delay(1000);               //   Останавливаем мотор. Обратите внимание на то, что ротор мотора останавливается резко.
}                                                            //
//                                                           //
//  Функция setStopNeutral() не останавливает                //
//  мотор, а указывает как он будет себя вести               //
//  при остановке.                                           //
//  Останавливается мотор функцией setStop().                //