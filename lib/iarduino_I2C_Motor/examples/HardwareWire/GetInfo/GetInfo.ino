// ПРИМЕР ВЫВОДИТ ИНФОРМАЦИЮ О МОДУЛЕ:                                                 // * Строки со звёздочкой являются необязательными.
                                                                                       //
// Драйвер с мотором 1000rpm 12V с   энкодером:                                        //   https://iarduino.ru/shop/Mehanika/motor-reduktor-s-enkoderom-n20-1000rpm-12v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  500rpm  6V без энкодера:                                         //   https://iarduino.ru/shop/Mehanika/N20-500rpm-6V.html
// Драйвер с мотором  300rpm  6V с   энкодером:                                        //   https://iarduino.ru/shop/Expansion-payments/n20-300rpm-6v-flash-i2c.html
// Драйвер с мотором  300rpm  6V без энкодера:                                         //   https://iarduino.ru/shop/Expansion-payments/motor-reduktor-bez-enkodera-n20-300rpm-6v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  100rpm 12V с   энкодером:                                        //   https://iarduino.ru/shop/Mehanika/motor-reduktor-n20-500rpm-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором  100rpm  6V без энкодера:                                         //   https://iarduino.ru/shop/Mehanika/motor-reduktor-bez-enkodera-n20-100rpm-12v-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Драйвер с мотором   50rpm  6V с   энкодером:                                        //   https://iarduino.ru/shop/Expansion-payments/n20-50rpm-6v-flash-i2c.html
// Драйвер без мотора, под ваш мотор-редуктор:                                         //   https://iarduino.ru/shop/Expansion-payments/drayver-motora-s-upravlyayuschim-kontrollerom-flash-i2c.html
// Настройка драйвера  под ваш мотор-редуктор:                                         //   https://wiki.iarduino.ru/page/motor-driver-controller-i2c/
// Информация о подключении модулей к шине I2C:                                        //   https://wiki.iarduino.ru/page/i2c_connection/
// Информация о модуле и описание библиотеки:                                          //   https://wiki.iarduino.ru/page/motor-driver-i2c/
                                                                                       //
#include <Wire.h>                                                                      //   Подключаем библиотеку для работы с аппаратной шиной I2C, до подключения библиотеки iarduino_I2C_Motor.
#include <iarduino_I2C_Motor.h>                                                        //   Подключаем библиотеку для работы с мотором I2C-flash.
iarduino_I2C_Motor mot(0x09);                                                          //   Объявляем объект mot для работы с функциями и методами библиотеки iarduino_I2C_Motor, указывая адрес модуля на шине I2C.
                                                                                       //   Если объявить объект без указания адреса (iarduino_I2C_Motor mot;), то адрес будет найден автоматически.
void setup(){                                                                          //
    delay(500);                                                                        // * Ждём завершение переходных процессов связанных с подачей питания.
    Serial.begin(9600);                                                                //
    while(!Serial){;}                                                                  // * Ждём завершения инициализации шины UART.
    mot.begin(&Wire); // &Wire1, &Wire2 ...                                            //   Инициируем работу с мотором, указав ссылку на объект для работы с шиной I2C на которой находится мотор (по умолчанию &Wire).
    bool f1 = mot.getInvGear()&1;                                                      //   Получаем флаг инверсии полярности мотора  из flash памяти модуля.
    bool f2 = mot.getInvGear()&2;                                                      //   Получаем флаг инверсии вращения редуктора из flash памяти модуля.
                                                                                       //
    Serial.print( F("Номинальное напряжение питания мотора:      ") );                 //
    Serial.print( mot.getVoltage() );                                                  //   Выводим номинальное напряжение питания мотора из flash памяти модуля.
    Serial.print( F(" В.\r\n") );                                                      //
                                                                                       //
    Serial.print( F("Номинальная скорость вращения вала:         ") );                 //
    Serial.print( mot.getNominalRPM() );                                               //   Выводим номинальную скорость вращения вала из flash памяти модуля.
    Serial.print( F(" об/мин.\r\n") );                                                 //
                                                                                       //
    Serial.print( F("Передаточное отношение редуктора:           1:") );               //
    Serial.print( mot.getReducer() );                                                  //   Выводим передаточное отношение редуктора из flash памяти модуля.
    Serial.print( F("\r\n") );                                                         //
                                                                                       //
    Serial.print( F("Количество полюсов кольцевого магнита:      ") );                 //
    Serial.print( mot.getMagnet() );                                                   //   Выводим количество однополярных полюсов кольцевого магнита из flash памяти модуля.
    Serial.print( F(" шт. (одной полярности)\r\n") );                                  //
                                                                                       //
    Serial.print( F("Полярность подключения мотора:              ") );                 //
    Serial.print( f1? "обратная. (это учитывается модулем)\r\n":"прямая.\r\n" );       //   Выводим флаг инверсии полярности мотора из flash памяти модуля.
                                                                                       //
    Serial.print( F("Направление вала редуктора и ротора мотора: ") );                 //
    Serial.print( f2? "встречное. (это учитывается модулем)\r\n":"прямое.\r\n" );      //   Выводим флаг инверсии редуктора из flash памяти модуля.
                                                                                       //
    Serial.print( F("Выходной вал редуктора вращается:           по ч.с. ") );         //   Инверсии полярности мотора и направления вращения редуктора хранятся в flash памяти модуля и учитываются им.
    Serial.print( F("(при положительных скоростях)\r\n") );                            //
                                                                                       //
    if( mot.getMagnet()==0 ){                                                          //   Если на роторе мотора отсутствует кольцевой магнит или у модуля отутствуют датчики Холла.
    Serial.print( F("Модуль не поддерживает функции:             ") );                 //
    Serial.print( F("setSpeed(), getSpeed(), setStop(), getStop() и getSum(),\r\n") ); //
    Serial.print( F("вызванные с параметрами:                    ") );                 //
    Serial.print( F("MOT_RPM, MOT_M_S, MOT_MET и MOT_REV.\r\n") );                     //
    }                                                                                  //
                                                                                       //
}                                                                                      //
                                                                                       //
void loop(){                                                                           //
}                                                                                      //
                                                                                       //   
// Для определения реального передаточного отношения редуктора                         //
// запустите пример библиотеки FindReducer                                             //
                                                                                       //   
// Для определения реального количества магнитных полюсов кольцевого магнита           //
// запустите пример библиотеки FindMagnet                                              //