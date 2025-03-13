// ВЫВОД КОЛИЧЕСТВА СОВЕРШЁННЫХ ОБОРОТОВ И ПРОЙДЕННОГО ПУТИ: // * Строки со звёздочкой являются необязательными.
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
    Serial.begin(9600);                                      //
    while(!Serial){;}                                        // * Ждём завершения инициализации шины UART.
    mot.begin(&sWire);                                       //   Инициируем работу с мотором, указав ссылку на объект для работы с шиной I2C на которой находится мотор (по умолчанию &Wire).
    mot.radius = 12.2;                                       //   Указываем радиус колеса в мм.
    mot.setSpeed(60, MOT_RPM);                               //   Запускаем мотор на скорости 60 об/мин.
}                                                            //
                                                             //
void loop(){                                                 //
    uint32_t i = millis()/1000;                              //   Получаем количество пройденных сеунд.
    Serial.print("Совершено ");                              //
    Serial.print( mot.getSum(MOT_REV) );                     //   Выводим количество совершённых оборотов.
    Serial.print(" полных оборотов, пройдено ");             //
    Serial.print( mot.getSum(MOT_MET) );                     //   Выводим количество метров пройденного пути.
    Serial.print(" м пути, за ");                            //
    Serial.print( i % 60 );                                  //   Выводим количество секунд в минуте.
    Serial.print(" секунд");                                 //
    if(i<10 || i>20){                                        //   Добавляем окончание к слову секунд...
        if(i % 10 == 1     ){Serial.print("у");}             //   за 1   секундУ.
        if(i%10>1 && i%10<5){Serial.print("ы");}             //   за 2-4 секундЫ.
    }                                                        //
    Serial.println(".");                                     //
    delay(100);                                              //
//  Сбрасываем количество оборотов 1 раз в минуту            //
    if( i%60 == 0 ){                                         //   Условие выполняется каждые 60 сек.
        mot.delSum();                                        //   Сбрасываем количество совершённых оборотов.
        Serial.println("======================");            //
        delay(1000);                                         //
    }                                                        //
}                                                            //
                                                             //
// ПРИМЕЧАНИЕ:                                               //
// Количество совершённых оборотов сбрасывается              //   Функция getSum() работает даже если остановить
// - при достижении 167'772,16 оборотов;                     //   мотор функцией setStop() и вращать ротор вручную.
// - при обращении к функции delSum();                       //
// - при обращении к любой функции кроме getSum()            //
//   с параметром MOT_REV или MOT_MET.                       //
