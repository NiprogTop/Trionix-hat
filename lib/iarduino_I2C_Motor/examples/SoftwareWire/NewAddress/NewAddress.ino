// ПРИМЕР СМЕНЫ АДРЕСА МОДУЛЯ:                               // * Строки со звёздочкой являются необязательными.
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
// Данный скетч демонстрирует не только                      //
// возможность смены адреса на указанный в                   //
// переменной newAddress, но и обнаружение,                  //
// и вывод текущего адреса модуля на шине I2C.               //
                                                             //
// Библиотека <iarduino_I2C_Motor.h> предназначена           //
// для работы с моторами серии I2C-flash                     //
// и меняет только их адреса.                                //
// Но, в разделе Файл/Примеры/.../findSortDevices            //
// находятся примеры смены и сортировки адресов              //
// любых модулей iarduino серии Flash I2C.                   //
                                                             //
uint8_t newAddress = 0x09;                                   //   Назначаемый модулю адрес (0x07 < адрес < 0x7F).
                                                             //
#include <iarduino_I2C_Software.h>                           //   Подключаем библиотеку для работы с программной шиной I2C, до подключения библиотеки iarduino_I2C_Motor.
SoftTwoWire sWire(3,4);                                      //   Создаём объект программной шины I2C указав выводы которым будет назначена роль линий: SDA, SCL.
                                                             //
#include <iarduino_I2C_Motor.h>                              //   Подключаем библиотеку для работы с мотором I2C-flash.
iarduino_I2C_Motor mot;                                      //   Объявляем объект mot для работы с функциями и методами библиотеки iarduino_I2C_Motor.
                                                             //   Если при объявлении объекта указать адрес, например, mot(0xBB), то пример будет работать с тем модулем, адрес которого был указан.
void setup(){                                                //
    delay(500);                                              // * Ждём завершение переходных процессов связанных с подачей питания.
    Serial.begin(9600);                                      //
    while(!Serial){;}                                        // * Ждём завершения инициализации шины UART.
    if( mot.begin(&sWire) ){                                 //   Инициируем работу с мотором, указав ссылку на объект для работы с шиной I2C на которой находится мотор (Wire, Wire1, Wire2 ...).
        Serial.print("Найден мотор 0x");                     //
        Serial.println( mot.getAddress(), HEX );             //   Выводим текущий адрес модуля.
        if( mot.changeAddress(newAddress) ){                 //   Меняем адрес модуля на newAddress.
            Serial.print("Адрес изменён на 0x");             //
            Serial.println(mot.getAddress(),HEX );           //   Выводим текущий адрес модуля.
        }else{                                               //
            Serial.println("Адрес не изменён!");             //
        }                                                    //
    }else{                                                   //
        Serial.println("Мотор не найден!");                  //
    }                                                        //
}                                                            //
                                                             //
void loop(){                                                 //
}                                                            //
