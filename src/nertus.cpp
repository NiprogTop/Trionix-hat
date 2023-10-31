// простейшие динамические эффекты
// сначала ознакомься с примером microLED_guide !!!

#define STRIP_PIN 6     // пин ленты
#define STRIP_PIN2 9
#define NUMLEDS 12      // кол-во светодиодов

#define COLOR_DEBTH 3
#include <microLED.h>   // подключаем библу
microLED<NUMLEDS, STRIP_PIN, MLED_NO_CLOCK, LED_WS2818, ORDER_GRB, CLI_AVER> strip;
microLED<NUMLEDS, STRIP_PIN2, MLED_NO_CLOCK, LED_WS2818, ORDER_GRB, CLI_AVER> strip2;




void colorCycle() {
  static byte counter = 0;
  strip.fill(mWheel8(counter));
  counter += 3;
}

void runningDots() {
  static byte counter = 0;
  // перемотка буфера со сдвигом (иллюзия движения пикселей)
  for (int i = 0; i < NUMLEDS - 1; i++) strip.leds[i] = strip.leds[i + 1];

  // каждый третий вызов - последний пиксель красным, иначе чёрным
  if (counter % 3 == 0) strip.leds[NUMLEDS - 1] = mRed;
  else strip.leds[NUMLEDS - 1] = mBlack;
  counter++;
  delay(100);   // дополнительная задержка
}

void breathing() {
  static int dir = 1;
  static int bright = 0;
  bright += dir * 5;    // 5 - множитель скорости изменения

  if (bright > 255) {
    bright = 255;
    dir = -1;
  }
  if (bright < 0) {
    bright = 0;
    dir = 1;
  }
  strip.setBrightness(bright);
}

void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  strip.setBrightness(250);
  strip2.setBrightness(250);
  strip.clear();
  strip2.clear();
}

void loop() {
  // раскомментируй нужный эффект
  
  // rainbow();      // бегущая радуга во всю ленту
  //filler();       // плавное заполнение
  // colorCycle();   // плавная смена цвета
  //runningDots();  // бегущие точки

  // вывод
  // breathing();    // "дыхание" яркости, применяется ко всем эффектам
  
  // strip.fill(mRed);
  // strip.oneLedIdle = mOrange;
  strip.fill(mRGB(255, 0, 0));
  strip.show();   // вывод
  strip2.fill(mRGB(255, 0, 0));
  strip2.show();   // вывод
  delay(1000);      // 30 кадров в секунду
}