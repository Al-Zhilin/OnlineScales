#include <GyverHX711.h>
#include <GyverDS18.h>
#include "passwords.h"
#include "Enumerations.h"

// --------------------------------------------- Пины Arduino ---------------------------------------------
#define RST_PIN 9                               // пин RST на SIM800L
#define SCL_PIN 8                               // SCL пин HX711
#define DT_PIN 7                                // DT пин HX711
#define DS_PIN 6                                // пин DS18b20
#define BUTT_PIN 10                             // пин кнопки


// ----------------------------------------------- Настройки ----------------------------------------------
#define POST_PERIOD 30 * 60 * 1000              // период отправки данных на OpenMonitoring
#define SLEEP_OUT 1 * 60 * 1000                 // за сколько времени до отправки разбудить модули, проверить соединение и подготовить данные к отправке
#define SCALE_WAIT_PERIOD 10 * 1000             // период ожидания готовности датчика веса для получения новых значения
#define ATTEMPS_NUM 3                           // количество попыток получить успешный результат от того или иного действия


// -------------------------------------- Периоды моргания светодиода --------------------------------------
#define BLINK_DELAY 150                   // период между морганиями светодиода
#define BLINK_PERIOD 200                  // длительность морганий светодиода
#define BLINK_CYCLE_PERIOD 500            // период между циклическим морганием



GyverHX711 sensor(DT_PIN, SCL_PIN, HX_GAIN64_A);
GyverDS18Single ds(DS_PIN);

class BlinkLed { 
  public:
    void showLed(ColorPins pin, BlinkModes mode) {
      static uint32_t cyclic_timer = millis();
      if (mode == BlinkModes::TOTAL_ERROR)  digitalWrite(static_cast<byte>(pin), HIGH);
      
      else if (mode == BlinkModes::SCALE) {
        for (byte i = 0; i < 2; i++) {
          digitalWrite(static_cast<byte>(pin), HIGH);
          delay(BLINK_PERIOD);
          digitalWrite(static_cast<byte>(pin), LOW);
          if (i != 1) delay(BLINK_DELAY); 
        }
      }

      else if (mode == BlinkModes::INFO && millis() - cyclic_timer >= BLINK_CYCLE_PERIOD) {
        cyclic_timer = millis();
        digitalWrite(static_cast<byte>(pin), HIGH);
        delay(BLINK_PERIOD);
        digitalWrite(static_cast<byte>(pin), LOW);
      }
    }

} led;

class Scales {
  public:
    ScaleErrors begin() {
      if (this->checkAvailable() == ScaleErrors::TIMEOUT_ERROR) {         // модуль не готов и вышел таймаут ожидания
        led.showLed(ColorPins::RED, BlinkModes::SCALE);
        return ScaleErrors::NEED_RECALL;
      }

      sensor.tare();                                // обнуляем вес, считаем текущие показания весов нулем 
      return ScaleErrors::SUCCESS;
    }

    ScaleErrors checkAvailable() {                  // функция проверяет доступность данных для чтения и/или ждет, пока это не произойдет
      uint32_t wait_period = millis();
      while (!sensor.available() && millis() - wait_period <= SCALE_WAIT_PERIOD)  led.showLed(ColorPins::BLUE, BlinkModes::INFO);                 // ждем пока измерения станут доступными или не выйдет таймаут 
      if (millis() - wait_period > SCALE_WAIT_PERIOD) return ScaleErrors::TIMEOUT_ERROR;                       // вышли из ожидания из-за истекшего таймаута
      return ScaleErrors::SUCCESS;
    }    

} scale;


void setup() {
  pinMode(static_cast<byte>(ColorPins::RED), OUTPUT);
  pinMode(static_cast<byte>(ColorPins::GREEN), OUTPUT);
  pinMode(static_cast<byte>(ColorPins::BLUE), OUTPUT);
  pinMode(RST_PIN, HIGH);
  pinMode(BUTT_PIN, INPUT_PULLUP);

  Serial.begin(9600);      // USB COM
  Serial1.begin(9600);     // Аппаратный UART (TX1=1, RX1=0 на Pro Micro)

  byte attemps = 0;
  while (scale.begin() == ScaleErrors::NEED_RECALL && attemps < ATTEMPS_NUM) {attemps++;}
  if (attemps == ATTEMPS_NUM) while (true) led.showLed(ColorPins::RED, BlinkModes::TOTAL_ERROR);

  led.showLed(ColorPins::GREEN, BlinkModes::SCALE);
}

void loop() {
  
}

