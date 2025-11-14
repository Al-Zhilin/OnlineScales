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
#define SCALE_READ_PERIOD 200                   // период чтения значений веса
#define MAX_TARE_ATTEMPS 20                     // максимальное количество попыток корректно оттарировать весы


// -------------------------------------- Периоды моргания светодиода --------------------------------------
#define BLINK_DELAY 150                   // период между морганиями светодиода
#define BLINK_PERIOD 200                  // длительность морганий светодиода
#define BLINK_CYCLE_PERIOD 500            // период между циклическим морганием

GyverHX711 sensor(DT_PIN, SCL_PIN, HX_GAIN64_A);
GyverDS18Single ds(DS_PIN);

struct Weight {
      float grams = 0;    // вес в граммах
      float kg = 0;       // вес в килограммах

      void update(float new_grams) {
        grams = new_grams;
        kg = grams/1000;
      }
    } weight;


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
          delay(BLINK_DELAY); 
        }
      }

      else if (mode == BlinkModes::INFO && millis() - cyclic_timer >= BLINK_CYCLE_PERIOD) {
        cyclic_timer = millis();
        digitalWrite(static_cast<byte>(pin), HIGH);
        delay(BLINK_PERIOD);
        digitalWrite(static_cast<byte>(pin), LOW);
      }

      else if (mode == BlinkModes::NONE) {
        digitalWrite(static_cast<byte>(ColorPins::RED), LOW);
        digitalWrite(static_cast<byte>(ColorPins::GREEN), LOW);
        digitalWrite(static_cast<byte>(ColorPins::BLUE), LOW);
      }
    }

} led;

class Scales {
  private:
    float raw_filtered = 0;
    int32_t raw = 0;
    float scale_factor = 11.472;                    // коэффициент для перевода значения с дитчиков веса в осмысленные граммы

  public:
    ScaleErrors begin() {
      if (this->checkAvailable() == ScaleErrors::NOT_AVAILABLE) {         // модуль не готов и вышел таймаут ожидания
        led.showLed(ColorPins::RED, BlinkModes::SCALE);
        return ScaleErrors::NOT_AVAILABLE;
      }
      
      int attemps = 0;     // количество попыток тарирования перед получением адекватных нулевых показаний
  
      do {
        delay(300);
        sensor.tare();     // обнуляем вес, считаем текущие показания весов нулем 
        this->readRaw();
        attemps++;
      } while (abs(raw) > 50 && attemps < MAX_TARE_ATTEMPS);
      if (attemps == MAX_TARE_ATTEMPS)  return ScaleErrors::TARE_ERROR;

      return ScaleErrors::SUCCESS;
    }

    ScaleErrors checkAvailable() {                  // функция проверяет доступность данных для чтения и/или ждет, пока это не произойдет
      byte attempts = 0;
      while (attempts < ATTEMPS_NUM) {
        uint32_t wait_period = millis();
        while (!sensor.available() && millis() - wait_period <= SCALE_WAIT_PERIOD)  led.showLed(ColorPins::BLUE, BlinkModes::INFO);                 // ждем пока измерения станут доступными или не выйдет таймаут 
        if (millis() - wait_period < SCALE_WAIT_PERIOD)   return ScaleErrors::SUCCESS;                                                              // вышли из ожидания из-за истекшего таймаута
        attempts++;
      }
      return ScaleErrors::NOT_AVAILABLE;
    }

    ScaleErrors readRaw() {
      if (this->checkAvailable() == ScaleErrors::SUCCESS) raw = sensor.read();
      else return ScaleErrors::NOT_AVAILABLE;

      float diff = abs(raw_filtered - raw);
      if (diff < 150.0)  raw_filtered += (raw - raw_filtered) * 0.07;
      else if (diff < 300.0) raw_filtered += (raw - raw_filtered) * 0.5;
      else raw_filtered += (raw - raw_filtered) * 0.9;

      weight.update(raw_filtered/scale_factor);
      Serial.println(weight.kg);
    }    

} scale;

class TempSensor {
private:
  float filtered_temp = 0;

public:
  void begin() {
    ds.requestTemp();
  }

}

void setup() {
  pinMode(static_cast<byte>(ColorPins::RED), OUTPUT);
  pinMode(static_cast<byte>(ColorPins::GREEN), OUTPUT);
  pinMode(static_cast<byte>(ColorPins::BLUE), OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(BUTT_PIN, INPUT_PULLUP);

  Serial.begin(9600);      // USB COM
  Serial1.begin(9600);     // Аппаратный UART (TX1=1, RX1=0 на Pro Micro)

  if (scale.begin() != ScaleErrors::SUCCESS)  led.showLed(ColorPins::RED, BlinkModes::TOTAL_ERROR);
  else led.showLed(ColorPins::GREEN, BlinkModes::SCALE);
}

void loop() {
  static uint32_t scale_read = millis();
  if (millis() - scale_read >= SCALE_READ_PERIOD) {
    scale_read = millis();
    scale.readRaw();                                      // нужно для поддержания корретного значения в фильтре
  }

  static uint32_t send_timer = millis();
  if (millis() - send_timer >= 2 * 60 * 1000) {
    Serial1.print(weight.kg);
    Serial1.print("/");
    Serial1.println(temp)
  }
  
}

