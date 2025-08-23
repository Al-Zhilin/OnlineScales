#include <GyverDS18.h>
#include <GyverHX711.h>
#include "passwords.h"
#include "error_codes.h"
#include <math.h>

// --------------------------------------------- Пины Arduino ---------------------------------------------
#define RST_PIN 9                               // пин RST на SIM800L
#define DS_PIN 6                                // DS пин HX711
#define DT_PIN 7                                // DT пин HX711
#define SCL_PIN 8                               // SCL пин HX711
#define RED_PIN 2                               // вывод красного пина RGB светодиода
#define BLUE_PIN 3                              // вывод синего пина RGB светодиода
#define GREEN_PIN 4                             // вывод зеленого пина RGB светодиода


// ------------------------------------------- Периоды и таймеры ------------------------------------------
#define POST_PERIOD 30 * 60 * 1000              // период отправки данных на OpenMonitoring
#define SLEEP_OUT 1 * 60 * 1000                 // за сколько времени до отправки разбудить модули, проверить соединение и подготовить данные к отправке
#define SCALE_WAIT_PERIOD 10 * 1000             // период ожидания готовности датчика веса для получения новых значения


// -------------------------------------- Периоды моргания светодиода --------------------------------------
#define 

GyverHX711 sensor(DT_PIN, SCL_PIN, HX_GAIN64_A); 
GyverDS18Single ds(DS_PIN);

class BlinkLed {
private:
  BlinkPresets {
    ERROR_BLINK,
    INFO_BLINK,
    SUCCESS_BLINK
  };

  BlinkPresets blink_preset;
  byte color_pin = 0;
  byte blink_num = 0;

public:
  void setBlinkMode(LedBlink blink_mode) {                // здесь устанавливаем режим, цвет и пресет моргания
    switch (blink_mode) {
      case SCALE_TIMEOUT: 
        blink_num = 2;                        // моргаем 2 раза за полный цикл
        blink_pin = RED_PIN;                  // красным светом
        blink_preset = ERROR_BLINK;           // с периодами, характерными для сигнализирования ошибки
        break;

      case SCALE_WAITING:
        blink_num = 1;
        blink_pin = BLUE_PIN;
        blink_preset = INFO_BLINK;
        break;

      case SCALE_SUCCESS:
        blink_num = 2;
        blink_pin = GREEN_PIN;
        blink_preset = SUCCESS_BLINK;
        break;
      }
  }

  void tick() {
    if (blink_preset == INFO_BLINK) {           // если циклически моргаем информационно

    }

    
  }

} led;

class Scales {
private:


public:
  ScaleErrors begin() {
    if (this->checkAvailable() == ScaleErrors::TIMEOUT_ERROR) {         // модуль не готов и вышел таймаут ожидания
      led.setBlinkMode(LedBlink::SCALE_TIMEOUT);
      return ScaleErrors::TIMEOUT_ERROR;
    }

    sensor.tare();                                // обнуляем вес, считаем текущие показания весов нулем 
    if (abs(this->getRaw()) > 70) {               // возможно откалибровалось неправильно 
      
    }
  }

  ScaleErrors checkAvailable() {                  // функция проверяет доступность данных для чтения и/или ждет, пока это не произойдет
    uint32_t wait_period = millis();
    led.setBlinkMode(SCALE_WAITING);              // будем сигнализировать об ожидании морганием светодиода
    while (!sensor.available() && millis() - wait_period <= SCALE_WAIT_PERIOD) {led.tick();}                 // ждем пока измерения станут доступными или не выйдет таймаут 
    if (millis() - wait_period > SCALE_WAIT_PERIOD) return ScaleErrors::TIMEOUT_ERROR;                       // вышли из ожидания из-за истекшего таймаута
    return ScaleErrors::SUCCESS;
  }

  /*long getRaw() {
    
  }*/

} scale;


void setup() {
  Serial.begin(9600);      // USB COM
  Serial1.begin(9600);     // Аппаратный UART (TX1=1, RX1=0 на Pro Micro)

  scale.begin();
}

void loop() {
  
}

