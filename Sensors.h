#pragma once
#include <GyverHX711.h>
#include <GyverDS18.h>

struct SensorsData {
  float weightKg = 0;
  float tempC = 0;
} sensorData;

class ScalesManager {
  private:
    GyverHX711 *_scales;
    float _calibation_factor;                             // коэффициент для перевода значения с датчиков веса в осмысленные граммы
    int32_t _offset = 0;                                  // оффсет для тарирования (при первом запуске - ноль)
    uint32_t tare_timer = 0;                              // для таймера от частого тарирования (напр., если пользователь удерживает кнопку дольше, чем нужно)
    EEManager EEOffset;                                   // объект для уравления переменной оффсета в EEPROM

  public:
    ScalesManager(float factor, int16_t dt_pin, int16_t scl_pin) : EEOffset(_offset) {
      _calibation_factor = factor;
      _scales = new GyverHX711(dt_pin, scl_pin, HX_GAIN64_A);
    }

    void begin() {
      uint8_t mem_stat = EEOffset.begin(0, 'Z');
      _scales->setOffset(_offset);
      if (!mem_stat)  LOG("Data read from EEPROM");
      else if (mem_stat == 1) LOG("New Key! Data wrote to EEPROM!");
      else LOG("ERROR! not enough space in the EEPROM!");
    }

    int8_t sensorTare() {         // зажата кнопка - нужно оттарировать заново. 1 - успех, 2 - не прошел тайм-аут, 0 - "ошибка"
      if (millis() - tare_timer < 2000) return 2;
      _offset = _scales->read();                // получили текущее значение
      _scales->setOffset(_offset);              // обнулили показания

      if (abs(_scales->read()) <= 50) {           // проверка на обнуление
        EEOffset.updateNow();                     // записали значение в память
        LOG("New weight offset saved to EEPROM");
        return 1;
      }

      else {
        LOG("Uncorrect tare!");
        return 0;
      }
    }
};