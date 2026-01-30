#pragma once
#include <GyverHX711.h>
#include <GyverDS18.h>

struct sensorData {
  float weightKg = 0;
  float tempC = 0;
} sensorData;

class ScalesManager {
  private:
    GyverHX711 *_scales;
    float _calibation_factor;                             // коэффициент для перевода значения с датчиков веса в осмысленные граммы
    int32_t _offset = 0;                                  // оффсет для тарирования (при первом запуске - ноль)
    int32_t _filtered = 0;                                // здесь актуальное отфильтрованное значение в попугаях
    uint32_t _tare_timer = 0;                             // для таймера от частого тарирования (напр., если пользователь удерживает кнопку дольше, чем нужно)
    EEManager EEOffset;                                   // объект для управления переменной оффсета в EEPROM памяти

  public:
    ScalesManager(float factor, int16_t dt_pin, int16_t scl_pin) : EEOffset(_offset) {
      _calibation_factor = factor;
      _scales = new GyverHX711(dt_pin, scl_pin, HX_GAIN64_A);
    }

    void begin() {
      uint8_t mem_stat = EEOffset.begin(0, 'Z');
      _scales->setOffset(_offset);                        // установили прочитанный из EEPROM оффсет
      if (!mem_stat)  LOG("Data read from EEPROM");
      else if (mem_stat == 1) LOG("New Key! Data wrote to EEPROM!");
      else LOG("ERROR! not enough space in the EEPROM!");
    }

    ScalesState sensorTare() {
      if (millis() - _tare_timer < 2000) return 2;        // защита от чрезмерного зажатия кнопки - тарировать можно не ранее, чем через 2 секунды
      _offset = _scales->read();                          // получили текущее значение
      _scales->setOffset(_offset);                        // обнулили показания

      if (abs(_scales->read()) <= 50) {                   // проверка на обнуление
        EEOffset.updateNow();                             // записали значение в память
        LOG("New weight offset saved to EEPROM");
        return ScalesState::SUCCESS;
      }

      else {
        LOG("Uncorrect tare!");
        return ScalesState::ERROR;
      }
    }

    ScalesState tick() {                                  // по таймауту обновляем фильтр
      float k;
      int32_t new_weight = _scales->read();

      if (abs(new_weight - _filtered) > STANDART_NOISE) k = 0.65;         // адаптивный коэффициент
      else k = 0.25;
      _filtered += (value - _filtered) * k;

      sensorData.weightKg = _filtered / _calibation_factor / 1000;        // обновили данные: попугаи / попугаи_в_граммы / 1000
    }
};