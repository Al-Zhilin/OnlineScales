#pragma once
#include <GyverHX711.h>
#include <GyverDS18.h>

#define STANDART_NOISE 100                         // амплитуда стандартного шума весовых датчиков

struct sensorData {
  float weightGr = 0;
  float weightKg = 0;
  float tempC = 0;
} sensorData;

class ScalesManager {
  private:
    GyverHX711 *_scales;
    float _calibation_factor;                             // коэффициент для перевода значения с датчиков веса в осмысленные граммы
    int32_t _offset = 0;                                  // оффсет для тарирования (при первом запуске - ноль)
    int32_t _filtered = 0;                                // здесь актуальное отфильтрованное значение
    uint32_t _tare_timer = 0;                             // для таймера от частого тарирования (напр., если пользователь удерживает кнопку дольше, чем нужно)
    EEManager EEOffset;                                   // объект для управления переменной оффсета в EEPROM памяти

  public:
    ScalesManager(float factor, int16_t dt_pin, int16_t scl_pin) : EEOffset(_offset) {
      _calibation_factor = factor;
      _scales = new GyverHX711(dt_pin, scl_pin, HX_GAIN64_A);
    }

    uint16_t begin() {
      uint8_t mem_stat = EEOffset.begin(0, 'Z');
      _scales->setOffset(_offset);                        // установили прочитанный из EEPROM оффсет
      if (!mem_stat)  LOG("Data read from EEPROM");
      else if (mem_stat == 1) LOG("New Key! Data wrote to EEPROM!");
      else LOG("ERROR! not enough space in the EEPROM!");

      return EEOffset.nextAddr();
    }

    ScalesState sensorTare() {
      if (millis() - _tare_timer < 5000) return ScalesState::ERROR;        // защита от чрезмерного зажатия кнопки - тарировать можно не ранее, чем через 2 секунды
      _tare_timer = millis();

      _scales->tare();

      if (abs(_scales->read()) <= STANDART_NOISE) {                   // проверка на обнуление
        _offset = _scales->getOffset();                   // запоминаем, чтобы после перезагрузки установить удачный оффсет
        EEOffset.updateNow();                             // записали значение в память
        LOG("New weight offset saved to EEPROM");
        return ScalesState::SUCCESS;
      }

      else {
        LOG("Uncorrect tare!");
        _scales->setOffset(_offset);                      // если процедура не удалась - устанавливаем последнее удачное значение
        return ScalesState::ERROR;
      }
    }

    ScalesState tick() {                                  // обновляем фильтр
      if (!_scales->available())  {
        LOG("Tick failed! Busy!");
        return ScalesState::BUSY;
      }

      float k;
      int32_t new_weight = _scales->read();

      if (abs(new_weight - _filtered) > STANDART_NOISE) k = 0.65;         // адаптивный коэффициент
      else k = 0.1;
      _filtered += (new_weight - _filtered) * k;

      sensorData.weightGr = _filtered / _calibation_factor;        // обновили данные: попугаи -> вес в граммах
      sensorData.weightKg = sensorData.weightGr / 1000;            // еще и вес в кг тоже обновили

      return ScalesState::SUCCESS;
    }
};

class TempManager {
  private:
    GyverDS18Single *_temp_sensor;                        // хранимое отфильтрованное значение

  public:
    TempManager(uint8_t pin) {
      _temp_sensor = new GyverDS18Single(pin);
    }

    void begin() {
      _temp_sensor->setParasite(false);
      _temp_sensor->setResolution(12);
    }

    TempState tick() {
      if (_temp_sensor->tick() == DS18_READY) {
        float new_temp = _temp_sensor->getTemp();

        float k = 0.05;
        if (fabsf(new_temp - sensorData.tempC) > 1.0)  k = 0.9;
        else if (fabsf(new_temp - sensorData.tempC) > 0.5) k = 0.75;

        sensorData.tempC += (new_temp - sensorData.tempC) * k;
        return TempState::SUCCESS;
      }
      else return TempState::BUSY;
    }
};