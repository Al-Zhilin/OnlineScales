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
    float _filtered = 0.0f;                               // здесь актуальное отфильтрованное значение
    uint32_t _tare_timer = 0;                             // для таймера от частого тарирования (напр., если пользователь удерживает кнопку дольше, чем нужно)
    uint32_t _start_timer = millis();                     // засекаем millis() после пробуждения HX711 - чтения будем вызыват не ранее, чем после 500мс после запуска (длительность инициализации и первого чтения)
    FileData EEOffset{&LittleFS, "/offset.dat", 'Z', &_offset, sizeof(_offset)};       // объект для управления переменной оффсета в файловой системе

  public:
    ScalesManager(float factor, int16_t dt_pin, int16_t scl_pin) {
      _calibation_factor = factor;
      _scales = new GyverHX711(dt_pin, scl_pin, HX_GAIN64_A);
    }

    void begin() {
      FDstat_t stat = EEOffset.read();
      _scales->setOffset(_offset);                        
      
      if (stat == FD_READ) {
          LOG("Offset read from LittleFS");
      } else {
          LOG("New offset file created in LittleFS!");
          EEOffset.write(); // Сразу создаем файл с дефолтным (нулевым) значением
      }
    }

    ScalesState sensorTare() {
      if (millis() - _tare_timer < 5000) return ScalesState::ERROR;        
      _tare_timer = millis();

      _scales->tare();

      if (abs(_scales->read()) <= STANDART_NOISE) {                   
        _offset = _scales->getOffset();                   
        
        EEOffset.updateNow();                             
        
        LOG("New weight offset saved to LittleFS");
        return ScalesState::SUCCESS;
      }
      else {
        LOG("Uncorrect tare!");
        _scales->setOffset(_offset);                      
        return ScalesState::ERROR;
      }
    }

    void sleepMode(bool mode) {                           // вкл/выкл режим сна
      _scales->sleepMode(mode);
      if (!mode) _start_timer = millis();                        // засекаем время после пробуждения, чтобы выждать таймаут перед первым чтением
    }

    bool isReady() {                                      // защита от слишком раннего чтения
      return (millis() - _start_timer) > 500;
    }

    ScalesState tick() {                                  // обновляем фильтр
      if (!_scales->available())  {
        //LOG("Tick failed! Busy!");
        return ScalesState::BUSY;
      }

      float k;
      int32_t new_weight = _scales->read();

      if (abs(new_weight - _filtered) > STANDART_NOISE) k = 0.65;         // адаптивный коэффициент
      else k = 0.05;
      _filtered += (new_weight - _filtered) * k;  
      sensorData.weightGr = (int32_t)_filtered / _calibation_factor;  // обновили данные: попугаи -> вес в граммах
      sensorData.weightKg = sensorData.weightGr / 1000;            // еще и вес в кг тоже обновили

      Serial.println(sensorData.weightGr);
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


float filtrateVolts(float new_meas) {
  static float filtrated = new_meas;
  float k;

  if (fabs(new_meas - filtrated) > 120) k = 0.85;
  else k = 0.45;
  filtrated += ((new_meas - filtrated) * k);
  
  return filtrated;
}