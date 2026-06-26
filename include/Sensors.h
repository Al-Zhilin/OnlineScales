#pragma once
#include <GyverHX711.h>
#include <GyverDS18.h>

struct SensorData {
  float weightGr = 0;
  float weightKg = 0;
  float tempC = 0;
};
extern SensorData sensorData;

class ScalesManager {
  private:
    GyverHX711 *_scales;
    float _calibation_factor;                             // коэффициент для перевода значения с датчиков веса в осмысленные граммы
    int32_t _offset = 0;                                  // оффсет для тарирования (при первом запуске - ноль)
    uint32_t _tare_timer = 0;                             // для таймера от частого тарирования (напр., если пользователь удерживает кнопку дольше, чем нужно)
    uint32_t _start_timer = millis();                     // засекаем millis() после пробуждения HX711 - чтения будем вызыват не ранее, чем после 500мс после запуска (длительность инициализации и первого чтения)
    float    _filtered        = 0.0f;
    bool     _inWarmup        = true;  // true до первого стабильного окна; сбрасывается при каждом пробуждении
    int32_t  _stableRef       = 0;    // первое чтение текущего окна стабильности
    int32_t  _stableAcc       = 0;    // накопитель для усреднения стабильного окна
    uint8_t  _stableCount     = 0;    // сколько подряд идущих чтений попало в окно
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
      if (millis() - _tare_timer < 5000) {
        LOG("Tare rejected: cooldown active");
        return ScalesState::ERROR;
      }

      uint32_t wait_available_timer = millis();
      while (millis() - wait_available_timer < 1000 && !this->isReady()) {
        yield();                                          // не блокируем планировщик и WDT
      }
      if (millis() - wait_available_timer >= 1000) {    // не дождались готовности
        return ScalesState::ERROR;                        // _tare_timer НЕ обновляем: HX711 завис, повтор доступен сразу
      }
      _tare_timer = millis();                             // кулдаун стартует только после подтверждения готовности датчика

      _scales->tare();

      // Ждём свежего отсчёта после тарирования, иначе read() вернёт устаревшее значение
      uint32_t verify_timer = millis();
      while (!_scales->available() && millis() - verify_timer < 500) yield();

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
      if (!mode) {
        _start_timer = millis();                          // засекаем время после пробуждения
        if (WARMUP_AFTER_SLEEP) {
            _inWarmup    = true;                              // каждое пробуждение — новый прогрев
            _stableRef   = 0;
            _stableAcc   = 0;
            _stableCount = 0;
        }
      }
    }

    bool isReady() {                                      // защита от слишком раннего чтения
      return _scales->available();
    }

    ScalesState tick() {                                  // обновляем фильтр
      if (!this->isReady())  {
        if (millis() - _start_timer > 2000) {             // Добавляем защиту от аппаратного зависания HX711
            return ScalesState::ERROR;                    // Датчик не ответил за 2 секунды! - признак зависания, а не просто "ожидания готовности измерения"
        }
        return ScalesState::BUSY;
      }
      
      _start_timer = millis(); // Сбрасываем таймер при успешном чтении

      int32_t new_weight = _scales->read();
      if (INVERT_WEIGHT_SIGN)   new_weight *= -1;

      // Прогрев по окну стабильности: принимаем первое чтение только после того,
      // как WARMUP_STABLE_N подряд идущих показаний HX711 уложились в 1.5*STANDART_NOISE друг от друга.
      // Гарантирует стабильный старт и после включения питания, и после каждого пробуждения из сна (включается опционально).
      static constexpr uint8_t WARMUP_STABLE_N = 5;
      if (_inWarmup) {
        if (_stableCount == 0 || abs(new_weight - _stableRef) > 1.5*STANDART_NOISE) {
          _stableRef   = new_weight;      // начинаем новое окно от текущего чтения
          _stableAcc   = new_weight;
          _stableCount = 1;
        } else {
          _stableAcc += new_weight;
          if (++_stableCount >= WARMUP_STABLE_N) {
            _filtered = float(_stableAcc) / _stableCount;   // _filtered = среднее стабильного окна
            _inWarmup = false;
          }
        }
        return ScalesState::BUSY;
      }

      float k;
      if (abs(new_weight - _filtered) > STANDART_NOISE) k = 0.65;
      else k = 0.05;
      _filtered += (new_weight - _filtered) * k;

      sensorData.weightGr = (float)_filtered / _calibation_factor;  // обновили данные: попугаи -> вес в граммах
      sensorData.weightKg = sensorData.weightGr / 1000;             // еще и вес в кг тоже обновили


      #ifdef USE_LOG
        #if WEIGHT_KOEFF_SELECTION == 1
            uint32_t sum_coeff = 0, count_coeff, weight = 0;
            String input = "";
            char c;
            if (USE_LOG.available()) {
                c = USE_LOG.read();
                if (c == '\n') {
                    // Обработка введенного потока
                    
                    input = "";
                }
                else if (c != '\r') input += c;
            }
        #endif
      #endif
      return ScalesState::SUCCESS;
    }
};

class TempManager {
  private:
    GyverDS18Single *_temp_sensor;
    uint32_t _start_timer = 0;                            // таймер между успешными измерениями. Если датчик не готов дольше этого времени - он работает некорректно!!
    bool first_read = true;
    bool _conv_started = false;
    uint8_t _skipCount = 2;                               // пропустить первые 2 конвертации: DS18B20 может вернуть значение по умолчанию из SRAM до первого реального преобразования

  public:
    TempManager(uint8_t pin) {
      _temp_sensor = new GyverDS18Single(pin);
    }

    void begin() {
      _temp_sensor->setParasite(false);
      _temp_sensor->setResolution(12);
    }

    TempState tick() {
      uint8_t state = _temp_sensor->tick();

      if (state == DS18_READY) {
        if (_skipCount > 0) {
          --_skipCount;
          _start_timer = millis();
          _conv_started = false;
          return TempState::BUSY;
        }

        if (first_read) {
          sensorData.tempC = _temp_sensor->getTemp();
          first_read = false;
        }

        else {
          float new_temp = _temp_sensor->getTemp();
          float k = 0.05;
          if (fabsf(new_temp - sensorData.tempC) > 1.0)       k = 0.9;
          else if (fabsf(new_temp - sensorData.tempC) > 0.5)  k = 0.75;
          sensorData.tempC += (new_temp - sensorData.tempC) * k;
        }
        _start_timer = millis();
        _conv_started = false;
        return TempState::SUCCESS;
      }

      if (_temp_sensor->isWaiting() && !_conv_started) {
        _conv_started = true;
        _start_timer = millis();
      }

      if (millis() - _start_timer < 2000) return TempState::BUSY;
      return TempState::ERROR;
    }
};

// Функция EMA для значения от делителя напряжения батареи (сглаживаем скачки и помехи)
inline float filtrateVolts(float new_meas) {
  static float filtrated = new_meas;
  float k;

  if (fabs(new_meas - filtrated) > 120) k = 0.85;
  else k = 0.45;
  filtrated += ((new_meas - filtrated) * k);
  
  return filtrated;
}