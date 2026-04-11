#include <math.h>

// __attribute__((packed)) защищает от непредвиденных отступов (padding) в памяти EEPROM
struct ModelEEData {
  uint8_t modelType = 0;            // 0 - линейная, 1 - квадратичная, 2 - кубическая
  float params[4] = {0, 0, 0, 0};   // параметры функции наилучшей модели
  bool calibrated = false;          // была ли калибровка?
} calibData;

extern AsyncLed led;
FileData calib_memory{&LittleFS, "/calib.dat", 'Z', &calibData, sizeof(calibData)};

template <int N>
class RLSModel {
  private:
    float theta[N] = {};            // коэффициенты компенсации
    float P[N][N];                  // матрица ковариации (неуверенности)
    float lambda = 0.999f;          // коэффициент забывания
    float last_t = -999.0f;         // для игнорирования "стоячей" температуры

  public:
    RLSModel() { reset(); }

    void reset() {
      for (int i = 0; i < N; i++) theta[i] = 0.0f;
      resetP(100.0f);
      last_t = -999.0f;
    }

    void resetP(float val) {
      for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) P[i][j] = (i == j) ? val : 0.0f;
      }
    }

    void setTheta(const float* source) {
      if (source == nullptr) return;
      for (int i = 0; i < N; i++) theta[i] = source[i];
    }

    const float* getTheta() const { return theta; }

    float getUncertainty() const {
      float trace = 0;
      for (int i = 0; i < N; i++) trace += P[i][i];
      // Защита от деления на 0 и нормализация
      float initial_trace = N * 100.0f;
      return (trace / initial_trace) * 100.0f;
    }

    // current_t вынесен явно для большей читаемости и надежности
    void update(const float* x, float y, float current_t) {
      // Защита от заражения матрицы NaN значениями (критично для матричных операций)
      if (isnan(y) || isnan(current_t) || isinf(y)) return; 
      
      // Дедбэнд фильтр (защита от сингулярности)
      if (fabsf(current_t - last_t) < 0.005f) return;
      last_t = current_t;

      float K[N], Px[N];
      float xPx = 0.0f;
      
      for (int i = 0; i < N; i++) {
        Px[i] = 0.0f;
        for (int j = 0; j < N; j++) Px[i] += P[i][j] * x[j];
      }
      for (int i = 0; i < N; i++) xPx += x[i] * Px[i];
      
      float denom = lambda + xPx;
      // Предотвращение деления на ноль (в RLS denom всегда должен быть > 0)
      if (denom < 1e-6f) return; 

      for (int i = 0; i < N; i++) K[i] = Px[i] / denom;
      
      float error = y - predict(x);
      for (int i = 0; i < N; i++) theta[i] += K[i] * error;
      
      float kxP[N];
      for (int j = 0; j < N; j++) {
        kxP[j] = 0;
        for (int i = 0; i < N; i++) kxP[j] += x[i] * P[i][j];
      }
      
      float trace = 0;
      for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
          P[i][j] = (P[i][j] - K[i] * kxP[j]) / lambda;
        }
        trace += P[i][i];
      }

      // Защита от "ковариационного взрыва" (covariance wind-up)
      if (trace > 1000.0f || isnan(trace)) {
        resetP(100.0f);
      }
    }

    float predict(const float* x) const {
      float res = 0.0f;
      for (int i = 0; i < N; i++) res += theta[i] * x[i];
      return res;
    }
};

class ScaleAutoCalibrator {
  private:
    RLSModel<2> linearModel;
    RLSModel<3> quadraticModel;
    RLSModel<4> cubicModel;
    
    // Используем EMA (скользящее среднее) вместо кумулятивной суммы
    float linearErrorEMA = 0.0f;
    float quadraticErrorEMA = 0.0f;
    float cubicErrorEMA = 0.0f;
    
    uint32_t samplesCount = 0; // uint32_t хватит на годы
    uint8_t bestModel = 0; 
    float referenceMass;
    bool isCalibrating = false;

    float normT(float t) const { return (t - 25.0f) / 10.0f; }

  public:
    ScaleAutoCalibrator(float refMass) : referenceMass(refMass) {}

    void begin() {
      FDstat_t stat = calib_memory.read();
      
      if (stat != FD_READ) {
          LOG("No calibration file found. Created default.");
          calib_memory.updateNow(); 
      }
      
      if (calibData.calibrated) loadData();
    }

    void startCalibration() {
      if (isCalibrating) {
        LOG("Already Calibrating! Resetting live params...");
        resetCalibrationModels();
      }
      
      isCalibrating = true;
      samplesCount = 0;
      linearErrorEMA = quadraticErrorEMA = cubicErrorEMA = 0.0f;
      led.setMode(LedModes::OK);
    }

    void finishCalibration() {
      if (!isCalibrating) {
        LOG("Calibration has not been started!");
        led.setMode(LedModes::ERROR);
        return;
      }
      
      isCalibrating = false;
      if (samplesCount > 10) { // Защита от сохранения после 1-2 случайных тиков
        calcBestModel();
        saveData();
      }
      led.setMode(LedModes::OK);
    }

    // Отдельный метод для сброса только математики (без EEPROM)
    void resetCalibrationModels() {
      linearModel.reset();
      quadraticModel.reset();
      cubicModel.reset();
      samplesCount = 0;
      linearErrorEMA = quadraticErrorEMA = cubicErrorEMA = 0.0f;
      bestModel = 0;
    }

    void resetCalibration() {
      resetCalibrationModels();
      isCalibrating = false;

      calibData.modelType = 0;
      calibData.calibrated = false;
      for(int i = 0; i < 4; i++) calibData.params[i] = 0.0f;

      // 2. Меняем updateNow() на write()
      calib_memory.updateNow(); 
      led.setMode(LedModes::OK);
      LOG("Reset complete!");
    }

    void saveData() {
      calibData.modelType = bestModel;
      const float* params = nullptr;
      
      if (bestModel == 0) params = linearModel.getTheta();
      else if (bestModel == 1) params = quadraticModel.getTheta();
      else params = cubicModel.getTheta();
      
      for (int i = 0; i < 4; i++) {
        calibData.params[i] = (params && i < (bestModel + 2)) ? params[i] : 0.0f;
      }
      calibData.calibrated = true;
      
      // 3. Меняем updateNow() на write()
      calib_memory.updateNow();
    }
    
    bool loadData() {
      if (!calibData.calibrated) return false;
      
      bestModel = calibData.modelType;
      // Сброс ковариации до малого значения, так как веса уже подобраны
      if (bestModel == 0) {
        linearModel.setTheta(calibData.params);
        linearModel.resetP(0.1f);
      } else if (bestModel == 1) {
        quadraticModel.setTheta(calibData.params);
        quadraticModel.resetP(0.1f);
      } else {
        cubicModel.setTheta(calibData.params);
        cubicModel.resetP(0.1f);
      }
      return true;
    }

    void printSavedModelData() {
      #ifdef USE_LOG
      USE_LOG.println(F("\n=== Сохраненные данные калибровки ==="));
      
      if (!calibData.calibrated) {
        USE_LOG.println(F("Статус: НЕТ ДАННЫХ (не откалибровано)"));
        USE_LOG.println(F("=====================================\n"));
        return;
      }

      USE_LOG.println(F("Статус: ОТКАЛИБРОВАНО"));
      USE_LOG.print(F("Лучшая модель: "));
      
      uint8_t n_params = calibData.modelType + 2;
      if (calibData.modelType == 0) USE_LOG.println(F("Линейная (2 параметра)"));
      else if (calibData.modelType == 1) USE_LOG.println(F("Квадратичная (3 параметра)"));
      else USE_LOG.println(F("Кубическая (4 параметра)"));

      USE_LOG.println(F("Коэффициенты (Theta):"));
      for (uint8_t i = 0; i < n_params; i++) {
        USE_LOG.print(F("  [")); USE_LOG.print(i); USE_LOG.print(F("] = "));
        USE_LOG.println(calibData.params[i], 6); 
      }
      USE_LOG.println(F("=====================================\n"));
      #endif
      
      led.setMode(LedModes::WAIT); 
    }
    
    void calibrationStep() {
      if (!isCalibrating) return;
      samplesCount++;
      
      float t = normT(sensorData.tempC);
      float error = sensorData.weightGr - referenceMass;
      
      // Векторы признаков
      float xL[2] = {t, 1.0f};
      float xQ[3] = {t * t, t, 1.0f};
      float xC[4] = {t * t * t, t * t, t, 1.0f};

      // Текущие ошибки моделей
      float errL = fabsf(error - linearModel.predict(xL));
      float errQ = fabsf(error - quadraticModel.predict(xQ));
      float errC = fabsf(error - cubicModel.predict(xC));

      // Обновление скользящего среднего (EMA)
      if (samplesCount <= 1) {
        linearErrorEMA = errL;
        quadraticErrorEMA = errQ;
        cubicErrorEMA = errC;
      } else {
        const float alpha = 0.02f; // Настраиваемая скорость реакции метрики (2%)
        linearErrorEMA = (1.0f - alpha) * linearErrorEMA + alpha * errL;
        quadraticErrorEMA = (1.0f - alpha) * quadraticErrorEMA + alpha * errQ;
        cubicErrorEMA = (1.0f - alpha) * cubicErrorEMA + alpha * errC;
      }

      // Обновляем модели (передаем t явно)
      linearModel.update(xL, error, t);
      quadraticModel.update(xQ, error, t);
      cubicModel.update(xC, error, t);

      if (samplesCount > 10) calcBestModel(); // Даем моделям "прогреться"
    }

    float getUncertainty() const {
      if (bestModel == 0) return linearModel.getUncertainty();
      if (bestModel == 1) return quadraticModel.getUncertainty();
      return cubicModel.getUncertainty();
    }
    
    // Оптимизированная компенсация (без массивов, схема Горнера)
    float compensate(float temperature, float rawWeight) {
      if (!calibData.calibrated) return rawWeight;

      float t = normT(temperature);
      float est = 0.0f;
      
      // Берем коэффициенты активной модели (из ОЗУ, они обновляются в реальном времени при калибровке)
      const float* p = nullptr;
      if (bestModel == 0) p = linearModel.getTheta();
      else if (bestModel == 1) p = quadraticModel.getTheta();
      else p = cubicModel.getTheta();

      // Метод Горнера для быстрого вычисления полинома
      // p = { a, b, c, d } соответствует a*t^3 + b*t^2 + c*t + d
      est = p[0];
      for (int i = 1; i < bestModel + 2; i++) {
        est = est * t + p[i];
      }
      
      return rawWeight - est;
    }

    void calcBestModel() {
      // Штрафы за сложность модели
      float sLin = linearErrorEMA * 1.0f; 
      float sQuad = quadraticErrorEMA * 1.05f;
      float sCub = cubicErrorEMA * 1.10f;
      
      if (sLin <= sQuad && sLin <= sCub) bestModel = 0;
      else if (sQuad <= sCub) bestModel = 1;
      else bestModel = 2;
    }

    uint8_t getBestModel() const { return bestModel; }
    bool isCalibratingMode() const { return isCalibrating; }
    bool isModelLoaded() const { return calibData.calibrated; }
};