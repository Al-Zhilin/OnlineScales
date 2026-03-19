struct ModelEEData {                        // структура данных для хранения их в EEPROM
  uint8_t modelType = 0;            // тип модели, которая показала себя наилучшим образом
  float params[4] = {0,0,0,0};      // параметры в поправочной функции наилучшей модели
  bool calibrated = false;          // была ли калибровка?
} calibData;

extern AsyncLed led;

EEManager calib_memory(calibData);                      // Объект менеджера памяти

template <int N>
class RLSModel {
  private:
    float theta[N] = {};                                // крэффициенты формули компенсации
    float P[N][N];                                      // матрица неуверенности
    float lambda = 0.999f;                              // коэффициент забывания
    float last_t = -999.0f;                             // для игнорирования стоячей температуры

  public:
    RLSModel() { reset(); }

    void reset() {
      for (int i = 0; i < N; i++) theta[i] = 0;
      resetP(100.0f);
      last_t = -999.0f;
    }

    void resetP(float val) {
      for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) P[i][j] = (i == j) ? val : 0.0f;
      }
    }

    void setTheta(const float* source) {                                  // уставнока коэффициентов (например, при чтении из памяти)
        if (source == nullptr) return;
        for (int i = 0; i < N; i++) theta[i] = source[i];
    }

    float getUncertainty() const {                                        // получение "неуверенности" модели
      float trace = 0;
      for (int i = 0; i < N; i++) trace += P[i][i];
      
      // Исходный след матрицы при старте: N * 100.0f. 
      // Нормализуем так, чтобы старт был 100%, а сходимость стремилась к 0%
      float initial_trace = N * 100.0f;
      float normalized = (trace / initial_trace) * 100.0f;
      
      return normalized;
    }
    
    void update(const float* x, float y) {                                // обновление модели
      if (fabsf(x[N-2] - last_t) < 0.005f) return;          // если данные слабо изменились - не выполняем обновление модели.
      last_t = x[N-2];

      float K[N], Px[N], xPx = 0.0f;
      for (int i = 0; i < N; i++) {
        Px[i] = 0.0f;
        for (int j = 0; j < N; j++) Px[i] += P[i][j] * x[j];
      }
      for (int i = 0; i < N; i++) xPx += x[i] * Px[i];
      
      float denom = lambda + xPx;
      for (int i = 0; i < N; i++) K[i] = Px[i] / denom;
      
      float error = y - predict(x);
      for (int i = 0; i < N; i++) theta[i] += K[i] * error;
      
      float kxP[N];
      for (int j = 0; j < N; j++) {
        kxP[j] = 0;
        for (int i = 0; i < N; i++) kxP[j] += x[i] * P[i][j];
      }
      for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
          P[i][j] = (P[i][j] - K[i] * kxP[j]) / lambda;
        }
      }

      float trace = 0;
      for (int i = 0; i < N; i++) trace += P[i][i];               // защита от "ковариационного взрыва"
      if (trace > 1000.0f) {
          resetP(100.0f);
      }
    }
    
    float predict(const float* x) const {
      float res = 0.0f;
      for (int i = 0; i < N; i++) res += theta[i] * x[i];
      return res;
    }

    const float* getTheta() const { return theta; }
};

class ScaleAutoCalibrator {
  private:
    RLSModel<2> linearModel;
    RLSModel<3> quadraticModel;
    RLSModel<4> cubicModel;
    
    float linearError = 0.0f, quadraticError = 0.0f, cubicError = 0.0f;   // ошибки предсказаний каждой моедли
    uint16_t updateCount = 0;
    uint8_t bestModel = 0; 
    float referenceMass;
    bool isCalibrating = false;

    float normT(float t) const { return (t - 25.0f) / 10.0f; }            // для корректных расчетов у моделей высоких порядков введдем нормализацию входной температуры

  public:
    ScaleAutoCalibrator(float refMass) : referenceMass(refMass) {}

    void begin(uint16_t addr) {
      calib_memory.begin(addr, 'Z');
      if (calibData.calibrated) loadData();           // если ранее была произведена калибровка - подтягиваем все параметры для лучшей модели
    }

    void startCalibration() {                         // начинаем калибровку
      if (isCalibrating)  {
        LOG("Already Calibrating! Resetting last params... (eeprom data won`t clear)");
        this->resetCalibration();
      }

      else {
        isCalibrating = true;
        updateCount = 0;
        linearError = quadraticError = cubicError = 0;
      }

      led.setMode(LedModes::OK);
    }

    void finishCalibration() {                        // заканчиваем
      if (!isCalibrating) {
        LOG("Calibration has not been started!");
        led.setMode(LedModes::ERROR);
      }
      else {
        isCalibrating = false;
        if (updateCount > 0) {
          calcBestModel();
          saveData();
        }
        led.setMode(LedModes::OK);
      }
    }

    void resetCalibration() {                         // сбрасываем калибровку всех моделей, данные в EEPROM не трогаются
      linearModel.reset();
      quadraticModel.reset();
      cubicModel.reset();

      bestModel = 0;  
      isCalibrating = false;
      updateCount = 0;

      linearError = quadraticError = cubicError = 0;

      calibData.modelType = 0;
      calibData.calibrated = false;

      for(int i = 0; i < 4; i++) calibData.params[i] = 0;

      led.setMode(LedModes::OK);
      LOG("Reset complete!");
    }

    void saveData() {
      calibData.modelType = bestModel;
      const float* params = nullptr;
      
      if (bestModel == 0) params = linearModel.getTheta();
      else if (bestModel == 1) params = quadraticModel.getTheta();
      else params = cubicModel.getTheta();
      
      // Копируем только реально существующие параметры
      for (int i = 0; i < 4; i++) {
          calibData.params[i] = (params && i < (bestModel + 2)) ? params[i] : 0.0f;
      }
      calibData.calibrated = true;
      calib_memory.updateNow();
    }
    
    bool loadData() {
      if (!calibData.calibrated)  return false;
      
      bestModel = calibData.modelType;
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
        uint8_t n_params = 0;
        if (calibData.modelType == 0) {
            USE_LOG.println(F("Линейная (2 параметра)"));
            n_params = 2;
        } else if (calibData.modelType == 1) {
            USE_LOG.println(F("Квадратичная (3 параметра)"));
            n_params = 3;
        } else {
            USE_LOG.println(F("Кубическая (4 параметра)"));
            n_params = 4;
        }

        USE_LOG.println(F("Коэффициенты (Theta):"));
        // Выводим с высокой точностью (6 знаков), так как числа могут быть очень маленькими
        for (uint8_t i = 0; i < n_params; i++) {
            USE_LOG.print(F("  ["));
            USE_LOG.print(i);
            USE_LOG.print(F("] = "));
            USE_LOG.println(calibData.params[i], 6); 
        }
        USE_LOG.println(F("=====================================\n"));
        #endif
        
        // Для визуального подтверждения моргнем диодом
        led.setMode(LedModes::WAIT); 
    }
    
    void calibrationStep() {        // скармливаем алгоритму новые данные
      if (!isCalibrating) return;
      updateCount++;
      
      float t = normT(sensorData.tempC);
      float error = sensorData.weightGr - referenceMass;
      
      float xL[2] = {t, 1.0f};
      float xQ[3] = {t * t, t, 1.0f};
      float xC[4] = {t * t * t, t * t, t, 1.0f};

      linearError += fabsf(error - linearModel.predict(xL));
      quadraticError += fabsf(error - quadraticModel.predict(xQ));
      cubicError += fabsf(error - cubicModel.predict(xC));

      linearModel.update(xL, error);
      quadraticModel.update(xQ, error);
      cubicModel.update(xC, error);

      calcBestModel();
    }

    float getUncertainty() const {                                    // Получить "неуверенность"
      if (bestModel == 0) return linearModel.getUncertainty();
      if (bestModel == 1) return quadraticModel.getUncertainty();
      return cubicModel.getUncertainty();

    }
    
    float compensate(float temperature, float rawWeight) {            // компенсировать вес относительно температуры
      float t = normT(temperature);
      float est = 0;
      
      if (bestModel == 0) {
          float x[2] = {t, 1.0f}; est = linearModel.predict(x);
      } else if (bestModel == 1) {
          float x[3] = {t * t, t, 1.0f}; est = quadraticModel.predict(x);
      } else {
          float x[4] = {t * t * t, t * t, t, 1.0f}; est = cubicModel.predict(x);
      }
      return rawWeight - est;
    }

    void calcBestModel() {                                            // на осноые ошибок предсказания просчитываем лучшую модель на текущий момент
      if (updateCount == 0) return;
      float sLin = (linearError / updateCount) * 1.0f; 
      float sQuad = (quadraticError / updateCount) * 1.05f;
      float sCub = (cubicError / updateCount) * 1.10f;
      
      if (sLin <= sQuad && sLin <= sCub) bestModel = 0;
      else if (sQuad <= sCub) bestModel = 1;
      else bestModel = 2;
    }

    uint8_t getBestModel() const { return bestModel; }
    bool isCalibratingMode() const { return isCalibrating; }
    bool isModelLoaded() const { return calibData.calibrated; }
};