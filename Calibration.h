#pragma once

#include <math.h>
#include <LittleFS.h>
#include <FileData.h>
#include <type_traits>

struct ModelEEData {
        uint8_t modelType = 0;            
        float params[4] = {0, 0, 0, 0};   
        bool calibrated = false;          
        float minCalibVal2 = 0.0f;
        float maxCalibVal2 = 0.0f;
    } calibData;

template <uint16_t N, typename T = double>
class RLSModel {
  static_assert(std::is_floating_point<T>::value, 
                  "RLSModel: Template type T must be a floating point type!");

  private:
    float theta[N] = {};       // коэффициенты компенсации
    T P[N][N];                 // матрица ковариации (неуверенности)
    T lambda = 1;              // коэффициент забывания. 1 - вообще не забывает, рекомендуется для единоразовой процедуры калибровки, 
                               //                       <1 - для использования в режиме длительного непрерывного цикла калибровки, постепенно забывает старые данные

  public:
    RLSModel() { reset(); }

    void reset() {
      for (int i = 0; i < N; i++) theta[i] = 0.0f;
      resetP(100.0f);
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

    void update(const float* x, float y, float currentRefParam) {
      // Защита матрицы от NaN значений (критично для матричных операций)
      if (isnan(y) || isnan(currentRefParam) || isinf(y)) return;

      T K[N], Px[N];
      T xPx = 0.0f;
      
      for (int i = 0; i < N; i++) {
        Px[i] = 0.0f;
        for (int j = 0; j < N; j++) Px[i] += P[i][j] * x[j];
      }
      for (int i = 0; i < N; i++) xPx += x[i] * Px[i];
      
      T denom = lambda + xPx;
      // Предотвращение деления на ноль (в RLS denom всегда должен быть > 0)
      if (denom < 1e-6f) return; 

      for (int i = 0; i < N; i++) K[i] = Px[i] / denom;
      
      T error = y - predict(x);
      for (int i = 0; i < N; i++) theta[i] += K[i] * error;
      
      T kxP[N];
      for (int j = 0; j < N; j++) {
        kxP[j] = 0;
        for (int i = 0; i < N; i++) kxP[j] += x[i] * P[i][j];
      }
      
      T trace = 0;
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

template <typename N = double>
class AdaptiveRLS {
  static_assert(std::is_floating_point<N>::value, 
                  "AdaptiveRLS: Template type T must be a floating point type!");

  private:
    FileData calib_memory{&LittleFS, "/calib.dat", 'Z', &calibData, sizeof(calibData)};             // объект в памяти для храниния откалиброванной модели

    RLSModel<2, N> linearModel;
    RLSModel<3, N> quadraticModel;
    RLSModel<4, N> cubicModel;
    
    float linearErrorCMA = 0.0f;
    float quadraticErrorCMA = 0.0f;
    float cubicErrorCMA = 0.0f;
    
    uint32_t samplesCount = 0;                  // uint32_t хватит надолго при грамотном вызове calibrationStep() и настроенном Threshold()
    uint8_t bestModel = 0; 
    float referenceVal1;
    bool isCalibrating = false;
    float min_val2 = 999.0f, max_val2 = -999.0f;
    float delta = 0.0f;
    float _minVal2Theshold = 0.5;               // минимальный шаг опорного параметра, если в calibrationStep передано меньшее значение - оно будет проигнорировано

    float _normZero = 0;
    float _normScale = 0;

    // нормировка val2 величины относительно нового нуля и разрешения
    float normVal2(float val2) const { return (val2 - _normZero) / _normScale;}


    Print* _debugOut = nullptr;                 // абстрактная основа любого потока вывода

    template <typename T>
    void debugPrint(T msg, bool newLine = true) {
        if (_debugOut == nullptr)    return;
        if (newLine) _debugOut->println(msg);
        else _debugOut->print(msg);
    }

    void debugPrintF(const __FlashStringHelper* msg, bool newLine = true) {
        if (_debugOut == nullptr)    return;
        if (newLine) _debugOut->println(msg);
        else _debugOut->print(msg);
    }

    // Отдельно для float
    void debugPrintFloat(float val, uint8_t decimals = 2, bool newLine = true) {
        if (_debugOut == nullptr) return;
        if (newLine) _debugOut->println(val, decimals);
        else _debugOut->print(val, decimals);
    }

  public:
    AdaptiveRLS(float refVal1) : referenceVal1(refVal1) {}

    void begin() {
      FDstat_t stat = calib_memory.read();      // читаем данные из памяти
      
      if (stat != FD_READ) {
          debugPrintF(F("Файл калибровки не найден, создан новый"));
          calib_memory.updateNow(); 
      }
      
      if (calibData.calibrated) loadData();
    }

    void setVal2Theshold(float new_theshold) {
        _minVal2Theshold = new_theshold;
    }
    
    // Установить новые параметры нормализации val2
    void setNormParams(float new_zero, float new_scale) {
        _normZero = new_zero;
        _normScale = new_scale;
    }

    void setDebugOut(Print* output) {
        _debugOut = output;
    }

    void startCalibration() {
      if (isCalibrating) {
        debugPrintF(F("Калбировка уже была проведена, сбрасываю параметры перед началом новой..."));
        resetCalibrationModels();
      }

      max_val2 = -999.0f;
      min_val2 = 999.0f;
      delta = 0.0f;
      
      isCalibrating = true;
      samplesCount = 0;
      linearErrorCMA = quadraticErrorCMA = cubicErrorCMA = 0.0f;
    }

    void finishCalibration() {
      if (!isCalibrating) {
        debugPrintF(F("Калбировка не была начата!"));
        return;
      }

      isCalibrating = false;

      // Защита 1: Слишком мало данных
      if (samplesCount <= 10) {
        debugPrintF(F("Калибровочная модель не сохранена. Слишком мало данных"));
        resetCalibrationModels();
        return;
      }

      // Защита 2: Недостаточный температурный диапазон
      if (delta < 5.0f) {
        debugPrintF(F("Калибровочная модель не сохранена. Использован слишком малый диапазон val2"));
        resetCalibrationModels();
        return;
      }
      
      calcBestModel();
      saveData();
    }

    void resetCalibrationModels() {     // Отдельный метод для сброса только математики (без EEPROM)
      linearModel.reset();
      quadraticModel.reset();
      cubicModel.reset();
      samplesCount = 0;
      linearErrorCMA = quadraticErrorCMA = cubicErrorCMA = 0.0f;
      bestModel = 0;
    }

    void resetCalibration() {           // метод принудительного сброса модельных переменных + данных в EEPROM
      resetCalibrationModels();
      isCalibrating = false;

      calibData.modelType = 0;
      calibData.calibrated = false;
      for(int i = 0; i < 4; i++) calibData.params[i] = 0.0f;

      calib_memory.updateNow(); 
      debugPrintF(F("Выполнен сброс моделей и сохраненных данных в памяти"));
    }

    void saveData() {
      calibData.modelType = bestModel;
      calibData.minCalibVal2 = min_val2;
      calibData.maxCalibVal2 = max_val2;
      const float* params = nullptr;
      
      if (bestModel == 0) params = linearModel.getTheta();
      else if (bestModel == 1) params = quadraticModel.getTheta();
      else params = cubicModel.getTheta();
      
      for (int i = 0; i < 4; i++) {
        calibData.params[i] = (params && i < (bestModel + 2)) ? params[i] : 0.0f;
      }
      calibData.calibrated = true;
      
      calib_memory.updateNow();
    }
    
    bool loadData() {
      if (!calibData.calibrated) return false;
      
      bestModel = calibData.modelType;
      max_val2 = calibData.maxCalibVal2;
      min_val2 = calibData.minCalibVal2;
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
      debugPrintF(F("Сохраненные данные калибровки:"), false);
      
      if (!calibData.calibrated) {
        debugPrintF(F("Нет сохраненных данных"));
        return;
      }

      debugPrintF(F("Лучшая модель: "), false);
      
      uint8_t n_params = calibData.modelType + 2;
      if (calibData.modelType == 0)         debugPrintF(F("Линейная"));
      else if (calibData.modelType == 1)    debugPrintF(F("Квадратичная"));
      else                                  debugPrintF(F("Кубическая"));

      debugPrintF(F("Коэффициенты уравнения:"));
      for (uint8_t i = 0; i < n_params; i++) {
        debugPrintF(F("  ["), false);
        debugPrint(i, false);
        debugPrintF(F(" = "), false);
        debugPrintFloat(calibData.params[i], 6);
      }
    }
    
    void calibrationStep(float val1, float val2) {
      if (!isCalibrating) return;

      min_val2 = min(min_val2, val2);
      max_val2 = max(max_val2, val2);
      delta = max_val2 - min_val2;
      
      static float _lastVal2 = -999.0f;
      if (fabs(val2 - _lastVal2) < _minVal2Theshold)    return;
      _lastVal2 = val2;
      samplesCount++;
      
      float norm_val2 = normVal2(val2);
      float error = val1 - referenceVal1;
      
      // Векторы признаков
      float xL[2] = {norm_val2, 1.0f};
      float xQ[3] = {norm_val2 * norm_val2, norm_val2, 1.0f};
      float xC[4] = {norm_val2 * norm_val2 * norm_val2, norm_val2 * norm_val2, norm_val2, 1.0f};

      // Текущие ошибки моделей
      float errL = fabsf(error - linearModel.predict(xL));
      float errQ = fabsf(error - quadraticModel.predict(xQ));
      float errC = fabsf(error - cubicModel.predict(xC));

      // Обновление кумулятивного среднего (CMA)
      if (samplesCount <= 1) {
        linearErrorCMA = errL;
        quadraticErrorCMA = errQ;
        cubicErrorCMA = errC;
      } else {
        // Классическая формула CMA без риска переполнения
        linearErrorCMA = linearErrorCMA + (errL - linearErrorCMA) / (float)samplesCount;
        quadraticErrorCMA = quadraticErrorCMA + (errQ - quadraticErrorCMA) / (float)samplesCount;
        cubicErrorCMA = cubicErrorCMA + (errC - cubicErrorCMA) / (float)samplesCount;
      }

      linearModel.update(xL, error, norm_val2);
      quadraticModel.update(xQ, error, norm_val2);
      cubicModel.update(xC, error, norm_val2);

      if (samplesCount > 10) calcBestModel();   // Лучшая модель будет рассчитываться "на лету" корректно только когда модели накопят достаточно данных
    }

    float getUncertainty() const {
      if (bestModel == 0) return linearModel.getUncertainty();
      if (bestModel == 1) return quadraticModel.getUncertainty();
      return cubicModel.getUncertainty();
    }
    
    // Оптимизированная компенсация (схема Горнера для расчета полиномов)
    float compensate(float rawVal1, float val2) {
      if (!isCalibrating && !isModelLoaded()) return rawVal1;
      if (!isCalibrating && isModelLoaded())  val2 = constrain(val2, min_val2, max_val2);          // защита от неадекватных величин при выходе val2 за границы своего откалиброванного диапазона

      float t = normVal2(val2);
      float est = 0.0f;
      
      // Берем коэффициенты активной модели (из ОЗУ, они обновляются в реальном времени при калибровке)
      const float* p = nullptr;
      if (bestModel == 0) p = linearModel.getTheta();
      else if (bestModel == 1) p = quadraticModel.getTheta();
      else p = cubicModel.getTheta();

      // Метод Горнера для быстрого вычисления полинома
      // p = { a, b, c, d } соответствует a*t^3 + b*t^2 + c*t + d
      est = p[0];
      Serial.print(p[0] + String(" "));
      for (int i = 1; i < bestModel + 2; i++) {
        est = est * t + p[i];
      }
      
      return rawVal1 - est;             // компенсируем сырую величину вычисленным калибровочным значением
    }

    void calcBestModel() {
        float sLin = linearErrorCMA * 1.0f; 
        float sQuad = quadraticErrorCMA * 1.05f;
        float sCub = cubicErrorCMA * 1.10f;
        
        if (sLin <= sQuad && sLin <= sCub) bestModel = 0;
        else if (sQuad <= sCub) bestModel = 1;
        else bestModel = 2;
    }

    String getPolynomialString(bool urlEncoded = false) const {
        if (!calibData.calibrated) return "0";

        String res = "";
        uint8_t n_params = calibData.modelType + 2;
        bool isFirst = true;

        // Определяем символы в зависимости от флага кодировки
        String space = urlEncoded ? "%20" : " ";
        String plus  = urlEncoded ? "%2B" : "+";
        String hat   = urlEncoded ? "%5E" : "^";

        for (uint8_t i = 0; i < n_params; i++) {
            if (fabs(calibData.params[i]) > 0.000001f) {
                if (calibData.params[i] < 0) {
                    if (isFirst) res += "-";
                    else res += space + "-" + space;
                } else {
                    // Плюс ставится ТОЛЬКО если элемент не первый
                    if (!isFirst) res += space + plus + space;
                }
                isFirst = false;

                res += String(fabs(calibData.params[i]), 2);
                
                if (i != n_params - 1) { // Если не свободный член
                    res += "x";
                    if (n_params - (i + 1) > 1) { // Степень больше 1
                        res += hat + String(n_params - (i + 1));
                    }
                }
            }
        }
        
        if (isFirst) return "0"; // Если все коэффициенты были нулями
        return res;
    }
    
    uint8_t getBestModel() const { return bestModel; }
    bool isCalibratingMode() const { return isCalibrating; }
    bool isModelLoaded() const { return calibData.calibrated; }
};