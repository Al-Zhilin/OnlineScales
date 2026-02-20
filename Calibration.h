
template <int N>
class RLSModel {
  private:
    float theta[N] = {};                  // массив неизвестных в функции компенсирвоания, именно их мы и ищем
    float P[N][N];                        // матрица "неуверенности"
    float lambda = 0.999f;                // коэффициент "забывания"

  public:
    RLSModel() { reset(); }

    void reset() {
      for (int i = 0; i < N; i++) theta[i] = 0;
      resetP(100.0f); 
    }

    void resetP(float val) {
      for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) P[i][j] = (i == j) ? val : 0.0f;
      }
    }

    // Возвращает среднее значение "неуверенности" по диагонали
    float getUncertainty() const {
      float sum = 0;
      for (int i = 0; i < N; i++) sum += P[i][i];
      return sum / N;
    }
    
    void update(const float* x, float y) {
      float K[N];          
      float Px[N];         
      float xPx = 0.0f;    
      
      for (int i = 0; i < N; i++) {
        Px[i] = 0.0f;
        for (int j = 0; j < N; j++) Px[i] += P[i][j] * x[j];
      }
      for (int i = 0; i < N; i++) xPx += x[i] * Px[i];
      
      float denom = lambda + xPx;
      for (int i = 0; i < N; i++) K[i] = Px[i] / denom;
      
      float y_pred = predict(x);
      float error = y - y_pred;
      
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
    }
    
    float predict(const float* x) const {
      float res = 0.0f;
      for (int i = 0; i < N; i++) res += theta[i] * x[i];
      return res;
    }

    const float* getTheta() const { return theta; }
};


class ScaleAutoCalibrator {         // менеждер моделей калибровочной функции, развивает разные модели поправочной функции одновременно и позволяет в конце калибровки выбрать наилучший вариант поправки ошибки
  private:
    RLSModel<2> linearModel;       // линейная модель
    RLSModel<3> quadraticModel;    // квадратичная
    RLSModel<4> cubicModel;        // кубическая
    
    float linearError = 0.0f;      // переменные для оценки ошибки каждой модели
    float quadraticError = 0.0f;
    float cubicError = 0.0f;
    
    int updateCount = 0;           // для расчета ошибки
    int bestModel = 0;             // номер наилучшей модели по итогу будет хранится здесь
    float referenceMass;           // "эталонная масса", относительно которой калибруем
    bool isCalibrating = false;    // калибруем сейчас?

  public:
    ScaleAutoCalibrator(float refMass) : referenceMass(refMass) {}

    void begin() {
      memory.begin(0, 'Z');
      if (calibData.calibrated) {
        loadData();
      }
    }

    void resetCalibration() {      // сбрасываем калибровку всех моделей
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
      
      memory.update(); 
      Serial.println(F("Reset complete"));
    }

    float getUncertainty() const {                                  // Получить "неуверенность" текущей лучшей модели
      if (bestModel == 0) return linearModel.getUncertainty();
      if (bestModel == 1) return quadraticModel.getUncertainty();
      return cubicModel.getUncertainty();
    }
    
    void saveData() {                                 // сохраняем данные лучшей модели в EEPROM память
      calibData.modelType = bestModel;
      const float* params = nullptr;
      
      if (bestModel == 0) params = linearModel.getTheta();               // заполянем прочитанными параметрами именно ту модель, которая сохранена с пометкой как наилучшая
      else if (bestModel == 1) params = quadraticModel.getTheta();
      else params = cubicModel.getTheta();
      
      for (int i = 0; i < 4; i++) calibData.params[i] = (params && i <= bestModel + 1) ? params[i] : 0;
      calibData.calibrated = true;                                       // если сохранили параметры - значит калибровка закончена -> ставим соответствущий флаг
      
      memory.update();
    }
    
    bool loadData() {                                 // чтение данных из EEPROM
      if (calibData.calibrated) {
        bestModel = calibData.modelType;
        float* target = nullptr;
        
        if (bestModel == 0) {
          target = const_cast<float*>(linearModel.getTheta());
          linearModel.resetP(0.1f);                   // Модель загружена -> мы в ней "уверены"
        } else if (bestModel == 1) {
          target = const_cast<float*>(quadraticModel.getTheta());
          quadraticModel.resetP(0.1f);
        } else {
          target = const_cast<float*>(cubicModel.getTheta());
          cubicModel.resetP(0.1f);
        }
        
        for(int i = 0; i <= bestModel + 1; i++) target[i] = calibData.params[i];
        return true;
      }
      return false;
    }
    
    void startCalibration() {
      isCalibrating = true;
      updateCount = 0;
      linearError = quadraticError = cubicError = 0;
    }
    
    void finishCalibration() {
      isCalibrating = false;
      if (updateCount > 0) {
  
        float sLin = (linearError / updateCount) * 1.0f;            // коэффициенты?? -> см. логику бритвы Окками
        float sQuad = (quadraticError / updateCount) * 1.05f;
        float sCub = (cubicError / updateCount) * 1.10f;
        
        if (sLin <= sQuad && sLin <= sCub) bestModel = 0;
        else if (sQuad <= sCub) bestModel = 1;
        else bestModel = 2;
        
        saveData();
      }
    }
    
    void calibrationStep(float temperature, float rawWeight) {                  // очередной шаг калибровки
      if (!isCalibrating) return;
      updateCount++;
      
      float error = rawWeight - referenceMass;
      
      float xL[2] = {temperature, 1.0f};
      float xQ[3] = {temperature * temperature, temperature, 1.0f};
      float xC[4] = {temperature * temperature * temperature, temperature * temperature, temperature, 1.0f};

      linearError += abs(error - linearModel.predict(xL));                      // просчитываем текущую ошибку каждого метода
      quadraticError += abs(error - quadraticModel.predict(xQ));
      cubicError += abs(error - cubicModel.predict(xC));

      linearModel.update(xL, error);                                            // Теперь обновляем ее
      quadraticModel.update(xQ, error);
      cubicModel.update(xC, error);
    }
    
    float compensate(float temperature, float rawWeight) {                      // функция компенсации "сырого" веса с помощьью полученной модели поправки
      float est = 0;
      if (bestModel == 0) {
          float x[2] = {temperature, 1.0f}; est = linearModel.predict(x);
      } else if (bestModel == 1) {
          float x[3] = {temperature * temperature, temperature, 1.0f}; est = quadraticModel.predict(x);
      } else {
          float x[4] = {temperature * temperature * temperature, temperature * temperature, temperature, 1.0f}; est = cubicModel.predict(x);
      }
      return rawWeight - est;
    }

    uint8_t getBestModel() const { return bestModel; }
    bool isCalibratingMode() const { return isCalibrating; }
    bool isModelLoaded() const { return calibData.calibrated; }
};