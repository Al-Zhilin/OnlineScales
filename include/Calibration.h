#pragma once

//  Выбор хранилища (задаётся через build_flags или до включения файла)

#define CALIB_STORAGE_MANUAL    0
#define CALIB_STORAGE_EEPROM    1
#define CALIB_STORAGE_LITTLEFS  2

#ifndef CALIB_STORAGE
  #define CALIB_STORAGE CALIB_STORAGE_LITTLEFS
#endif

#if CALIB_STORAGE == CALIB_STORAGE_LITTLEFS
  #include <LittleFS.h>
  #include <FileData.h>
  #include <Arduino.h>
#elif CALIB_STORAGE == CALIB_STORAGE_EEPROM
  #include <EEPROM.h>
  #include <Arduino.h>
#endif

#include <math.h>
#include <float.h>
#include <type_traits>


//  Данные калибровки для сохранения во flash / EEPROM

struct ModelEEData {
    uint8_t  version          = 2;       // версия формата; увеличивать при изменении раскладки структуры
    uint8_t  modelType        = 0;       // тип модели: 0=линейная, 1=квадратичная, 2=кубическая
    uint8_t  numParams        = 0;       // реальная длина theta для лучшей модели (LN/QN/CN, не более 6)
    bool     calibrated       = false;   // флаг успешно завершённой калибровки
    float    params[6]        = {};      // коэффициенты theta в нормализованных координатах (до 6 для EMA-моделей)
    float    minCalibVal2     = 0.0f;    // минимальный val2, наблюдавшийся при калибровке
    float    maxCalibVal2     = 0.0f;    // максимальный val2, наблюдавшийся при калибровке
    float    savedUncertainty = 100.0f;  // неопределённость P в момент finishCalibration(); восстанавливается после загрузки
};


//  RLSModel<N, T> — одна рекуррентная регрессионная модель с N признаками
//    N — размерность вектора признаков
//    T — тип вычислений (рекомендуется double)

template <uint16_t N, typename T = double>
class RLSModel {
    static_assert(std::is_floating_point<T>::value, "RLSModel requires floating-point T");

private:
    float theta[N]   = {};         // вектор коэффициентов регрессии (веса модели)
    T     P[N][N]    = {};         // ковариационная матрица неопределённости; диагональ сужается по мере сходимости
    T     _lambda    = T(1);       // коэффициент забывания λ ∈ (0,1]; 1 = без забывания
    T     _pfloor    = T(0);       // нижняя граница диагонали P; предотвращает «замерзание» усиления Калмана
    float _p0Scale   = 100000.0f; // начальное значение диагонали P; задаёт масштаб 100%-ной неопределённости

public:
    // Конструктор: инициализирует theta=0, P=_p0Scale*I.
    RLSModel() { reset(); }

    // Сбрасывает theta к нулю и P к начальной диагональной матрице (100000*I).
    void reset() {
        for (uint16_t i = 0; i < N; i++) theta[i] = 0.0f;
        resetP(100000.0f);
    }

    // Сбрасывает P к val*I и сохраняет val как масштаб неопределённости (_p0Scale).
    // Вход: val — новое начальное значение диагонали P; используется как 100% для getUncertainty().
    void resetP(float val) {
        _p0Scale = val;
        for (uint16_t i = 0; i < N; i++)
            for (uint16_t j = 0; j < N; j++)
                P[i][j] = (i == j) ? T(val) : T(0);
    }

    // Устанавливает коэффициент забывания λ. Значения ≤0 заменяются на 1 (без забывания).
    // Вход: lam — λ ∈ (0,1]; чем меньше значение, тем быстрее «забываются» старые точки.
    void setLambda(T lam) { _lambda = (lam > T(0)) ? lam : T(1); }

    // Устанавливает нижнюю границу диагонали P.
    // Вход: pf — минимальное значение элемента диагонали P; не даёт усилению Калмана обнулиться/упасть слишком низко.
    void setPfloor(T pf)  { _pfloor = pf; }

    // Масштабирует только диагональ P на factor (не более diagonalCeiling); внедиагональные элементы не трогаются.
    // Вход: factor — множитель для диагонали; diagonalCeiling — верхняя граница диагонального элемента.
    // Масштабирование только диагонали сохраняет положительную определённость (PSD): при factor≥1
    // диагональ строго доминирует, а выученная структура корреляций (внедиагональ) остаётся нетронутой.
    // Масштабирование всех элементов с ограничением только диагонали нарушает PSD — знак усиления Калмана инвертируется.
    void scaleP(float factor, float diagonalCeiling = 100000.0f) {
        for (uint16_t i = 0; i < N; i++) {
            P[i][i] *= T(factor);
            if (P[i][i] > T(diagonalCeiling)) P[i][i] = T(diagonalCeiling);
        }
    }

    // Загружает вектор коэффициентов из внешнего буфера.
    // Вход: src — указатель на массив из N float; nullptr игнорируется.
    void setTheta(const float* src) {
        if (!src) return;
        for (uint16_t i = 0; i < N; i++) theta[i] = src[i];
    }

    // Возвращает указатель на внутренний вектор коэффициентов theta (только чтение).
    const float* getTheta() const { return theta; }

    // Возвращает глобальную неопределённость как % от начального масштаба (0..100%).
    // Вычисляется как trace(P)/(N*_p0Scale)*100. Ограничена 100%: _tryReproject() может
    // кратковременно раздувать P выше начального масштаба.
    float getUncertainty() const {
        T trace = T(0);
        for (uint16_t i = 0; i < N; i++) trace += P[i][i];
        float u = float(trace / (T(N) * T(_p0Scale))) * 100.0f;
        return (u > 100.0f) ? 100.0f : u;
    }

    // Выполняет один шаг рекуррентного МНК: обновляет theta и P по новой точке (x, y).
    // Вход: x — вектор признаков длиной N; y — целевое значение (ошибка сенсора).
    // NaN/Inf в x или y пропускаются без изменения состояния.
    void update(const float* x, T y) {
        if (isnan(float(y)) || isinf(float(y))) return;
        for (uint16_t i = 0; i < N; i++)
            if (isnan(x[i]) || isinf(x[i])) return;

        T Px[N] = {}, xPx = T(0);
        for (uint16_t i = 0; i < N; i++)
            for (uint16_t j = 0; j < N; j++)
                Px[i] += P[i][j] * T(x[j]);
        for (uint16_t i = 0; i < N; i++) xPx += T(x[i]) * Px[i];

        T denom = _lambda + xPx;
        if (denom < T(1e-10)) return;

        T pred = T(0);
        for (uint16_t i = 0; i < N; i++) pred += T(theta[i]) * T(x[i]);
        T err = y - pred;

        T K[N];
        for (uint16_t i = 0; i < N; i++) K[i] = Px[i] / denom;
        for (uint16_t i = 0; i < N; i++) theta[i] += float(K[i] * err);

        T kxP[N] = {};
        for (uint16_t j = 0; j < N; j++)
            for (uint16_t i = 0; i < N; i++)
                kxP[j] += T(x[i]) * P[i][j];

        T trace = T(0);
        for (uint16_t i = 0; i < N; i++) {
            for (uint16_t j = 0; j < N; j++)
                P[i][j] = (P[i][j] - K[i] * kxP[j]) / _lambda;
            if (P[i][i] < _pfloor) P[i][i] = _pfloor;
            trace += P[i][i];
        }

        // Симметризация P для компенсации накопления ошибок округления float
        for (uint16_t i = 0; i < N; i++)
            for (uint16_t j = i + 1; j < N; j++)
                P[i][j] = P[j][i] = (P[i][j] + P[j][i]) * T(0.5);

        if (float(trace) > 1e6f || isnan(float(trace))) {
            // Переполнение: theta и P численно недостоверны — сброс обоих.
            // P восстанавливается до p0Scale*I: неопределённость возвращается к 100% и обучение продолжается.
            for (uint16_t ii = 0; ii < N; ii++) {
                theta[ii] = 0.0f;
                for (uint16_t jj = 0; jj < N; jj++)
                    P[ii][jj] = (ii == jj) ? T(_p0Scale) : T(0);
            }
        }
    }

    // Вычисляет предсказание модели: y = theta^T * x. Возвращает T для сохранения точности double.
    // Вход: x — вектор признаков длиной N.
    // Выход: скалярное предсказание в типе T (double сохраняет точность в compensate()).
    T predict(const float* x) const {
        T res = T(0);
        for (uint16_t i = 0; i < N; i++) res += T(theta[i]) * T(x[i]);
        return res;
    }

    // Вычисляет точечную неопределённость модели в направлении вектора x: (x^T P x)/(p0Scale*||x||^2)*100%.
    // Вход: x — вектор признаков длиной N (то же, что передаётся в predict()).
    // Выход: 100% в начале калибровки; убывает к 0% по мере сходимости именно в точке x. Ограничена 100%.
    float getUncertaintyAt(const float* x) const {
        T xPx = T(0), norm2 = T(0);
        for (uint16_t i = 0; i < N; i++) {
            T Pxi = T(0);
            for (uint16_t j = 0; j < N; j++) Pxi += P[i][j] * T(x[j]);
            xPx  += T(x[i]) * Pxi;
            norm2 += T(x[i]) * T(x[i]);
        }
        if (norm2 < T(1e-20)) return 0.0f;
        float u = float(xPx / (T(_p0Scale) * norm2)) * 100.0f;
        return (u > 100.0f) ? 100.0f : u;
    }

    // Применяет замену координат к матрице P: P_new = M^T * P_old * M (x_old = M * x_new).
    // Вход: M — матрица преобразования N×N (x_old = M * x_new).
    // Используется после _reprojectPoly() при смене нормировки val2 для согласованности P и theta.
    // После преобразования P симметризуется для компенсации накопления ошибок float.
    void applyPTransform(const float (*M)[N]) {
        T temp[N][N] = {};
        for (uint16_t i = 0; i < N; i++)
            for (uint16_t j = 0; j < N; j++)
                for (uint16_t k = 0; k < N; k++)
                    temp[i][j] += T(M[k][i]) * P[k][j];
        for (uint16_t i = 0; i < N; i++)
            for (uint16_t j = 0; j < N; j++) {
                P[i][j] = T(0);
                for (uint16_t k = 0; k < N; k++)
                    P[i][j] += temp[i][k] * T(M[k][j]);
            }
        for (uint16_t i = 0; i < N; i++)
            for (uint16_t j = i + 1; j < N; j++)
                P[i][j] = P[j][i] = (P[i][j] + P[j][i]) * T(0.5);
    }
};


//  AdaptiveRLS<T, EnableDynamic, NumEma> — адаптивный RLS-компенсатор дрейфа
//    T             — точность вычислений (рекомендуется double)
//    EnableDynamic — включает адаптивный λ и EMA-признаки
//    NumEma        — количество EMA-фильтров, добавляемых как дополнительные признаки

template <typename T = double, bool EnableDynamic = false, uint8_t NumEma = 0>
class AdaptiveRLS {
private:
    static constexpr uint8_t _NE    = EnableDynamic ? NumEma : 0; // эффективное число EMA (0 если !EnableDynamic)
    static constexpr uint8_t LN    = uint8_t(2 + _NE); // длина вектора признаков линейной модели: [t, ema..., 1]
    static constexpr uint8_t QN    = uint8_t(3 + _NE); // длина вектора признаков квадратичной модели: [t^2, t, ema..., 1]
    static constexpr uint8_t CN    = uint8_t(4 + _NE); // длина вектора признаков кубической модели: [t^3, t^2, t, ema..., 1]
    static constexpr uint8_t _EABUF = (_NE > 0) ? _NE : 1; // размер буфера EMA (минимум 1 для корректной компиляции)

    // ── Общие состояния ─────────────────────────────────────────────────────
    //bool _isBegined = false;

    // ── Модели ──────────────────────────────────────────────────────────────
    RLSModel<LN, T> linearModel;      // линейная модель: y = a*t + b (+ EMA-члены)
    RLSModel<QN, T> quadraticModel;   // квадратичная модель: y = a*t^2 + b*t + c (+ EMA-члены)
    RLSModel<CN, T> cubicModel;       // кубическая модель: y = a*t^3 + ... + d (+ EMA-члены)

    // ── Состояние калибровки ─────────────────────────────────────────────────
    T           referenceVal1;               // эталонное val1 при калибровке (целевая точка без дрейфа)
    ModelEEData _calibData;                  // данные, сохраняемые во flash/EEPROM
    bool        isCalibrating    = false;    // true в процессе calibrationStep(); false после finish/reset
    uint8_t     bestModel        = 0;        // индекс лучшей модели: 0=линейная, 1=квадратичная, 2=кубическая
    uint32_t    samplesCount     = 0;        // количество принятых шагов calibrationStep()
    float       max_val2         = -FLT_MAX; // максимальный val2 за сессию (для delta и нормировки)
    float       min_val2         =  FLT_MAX; // минимальный val2 за сессию (для delta и нормировки)
    float       delta            = 0.0f;     // размах val2: max_val2 - min_val2; проверяется в finishCalibration()
    float       linearErrorCMA   = 0.0f;     // CMA абсолютной ошибки линейной модели (на данных до update)
    float       quadraticErrorCMA = 0.0f;    // CMA абсолютной ошибки квадратичной модели
    float       cubicErrorCMA    = 0.0f;     // CMA абсолютной ошибки кубической модели

    // ── Нормировка ───────────────────────────────────────────────────────────
    float _normZero           = 0.0f;  // центр нормировки: t = (val2 - _normZero) / _normScale
    float _normScale          = 0.0f;  // полуширина диапазона нормировки; 0 = не инициализировано
    float _preferredNormZero  = 0.0f;  // предпочтительный центр нормировки (задаётся setNormParams)
    float _preferredNormScale = 0.0f;  // предпочтительная полуширина нормировки (задаётся setNormParams)
    float _lastVal2           = NAN;   // предыдущий val2; NAN = нет предыдущего (для фильтрации малых шагов)

    // ── Состояние EMA ────────────────────────────────────────────────────────
    float _emaState [_EABUF] = {};     // текущие значения EMA-фильтров в нормированных координатах
    float _emaAlphas[_EABUF] = {};     // коэффициенты сглаживания EMA: (1 - alpha_i) = вес нового значения
    bool  _emaInitialized    = false;  // true после первого обращения к _updateEmas(); сброс в startCalibration()

    // ── Параметры ────────────────────────────────────────────────────────────
    uint32_t _minSamples    = 10;     // минимум принятых шагов для успешного finishCalibration()
    float    _minDelta      = 5.0f;   // минимальный размах val2 для успешного finishCalibration()
    float    _val2Threshold = 0.5f;   // минимальный шаг val2 между calibrationStep(); меньшие пропускаются
    float    _penaltyQuad   = 1.10f;  // штраф CMA-ошибки квадратичной модели при выборе лучшей (сложность)
    float    _penaltyCubic  = 1.20f;  // штраф CMA-ошибки кубической модели при выборе лучшей
    float    _lambdaBase    = 1.0f;   // минимальный λ при адаптивном забывании (EnableDynamic); 1.0 = без забывания
    float    _slopeThresh   = 8.0f;   // порог |Δval2|, при превышении которого λ начинает уменьшаться
    float    _slopeGain     = 25.0f;  // крутизна снижения λ: 1/_slopeGain — ширина переходной зоны
    float    _pfloor        = 0.0f;   // нижняя граница диагонали P для всех моделей
    uint8_t  _cmaWarmup     = 3;      // первые N шагов обучают модели, но не участвуют в CMA-оценке

    // ── Состояние после калибровки ───────────────────────────────────────────
    float _savedUncertainty = 100.0f; // неопределённость, сохранённая в finishCalibration(); возвращается при _isLoaded
    bool  _isLoaded         = false;  // true после _loadData(); сбрасывается в startCalibration()

    // ── Авто-буст во время калибровки ────────────────────────────────────────
    float   _driftThreshold     = 0.0f;  // порог остатка (в единицах rawVal1) для буста P; 0 = отключено
    float   _driftBoostFactor   = 10.0f; // множитель для boostP() при превышении _driftThreshold
    uint8_t _boostCooldownSteps = 5;     // число шагов блокировки буста после срабатывания
    uint8_t _boostCooldown      = 0;     // оставшийся кулдаун; 0 = буст разрешён

    // ── Отладка ──────────────────────────────────────────────────────────────
    Print* _debugOut = nullptr;         // поток отладочного вывода; nullptr = отключено

#if CALIB_STORAGE == CALIB_STORAGE_LITTLEFS
    FileData _fileData{&LittleFS, "/calib.dat", 'Z', &_calibData, sizeof(ModelEEData)};
#endif

    // ─────────────────────────── приватные вспомогательные ───────────────────

    // Нормализует val2 в безразмерный признак t = (v - _normZero) / _normScale.
    // Вход: v — значение val2 в физических единицах.
    // Выход: нормализованный признак t; 0 при незаданной нормировке (_normScale≤0).
    float normVal2(float v) const {
        if (_normScale <= 0.0f) return 0.0f;
        return (v - _normZero) / _normScale;
    }

    // Ограничивает val2 диапазоном калибровки [minCalibVal2, maxCalibVal2].
    // Вход: v — произвольный val2.
    // Выход: v, зажатый в сохранённый диапазон; предотвращает экстраполяцию за пределы калибровки.
    float _clampVal2(float v) const {
        if (v < _calibData.minCalibVal2) return _calibData.minCalibVal2;
        if (v > _calibData.maxCalibVal2) return _calibData.maxCalibVal2;
        return v;
    }

    // Обновляет EMA-фильтры по нормализованному значению norm_v.
    // При первом вызове инициализирует все EMA в norm_v (холодный старт).
    // Вход: norm_v — нормализованный val2 (t). Нет эффекта если !EnableDynamic || NumEma==0.
    void _updateEmas(float norm_v) {
        if constexpr (EnableDynamic && NumEma > 0) {
            if (!_emaInitialized) {
                for (uint8_t i = 0; i < NumEma; i++) _emaState[i] = norm_v;
                _emaInitialized = true;
            } else {
                for (uint8_t i = 0; i < NumEma; i++)
                    _emaState[i] = _emaAlphas[i] * _emaState[i]
                                 + (1.0f - _emaAlphas[i]) * norm_v;
            }
        }
    }

    // Формирует векторы признаков для трёх моделей из нормализованного t и текущих EMA.
    // Вход: t — нормализованный val2; xL[LN]/xQ[QN]/xC[CN] — выходные буферы признаков.
    // Структура: [t^deg, ..., t, ema_0, ..., ema_k, 1.0] (bias всегда последний).
    void _buildFeatures(float t, float* xL, float* xQ, float* xC) const {
        xL[0] = t;
        xQ[0] = t*t;     xQ[1] = t;
        xC[0] = t*t*t;   xC[1] = t*t;   xC[2] = t;

        if constexpr (EnableDynamic && NumEma > 0) {
            for (uint8_t i = 0; i < NumEma; i++) {
                xL[1 + i] = _emaState[i];
                xQ[2 + i] = _emaState[i];
                xC[3 + i] = _emaState[i];
            }
            xL[1 + NumEma] = 1.0f;
            xQ[2 + NumEma] = 1.0f;
            xC[3 + NumEma] = 1.0f;
        } else {
            xL[1] = 1.0f;
            xQ[2] = 1.0f;
            xC[3] = 1.0f;
        }
    }

    // Строит верхнетреугольную матрицу преобразования M[Sz][Sz] для замены координат
    // t_old = alpha*t_new + beta в полиномиальном пространстве признаков.
    // Вход: M — выходной буфер Sz×Sz; alpha, beta — параметры аффинного преобразования val2;
    //       deg — степень полинома (1/2/3).
    // Строка p (степень deg-p): биномиальное разложение t_old^(deg-p) по степеням t_new.
    // EMA-строки (deg..Sz-2): линейное преобразование M[p][p]=alpha, M[p][Sz-1]=beta.
    // Строка смещения (Sz-1): тождественная.
    template<int Sz>
    static void _buildPolyM(float (&M)[Sz][Sz], float alpha, float beta, int deg) {
        for (int i = 0; i < Sz; i++) for (int j = 0; j < Sz; j++) M[i][j] = 0.0f;

        float ap[Sz], bp[Sz];
        ap[0] = bp[0] = 1.0f;
        for (int i = 1; i < Sz; i++) { ap[i] = ap[i-1]*alpha; bp[i] = bp[i-1]*beta; }

        for (int p = 0; p < deg; p++) {
            int k = deg - p;
            uint32_t binom = 1;
            for (int j = 0; j <= k; j++) {
                int q = (j == 0) ? (Sz - 1) : (deg - j);
                if (q >= 0 && q < Sz) M[p][q] += float(binom) * ap[j] * bp[k - j];
                if (j < k) binom = binom * uint32_t(k - j) / uint32_t(j + 1);
            }
        }
        // EMA-признаки преобразуются линейно (так же как t): ema_old = alpha*ema_new + beta
        for (int p = deg; p < Sz - 1; p++) {
            M[p][p]     = alpha;
            M[p][Sz-1]  = beta;
        }
        M[Sz-1][Sz-1] = 1.0f;  // строка смещения: тождественная
    }

    // Перепроецирует коэффициенты theta полинома при замене нормировки t_old = alpha*t_new + beta.
    // Вход: th — вектор theta; degree — степень полинома (1/2/3); total — полная длина th;
    //       alpha, beta — параметры аффинного преобразования нормировки.
    // EMA-коэффициенты (индексы degree..total-2) масштабируются на alpha;
    // их сдвиг beta суммируется в bias (последний элемент th).
    static void _reprojectPoly(float* th, uint8_t degree, uint8_t total,
                                float alpha, float beta) {
        uint8_t bi = uint8_t(total - 1);
        // Масштабирование EMA-коэффициентов на alpha; накопление их beta-сдвига в смещение
        float emaBeta = 0.0f;
        for (uint8_t i = degree; i < bi; i++) {
            emaBeta += th[i] * beta;
            th[i]   *= alpha;
        }
        if (degree == 1) {
            float a = th[0], b = th[bi];
            th[0]  = a * alpha;
            th[bi] = a * beta + b + emaBeta;
        } else if (degree == 2) {
            float a = th[0], b = th[1], c = th[bi];
            th[0]  = a * alpha * alpha;
            th[1]  = 2.0f*a*alpha*beta + b*alpha;
            th[bi] = a*beta*beta + b*beta + c + emaBeta;
        } else {
            float a = th[0], b = th[1], c = th[2], d = th[bi];
            th[0]  = a*alpha*alpha*alpha;
            th[1]  = 3.0f*a*alpha*alpha*beta + b*alpha*alpha;
            th[2]  = 3.0f*a*alpha*beta*beta  + 2.0f*b*alpha*beta + c*alpha;
            th[bi] = a*beta*beta*beta + b*beta*beta + c*beta + d + emaBeta;
        }
    }

    // Расширяет диапазон нормировки чтобы охватить val2, затем перепроецирует
    // theta и P всех трёх моделей в новые координаты.
    // При первом вызове инициализирует нормировку: _normZero=val2, _normScale=50
    // (типичная полуширина для 100°C-диапазона — предотвращает медленную сходимость с малыми признаками).
    // Вход: val2 — новое значение val2, возможно выходящее за текущий нормировочный диапазон.
    void _tryReproject(float val2) {
        if (_normScale <= 0.0f) {
            _normZero  = val2;
            _normScale = 50.0f;
            return;
        }
        if (val2 >= _normZero - _normScale && val2 <= _normZero + _normScale) return;

        float cur_min = _normZero - _normScale;
        float cur_max = _normZero + _normScale;
        float new_min = (val2 < cur_min) ? val2 : cur_min;
        float new_max = (val2 > cur_max) ? val2 : cur_max;
        float new_zero  = (new_min + new_max) * 0.5f;
        float new_scale = (new_max - new_min) * 0.5f;
        if (new_scale < 1e-4f) new_scale = 1e-4f;

        float alpha = new_scale / _normScale;                // new/old: коэффициенты полинома растут при расширении диапазона
        float beta  = (new_zero - _normZero) / _normScale;  // сдвиг в единицах старой нормировки

        {
            float buf[LN];
            const float* s = linearModel.getTheta();
            for (int i = 0; i < LN; i++) buf[i] = s[i];
            _reprojectPoly(buf, 1, LN, alpha, beta);
            linearModel.setTheta(buf);
            float ML[LN][LN] = {};
            _buildPolyM(ML, alpha, beta, 1);
            linearModel.applyPTransform(ML);
        }
        {
            float buf[QN];
            const float* s = quadraticModel.getTheta();
            for (int i = 0; i < QN; i++) buf[i] = s[i];
            _reprojectPoly(buf, 2, QN, alpha, beta);
            quadraticModel.setTheta(buf);
            float MQ[QN][QN] = {};
            _buildPolyM(MQ, alpha, beta, 2);
            quadraticModel.applyPTransform(MQ);
        }
        {
            float buf[CN];
            const float* s = cubicModel.getTheta();
            for (int i = 0; i < CN; i++) buf[i] = s[i];
            _reprojectPoly(buf, 3, CN, alpha, beta);
            cubicModel.setTheta(buf);
            float MC[CN][CN] = {};
            _buildPolyM(MC, alpha, beta, 3);
            cubicModel.applyPTransform(MC);
        }

        // Перепересчёт существующих EMA-состояний в новую нормировку
        if constexpr (EnableDynamic && NumEma > 0) {
            if (_emaInitialized) {
                for (uint8_t i = 0; i < NumEma; i++)
                    _emaState[i] = (_emaState[i]*_normScale + _normZero - new_zero) / new_scale;
            }
        }

        _normZero  = new_zero;
        _normScale = new_scale;
    }

    // Вычисляет bestModel по взвешенным CMA-ошибкам с учётом штрафов за сложность.
    // Выбирает модель с наименьшим произведением CMA * penalty. Результат записывается в bestModel.
    void _calcBestModel() {
        float sL = linearErrorCMA;
        float sQ = quadraticErrorCMA * _penaltyQuad;
        float sC = cubicErrorCMA * _penaltyCubic;
        if (sL <= sQ && sL <= sC) bestModel = 0;
        else if (sQ <= sC)  bestModel = 1;
        else bestModel = 2;
    }

    // Возвращает индекс «живой» лучшей модели в текущий момент.
    // Во время прогрева (samplesCount ≤ _cmaWarmup) — всегда 0 (линейная, CMA ещё не достоверна).
    // После прогрева — модель с наименьшей взвешенной CMA-ошибкой.
    // После finishCalibration() — сохранённый bestModel.
    uint8_t _liveModelType() const {
        if (isCalibrating) {
            if (samplesCount <= _cmaWarmup) return 0;
            float sL = linearErrorCMA;
            float sQ = quadraticErrorCMA * _penaltyQuad;
            float sC = cubicErrorCMA * _penaltyCubic;
            if (sL <= sQ && sL <= sC) return 0;
            if (sQ <= sC)   return 1;
            return 2;
        }
        return bestModel;
    }

    // Загружает параметры из _calibData во внутреннее состояние (theta, нормировка, bestModel).
    // Выход: true — данные корректны и загружены; false — данные не прошли валидацию.
    // При успехе устанавливает _isLoaded=true; нормировка восстанавливается по min/max_val2.
    bool _loadData() {
        if (_calibData.version != 2)    return false;
        if (!_calibData.calibrated) return false;
        if (_calibData.numParams < 2 || _calibData.numParams > 6)  return false;
        // numParams должен совпадать с реальной длиной theta для сохранённого типа модели
        uint8_t expectedParams;
        if      (_calibData.modelType == 0) expectedParams = uint8_t(LN < 7 ? LN : 6);
        else if (_calibData.modelType == 1) expectedParams = uint8_t(QN < 7 ? QN : 6);
        else if (_calibData.modelType == 2) expectedParams = uint8_t(CN < 7 ? CN : 6);
        else return false;
        if (_calibData.numParams != expectedParams) return false;
        for (uint8_t i = 0; i < _calibData.numParams; i++)
            if (isnan(_calibData.params[i]) || isinf(_calibData.params[i])) return false;

        min_val2 = _calibData.minCalibVal2;
        max_val2 = _calibData.maxCalibVal2;
        delta    = max_val2 - min_val2;

        _normZero  = (min_val2 + max_val2) * 0.5f;
        _normScale = (max_val2 - min_val2) * 0.5f;
        if (_normScale < 1e-4f) _normScale = 1e-4f;

        bestModel = _calibData.modelType;
        const float* p = _calibData.params;
        if      (bestModel == 0) linearModel.setTheta(p);
        else if (bestModel == 1) quadraticModel.setTheta(p);
        else                     cubicModel.setTheta(p);

        float su = _calibData.savedUncertainty;
        _savedUncertainty = (isnan(su) || su < 0.0f || su > 100.0f) ? 100.0f : su;
        _isLoaded = true;
        return true;
    }

    // Сбрасывает все три модели к начальному состоянию и устанавливает pfloor.
    void _resetModels() {
        linearModel.reset();
        quadraticModel.reset();
        cubicModel.reset();
        T pf = T(_pfloor);
        linearModel.setPfloor(pf);
        quadraticModel.setPfloor(pf);
        cubicModel.setPfloor(pf);
    }

    // Адаптирует λ всех моделей в зависимости от скорости изменения val2 (только если EnableDynamic).
    // При |Δval2| > _slopeThresh λ плавно снижается до _lambdaBase; ниже порога λ=1 (без забывания).
    // Вход: val2 — текущее значение val2; Δval2 вычисляется относительно _lastVal2.
    void _applyAdaptiveLambda(float val2) {
        if constexpr (EnableDynamic) {
            if (!isnan(_lastVal2)) {
                // Кусочно-линейная функция: нет забывания ниже slopeThresh, полное забывание
                // при slopeThresh + 1/slopeGain. slopeGain=25 → 0.04°C переходная зона;
                // slopeGain=0.5 → 2°C переходная зона.
                float excess = fabsf(val2 - _lastVal2) - _slopeThresh;
                float t = (excess > 0.0f) ? fminf(excess * _slopeGain, 1.0f) : 0.0f;
                T lam = T(1.0f - t * (1.0f - _lambdaBase));
                linearModel.setLambda(lam);
                quadraticModel.setLambda(lam);
                cubicModel.setLambda(lam);
            }
        }
    }

public:
    // ──────────────────────────── конструктор ──────────────────────────────
    // Вход: refVal1 — эталонное значение val1 (целевая точка без ошибки, например 0.0 для нулевого дрейфа).
    explicit AdaptiveRLS()  {}

    // ──────────────────────────── конфигурация ─────────────────────────────

    // Устанавливает поток отладочного вывода (Serial и т.п.).
    // Вход: p — указатель на Print; nullptr отключает вывод.
    void setDebugOut(Print* p)      { _debugOut = p; }

    // Устанавливает минимальный размах val2 для успешного finishCalibration().
    // Вход: d — минимальный delta (max_val2 - min_val2); при меньшем размахе finish вернёт false.
    void setMinDelta(float d)       { _minDelta = d; }

    // Устанавливает минимальный шаг val2 между соседними calibrationStep().
    // Вход: thr — порог; calibrationStep() пропускает точку если |Δval2| < thr.
    void setVal2Threshold(float thr){ _val2Threshold = thr; }

    // Устанавливает минимальное число шагов для успешного finishCalibration().
    // Вход: n — минимальное число принятых calibrationStep(); автоматически корректирует _cmaWarmup
    //       так, чтобы хотя бы один шаг попал в CMA-оценку после прогрева.
    void setMinSamples(uint32_t n) {
        _minSamples = n;
        if (_cmaWarmup >= _minSamples && _minSamples > 0) {
            uint32_t w = _minSamples - 1;
            _cmaWarmup = (w > 255u) ? uint8_t(255) : uint8_t(w);
        }
    }

    // Устанавливает количество шагов прогрева CMA.
    // Вход: n — первые n шагов обучают модели, но не вносятся в CMA-оценку (theta≈0 до сходимости).
    //           Автоматически увеличивает _minSamples до n+1 при нарушении инварианта.
    void setCmaWarmup(uint8_t n) {
        _cmaWarmup = n;
        if (_minSamples <= (uint32_t)_cmaWarmup)
            _minSamples = (uint32_t)_cmaWarmup + 1;
    }

    // Устанавливает штрафы за сложность при выборе лучшей модели в finishCalibration().
    // Вход: pq — множитель CMA-ошибки квадратичной модели (>1.0 штрафует за усложнение);
    //       pc — множитель CMA-ошибки кубической модели.
    void setComplexityPenalties(double pq, double pc) {
        _penaltyQuad  = float(pq);
        _penaltyCubic = float(pc);
    }

    // Задаёт предпочтительные начальные параметры нормировки val2 чтобы избежать крупных перепроецирований.
    // Вход: zero — ожидаемый центр диапазона val2; scale — ожидаемая полуширина (0 → умолчание 50).
    void setNormParams(float zero, float scale) {
        _preferredNormZero  = zero;
        _preferredNormScale = (scale > 1e-4f) ? scale : 50.0f;
    }

    // Настраивает адаптивное забывание (только при EnableDynamic=true).
    // Вход: lambdaBase — минимальный λ при быстром изменении val2 (например, 0.9);
    //       slopeThresh — порог |Δval2|, при котором начинается снижение λ;
    //       slopeGain — обратная ширина переходной зоны (25 → 0.04 ед.; 0.5 → 2 ед. val2).
    void setInflationParams(float lambdaBase, float slopeThresh, float slopeGain) {
        _lambdaBase  = lambdaBase;
        _slopeThresh = slopeThresh;
        _slopeGain   = slopeGain;
    }

    // Устанавливает нижнюю границу диагонали P для всех трёх моделей.
    // Вход: pf — минимальное значение диагонального элемента P;
    //            предотвращает полное «замерзание» усиления Калмана после сходимости.
    void setPfloor(float pf) {
        _pfloor = pf;
        T t = T(pf);
        linearModel.setPfloor(t);
        quadraticModel.setPfloor(t);
        cubicModel.setPfloor(t);
    }

    // Устанавливает нижнюю границу P в процентах от масштаба неопределённости (0..100).
    // Вход: pct — процент; конвертируется: pf = (pct/100)*100000 (_p0Scale=100000).
    // Пример: setPfloorPercent(0.01) → floor 0.01% → усиление Калмана ≈ 10 после сходимости.
    void setPfloorPercent(float pct) {
        setPfloor((pct / 100.0f) * 100000.0f);
    }

    // Возвращает текущий pfloor в процентах от масштаба неопределённости.
    float getPfloorPercent() const {
        return (_pfloor / 100000.0f) * 100.0f;
    }

    // Умножает диагональ P всех моделей на factor (ограничено p0Scale=100000).
    // Вход: factor — множитель; используется для принудительного открытия неопределённости
    //               когда модель устарела. Внедиагональные элементы не изменяются.
    void boostP(float factor) {
        linearModel.scaleP(factor, 100000.0f);
        quadraticModel.scaleP(factor, 100000.0f);
        cubicModel.scaleP(factor, 100000.0f);
    }

    // Настраивает автоматический буст P при calibrationStep().
    // Если остаток ведущей модели (до update) превышает threshold, P умножается на boostFactor
    // перед шагом RLS — усиление Калмана растёт и модель адаптируется быстрее.
    // После срабатывания буст блокируется на cooldownSteps шагов: за это время модель
    // переобучается с повышенным P, после чего, если остаток всё ещё высок, буст повторяется.
    // Вход: threshold     — порог остатка в единицах rawVal1; 0 = отключено.
    //       boostFactor   — множитель P при превышении порога.
    //       cooldownSteps — число шагов блокировки после каждого буста (по умолчанию 5).
    void setDriftBoost(float threshold, float boostFactor, uint8_t cooldownSteps = 5) {
        _driftThreshold     = threshold;
        _driftBoostFactor   = boostFactor;
        _boostCooldownSteps = cooldownSteps;
    }

    // Устанавливает коэффициенты сглаживания EMA-фильтров (только при EnableDynamic && NumEma>0).
    // Вход: alphas — массив alpha_i строго ∈ (0,1); count — число элементов.
    //       Значения вне интервала (0,1) игнорируются. Ненастроенные каналы получают умолчания в startCalibration().
    void setEmaAlphas(const float* alphas, uint8_t count) {
        if constexpr (EnableDynamic && NumEma > 0) {
            uint8_t n = (count < NumEma) ? count : NumEma;
            for (uint8_t i = 0; i < n; i++)
                if (alphas[i] > 0.0f && alphas[i] < 1.0f)
                    _emaAlphas[i] = alphas[i];
        }
    }

    // ──────────────────────────── хранилище ─────────────────────────────────

    // Инициализирует хранилище и загружает сохранённые данные калибровки.
    // LittleFS: читает /calib.dat; при отсутствии валидных данных записывает начальную структуру.
    // EEPROM: читает из адреса 0; вызывает _loadData() для валидации.
    void begin(T RefVal1) {
        referenceVal1 = RefVal1;
        #if CALIB_STORAGE == CALIB_STORAGE_LITTLEFS
                FDstat_t stat = _fileData.read();
                if (stat == FD_READ) {
                    _loadData();
                } else {
                    _fileData.write();
                }
        #elif CALIB_STORAGE == CALIB_STORAGE_EEPROM
                EEPROM.begin(sizeof(ModelEEData));
                EEPROM.get(0, _calibData);
                _loadData();
        #endif
    }

    bool isBegined() {
        return this->_isBegined;
    }

    // ──────────────────────────── калибровка ────────────────────────────────

    // Сбрасывает состояние и начинает новую сессию калибровки.
    // Очищает theta, P, EMA, CMA-статистику, устанавливает нормировку из _preferredNorm*.
    // Ненастроенным EMA-каналам присваиваются умолчания: alpha_i = 1 - 1/(10*2^i).
    bool startCalibration() {
        isCalibrating     = true;
        samplesCount      = 0;
        max_val2          = -FLT_MAX;
        min_val2          =  FLT_MAX;
        delta             = 0.0f;
        linearErrorCMA    = 0.0f;
        quadraticErrorCMA = 0.0f;
        cubicErrorCMA     = 0.0f;
        _lastVal2         = NAN;
        _normZero         = _preferredNormZero;
        _normScale        = (_preferredNormScale > 1e-4f) ? _preferredNormScale : 0.0f;
        _emaInitialized   = false;
        _isLoaded         = false;
        _boostCooldown    = 0;
        _resetModels();

        // Если setEmaAlphas() не вызывался (alphas==0), заполнить разумными умолчаниями:
        // alpha_i = 1 - 1/(10*2^i) → 0.9, 0.95, 0.975, …
        if constexpr (EnableDynamic && NumEma > 0) {
            for (uint8_t i = 0; i < NumEma; i++)
                if (_emaAlphas[i] <= 0.0f || _emaAlphas[i] >= 1.0f)
                    _emaAlphas[i] = 1.0f - 1.0f / float(10u << i);
        }

        // Инвариант: минимум один шаг должен попасть в CMA после прогрева
        if (_minSamples > 0 && _cmaWarmup >= _minSamples) {
            uint32_t w = _minSamples - 1;
            _cmaWarmup = (w > 255u) ? uint8_t(255) : uint8_t(w);
        }

        return true;
    }

    // Добавляет одну точку в калибровочную выборку и обновляет модели.
    // Вход: val1 — «сырое» показание сенсора; val2 — вторичная переменная (например, температура).
    // Пропускает точку если: !isCalibrating, NaN/Inf, или |Δval2| < _val2Threshold.
    // Последовательность: адаптация λ → расширение нормировки → обновление EMA →
    //                     сборка признаков → предсказание (leave-one-out) → авто-буст → update → CMA.
    void calibrationStep(float val1, float val2) {
        if (!isCalibrating) return;
        if (isnan(val1) || isinf(val1) || isnan(val2) || isinf(val2)) return;
        if (!isnan(_lastVal2) && fabsf(val2 - _lastVal2) < _val2Threshold) return;

        _applyAdaptiveLambda(val2);
        _tryReproject(val2);

        float t = normVal2(val2);
        _updateEmas(t);

        if (val2 > max_val2) max_val2 = val2;
        if (val2 < min_val2) min_val2 = val2;
        delta = max_val2 - min_val2;

        float xL[LN], xQ[QN], xC[CN];
        _buildFeatures(t, xL, xQ, xC);

        T error = T(val1) - referenceVal1;

        // Ошибки предсказания ДО update (leave-one-out оценка для выбора модели)
        float eLin  = fabsf(float(error) - float(linearModel.predict(xL)));
        float eQuad = fabsf(float(error) - float(quadraticModel.predict(xQ)));
        float eCub  = fabsf(float(error) - float(cubicModel.predict(xC)));

        // Авто-буст: если остаток ведущей модели превышает порог — раздуваем P перед update,
        // чтобы усиление Калмана выросло и текущая точка поглотилась агрессивнее.
        // Кулдаун предотвращает рэтчет P к потолку при нескольких подряд превышениях:
        // модель обновляется N шагов с повышенным P, затем снова проверяется остаток.
        if (_driftThreshold > 0.0f && samplesCount > _cmaWarmup) {
            if (_boostCooldown > 0) {
                _boostCooldown--;
            } else {
                uint8_t m = _liveModelType();
                float leadErr = (m == 0) ? eLin : (m == 1) ? eQuad : eCub;
                if (leadErr > _driftThreshold) {
                    boostP(_driftBoostFactor);
                    _boostCooldown = _boostCooldownSteps;
                }
            }
        }

        linearModel.update(xL, error);
        quadraticModel.update(xQ, error);
        cubicModel.update(xC, error);

        samplesCount++;
        // Первые _cmaWarmup шагов пропускаются в CMA: theta≈0 → все модели предсказывают одинаково плохо.
        // Обучение (update выше) идёт с первого шага.
        if (samplesCount > _cmaWarmup) {
            float n = float(samplesCount - _cmaWarmup);
            linearErrorCMA    += (eLin  - linearErrorCMA)    / n;
            quadraticErrorCMA += (eQuad - quadraticErrorCMA) / n;
            cubicErrorCMA     += (eCub  - cubicErrorCMA)     / n;
        }

        _lastVal2 = val2;
    }

    // Завершает калибровку: выбирает лучшую модель, перепроецирует в финальную нормировку, сохраняет во flash/EEPROM.
    // Выход: true — калибровка успешна; false — недостаточно шагов (<_minSamples) или размах val2 (<_minDelta).
    // После успеха compensate() использует bestModel в нормировке centred по [min_val2, max_val2].
    bool finishCalibration() {
        if (!isCalibrating) return false;
        isCalibrating = false;

        if (samplesCount < _minSamples) return false;
        if (delta < _minDelta)          return false;

        _calcBestModel();
        _savedUncertainty = getUncertainty(); // снимаем живое P до обновления нормировки

        _calibData.calibrated   = true;
        _calibData.modelType    = bestModel;
        // Сохранение полного вектора theta; обрезка до 6 (размер params[6])
        uint8_t nSave = (bestModel == 0) ? uint8_t(LN) : (bestModel == 1) ? uint8_t(QN) : uint8_t(CN);
        if (nSave > 6) nSave = 6;
        _calibData.numParams    = nSave;
        _calibData.minCalibVal2 = min_val2;
        _calibData.maxCalibVal2 = max_val2;

        // Перепроецирование theta из тренировочной нормировки в финальную (центрированную по min/max).
        // Должно произойти ДО обновления _normZero/_normScale, чтобы знать старый масштаб.
        float fin_zero  = (min_val2 + max_val2) * 0.5f;
        float fin_scale = (max_val2 - min_val2) * 0.5f;
        if (fin_scale < 1e-4f) fin_scale = 1e-4f;

        if (_normScale > 1e-4f) {
            float alpha_fin = fin_scale / _normScale;
            float beta_fin  = (fin_zero - _normZero) / _normScale;

            float bufL[LN]; const float* sL = linearModel.getTheta();
            for (int i = 0; i < LN; i++) bufL[i] = sL[i];
            _reprojectPoly(bufL, 1, LN, alpha_fin, beta_fin);
            linearModel.setTheta(bufL);

            float bufQ[QN]; const float* sQ = quadraticModel.getTheta();
            for (int i = 0; i < QN; i++) bufQ[i] = sQ[i];
            _reprojectPoly(bufQ, 2, QN, alpha_fin, beta_fin);
            quadraticModel.setTheta(bufQ);

            float bufC[CN]; const float* sC = cubicModel.getTheta();
            for (int i = 0; i < CN; i++) bufC[i] = sC[i];
            _reprojectPoly(bufC, 3, CN, alpha_fin, beta_fin);
            cubicModel.setTheta(bufC);
        }

        _normZero  = fin_zero;
        _normScale = fin_scale;

        const float* src = (bestModel == 0) ? linearModel.getTheta()
                         : (bestModel == 1) ? quadraticModel.getTheta()
                                            : cubicModel.getTheta();
        for (uint8_t i = 0; i < 6; i++)
            _calibData.params[i] = (i < nSave) ? src[i] : 0.0f;
        _calibData.savedUncertainty = _savedUncertainty;

#if CALIB_STORAGE == CALIB_STORAGE_LITTLEFS
        _fileData.updateNow();
#elif CALIB_STORAGE == CALIB_STORAGE_EEPROM
        EEPROM.put(0, _calibData);
        EEPROM.commit();
#endif
        return true;
    }

    // ──────────────────────────── компенсация ───────────────────────────────

    // Вычисляет компенсированное значение rawVal1 и обновляет EMA-состояние (для работы в реальном времени).
    // Вход: rawVal1 — «сырое» показание сенсора; val2 — текущая вторичная переменная.
    // Выход: rawVal1 - ошибка_модели; возвращает rawVal1 без изменений при NaN/Inf или отсутствии калибровки.
    // Во время калибровки использует CMA-лучшую живую модель; после finishCalibration() — bestModel.
    float compensate(float rawVal1, float val2) {
        if (isnan(val2) || isinf(val2)) return rawVal1;

        if (isCalibrating) {
            if (samplesCount == 0) return rawVal1;
            float t = normVal2(val2);
            _updateEmas(t);
            float xL[LN], xQ[QN], xC[CN];
            _buildFeatures(t, xL, xQ, xC);
            uint8_t m = _liveModelType();
            T pred = (m == 0) ? linearModel.predict(xL)
                   : (m == 1) ? quadraticModel.predict(xQ)
                              : cubicModel.predict(xC);
            return float(T(rawVal1) - pred);
        }

        if (!_calibData.calibrated) return rawVal1;

        float v2 = _clampVal2(val2);
        float t = normVal2(v2);
        _updateEmas(t);
        float xL[LN], xQ[QN], xC[CN];
        _buildFeatures(t, xL, xQ, xC);
        T pred = (bestModel == 0) ? linearModel.predict(xL)
               : (bestModel == 1) ? quadraticModel.predict(xQ)
                                  : cubicModel.predict(xC);

        return float(T(rawVal1) - pred);
    }

    // Вычисляет компенсированное значение без изменения EMA-состояния (только чтение).
    // Вход: rawVal1 — «сырое» показание сенсора; val2 — значение вторичной переменной.
    // Выход: rawVal1 - ошибка_модели; аналогично compensate(), но не меняет внутреннее состояние.
    // Используется для зондирования модели в гипотетической точке val2 без побочных эффектов.
    float getCompensation(float rawVal1, float val2) const {
        if (isnan(val2) || isinf(val2)) return rawVal1;

        if (isCalibrating) {
            if (samplesCount == 0) return rawVal1;
            float t = normVal2(val2);
            float xL[LN], xQ[QN], xC[CN];
            _buildFeatures(t, xL, xQ, xC);
            uint8_t m = _liveModelType();
            T pred = (m == 0) ? linearModel.predict(xL)
                   : (m == 1) ? quadraticModel.predict(xQ)
                              : cubicModel.predict(xC);
            return float(T(rawVal1) - pred);
        }

        if (!_calibData.calibrated) return rawVal1;

        float v2 = _clampVal2(val2);
        float t = normVal2(v2);
        float xL[LN], xQ[QN], xC[CN];
        _buildFeatures(t, xL, xQ, xC);
        T pred = (bestModel == 0) ? linearModel.predict(xL)
               : (bestModel == 1) ? quadraticModel.predict(xQ)
                                  : cubicModel.predict(xC);
        return float(T(rawVal1) - pred);
    }

    // ──────────────────────────── сброс ─────────────────────────────────────

    // Полностью сбрасывает калибровку: очищает _calibData, theta, P, всё состояние.
    // Записывает сброшенную структуру во flash/EEPROM. После вызова isModelLoaded() = false.
    void resetCalibration() {
        _calibData        = ModelEEData{};
        isCalibrating     = false;
        bestModel         = 0;
        samplesCount      = 0;
        max_val2          = -FLT_MAX;
        min_val2          =  FLT_MAX;
        delta             = 0.0f;
        linearErrorCMA    = quadraticErrorCMA = cubicErrorCMA = 0.0f;
        _normZero         = 0.0f;
        _normScale        = 0.0f;
        _lastVal2         = NAN;
        _emaInitialized   = false;
        _isLoaded         = false;
        _savedUncertainty = 100.0f;
        _resetModels();

#if CALIB_STORAGE == CALIB_STORAGE_LITTLEFS
        _fileData.updateNow();
#elif CALIB_STORAGE == CALIB_STORAGE_EEPROM
        EEPROM.put(0, _calibData);
        EEPROM.commit();
#endif
    }

    // ──────────────────────────── запросы ────────────────────────────────────

    // Возвращает true если загружена валидная калибровка (после begin() или loadFromData()).
    bool    isModelLoaded()     const { return _calibData.calibrated; }

    // Возвращает true если выполняется сессия калибровки (после startCalibration(), до finish/reset).
    bool    isCalibratingMode() const { return isCalibrating; }

    // Возвращает индекс выбранной модели: 0=линейная, 1=квадратичная, 2=кубическая.
    uint8_t getBestModel()      const { return bestModel; }

    // Возвращает текущую глобальную неопределённость (0..100%).
    // При _isLoaded — сохранённое значение момента finishCalibration().
    // Иначе — trace(P) лучшей живой модели, нормированный к _p0Scale.
    float getUncertainty() const {
        if (_isLoaded) return _savedUncertainty;
        uint8_t m = _liveModelType();
        if (m == 0) return linearModel.getUncertainty();
        if (m == 1) return quadraticModel.getUncertainty();
        return           cubicModel.getUncertainty();
    }

    // Возвращает точечную неопределённость модели в произвольной точке val2.
    // Вход: val2 — значение вторичной переменной.
    // Выход: (x^T P x)/(p0Scale*||x||^2)*100% для bestModel; 100% если нет калибровки или нормировки.
    float getUncertaintyAt(float val2) const {
        if (_normScale <= 0.0f || !_calibData.calibrated) return 100.0f;
        float t = normVal2(val2);
        float xL[LN], xQ[QN], xC[CN];
        _buildFeatures(t, xL, xQ, xC);
        if (bestModel == 0) return linearModel.getUncertaintyAt(xL);
        if (bestModel == 1) return quadraticModel.getUncertaintyAt(xQ);
        return                     cubicModel.getUncertaintyAt(xC);
    }

    // ── Диагностика калибровки ───────────────────────────────────────────────

    // Возвращает количество принятых шагов calibrationStep() в текущей сессии.
    uint32_t getNumSamples()       const { return samplesCount; }

    // Возвращает размах val2 (max - min) за текущую сессию калибровки.
    float    getCalibrationDelta() const { return delta; }

    // Возвращает диапазон val2, охваченный при калибровке.
    // Вход: outMin, outMax — выходные параметры (по ссылке).
    // При наличии сохранённой калибровки — из _calibData; иначе из текущей сессии.
    void getCalibrationRange(float& outMin, float& outMax) const {
        outMin = (_calibData.calibrated) ? _calibData.minCalibVal2 : min_val2;
        outMax = (_calibData.calibrated) ? _calibData.maxCalibVal2 : max_val2;
    }

    // Возвращает true если val2 находится в пределах диапазона калибровки.
    // Вход: val2 — проверяемое значение. Возвращает false если нет валидной калибровки.
    bool isInCalibrationRange(float val2) const {
        if (!_calibData.calibrated) return false;
        return (val2 >= _calibData.minCalibVal2 && val2 <= _calibData.maxCalibVal2);
    }

    // Возвращает накопленные CMA-ошибки трёх моделей для диагностики выбора модели.
    // Выход: lin, quad, cub — средние абсолютные ошибки по принятым шагам после прогрева.
    void getCmaErrors(float& lin, float& quad, float& cub) const {
        lin  = linearErrorCMA;
        quad = quadraticErrorCMA;
        cub  = cubicErrorCMA;
    }

    // Принудительно переключает активную модель после finishCalibration().
    // Вход: modelType — 0/1/2. При успехе сохраняет выбор во flash/EEPROM.
    // Выход: false при некорректном modelType, во время калибровки, без данных,
    //        или при попытке переключить на другую модель после loadFromData() (_isLoaded).
    bool forceModel(uint8_t modelType) {
        if (modelType > 2)           return false;
        if (isCalibrating)           return false;
        if (!_calibData.calibrated)  return false;
        if (_isLoaded && modelType != _calibData.modelType) return false;

        const float* src = (modelType == 0) ? linearModel.getTheta()
                         : (modelType == 1) ? quadraticModel.getTheta()
                                            : cubicModel.getTheta();
        bestModel = modelType;
        _calibData.modelType = modelType;
        uint8_t nSave = (modelType == 0) ? uint8_t(LN) : (modelType == 1) ? uint8_t(QN) : uint8_t(CN);
        if (nSave > 6) nSave = 6;
        _calibData.numParams = nSave;
        for (uint8_t i = 0; i < 6; i++)
            _calibData.params[i] = (i < nSave) ? src[i] : 0.0f;
        _calibData.savedUncertainty = getUncertainty();
        _savedUncertainty = _calibData.savedUncertainty;

#if CALIB_STORAGE == CALIB_STORAGE_LITTLEFS
        _fileData.updateNow();
#elif CALIB_STORAGE == CALIB_STORAGE_EEPROM
        EEPROM.put(0, _calibData);
        EEPROM.commit();
#endif
        return true;
    }

    // Возвращает ссылку на текущую структуру данных калибровки (только чтение).
    const ModelEEData& getSavedData() const { return _calibData; }

    // Загружает калибровочные данные из внешней структуры (например, полученной по сети).
    // Вход: d — структура ModelEEData; проходит те же валидационные проверки что и _loadData().
    // Выход: true — данные корректны и загружены; false — данные не прошли валидацию.
    bool loadFromData(const ModelEEData& d) {
        if (d.version != 2)                        return false;
        if (!d.calibrated)                         return false;
        if (d.numParams < 2 || d.numParams > 6)   return false;
        uint8_t expectedParams;
        if      (d.modelType == 0) expectedParams = uint8_t(LN < 7 ? LN : 6);
        else if (d.modelType == 1) expectedParams = uint8_t(QN < 7 ? QN : 6);
        else if (d.modelType == 2) expectedParams = uint8_t(CN < 7 ? CN : 6);
        else return false;
        if (d.numParams != expectedParams)         return false;
        for (uint8_t i = 0; i < d.numParams; i++)
            if (isnan(d.params[i]) || isinf(d.params[i])) return false;
        _calibData = d;
        return _loadData();
    }

    // ──────────────────────────── отладочный вывод ───────────────────────────

    // Выводит параметры сохранённой модели в _debugOut (тип, коэффициенты, нормировку, диапазон val2).
    // Не выполняет никаких действий если _debugOut==nullptr или нет данных калибровки.
    void printSavedModelData() {
        if (!_debugOut) return;
        if (!_calibData.calibrated) {
            _debugOut->println(F("Calibration: no data saved"));
            return;
        }
        static const char* names[] = {"Linear", "Quadratic", "Cubic"};
        _debugOut->print(F("Model: "));
        if (_calibData.modelType < 3) _debugOut->println(names[_calibData.modelType]);
        else                          _debugOut->println(_calibData.modelType);

        _debugOut->print(F("Params["));
        _debugOut->print(_calibData.numParams);
        _debugOut->print(F("]: "));
        for (uint8_t i = 0; i < _calibData.numParams; i++) {
            _debugOut->print(_calibData.params[i], 5);
            _debugOut->print(' ');
        }
        _debugOut->println();

        _debugOut->print(F("Norm: zero="));
        _debugOut->print(_normZero, 3);
        _debugOut->print(F(" scale="));
        _debugOut->println(_normScale, 3);

        _debugOut->print(F("Val2 range: ["));
        _debugOut->print(_calibData.minCalibVal2, 2);
        _debugOut->print(F(", "));
        _debugOut->print(_calibData.maxCalibVal2, 2);
        _debugOut->println(']');
    }

#if CALIB_STORAGE != CALIB_STORAGE_MANUAL
    // Возвращает денормализованное строковое представление полинома в физических координатах.
    // Вход: urlEncoded — если true, '+' заменяется на '%2B' для безопасной передачи в URL.
    // Выход: строка вида "0.0123t+1.456" (линейная), "...t^2..." (квадратичная), "...t'^3..." (кубическая).
    // EMA-коэффициенты не отображаются — только чистые полиномиальные члены.
    // Возвращает "No data" (или "No%20data") если нет валидной калибровки.
    String getPolynomialString(bool urlEncoded = false) const {
        if (!_calibData.calibrated)
            return String(urlEncoded ? "No%20data" : "No data");

        const float* p  = _calibData.params;
        float Z = _normZero;
        float S = (_normScale > 1e-4f) ? _normScale : 1.0f;
        uint8_t type = _calibData.modelType;
        // Смещение всегда хранится на последнем сохранённом индексе params
        float bias = p[_calibData.numParams - 1];

        char buf[100] = {};
        if (type == 0) {
            // y = (p[0]/S)*t + (bias - p[0]*Z/S)
            float A = p[0] / S;
            float B = bias - p[0]*Z / S;
            snprintf(buf, sizeof(buf), "%.4ft%c%.4f",
                     A, (B >= 0.0f ? '+' : '-'), fabsf(B));
        } else if (type == 1) {
            // y = (p[0]/S^2)*t^2 + (p[1]/S - 2*p[0]*Z/S^2)*t + (bias - p[1]*Z/S + p[0]*Z^2/S^2)
            float A =  p[0] / (S*S);
            float B =  p[1]/S - 2.0f*p[0]*Z/(S*S);
            float C =  bias - p[1]*Z/S + p[0]*Z*Z/(S*S);
            snprintf(buf, sizeof(buf), "%.4ft^2%c%.4ft%c%.4f",
                     A,
                     (B >= 0.0f ? '+' : '-'), fabsf(B),
                     (C >= 0.0f ? '+' : '-'), fabsf(C));
        } else {
            // Кубическая: нормализованная форма (t' = (t-Z)/S); смещение на последнем индексе
            snprintf(buf, sizeof(buf), "%.3ft'^3%c%.3ft'^2%c%.3ft'%c%.3f",
                     p[0],
                     (p[1]>=0.0f?'+':'-'), fabsf(p[1]),
                     (p[2]>=0.0f?'+':'-'), fabsf(p[2]),
                     (bias >=0.0f?'+':'-'), fabsf(bias));
        }

        String s(buf);
        if (urlEncoded) s.replace("+", "%2B");
        return s;
    }
#endif
};
