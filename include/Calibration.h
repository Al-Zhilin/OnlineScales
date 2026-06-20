#pragma once

// ──────────────────────────────────────────────────────────────────────────────
//  Storage backend selection  (set via build_flags or before including)
// ──────────────────────────────────────────────────────────────────────────────
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

// ──────────────────────────────────────────────────────────────────────────────
//  Persistent calibration data  (stored in flash / EEPROM)
// ──────────────────────────────────────────────────────────────────────────────
struct ModelEEData {
    uint8_t  version          = 1;       // format version; increment when layout changes
    uint8_t  modelType        = 0;       // 0=linear  1=quadratic  2=cubic
    uint8_t  numParams        = 0;       // modelType + 2  (2/3/4)
    bool     calibrated       = false;
    float    params[4]        = {};      // theta in normalised coordinates
    float    minCalibVal2     = 0.0f;
    float    maxCalibVal2     = 0.0f;
    float    savedUncertainty = 100.0f; // P-uncertainty at finishCalibration(); restored after load
};

// ──────────────────────────────────────────────────────────────────────────────
//  RLSModel<N, T>  —  single recursive least-squares model with N features
// ──────────────────────────────────────────────────────────────────────────────
template <uint16_t N, typename T = double>
class RLSModel {
    static_assert(std::is_floating_point<T>::value, "RLSModel requires floating-point T");

private:
    float theta[N]   = {};
    T     P[N][N]    = {};
    T     _lambda    = T(1);
    T     _pfloor    = T(0);
    float _p0Scale   = 100000.0f;

public:
    RLSModel() { reset(); }

    void reset() {
        for (uint16_t i = 0; i < N; i++) theta[i] = 0.0f;
        resetP(100000.0f);
    }

    void resetP(float val) {
        _p0Scale = val;
        for (uint16_t i = 0; i < N; i++)
            for (uint16_t j = 0; j < N; j++)
                P[i][j] = (i == j) ? T(val) : T(0);
    }

    void setLambda(T lam) { _lambda = (lam > T(0)) ? lam : T(1); }
    void setPfloor(T pf)  { _pfloor = pf; }

    void setTheta(const float* src) {
        if (!src) return;
        for (uint16_t i = 0; i < N; i++) theta[i] = src[i];
    }

    const float* getTheta() const { return theta; }

    // Returns 100% before any data; shrinks as P converges toward 0.
    float getUncertainty() const {
        T trace = T(0);
        for (uint16_t i = 0; i < N; i++) trace += P[i][i];
        return float(trace / (T(N) * T(_p0Scale))) * 100.0f;
    }

    // 2-argument update: feature vector x[N], target y
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

        // Symmetrize to prevent float drift accumulation
        for (uint16_t i = 0; i < N; i++)
            for (uint16_t j = i + 1; j < N; j++)
                P[i][j] = P[j][i] = (P[i][j] + P[j][i]) * T(0.5);

        if (float(trace) > 1e6f || isnan(float(trace))) {
            // Overflow: reset P and theta (both numerically suspect)
            for (uint16_t ii = 0; ii < N; ii++) {
                theta[ii] = 0.0f;
                for (uint16_t jj = 0; jj < N; jj++)
                    P[ii][jj] = (ii == jj) ? T(100.0f) : T(0);
            }
        }
    }

    // Higher-precision predict — returns T so full double accuracy is preserved in compensate().
    T predict(const float* x) const {
        T res = T(0);
        for (uint16_t i = 0; i < N; i++) res += T(theta[i]) * T(x[i]);
        return res;
    }

    // Point-wise predictive variance: (x^T P x) / (p0Scale * ||x||^2) * 100 %.
    // At calibration start = 100 %; shrinks to 0 % as the model converges at x.
    float getUncertaintyAt(const float* x) const {
        T xPx = T(0), norm2 = T(0);
        for (uint16_t i = 0; i < N; i++) {
            T Pxi = T(0);
            for (uint16_t j = 0; j < N; j++) Pxi += P[i][j] * T(x[j]);
            xPx  += T(x[i]) * Pxi;
            norm2 += T(x[i]) * T(x[i]);
        }
        if (norm2 < T(1e-20)) return 0.0f;
        return float(xPx / (T(_p0Scale) * norm2)) * 100.0f;
    }

    // Apply coordinate change P = M^T * P * M  (x_old = M * x_new)
    // Symmetrizes after the transform to prevent float drift from accumulating.
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

// ──────────────────────────────────────────────────────────────────────────────
//  AdaptiveRLS<T, EnableDynamic, NumEma>
//    T             – floating-point precision  (double recommended)
//    EnableDynamic – when true: adaptive lambda + EMA feature extension
//    NumEma        – number of EMA filters appended as extra features
// ──────────────────────────────────────────────────────────────────────────────
template <typename T = double, bool EnableDynamic = false, uint8_t NumEma = 0>
class AdaptiveRLS {
private:
    static constexpr uint8_t _NE   = EnableDynamic ? NumEma : 0;
    static constexpr uint8_t LN    = uint8_t(2 + _NE);
    static constexpr uint8_t QN    = uint8_t(3 + _NE);
    static constexpr uint8_t CN    = uint8_t(4 + _NE);
    static constexpr uint8_t _EABUF = (_NE > 0) ? _NE : 1;

    // ── Models ──────────────────────────────────────────────────────────────
    RLSModel<LN, T> linearModel;
    RLSModel<QN, T> quadraticModel;
    RLSModel<CN, T> cubicModel;

    // ── Calibration state ───────────────────────────────────────────────────
    T           referenceVal1;
    ModelEEData _calibData;
    bool        isCalibrating    = false;
    uint8_t     bestModel        = 0;
    uint32_t    samplesCount     = 0;
    float       max_val2         = -FLT_MAX;
    float       min_val2         =  FLT_MAX;
    float       delta            = 0.0f;
    float       linearErrorCMA   = 0.0f;
    float       quadraticErrorCMA = 0.0f;
    float       cubicErrorCMA    = 0.0f;

    // ── Normalisation ────────────────────────────────────────────────────────
    float _normZero           = 0.0f;
    float _normScale          = 0.0f;   // 0 = uninitialized sentinel
    float _preferredNormZero  = 0.0f;
    float _preferredNormScale = 0.0f;
    float _lastVal2           = NAN;    // NAN = no previous value

    // ── EMA state ───────────────────────────────────────────────────────────
    float _emaState [_EABUF] = {};
    float _emaAlphas[_EABUF] = {};
    bool  _emaInitialized    = false;

    // ── Parameters ──────────────────────────────────────────────────────────
    uint32_t _minSamples    = 10;
    float    _minDelta      = 5.0f;
    float    _val2Threshold = 0.5f;
    float    _penaltyQuad   = 1.10f;
    float    _penaltyCubic  = 1.20f;
    float    _lambdaBase    = 1.0f;
    float    _slopeThresh   = 8.0f;
    float    _slopeGain     = 25.0f;
    float    _pfloor        = 0.0f;
    uint8_t  _cmaWarmup     = 3;    // first N samples train model but skip CMA scoring

    // ── Post-calibration state ───────────────────────────────────────────────
    float _savedUncertainty = 100.0f; // stored at finishCalibration(); returned when _isLoaded
    bool  _isLoaded         = false;  // set by _loadData(); cleared by startCalibration()

    // ── Debug ────────────────────────────────────────────────────────────────
    Print* _debugOut = nullptr;

#if CALIB_STORAGE == CALIB_STORAGE_LITTLEFS
    FileData _fileData{&LittleFS, "/calib.dat", 'Z', &_calibData, sizeof(ModelEEData)};
#endif

    // ─────────────────────────── private helpers ─────────────────────────────

    float normVal2(float v) const {
        if (_normScale <= 0.0f) return 0.0f;
        return (v - _normZero) / _normScale;
    }

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

    // Build upper-triangular M[Sz][Sz] for polynomial coord change x_old = M * x_new.
    // Row p: t_old^(deg-p) = sum_j C(deg-p,j)*alpha^j*beta^(deg-p-j)*t_new^j
    // Constant term (j=0) → bias at position Sz-1; t^j (j>0) → position deg-j.
    // EMA rows (deg..Sz-2): same linear transform as t → M[p][p]=alpha, M[p][Sz-1]=beta.
    // Bias row (Sz-1): identity.
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
        // EMA features undergo the same linear transform as t: ema_old = alpha*ema_new + beta
        for (int p = deg; p < Sz - 1; p++) {
            M[p][p]     = alpha;
            M[p][Sz-1]  = beta;
        }
        M[Sz-1][Sz-1] = 1.0f;  // bias row: identity
    }

    // Reproject polynomial theta under t_old = alpha*t_new + beta.
    // `degree` = polynomial degree (1/2/3); `total` = total theta length.
    // EMA entries (indices degree..total-2) undergo the same linear transform as t:
    //   ema_old = alpha*ema_new + beta  →  theta_ema *= alpha, bias += theta_ema*beta.
    static void _reprojectPoly(float* th, uint8_t degree, uint8_t total,
                                float alpha, float beta) {
        uint8_t bi = uint8_t(total - 1);
        // Scale EMA coefficients by alpha; accumulate their beta shift into bias.
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

    // Expand normalisation to cover val2 and reproject all model thetas.
    void _tryReproject(float val2) {
        if (_normScale <= 0.0f) {
            // Seed normalization so the full calibration range maps to roughly t_norm in [0,1]
            // for typical 100°C-wide calibrations (avoiding slow RLS convergence with tiny features)
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

        float alpha = new_scale / _normScale;                // new/old: polynomial coeffs grow as range expands
        float beta  = (new_zero - _normZero) / _normScale;  // shift in old-normalised units

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

        // Transform existing EMA states to the new normalisation scale
        if constexpr (EnableDynamic && NumEma > 0) {
            if (_emaInitialized) {
                for (uint8_t i = 0; i < NumEma; i++)
                    _emaState[i] = (_emaState[i]*_normScale + _normZero - new_zero) / new_scale;
            }
        }

        _normZero  = new_zero;
        _normScale = new_scale;
    }

    void _calcBestModel() {
        float sL = linearErrorCMA;
        float sQ = quadraticErrorCMA * _penaltyQuad;
        float sC = cubicErrorCMA     * _penaltyCubic;
        if      (sL <= sQ && sL <= sC) bestModel = 0;
        else if (sQ <= sC)             bestModel = 1;
        else                           bestModel = 2;
    }

    bool _loadData() {
        if (_calibData.version != 1)                               return false;
        if (!_calibData.calibrated)                                return false;
        if (_calibData.numParams < 2 || _calibData.numParams > 4)  return false;
        if (_calibData.numParams != _calibData.modelType + 2)      return false;
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

    void _resetModels() {
        linearModel.reset();
        quadraticModel.reset();
        cubicModel.reset();
        T pf = T(_pfloor);
        linearModel.setPfloor(pf);
        quadraticModel.setPfloor(pf);
        cubicModel.setPfloor(pf);
    }

    void _applyAdaptiveLambda(float val2) {
        if constexpr (EnableDynamic) {
            if (!isnan(_lastVal2)) {
                // Piecewise-linear: no forgetting below slopeThresh, full forgetting at
                // slopeThresh + 1/slopeGain.  slopeGain=25 → 0.04 °C wide (≈ step);
                // slopeGain=0.5 → 2 °C wide (smooth ramp).
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
    // ──────────────────────────── construction ──────────────────────────────
    explicit AdaptiveRLS(T refVal1) : referenceVal1(refVal1) {}

    // ──────────────────────────── configuration ─────────────────────────────
    void setDebugOut(Print* p)      { _debugOut = p; }
    void setMinDelta(float d)       { _minDelta = d; }
    void setVal2Threshold(float thr){ _val2Threshold = thr; }

    void setMinSamples(uint32_t n) {
        _minSamples = n;
        // Invariant: at least 1 sample must contribute to CMA after warmup.
        if (_cmaWarmup >= _minSamples && _minSamples > 0)
            _cmaWarmup = (uint8_t)(_minSamples - 1);
    }

    void setCmaWarmup(uint8_t n) {
        _cmaWarmup = n;
        // Mirror invariant: minSamples must be > cmaWarmup.
        if (_minSamples <= (uint32_t)_cmaWarmup)
            _minSamples = (uint32_t)_cmaWarmup + 1;
    }

    void setComplexityPenalties(double pq, double pc) {
        _penaltyQuad  = float(pq);
        _penaltyCubic = float(pc);
    }

    // Pre-seed normalisation; first points won't cause large reprojections.
    // scale=0 silently defaults to 50 (typical half-span for 100°C-wide calibrations).
    void setNormParams(float zero, float scale) {
        _preferredNormZero  = zero;
        _preferredNormScale = (scale > 1e-4f) ? scale : 50.0f;
    }

    // Adaptive forgetting: lambdaBase = minimum lambda (e.g. 0.9), slopeThresh = |Δval2| onset,
    // slopeGain = reciprocal transition width (e.g. 25 → 0.04 °C, 0.5 → 2 °C).
    void setInflationParams(float lambdaBase, float slopeThresh, float slopeGain) {
        _lambdaBase  = lambdaBase;
        _slopeThresh = slopeThresh;
        _slopeGain   = slopeGain;
    }

    void setPfloor(float pf) {
        _pfloor = pf;
        T t = T(pf);
        linearModel.setPfloor(t);
        quadraticModel.setPfloor(t);
        cubicModel.setPfloor(t);
    }

    // Accepts only valid EMA alphas strictly in (0, 1).
    void setEmaAlphas(const float* alphas, uint8_t count) {
        if constexpr (EnableDynamic && NumEma > 0) {
            uint8_t n = (count < NumEma) ? count : NumEma;
            for (uint8_t i = 0; i < n; i++)
                if (alphas[i] > 0.0f && alphas[i] < 1.0f)
                    _emaAlphas[i] = alphas[i];
        }
    }

    // ──────────────────────────── storage ───────────────────────────────────
    void begin() {
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

    // ──────────────────────────── calibration ───────────────────────────────
    void startCalibration() {
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
        _resetModels();

        // Guard: if setEmaAlphas() was never called (alphas still 0), fill with sensible defaults.
        // alpha_i = 1 - 1/(10*2^i) → 0.9, 0.95, 0.975, …
        if constexpr (EnableDynamic && NumEma > 0) {
            for (uint8_t i = 0; i < NumEma; i++)
                if (_emaAlphas[i] <= 0.0f || _emaAlphas[i] >= 1.0f)
                    _emaAlphas[i] = 1.0f - 1.0f / float(10u << i);
        }

        // Guard: ensure at least 1 sample contributes to CMA.
        if (_minSamples > 0 && _cmaWarmup >= _minSamples)
            _cmaWarmup = (uint8_t)(_minSamples - 1);
    }

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

        // Pre-update prediction errors (leave-one-out estimate for model selection)
        float eLin  = fabsf(float(error) - float(linearModel.predict(xL)));
        float eQuad = fabsf(float(error) - float(quadraticModel.predict(xQ)));
        float eCub  = fabsf(float(error) - float(cubicModel.predict(xC)));

        linearModel.update(xL, error);
        quadraticModel.update(xQ, error);
        cubicModel.update(xC, error);

        samplesCount++;
        // Skip first _cmaWarmup samples from model-selection scoring:
        // theta≈0 makes all models predict equally poorly before convergence.
        // Training (update above) still happens from sample 1.
        if (samplesCount > _cmaWarmup) {
            float n = float(samplesCount - _cmaWarmup);
            linearErrorCMA    += (eLin  - linearErrorCMA)    / n;
            quadraticErrorCMA += (eQuad - quadraticErrorCMA) / n;
            cubicErrorCMA     += (eCub  - cubicErrorCMA)     / n;
        }

        _lastVal2 = val2;
    }

    bool finishCalibration() {
        if (!isCalibrating) return false;
        isCalibrating = false;

        if (samplesCount < _minSamples) return false;
        if (delta < _minDelta)          return false;

        _calcBestModel();
        _savedUncertainty = getUncertainty(); // capture live P before normZero/Scale update

        _calibData.calibrated   = true;
        _calibData.modelType    = bestModel;
        _calibData.numParams    = bestModel + 2;
        _calibData.minCalibVal2 = min_val2;
        _calibData.maxCalibVal2 = max_val2;

        // Reproject all model thetas from training normalisation to final (min/max-centred) coords.
        // Must happen BEFORE updating _normZero/_normScale so we know the old scale.
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
        for (uint8_t i = 0; i < 4; i++)
            _calibData.params[i] = (i < _calibData.numParams) ? src[i] : 0.0f;
        _calibData.savedUncertainty = _savedUncertainty;

#if CALIB_STORAGE == CALIB_STORAGE_LITTLEFS
        _fileData.updateNow();
#elif CALIB_STORAGE == CALIB_STORAGE_EEPROM
        EEPROM.put(0, _calibData);
        EEPROM.commit();
#endif
        return true;
    }

    // ──────────────────────────── compensation ───────────────────────────────

    // Live compensation — also advances EMA state (correct for real-time use).
    float compensate(float rawVal1, float val2) {
        if (isnan(val2) || isinf(val2)) return rawVal1;
        if (!_calibData.calibrated)     return rawVal1;

        float t = normVal2(val2);
        _updateEmas(t);

        float xL[LN], xQ[QN], xC[CN];
        _buildFeatures(t, xL, xQ, xC);

        T pred = (bestModel == 0) ? linearModel.predict(xL)
               : (bestModel == 1) ? quadraticModel.predict(xQ)
                                  : cubicModel.predict(xC);
        return float(T(rawVal1) - pred);  // full-precision subtraction
    }

    // Read-only compensation — uses current EMA snapshot without advancing it.
    // Useful for probing the model at a hypothetical val2 without side-effects.
    float getCompensation(float rawVal1, float val2) const {
        if (isnan(val2) || isinf(val2)) return rawVal1;
        if (!_calibData.calibrated)     return rawVal1;

        float t = normVal2(val2);
        float xL[LN], xQ[QN], xC[CN];
        _buildFeatures(t, xL, xQ, xC);

        T pred = (bestModel == 0) ? linearModel.predict(xL)
               : (bestModel == 1) ? quadraticModel.predict(xQ)
                                  : cubicModel.predict(xC);
        return float(T(rawVal1) - pred);
    }

    // ──────────────────────────── reset ─────────────────────────────────────
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

    // ──────────────────────────── queries ───────────────────────────────────
    bool    isModelLoaded()     const { return _calibData.calibrated; }
    bool    isCalibratingMode() const { return isCalibrating; }
    uint8_t getBestModel()      const { return bestModel; }

    float getUncertainty() const {
        if (_isLoaded) return _savedUncertainty;
        if (isCalibrating && samplesCount > _cmaWarmup) {
            // During active calibration: show uncertainty of the currently leading model
            // (CMA determines current winner; bestModel is updated only at finishCalibration).
            float sL = linearErrorCMA;
            float sQ = quadraticErrorCMA * _penaltyQuad;
            float sC = cubicErrorCMA     * _penaltyCubic;
            if (sL <= sQ && sL <= sC) return linearModel.getUncertainty();
            if (sQ <= sC)             return quadraticModel.getUncertainty();
            return                           cubicModel.getUncertainty();
        }
        if (bestModel == 0) return linearModel.getUncertainty();
        if (bestModel == 1) return quadraticModel.getUncertainty();
        return                     cubicModel.getUncertainty();
    }

    // Point-wise predictive uncertainty at an arbitrary val2 using the current best model.
    float getUncertaintyAt(float val2) const {
        if (_normScale <= 0.0f || !_calibData.calibrated) return 100.0f;
        float t = normVal2(val2);
        float xL[LN], xQ[QN], xC[CN];
        _buildFeatures(t, xL, xQ, xC);
        if (bestModel == 0) return linearModel.getUncertaintyAt(xL);
        if (bestModel == 1) return quadraticModel.getUncertaintyAt(xQ);
        return                     cubicModel.getUncertaintyAt(xC);
    }

    // ── Calibration diagnostics ──────────────────────────────────────────────
    uint32_t getNumSamples()       const { return samplesCount; }
    float    getCalibrationDelta() const { return delta; }

    void getCalibrationRange(float& outMin, float& outMax) const {
        outMin = (_calibData.calibrated) ? _calibData.minCalibVal2 : min_val2;
        outMax = (_calibData.calibrated) ? _calibData.maxCalibVal2 : max_val2;
    }

    bool isInCalibrationRange(float val2) const {
        if (!_calibData.calibrated) return false;
        return (val2 >= _calibData.minCalibVal2 && val2 <= _calibData.maxCalibVal2);
    }

    void getCmaErrors(float& lin, float& quad, float& cub) const {
        lin  = linearErrorCMA;
        quad = quadraticErrorCMA;
        cub  = cubicErrorCMA;
    }

    // Override model selection after finishCalibration() — all three models are trained
    // and can be switched freely.  Calling after loadFromData() is only safe when forcing
    // to the same model that was loaded (other models have theta=0).
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
        _calibData.numParams = modelType + 2;
        for (uint8_t i = 0; i < 4; i++)
            _calibData.params[i] = (i < _calibData.numParams) ? src[i] : 0.0f;
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

    const ModelEEData& getSavedData() const { return _calibData; }

    bool loadFromData(const ModelEEData& d) {
        if (d.version != 1)                        return false;
        if (!d.calibrated)                         return false;
        if (d.numParams < 2 || d.numParams > 4)   return false;
        if (d.numParams != d.modelType + 2)        return false;
        for (uint8_t i = 0; i < d.numParams; i++)
            if (isnan(d.params[i]) || isinf(d.params[i])) return false;
        _calibData = d;
        return _loadData();
    }

    // ──────────────────────────── debug output ───────────────────────────────
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
    // Returns de-normalised polynomial string for display (e.g. VK messages).
    // urlEncoded=true escapes '+' as '%2B' for safe URL form encoding.
    String getPolynomialString(bool urlEncoded = false) const {
        if (!_calibData.calibrated)
            return String(urlEncoded ? "No%20data" : "No data");

        const float* p  = _calibData.params;
        float Z = _normZero;
        float S = (_normScale > 1e-4f) ? _normScale : 1.0f;
        uint8_t type = _calibData.modelType;

        char buf[100] = {};
        if (type == 0) {
            // y = (p0/S)*t + (p1 - p0*Z/S)
            float A = p[0] / S;
            float B = p[1] - p[0]*Z / S;
            snprintf(buf, sizeof(buf), "%.4ft%c%.4f",
                     A, (B >= 0.0f ? '+' : '-'), fabsf(B));
        } else if (type == 1) {
            // y = (p0/S^2)*t^2 + (p1/S - 2*p0*Z/S^2)*t + (p2 - p1*Z/S + p0*Z^2/S^2)
            float A =  p[0] / (S*S);
            float B =  p[1]/S - 2.0f*p[0]*Z/(S*S);
            float C =  p[2] - p[1]*Z/S + p[0]*Z*Z/(S*S);
            snprintf(buf, sizeof(buf), "%.4ft^2%c%.4ft%c%.4f",
                     A,
                     (B >= 0.0f ? '+' : '-'), fabsf(B),
                     (C >= 0.0f ? '+' : '-'), fabsf(C));
        } else {
            // Cubic: show normalised-space form (t' = (t-Z)/S)
            snprintf(buf, sizeof(buf), "%.3ft'^3%c%.3ft'^2%c%.3ft'%c%.3f",
                     p[0],
                     (p[1]>=0.0f?'+':'-'), fabsf(p[1]),
                     (p[2]>=0.0f?'+':'-'), fabsf(p[2]),
                     (p[3]>=0.0f?'+':'-'), fabsf(p[3]));
        }

        String s(buf);
        if (urlEncoded) s.replace("+", "%2B");
        return s;
    }
#endif
};
