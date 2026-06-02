#pragma once
#include <Arduino.h>

template <int PIN>
class AsyncLed {
private:
    uint8_t _switch_pin;
    
    LedModes _queue[5];
    uint8_t _qHead = 0;
    uint8_t _qTail = 0;

    LedModes _currentMode = LedModes::NONE;
    uint32_t _timer = 0;
    bool _isOn = false;
    uint8_t _blinkCount = 0;
    uint8_t _targetBlinks = 0;
    uint32_t _interval = 0;
    uint32_t _color = 0;

    bool isEnabled() { return !digitalRead(_switch_pin); }
    void _clearQueue() { _qHead = 0; _qTail = 0; }

    // --- МАГИЯ ЗДЕСЬ: Нативная функция без сторонних библиотек ---
    void _setPixel(uint8_t r, uint8_t g, uint8_t b) {
        // Если вдруг твоя лента искажает цвета (например, красный горит зеленым),
        // просто поменяй местами переменные ниже, например: (PIN, g, r, b)
        #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
            rgbLedWrite(PIN, r, g, b); // Для нового ядра ESP32 (3.x)
        #else
            neopixelWrite(PIN, r, g, b); // Для классического ядра ESP32 (2.x)
        #endif
    }
    // -------------------------------------------------------------

    void _popAndStart() {
        if (_qHead == _qTail) {
            _currentMode = LedModes::NONE;
            _setPixel(0, 0, 0);
            return;
        }
        _startMode(_queue[_qHead]);
        _qHead = (_qHead + 1) % 5;
    }

    void _startMode(LedModes mode) {
        _currentMode = mode;
        _blinkCount = 0; 
        _isOn = false;
        _targetBlinks = 0;

        switch (mode) {
            case LedModes::BREATH_INIT: _color = 0x0000FF; break;
            case LedModes::BREATH_NET:  _color = 0xFF9600; break;
            case LedModes::BREATH_HTTP: _color = 0xFF00FF; break;
            case LedModes::REP_SENS_OK:  _color = 0x00FF00; _targetBlinks = 1; _interval = 500; break;
            case LedModes::REP_SENS_ERR: _color = 0xFF0000; _targetBlinks = 1; _interval = 500; break;
            case LedModes::REP_MOD_OK:       _color = 0x00FF00; _targetBlinks = 2; _interval = 150; break;
            case LedModes::REP_MOD_ERR_HW:   _color = 0xFF0000; _targetBlinks = 2; _interval = 150; break;
            case LedModes::REP_MOD_ERR_NET:  _color = 0xFF0000; _targetBlinks = 3; _interval = 150; break;
            case LedModes::REP_MOD_ERR_SERV: _color = 0xFF0000; _targetBlinks = 4; _interval = 150; break;
            case LedModes::ACT_TARE_OK:     _color = 0x00FFFF; _targetBlinks = 3; _interval = 100; break;
            case LedModes::ACT_TARE_ERR:    _color = 0xFF6400; _targetBlinks = 3; _interval = 100; break;
            case LedModes::ACT_CALIB_START: _color = 0xFFFFFF; _targetBlinks = 2; _interval = 150; break;
            case LedModes::ACT_CALIB_OK:    _color = 0xFFFFFF; _targetBlinks = 1; _interval = 800; break;
            case LedModes::ACT_CALIB_ERR:   _color = 0xFF0000; _targetBlinks = 4; _interval = 100; break;
            case LedModes::ACT_PRINT:       _color = 0x0000FF; _targetBlinks = 1; _interval = 800; break;
            default: break;
        }

        if (_targetBlinks > 0) { 
            _isOn = true;
            uint8_t r = (uint8_t)(_color >> 16);
            uint8_t g = (uint8_t)(_color >> 8);
            uint8_t b = (uint8_t)_color;
            _setPixel(r, g, b);
        }
        _timer = millis();
    }

public:
    AsyncLed(uint8_t switch_pin) : _switch_pin(switch_pin) {}

    void begin() {
        pinMode(_switch_pin, INPUT_PULLUP);
        _setPixel(0, 0, 0);
    }

    void powerOff() {
        _setPixel(0, 0, 0);
        delay(5);
        // Теперь мы можем смело обрывать пин! Нативный драйвер уже завершил работу.
        // Ток не утекает, плата холодная, ядро не ругается.
        pinMode(PIN, OUTPUT);
        digitalWrite(PIN, LOW);
    }

    void setMode(LedModes mode) {
        if (!isEnabled()) return;
        if (_currentMode == mode) return;
        _clearQueue(); 
        _startMode(mode);
    }

    void pushReport(LedModes mode) {
        if (!isEnabled() || mode == LedModes::NONE) return;
        
        if (_currentMode == LedModes::BREATH_INIT || _currentMode == LedModes::BREATH_NET || _currentMode == LedModes::BREATH_HTTP) {
            _currentMode = LedModes::NONE;
        }

        uint8_t nextTail = (_qTail + 1) % 5;
        if (nextTail != _qHead) {
            _queue[_qTail] = mode;
            _qTail = nextTail;
        }

        if (_currentMode == LedModes::NONE) _popAndStart();
    }

    bool tick() {
        if (!isEnabled()) {
            _clearQueue();
            if (_currentMode != LedModes::NONE) {
                _currentMode = LedModes::NONE;
                _setPixel(0, 0, 0);
            }
            return true;
        }

        if (_currentMode == LedModes::NONE) return true;

        if (_targetBlinks == 0) {
            static uint32_t lastFrame = 0;
            if (millis() - lastFrame >= 33) {  
                lastFrame = millis();
                float wave = (sin(millis() / 318.3f) + 1.0f) / 2.0f; 
                
                uint8_t r = (uint8_t)(_color >> 16);
                uint8_t g = (uint8_t)(_color >> 8);
                uint8_t b = (uint8_t)_color;
                
                float scale = 0.05f + (wave * 0.35f); 
                _setPixel((uint8_t)(r * scale), (uint8_t)(g * scale), (uint8_t)(b * scale));
            }
            return false; 
        }

        uint32_t currentInterval = _interval;
        if (!_isOn && _blinkCount >= _targetBlinks) currentInterval = 800;

        if (millis() - _timer >= currentInterval) {
            _timer = millis();
            
            if (!_isOn) {
                if (_blinkCount >= _targetBlinks) {
                    _popAndStart();                                                 
                    return false;
                }
                _isOn = true;
                uint8_t r = (uint8_t)(_color >> 16);
                uint8_t g = (uint8_t)(_color >> 8);
                uint8_t b = (uint8_t)_color;
                _setPixel(r, g, b);
            } else {
                _isOn = false;
                _setPixel(0, 0, 0);
                _blinkCount++;
            }
        }
        return false;
    }
};