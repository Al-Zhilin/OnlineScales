#pragma once

// Вынесем enum, чтобы он был доступен везде
enum class LedModes {
    NONE,
    OK,
    ERROR,
    WAIT
};

class AsyncLed {
private:
    struct Pins {
        byte red, green, blue;
    } _pins;

    uint32_t _timer = 0;
    bool _state = false;
    uint8_t _cycle_count = 0;
    LedModes _currentMode = LedModes::NONE;

    void turnOffAll() {
        digitalWrite(_pins.red, LOW);
        digitalWrite(_pins.green, LOW);
        digitalWrite(_pins.blue, LOW);
        _state = false;
    }

public:
    AsyncLed(byte red, byte green, byte blue) {
        _pins.red = red;
        _pins.green = green;
        _pins.blue = blue;
    }

    void begin() {
        pinMode(_pins.red, OUTPUT);
        pinMode(_pins.green, OUTPUT);
        pinMode(_pins.blue, OUTPUT);
        turnOffAll();
    }

    void setMode(LedModes mode) {
        if (_currentMode == mode) return;

        _currentMode = mode;
        _cycle_count = 0;
        turnOffAll();
        
        if (_currentMode != LedModes::NONE) {
            _state = true;
            _timer = millis();
            
            switch (_currentMode) {
                case LedModes::OK:    digitalWrite(_pins.green, HIGH); break;
                case LedModes::ERROR: digitalWrite(_pins.red, HIGH);   break;
                case LedModes::WAIT:  digitalWrite(_pins.blue, HIGH);  break;
            }
        }
    }

    void tick() {
        if (_currentMode == LedModes::NONE) return;

        uint32_t interval = 1000;
        uint8_t targetBlinks = 0;

        switch (_currentMode) {
            case LedModes::OK:    interval = 350; targetBlinks = 2; break;
            case LedModes::ERROR: interval = 200; targetBlinks = 3; break;
            case LedModes::WAIT:  interval = 800; targetBlinks = 0; break;
        }

        if (millis() - _timer >= interval) {
            _timer = millis();
            _state = !_state;

            switch (_currentMode) {
                case LedModes::OK:    digitalWrite(_pins.green, _state); break;
                case LedModes::ERROR: digitalWrite(_pins.red, _state);   break;
                case LedModes::WAIT:  digitalWrite(_pins.blue, _state);  break;
            }

            if (!_state && targetBlinks > 0) {
                _cycle_count++;
                if (_cycle_count >= targetBlinks) {
                    setMode(LedModes::NONE);
                }
            }
        }
    }
};