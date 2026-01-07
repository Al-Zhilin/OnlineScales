#pragma once

class AsyncLed {
private:
    struct Pins {
      byte red, green, blue;
    }_pins;

    uint32_t _tick_timer = 0;
    bool _state = false;                        // вкл/выкл для циклов мигания
    LedModes _currentMode = LedModes::NONE;

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
        digitalWrite(_pins.red, LOW);
        digitalWrite(_pins.green, LOW);
        digitalWrite(_pins.blue, LOW);
    }

    // Установить режим мигания
    void setMode(LedModes mode) {
        _currentMode = mode;
        _tick_timer = millis();
    }

    // Эту функцию надо вызывать в loop() постоянно
    void tick() {
        if (_currentMode == LedModes::NONE || millis() - _tick_timer <= 200)  return;           // думаю тикера 5 раз в секунду хватит для светодиода
        _tick_timer = millis();
    }
};