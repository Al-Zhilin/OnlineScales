#pragma once

enum class SystemState : uint8_t {                    // состояния конечного автомата
    CALIBRATION,
    SLEEP,
    WAKEUP,
    MEASURE,
    CONNECT_GSM,
    DATA_SEND,
    ERROR_HANDLING,
    TARE_PROCESS
};

enum class LedModes : uint8_t {                       // режимы световод индикации
    OK,
    ERROR,
    WAIT,
    NONE
};

enum class ScalesState : uint8_t {                    // состояния весовой схемы (тензодатчики + HX711)
    SUCCESS,
    ERROR
};