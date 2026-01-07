#pragma once

enum class SystemState {                    // состояния конечного автомата
    CALIBRATION,
    SLEEP,
    WAKEUP,
    MEASURE,
    CONNECT_GSM,
    DATA_SEND,
    ERROR_HANDLING,
    TARE_PROCESS
};

enum class LedModes {                       // режимы световод индикации
    OK,
    ERROR,
    WAIT,
    TARE,
    NONE
};

enum class ScalesState {                    // состояния весовой схемы (тензодатчики + HX711)

};