#pragma once

enum class SystemState : uint8_t {                   // состояния конечного автомата (главного цикла loop())
    CALIBRATION,                            // в процессе калибровки
    SLEEP_SENSORS,                          // усыпляем temp и weight датчики
    SLEEP_MODEM,                            // заканчиваем работу и усыпляем SIM800L
    WAKEUP,                                 // пробуждаем датчики
    MEASURE,                                // измеряем показания
    CONNECT_GSM,                            // начинаем работу модема, устанавливаем GSM связь
    DATA_SEND,                              // отправляем данные
    ERROR_HANDLING,                         // обрабатываем ошибки, в это состояние будем проваливаться при наличии внештатных ситуаций на любом этапе работы
    TARE_PROCESS,                           // тарирование
    END_CALIBRATION,                        // закончить калибровку
    START_CALIBRATION,                      // начать калибровку
    PREV_STATE,                             // специальное значение, указывает функции changeState о возврате на предыдущее состояние
};

enum class LedModes : uint8_t {                      // режимы светодиодной индикации
    OK,
    ERROR,
    WAIT,
    NONE
};

enum class ScalesState : uint8_t {                   // состояния весовой схемы (тензодатчики + HX711)
    SUCCESS,
    ERROR,
    BUSY
};

enum class TempState : uint8_t {
    SUCCESS,
    BUSY
};

enum class CalibrationState : uint8_t {              // статусы процесса калибровки
    SUCCESS,
};