#pragma once

enum class SystemState : uint8_t {                   // состояния конечного автомата (главного цикла loop())
    SLEEP_SENSORS,                          // усыпляем temp и weight датчики
    SLEEP_MODEM,                            // заканчиваем работу и усыпляем SIM800L
    WAKEUP_SENSORS,                         // пробуждаем датчики
    MEASURE,                                // измеряем показания
    START_MODEM,                            // начинаем работу модема, устанавливаем GSM связь
    DATA_SEND,                              // отправляем данные
    ERROR_HANDLING,                         // обрабатываем ошибки, в это состояние будем проваливаться при наличии внештатных ситуаций на любом этапе работы
    TARE_PROCESS,                           // тарирование
    PREV_STATE,                             // специальное значение, указывает функции changeState о возврате на предыдущее состояние
};

/*enum class ModificationRequest : uint8_t {      // влияние внешних событий на стандартный цикл смены состояний FSM
    NONE,
    TARE,                                   // нужно провести тарирование
    START_CALIBRATION,                      // нужно начать калибровку
    END_CALIBRATION,                        // нужно закончить калибровку
    FORCE_SEND,                             // в следующем цикле, не смотря на таймер, нужно отправить данные на сервер
};*/

struct ModificationRequests {
    bool tare = false;
    bool start_calibration = false;
    bool end_calibration = false;
    bool force_send = false;
};

enum class LedModes : uint8_t {
    NONE,
    BREATH_INIT, BREATH_NET, BREATH_HTTP,                                         // Дыхане: инициализация модема, настройка сети, https запрос
    REP_SENS_OK, REP_SENS_ERR,                                                    // Отчеты Датчиков: успех/ошибка
    REP_MOD_OK, REP_MOD_ERR_HW, REP_MOD_ERR_NET, REP_MOD_ERR_SERV,                // Отчеты Модема: успех, ошибка hardware, ошибка настройки сети, ошибка https запроса или соединения с сервером
    ACT_TARE_OK, ACT_TARE_ERR,                                                    // Action тарирование: успех/ошибка
    ACT_CALIB_START, ACT_CALIB_OK, ACT_CALIB_ERR, ACT_PRINT                       // Калибровка: начата, успешно завершена, ошибка, печать данны
};

enum class ModemStatus : uint8_t {
    IDLE, 
    BUSY, BUSY_INIT, BUSY_NET, BUSY_HTTP, 
    SUCCESS, SUCCESS_WITH_RESTARTS,
    ERR_NO_SIM, ERR_BOOT_TIMEOUT, ERR_PPP_TIMEOUT, ERR_SERVER_CONNECT, ERR_HTTP_TIMEOUT
};

enum class ScalesState : uint8_t {                   // состояния весовой схемы (тензодатчики + HX711)
    SUCCESS,
    ERROR,
    BUSY
};

enum class TempState : uint8_t {
    SUCCESS,
    ERROR,
    BUSY
};

enum class CalibrationState : uint8_t {              // статусы процесса калибровки
    SUCCESS,
};