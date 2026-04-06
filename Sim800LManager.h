#ifndef SIM800L_MANAGER_H
#define SIM800L_MANAGER_H

#include <Arduino.h>
#include <NetworkClientSecure.h>
#include <PPP.h>

// Добавлены статусы IDLE и BUSY для конечного автомата
enum class ModemStatus {
    IDLE,                               // простой
    BUSY,                               // занят выполнением
    SUCCESS,                            // успешность выполнения
    SUCCESS_WITH_RESTARTS,              // выполнено успешно, но понадобились перезагрузки
    ERR_NO_SIM,                         // ошибка с SIM картой
    ERR_BOOT_TIMEOUT,                   // ошибка boot таймаута
    ERR_PPP_TIMEOUT,                    // ошибка подъема PPPoS
    ERR_SERVER_CONNECT,                 // ошибка подключения к серверу
    ERR_HTTP_TIMEOUT                    // ошибка отправки запроса (таймаут)
};

struct Config {
    uint8_t pwr_pin;                    // пин управления питанием
    uint8_t rst_pin;                    // пин, соединенный с RST SIM800L
    uint8_t tx_pin;                     // TX pin
    uint8_t rx_pin;                     // RX pin
    const char* apn;                    // apn оператора
};

// Внутренние задачи и шаги автомата
enum class JobType { NONE, INIT, REQUEST, POWER_OFF };
enum class Step {
    // Шаги инициализации
    INIT_START, INIT_PWR_DELAY, INIT_RST_LOW, INIT_RST_HIGH,
    INIT_AT_SEND, INIT_AT_WAIT, INIT_AT_DELAY,
    INIT_SIM_DELAY, INIT_SIM_SEND, INIT_SIM_WAIT,
    INIT_CSQ_SEND, INIT_CSQ_WAIT, INIT_CSQ_DELAY,
    
    // Шаги HTTP запроса
    REQ_PPP_BEGIN, REQ_PPP_ATTACH_WAIT, REQ_PPP_CONN_WAIT,
    REQ_HTTP_CONNECT, REQ_HTTP_WAIT_RES,
    
    // Шаги выключения
    OFF_START, OFF_DELAY
};

class Sim800LManager {
public:
    Sim800LManager(Config cfg);

    // Методы "запуска" процессов (возвращают управление мгновенно)
    void startInitHardware(uint8_t max_retries);
    void startRequest(const char* host, const char* path, const String& payload, String* responsePtr, uint32_t network_timeout_ms);
    void startPowerOff();

    // Главный метод, который нужно вызывать в loop()
    ModemStatus tick();

private:
    Config _cfg;
    bool _isPppActive;

    // Переменные конечного автомата
    JobType _currentJob = JobType::NONE;
    Step _currentStep = Step::INIT_START;
    unsigned long _timer = 0;
    
    // Счетчики попыток
    uint8_t _maxRetries = 0;
    uint8_t _attempts = 0;
    uint8_t _subAttempts = 0;
    bool _hardwareRestarted = false;

    // Данные для HTTP запроса
    const char* _host;
    const char* _path;
    String _payload;
    String* _responsePtr;
    uint32_t _networkTimeout;
    NetworkClientSecure _client;

    // Буфер для асинхронного чтения UART и HTTP
    String _uartBuffer;

    ModemStatus finishJob(ModemStatus status);
    void clearUART();
};

#endif