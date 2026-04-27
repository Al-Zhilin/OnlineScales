#ifndef SIM800L_MANAGER_H
#define SIM800L_MANAGER_H

#include <NetworkClientSecure.h>
#include <PPP.h>

// --- Унифицированные настройки (constexpr) ---
namespace ModemCfg {
    constexpr uint8_t  MAX_RETRIES        = 3;      // Кол-во попыток аппаратного рестарта
    constexpr uint32_t AT_TIMEOUT_MS      = 1000;   // Базовый таймаут AT команд
    constexpr uint32_t SIM_TIMEOUT_MS     = 2000;   // Таймаут ответа SIM-карты
    constexpr uint32_t ATTACH_TIMEOUT_MS  = 40000;  // Таймаут ожидания attach() 
    constexpr uint32_t NETWORK_TIMEOUT_MS = 60000;  // Ожидание регистрации в сети
    constexpr uint32_t HTTP_TIMEOUT_MS    = 17000;  // Ожидание ответа от сервера (суммарно на получение всего ответа)
    constexpr uint16_t MAX_UART_LEN       = 256;    // Защита от бесконечного мусора из UART
    constexpr uint16_t MAX_HTTP_LEN       = 4096;   // Защита от переполнения памяти при ответе сервера
    
    // Настройки сети и сервера
    constexpr const char* APN         = "internet";
    constexpr const char* SERVER_HOST = "api.vk.com";
    constexpr const char* SERVER_PATH = "/method/messages.send";
};

enum class WaitResult {
    OK,
    ERROR,
    TIMEOUT,
    BUSY
};

struct Config {
    uint8_t pwr_pin;
    uint8_t rst_pin;
    uint8_t tx_pin;
    uint8_t rx_pin;
};

// Внутренние задачи и шаги автомата
enum class JobType { NONE, INIT, REQUEST, POWER_OFF };
enum class Step {
    INIT_START_DELAY, INIT_START, INIT_PWR_DELAY, INIT_RST_LOW, INIT_RST_HIGH,
    INIT_AT_SEND, INIT_AT_WAIT, INIT_AT_DELAY,
    INIT_SIM_DELAY, INIT_SIM_SEND, INIT_SIM_WAIT,
    INIT_CSQ_SEND, INIT_CSQ_WAIT, INIT_CSQ_DELAY,
    
    REQ_PPP_BEGIN, REQ_PPP_ATTACH_WAIT, REQ_PPP_CONN_WAIT,
    REQ_HTTP_CONNECT, REQ_HTTP_RETRY, REQ_HTTP_WAIT_RES,
    
    OFF_START, OFF_DELAY
};

class Sim800LManager {
public:
    Sim800LManager();

    void begin(Config cfg);

    // Главные методы для вызова в loop()
    // Возвращают BUSY, пока выполняются. При завершении возвращают SUCCESS или ERR_...
    ModemStatus processInit();
    ModemStatus processRequest(const String& payload, String& response);
    ModemStatus processPowerOff();

private:
    Config _cfg;
    bool _isPppActive;

    JobType _currentJob = JobType::NONE;
    Step _currentStep;
    unsigned long _timer = 0;
    
    uint8_t _attempts = 0;                              // макро попытки, счетчики ретраев модема
    uint8_t _subAttempts = 0;                           // микро попытки, т.е. счетчики повторов в конкретных шагах
    bool _hardwareRestarted = false;

    NetworkClientSecure _client;                        // выбираем HTTPS т.к. api.vk только по нему общается

    String _uartBuffer;                                 // буфер парсинга ответа модема
    const char* _expectedAtResponse;                    // ожидаемый ответ (обычно "OK" или подобное)
    uint32_t _currentAtTimeout;

    ModemStatus finishJob(ModemStatus status);
    
    void clearUART();
    void sendAT(const char* cmd, const char* expected, uint32_t timeout);
    WaitResult waitAT(String& outResponse);
};

#endif