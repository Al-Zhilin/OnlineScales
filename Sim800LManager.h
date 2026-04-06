#ifndef SIM800L_MANAGER_H
#define SIM800L_MANAGER_H

#include <Arduino.h>
#include <PPP.h>
#include <NetworkClientSecure.h>

// Строгие статусы для контроля потока выполнения
enum class ModemStatus {
    SUCCESS,
    SUCCESS_WITH_RESTARTS, // Успешно, но потребовались перезагрузки (для логов)
    ERR_BOOT_TIMEOUT,      // Модем не отвечает на AT-команды
    ERR_NO_SIM,            // SIM-карта не обнаружена
    ERR_NO_SIGNAL,         // Нет сети (CSQ слишком мал)
    ERR_PPP_TIMEOUT,       // Не удалось поднять GPRS-сессию
    ERR_SERVER_CONNECT,    // Не удалось подключиться к серверу по HTTPS
    ERR_HTTP_TIMEOUT       // Сервер не ответил вовремя
};

class Sim800LManager {
public:
    struct Config {
        int8_t tx_pin;
        int8_t rx_pin;                      
        int8_t rst_pin;
        int8_t pwr_pin;
        const char* apn;
    };

    Sim800LManager(Config cfg);

    // Этап 1: Включение, инициализация и базовая проверка (AT, SIM, Сигнал)
    // max_retries - сколько раз можно аппаратно перезагружать модем при сбоях
    ModemStatus initHardware(uint8_t max_retries = 3);

    // Этап 2: Поднятие стека PPP, отправка запроса и получение ответа
    ModemStatus sendRequest(const char* host, const char* path, const String& payload, String& response, uint32_t network_timeout_ms = 40000);

    // Этап 3: Корректное завершение сессии и полное обесточивание
    void powerOff();

private:
    Config _cfg;
    bool _isPppActive;

    void hardwareReset();
    String sendATCommand(const char* cmd, uint32_t timeout_ms);
    void clearUART();
};

#endif