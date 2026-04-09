#include "Sim800LManager.h"

Sim800LManager::Sim800LManager(Config cfg) : _cfg(cfg), _isPppActive(false) {
    pinMode(_cfg.pwr_pin, OUTPUT);
    pinMode(_cfg.rst_pin, OUTPUT);
    digitalWrite(_cfg.pwr_pin, LOW);                // по умолчанию модем выкл
    digitalWrite(_cfg.rst_pin, HIGH);
}

// ==========================================
// УНИФИЦИРОВАННЫЕ МЕТОДЫ AT-КОМАНД
// ==========================================

void Sim800LManager::sendAT(const char* cmd, const char* expected, uint32_t timeout) {
    clearUART();
    Serial1.println(cmd);
    _expectedAtResponse = expected;
    _currentAtTimeout = timeout;
    _uartBuffer = "";
    _timer = millis();
}

bool Sim800LManager::waitAT(String& outResponse, bool& isTimeout) {
    // 1. Вычитываем UART, но жестко ограничиваем длину (защита от бесконечного мусора)
    while (Serial1.available() && _uartBuffer.length() < ModemCfg::MAX_UART_LEN) {
        _uartBuffer += (char)Serial1.read();
    }

    if (_uartBuffer.indexOf(_expectedAtResponse) != -1 || _uartBuffer.indexOf("ERROR") != -1) {
        outResponse = _uartBuffer;
        isTimeout = false;
        return true; // Ответ получен
    }

    // 3. Проверяем таймаут
    if (millis() - _timer >= _currentAtTimeout) {
        outResponse = _uartBuffer;
        isTimeout = true;
        return true; // Вышли по таймауту
    }

    return false; // Всё еще ждем
}

void Sim800LManager::clearUART() {
    while (Serial1.available()) Serial1.read();
}

ModemStatus Sim800LManager::finishJob(ModemStatus status) {                        // синтаксический сахар: прокидывает через себя возвращаемый статус функции, скрывая под капотом деинициализацию _currentJob
    _currentJob = JobType::NONE;
    return status;
}

// ==========================================
// ЭТАП 1: ИНИЦИАЛИЗАЦИЯ
// ==========================================
ModemStatus Sim800LManager::processInit() {
    if (_currentJob != JobType::INIT) {                                             // Если задача только поступила, настраиваем первый шаг
        _currentJob = JobType::INIT;
        _currentStep = Step::INIT_START;
        _attempts = 0;
        _hardwareRestarted = false;
    }

    String res;
    bool timeout;

    switch (_currentStep) {
        case Step::INIT_START:                                                      // включение модема
            digitalWrite(_cfg.pwr_pin, HIGH);                   // включаем модем
            _timer = millis();                                  // засекаем время
            _currentStep = Step::INIT_PWR_DELAY;                // перешагиваем
            break;

        case Step::INIT_PWR_DELAY:                                                  // настройка Serial или отправка на перезагрузку
            if (millis() - _timer >= 1000) {
                if (_attempts > 0) {                                                // если пришли сюда после неудачной попытки
                    digitalWrite(_cfg.rst_pin, LOW);                                    // выключаем
                    _timer = millis();
                    _currentStep = Step::INIT_RST_LOW;                                  //отправляемся ждать
                } else {                                                            // если первая попытка
                    Serial1.begin(115200, SERIAL_8N1, _cfg.tx_pin, _cfg.rx_pin);
                    _subAttempts = 0;
                    _currentStep = Step::INIT_AT_SEND;
                }
            }
            break;

        case Step::INIT_RST_LOW:                                                    // пробуждение после перезагрузки
            if (millis() - _timer >= 300) {
                digitalWrite(_cfg.rst_pin, HIGH);
                _hardwareRestarted = true;
                _timer = millis();
                _currentStep = Step::INIT_RST_HIGH;
            }
            break;

        case Step::INIT_RST_HIGH:                                                   // настройка после перезагрузки
            if (millis() - _timer >= 4000) {
                Serial1.begin(115200, SERIAL_8N1, _cfg.tx_pin, _cfg.rx_pin);
                _subAttempts = 0;
                _currentStep = Step::INIT_AT_SEND;
            }
            break;

        // --- Блок проверки UART ---
        case Step::INIT_AT_SEND:                                                    // тест на простую команду "AT"
            sendAT("AT", "OK", ModemCfg::AT_TIMEOUT_MS);
            _currentStep = Step::INIT_AT_WAIT;
            break;

        case Step::INIT_AT_WAIT:                                                    // ждем ответа на отправленную команду
            if (waitAT(res, timeout)) {
                if (!timeout && res.indexOf("OK") != -1) {
                    _timer = millis();
                    _currentStep = Step::INIT_SIM_DELAY;
                } else {
                    _subAttempts++;
                    if (_subAttempts < 5) {
                        _timer = millis();
                        _currentStep = Step::INIT_AT_DELAY;
                    } else {
                        Serial1.end();
                        _attempts++;
                        if (_attempts >= ModemCfg::MAX_RETRIES) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);
                        _currentStep = Step::INIT_START;
                    }
                }
            }
            break;

        case Step::INIT_AT_DELAY:                                                   // выжидаем таймаут перед очередной попыткой отправки команды
            if (millis() - _timer >= 500) _currentStep = Step::INIT_AT_SEND;
            break;

        // --- Блок проверки SIM ---
        case Step::INIT_SIM_DELAY:                                                  // проверяем готовность SIM карты
            if (millis() - _timer >= 2000) {
                sendAT("AT+CPIN?", "READY", ModemCfg::SIM_TIMEOUT_MS);
                _currentStep = Step::INIT_SIM_WAIT;
            }
            break;

        case Step::INIT_SIM_WAIT:                                                   // ждем ответ и проверяем ответ
            if (waitAT(res, timeout)) {
                if (!timeout && res.indexOf("READY") != -1) {
                    _subAttempts = 0;
                    _currentStep = Step::INIT_CSQ_SEND;
                } else {
                    Serial1.end();
                    return finishJob(ModemStatus::ERR_NO_SIM);
                }
            }
            break;

        // --- Блок проверки Сигнала (CSQ) ---
        case Step::INIT_CSQ_SEND:                                                   // проверка уровня сигнала через CSQ
            sendAT("AT+CSQ", "OK", ModemCfg::AT_TIMEOUT_MS);
            _currentStep = Step::INIT_CSQ_WAIT;
            break;

        case Step::INIT_CSQ_WAIT:                                                   // ожидаем получение ответа
            if (waitAT(res, timeout)) {
                bool signal_ok = false;
                if (!timeout) {
                    // Используем полученный ответ (парсинг уровня сигнала)
                    int commaIndex = res.indexOf(',');
                    if (commaIndex != -1) {
                        int rssi = res.substring(res.indexOf(':') + 2, commaIndex).toInt();
                        if (rssi >= 7 && rssi != 99) signal_ok = true;
                    }
                }

                if (signal_ok) {
                    Serial1.end();
                    return finishJob(_hardwareRestarted ? ModemStatus::SUCCESS_WITH_RESTARTS : ModemStatus::SUCCESS);
                } else {
                    _subAttempts++;
                    if (_subAttempts < 15) {
                        _timer = millis();
                        _currentStep = Step::INIT_CSQ_DELAY;
                    } else {
                        Serial1.end();
                        _attempts++;
                        if (_attempts >= ModemCfg::MAX_RETRIES) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);
                        _currentStep = Step::INIT_START;
                    }
                }
            }
            break;

        case Step::INIT_CSQ_DELAY:                                                  // ждем перед очередной попыткой отправки команды
            if (millis() - _timer >= 1000) _currentStep = Step::INIT_CSQ_SEND;
            break;
    }
    return ModemStatus::BUSY;                                                       // возвращаем занятость, пока не выполнили блок операций полностью
}

// ==========================================
// ЭТАП 2: ЗАПРОС
// ==========================================
ModemStatus Sim800LManager::processRequest(const String& payload, String& response) {
    if (_currentJob != JobType::REQUEST) {                                          // инициализируем блок
        _currentJob = JobType::REQUEST;
        _currentStep = Step::REQ_PPP_BEGIN;
    }

    switch (_currentStep) {
        case Step::REQ_PPP_BEGIN:                                                   // открываем PPP и проверяем успешность открытия
            PPP.setApn(ModemCfg::APN);
            PPP.setPins(_cfg.tx_pin, _cfg.rx_pin);
            if (!PPP.begin(PPP_MODEM_SIM800)) {
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            _isPppActive = true;
            _timer = millis();
            _currentStep = Step::REQ_PPP_ATTACH_WAIT;
            break;

        case Step::REQ_PPP_ATTACH_WAIT:                                             // выполняем attached() и проверяем успешность
            if (PPP.attached()) {
                _timer = millis();
                _currentStep = Step::REQ_PPP_CONN_WAIT;
            } else if (millis() - _timer > ModemCfg::NETWORK_TIMEOUT_MS) {
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            break;

        case Step::REQ_PPP_CONN_WAIT:                                               // ожидаем connected() и проверяем успешность
            if (PPP.connected()) {
                _currentStep = Step::REQ_HTTP_CONNECT;
            } else if (millis() - _timer > 20000) {
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            break;

        case Step::REQ_HTTP_CONNECT: {                                              // соединяемся с сервером и отправляем POST
            _client.setInsecure();
            _client.setTimeout(15);
            
            if (!_client.connect(ModemCfg::SERVER_HOST, 443)) {
                return finishJob(ModemStatus::ERR_SERVER_CONNECT);
            }

            String request = String("POST ") + ModemCfg::SERVER_PATH + " HTTP/1.1\r\n" +
                             "Host: " + ModemCfg::SERVER_HOST + "\r\n" +
                             "Content-Type: application/json\r\n" +
                             "Content-Length: " + payload.length() + "\r\n" +
                             "Connection: close\r\n\r\n" + 
                             payload;
            _client.print(request);
            
            _timer = millis();
            _uartBuffer = ""; // Используем буфер для сборки ответа HTTP
            _currentStep = Step::REQ_HTTP_WAIT_RES;
            break;
        }

        case Step::REQ_HTTP_WAIT_RES:                                               // ожидаем ответ от сервера и собираем его в буфер
            // 1. Сначала читаем всё, что пришло (с ограничением длины)
            while (_client.available() && _uartBuffer.length() < ModemCfg::MAX_HTTP_LEN) {
                _uartBuffer += (char)_client.read();
            }
            
            // 2. Проверяем условие завершения: сервер закрыл соединение и буфер пуст
            if (!_client.connected() && _client.available() == 0) {
                _client.stop();
                if (_uartBuffer.indexOf("200 OK") != -1) {
                    response = _uartBuffer; // Записываем ответ только при успехе
                    return finishJob(ModemStatus::SUCCESS);
                } else {
                    return finishJob(ModemStatus::ERR_HTTP_TIMEOUT);
                }
            } 
            // 3. Только потом проверяем таймаут (защита от редкого вызова tick() в loop())
            else if (millis() - _timer >= ModemCfg::HTTP_TIMEOUT_MS) { 
                _client.stop();
                return finishJob(ModemStatus::ERR_HTTP_TIMEOUT);
            }
            break;
    }
    return ModemStatus::BUSY;                                                       // возвращаем занятость, пока не дойдем до конца команд
}

// ==========================================
// ЭТАП 3: ВЫКЛЮЧЕНИЕ
// ==========================================
ModemStatus Sim800LManager::processPowerOff() {
    if (_currentJob != JobType::POWER_OFF) {                                        // инициализируем блок
        _currentJob = JobType::POWER_OFF;
        _currentStep = Step::OFF_START;
    }

    switch (_currentStep) {
        case Step::OFF_START:                                                       // завершаем PPP, Serial и выключаем питание модема
            if (_isPppActive) {
                PPP.end();
                _isPppActive = false;
                _timer = millis();
                _currentStep = Step::OFF_DELAY;
            } else {
                Serial1.end();
                digitalWrite(_cfg.pwr_pin, LOW);
                return finishJob(ModemStatus::SUCCESS);
            }
            break;

        case Step::OFF_DELAY:                                                       // если PPP был активен - ждем 100мс без какой либо логики или проверки, после истечения просто выключим модем
            if (millis() - _timer >= 1000) {
                Serial1.end();
                digitalWrite(_cfg.pwr_pin, LOW);
                return finishJob(ModemStatus::SUCCESS);
            }
            break;
    }
    return ModemStatus::BUSY;                                                       // аналогично реализуем внешний мониторинг статуса
}