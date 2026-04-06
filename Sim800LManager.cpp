#include "Sim800LManager.h"

Sim800LManager::Sim800LManager(Config cfg) : _cfg(cfg), _isPppActive(false) {
    pinMode(_cfg.pwr_pin, OUTPUT);
    pinMode(_cfg.rst_pin, OUTPUT);
    digitalWrite(_cfg.pwr_pin, LOW);        // выключено по умолчанию
    digitalWrite(_cfg.rst_pin, HIGH);
}

// --- Методы запуска задач ---
void Sim800LManager::startInitHardware(uint8_t max_retries) {
    _maxRetries = max_retries;
    _attempts = 0;
    _hardwareRestarted = false;
    _currentJob = JobType::INIT;
    _currentStep = Step::INIT_START;
}

void Sim800LManager::startRequest(const char* host, const char* path, const String& payload, String* responsePtr, uint32_t network_timeout_ms) {
    _host = host;
    _path = path;
    _payload = payload;
    _responsePtr = responsePtr;
    _networkTimeout = network_timeout_ms;
    _currentJob = JobType::REQUEST;
    _currentStep = Step::REQ_PPP_BEGIN;
}

void Sim800LManager::startPowerOff() {
    _currentJob = JobType::POWER_OFF;
    _currentStep = Step::OFF_START;
}

// --- Главный TICK (Конечный автомат) ---

ModemStatus Sim800LManager::tick() {
    if (_currentJob == JobType::NONE) {
        return ModemStatus::IDLE;
    }

    switch (_currentJob) {
        
        // ==========================================
        // ЭТАП 1: ИНИЦИАЛИЗАЦИЯ
        // ==========================================
        case JobType::INIT:
            switch (_currentStep) {
                case Step::INIT_START:
                    digitalWrite(_cfg.pwr_pin, HIGH);
                    _timer = millis();
                    _currentStep = Step::INIT_PWR_DELAY;
                    break;

                case Step::INIT_PWR_DELAY:
                    if (millis() - _timer >= 1000) {
                        if (_attempts > 0) {
                            digitalWrite(_cfg.rst_pin, LOW);
                            _timer = millis();
                            _currentStep = Step::INIT_RST_LOW;
                        } else {
                            Serial1.begin(115200, SERIAL_8N1, _cfg.tx_pin, _cfg.rx_pin);
                            _subAttempts = 0;
                            _currentStep = Step::INIT_AT_SEND;
                        }
                    }
                    break;

                case Step::INIT_RST_LOW:
                    if (millis() - _timer >= 300) {
                        digitalWrite(_cfg.rst_pin, HIGH);
                        _hardwareRestarted = true;
                        _timer = millis();
                        _currentStep = Step::INIT_RST_HIGH;
                    }
                    break;

                case Step::INIT_RST_HIGH:
                    if (millis() - _timer >= 4000) {
                        Serial1.begin(115200, SERIAL_8N1, _cfg.tx_pin, _cfg.rx_pin);
                        _subAttempts = 0;
                        _currentStep = Step::INIT_AT_SEND;
                    }
                    break;

                // 1. Проверка UART
                case Step::INIT_AT_SEND:
                    clearUART();
                    Serial1.println("AT");
                    _uartBuffer = "";
                    _timer = millis();
                    _currentStep = Step::INIT_AT_WAIT;
                    break;

                case Step::INIT_AT_WAIT:
                    while (Serial1.available() && _uartBuffer.length() < 127) _uartBuffer += (char)Serial1.read();
                    if (_uartBuffer.indexOf("OK") != -1) {
                        _timer = millis();
                        _currentStep = Step::INIT_SIM_DELAY;
                    } else if (millis() - _timer > 1000) {
                        _subAttempts++;
                        if (_subAttempts < 5) {
                            _timer = millis();
                            _currentStep = Step::INIT_AT_DELAY;
                        } else {
                            Serial1.end();
                            _attempts++;
                            if (_attempts >= _maxRetries) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);
                            _currentStep = Step::INIT_START; // Рестарт
                        }
                    }
                    break;

                case Step::INIT_AT_DELAY:
                    if (millis() - _timer >= 500) _currentStep = Step::INIT_AT_SEND;
                    break;

                // 2. Проверка SIM
                case Step::INIT_SIM_DELAY:
                    if (millis() - _timer >= 2000) {
                        clearUART();
                        Serial1.println("AT+CPIN?");
                        _uartBuffer = "";
                        _timer = millis();
                        _currentStep = Step::INIT_SIM_WAIT;
                    }
                    break;

                case Step::INIT_SIM_WAIT:
                    while (Serial1.available()) _uartBuffer += (char)Serial1.read();
                    if (_uartBuffer.indexOf("READY") != -1) {
                        _subAttempts = 0;
                        _currentStep = Step::INIT_CSQ_SEND;
                    } else if (millis() - _timer > 2000) {
                        Serial1.end();
                        return finishJob(ModemStatus::ERR_NO_SIM); // Физическая проблема
                    }
                    break;

                // 3. Проверка сигнала (CSQ)
                case Step::INIT_CSQ_SEND:
                    clearUART();
                    Serial1.println("AT+CSQ");
                    _uartBuffer = "";
                    _timer = millis();
                    _currentStep = Step::INIT_CSQ_WAIT;
                    break;

                case Step::INIT_CSQ_WAIT:
                    while (Serial1.available()) _uartBuffer += (char)Serial1.read();
                    if (_uartBuffer.indexOf("OK") != -1 || _uartBuffer.indexOf("ERROR") != -1) {
                        int commaIndex = _uartBuffer.indexOf(',');
                        bool signal_ok = false;
                        if (commaIndex != -1) {
                            int rssi = _uartBuffer.substring(_uartBuffer.indexOf(':') + 2, commaIndex).toInt();
                            if (rssi >= 7 && rssi != 99) signal_ok = true;
                        }

                        if (signal_ok) {
                            Serial1.end(); // Подготовка к PPP
                            return finishJob(_hardwareRestarted ? ModemStatus::SUCCESS_WITH_RESTARTS : ModemStatus::SUCCESS);
                        } else {
                            _subAttempts++;
                            if (_subAttempts < 15) {
                                _timer = millis();
                                _currentStep = Step::INIT_CSQ_DELAY;
                            } else {
                                Serial1.end();
                                _attempts++;
                                if (_attempts >= _maxRetries) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);
                                _currentStep = Step::INIT_START; // Плохой сигнал - аппаратный рестарт
                            }
                        }
                    } else if (millis() - _timer > 1000) {
                        // Таймаут ответа на команду CSQ
                        _subAttempts++;
                        if (_subAttempts < 15) {
                            _timer = millis();
                            _currentStep = Step::INIT_CSQ_DELAY;
                        } else {
                            Serial1.end();
                            _attempts++;
                            if (_attempts >= _maxRetries) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);
                            _currentStep = Step::INIT_START;
                        }
                    }
                    break;

                case Step::INIT_CSQ_DELAY:
                    if (millis() - _timer >= 1000) _currentStep = Step::INIT_CSQ_SEND;
                    break;
            }
            break;

        // ==========================================
        // ЭТАП 2: СЕТЬ И ЗАПРОС
        // ==========================================
        case JobType::REQUEST:
            switch (_currentStep) {
                case Step::REQ_PPP_BEGIN:
                    PPP.setApn(_cfg.apn);
                    PPP.setPins(_cfg.tx_pin, _cfg.rx_pin);
                    if (!PPP.begin(PPP_MODEM_SIM800)) {
                        return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
                    }
                    _isPppActive = true;
                    _timer = millis();
                    _currentStep = Step::REQ_PPP_ATTACH_WAIT;
                    break;

                case Step::REQ_PPP_ATTACH_WAIT:
                    if (PPP.attached()) {
                        _timer = millis();
                        _currentStep = Step::REQ_PPP_CONN_WAIT;
                    } else if (millis() - _timer > _networkTimeout) {
                        return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
                    }
                    break;

                case Step::REQ_PPP_CONN_WAIT:
                    // Неблокирующая замена функции waitStatusBits
                    if (PPP.connected()) {
                        _currentStep = Step::REQ_HTTP_CONNECT;
                    } else if (millis() - _timer > 20000) {
                        return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
                    }
                    break;

                case Step::REQ_HTTP_CONNECT: {
                    _client.setInsecure();
                    _client.setTimeout(15);
                    
                    // Установка SSL соединения (может занять 1-3 секунды)
                    // Это единственный полублокирующий вызов, так устроена библиотека,
                    // но это время не приведет к срабатыванию WDT ESP32.
                    if (!_client.connect(_host, 443)) {
                        return finishJob(ModemStatus::ERR_SERVER_CONNECT);
                    }

                    String request = String("POST ") + _path + " HTTP/1.1\r\n" +
                                     "Host: " + _host + "\r\n" +
                                     "Content-Type: application/json\r\n" +
                                     "Content-Length: " + _payload.length() + "\r\n" +
                                     "Connection: close\r\n\r\n" + 
                                     _payload;
                    _client.print(request);
                    
                    _timer = millis();
                    _uartBuffer = "";
                    _currentStep = Step::REQ_HTTP_WAIT_RES;
                    break;
                }

                case Step::REQ_HTTP_WAIT_RES:
                    // Читаем посимвольно всё, что прилетело на данный момент, и отдаем управление
                    while (_client.available()) {
                        _uartBuffer += (char)_client.read();
                    }
                    
                    // Если соединение закрыто сервером и читать больше нечего
                    if (!_client.connected() && _client.available() == 0) {
                        _client.stop();
                        if (_uartBuffer.indexOf("200 OK") != -1) {
                            if (_responsePtr) *_responsePtr = _uartBuffer;
                            return finishJob(ModemStatus::SUCCESS);
                        } else {
                            return finishJob(ModemStatus::ERR_HTTP_TIMEOUT);
                        }
                    } else if (millis() - _timer > 15000) { // Глобальный таймаут HTTP
                        _client.stop();
                        return finishJob(ModemStatus::ERR_HTTP_TIMEOUT);
                    }
                    break;
            }
            break;

        // ==========================================
        // ЭТАП 3: ОТКЛЮЧЕНИЕ
        // ==========================================
        case JobType::POWER_OFF:
            switch (_currentStep) {
                case Step::OFF_START:
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

                case Step::OFF_DELAY:
                    // Даем LwIP сокетам 1 секунду на закрытие без блокировки
                    if (millis() - _timer >= 1000) {
                        Serial1.end();
                        digitalWrite(_cfg.pwr_pin, LOW);
                        return finishJob(ModemStatus::SUCCESS);
                    }
                    break;
            }
            break;
    }

    return ModemStatus::BUSY;
}

// --- Приватные утилиты ---

ModemStatus Sim800LManager::finishJob(ModemStatus status) {
    _currentJob = JobType::NONE; // Освобождаем автомат для следующих задач
    return status;
}

void Sim800LManager::clearUART() {
    while (Serial1.available()) Serial1.read();
}