#include "Sim800LManager.h"

Sim800LManager::Sim800LManager() : _isPppActive(false) {    }

void Sim800LManager::begin(Config cfg) {
    _cfg = cfg; // Принимаем уже заполненный конфиг из setup()
    
    pinMode(_cfg.pwr_pin, OUTPUT);
    pinMode(_cfg.rst_pin, OUTPUT);
    digitalWrite(_cfg.pwr_pin, LOW);  // по умолчанию модем выкл
    pinMode(_cfg.rst_pin, INPUT);     // RST "в воздухе", чтобы не держать модем в вечном сбросе. Боюсь, что 3.3В логики не хватит для отпускания RST, попробую подвешенное состояние
}

void Sim800LManager::sendAT(const char* cmd, const char* expected, uint32_t timeout) {
    clearUART();
    Serial1.println(cmd);

    _expectedAtResponse = expected;
    _currentAtTimeout = timeout;
    _uartBuffer = "";
    _uartBuffer.reserve(ModemCfg::MAX_UART_LEN); // Резервируем мало памяти для обычных AT
    _timer = millis();
}

bool Sim800LManager::waitAT(String& outResponse, bool& isTimeout) {
    // 1. Вычитываем UART, жестко ограничивая длину (защита от переполнения кучи)
    while (Serial1.available() && _uartBuffer.length() < ModemCfg::MAX_UART_LEN) {
        _uartBuffer += (char)Serial1.read();
    }

    // 2. Ищем ожидаемый ответ или ошибку
    if (_uartBuffer.indexOf(_expectedAtResponse) != -1 || _uartBuffer.indexOf("ERROR") != -1) {
        outResponse = _uartBuffer;
        isTimeout = false;

        return true; // Ответ получен
    }

    // 3. Проверяем таймаут
    if (millis() - _timer >= _currentAtTimeout) {
        outResponse = _uartBuffer;
        Serial.println("<<< [TIMEOUT ERROR] waitAT timeout!!");
        isTimeout = true;
        return true; // Вышли по таймауту
    }

    return false; // Всё еще ждем
}

void Sim800LManager::clearUART() {
    while (Serial1.available()) Serial1.read();
}

ModemStatus Sim800LManager::finishJob(ModemStatus status) {                    // синтаксический "сахар": прогоняет через себя возвратное значение + делает рутинную работу, чтобы не писать каждый раз одно и то же       
    _currentJob = JobType::NONE;
    return status;
}

// -------------------------------------- Инициализация простого Serial соединения, проверка SIM, уровня сигнала --------------------------------------
ModemStatus Sim800LManager::processInit() {
    if (_currentJob != JobType::INIT) {                                             
        _currentJob = JobType::INIT;
        _currentStep = Step::INIT_START;
        _attempts = 0;
        // _hardwareRestarted = false; // Раскомментируй, если используешь в логике
    }

    String res;
    bool timeout;

    switch (_currentStep) {
        case Step::INIT_START:                                                      
            LOG("Модем: Включение питания и инициализация интерфейсов...");
            
            // Воскрешаем Serial1 на случай, если он был выключен в processPowerOff()
            Serial1.begin(9600, SERIAL_8N1, _cfg.rx_pin, _cfg.tx_pin); 
            
            pinMode(_cfg.rst_pin, INPUT);      
            digitalWrite(_cfg.pwr_pin, HIGH);  
            
            _timer = millis();
            _currentStep = Step::INIT_PWR_DELAY; 
            break;

        case Step::INIT_PWR_DELAY:
            // Ждем 3.5 сек загрузки ОС внутри SIM800L
            if (millis() - _timer >= 3500) {
                LOG("Модем запустился. Запуск синхронизации UART...");
                _currentStep = Step::INIT_AT_SEND;
            }
            break;

        case Step::INIT_AT_SEND:
            // "Будим" UART модема
            for (int i = 0; i < 5; i++) {                   // спамим чтобы убедить его в принятии нашей скорости передачи
                Serial1.println("AT");
                delay(50); 
            }
            clearUART(); 
            
            // --- НОВЫЙ БЛОК: СТРОГАЯ НАСТРОЙКА ДЛЯ PPP ---
            // Жестко фиксируем скорость
            Serial1.println("AT+IPR=9600");
            delay(100);
            
            // ОТКЛЮЧАЕМ ЭХО (ATE0). Без этого драйвер PPP не поймет ответы!
            Serial1.println("ATE0");
            delay(100);
            clearUART();
            // ----------------------------------------------
            
            sendAT("AT", "OK", ModemCfg::AT_TIMEOUT_MS);
            _currentStep = Step::INIT_AT_WAIT;
            break;

        case Step::INIT_AT_WAIT:                                                    
            if (waitAT(res, timeout)) {
                if (!timeout && res.indexOf("OK") != -1) {
                    _timer = millis();
                    _currentStep = Step::INIT_SIM_DELAY;
                    LOG("Модем: Успешный ответ на AT. UART синхронизирован!");
                } else {
                    _subAttempts++;
                    if (_subAttempts < 5) {
                        _timer = millis();
                        _currentStep = Step::INIT_AT_DELAY;
                    } else {
                        LOG("Модем: КРИТИЧЕСКАЯ ОШИБКА. Нет ответа на AT. Рестарт FSM.");
                        Serial1.end();
                        digitalWrite(_cfg.pwr_pin, LOW);
                        _attempts++;
                        if (_attempts >= ModemCfg::MAX_RETRIES) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);
                        _currentStep = Step::INIT_START;
                    }
                }
            }
            break;

        case Step::INIT_AT_DELAY:                                                   
            if (millis() - _timer >= 500) _currentStep = Step::INIT_AT_SEND;
            break;

        // --- Блок проверки SIM ---
        case Step::INIT_SIM_DELAY:                                                  
            if (millis() - _timer >= 2000) {
                sendAT("AT+CPIN?", "READY", ModemCfg::SIM_TIMEOUT_MS);
                _currentStep = Step::INIT_SIM_WAIT;
            }
            break;

        case Step::INIT_SIM_WAIT:                                                   
            if (waitAT(res, timeout)) {
                if (!timeout && res.indexOf("READY") != -1) {
                    LOG("Модем: SIM-карта READY.");
                    _subAttempts = 0;
                    _currentStep = Step::INIT_CSQ_SEND;
                } else {
                    LOG("Модем: ОШИБКА SIM-КАРТЫ (Не вставлена или заблокирована).");
                    Serial1.end();
                    digitalWrite(_cfg.pwr_pin, LOW);
                    return finishJob(ModemStatus::ERR_NO_SIM);
                }
            }
            break;

        // --- Блок проверки Сигнала (CSQ) ---
        case Step::INIT_CSQ_SEND:                                                   
            sendAT("AT+CSQ", "OK", ModemCfg::AT_TIMEOUT_MS);
            _currentStep = Step::INIT_CSQ_WAIT;
            break;

        case Step::INIT_CSQ_WAIT:                                                                                                      
            if (waitAT(res, timeout)) {
                bool signal_ok = false;
                if (!timeout) {
                    int commaIndex = res.indexOf(',');
                    if (commaIndex != -1) {
                        int rssi = res.substring(res.indexOf(':') + 2, commaIndex).toInt();
                        if (rssi >= 7 && rssi != 99) {
                            signal_ok = true;
                            // ИСПРАВЛЕНИЕ: Выводим переменную отдельно от макроса LOG
                            Serial.println(">>> Модем: Отличный уровень сигнала (CSQ: " + String(rssi) + ")");
                            LOG("Модем: Отличный уровень сигнала");
                        } else {
                            // ИСПРАВЛЕНИЕ: Выводим переменную отдельно
                            Serial.println(">>> Модем: Слабый сигнал (CSQ: " + String(rssi) + "). Ждем...");
                            LOG("Модем: Слабый сигнал. Ждем...");
                        }
                    }
                }

                if (signal_ok) {
                    // Оставляем Serial1 работать! Нам нужен UART для PPP.
                    // return finishJob(_hardwareRestarted ? ModemStatus::SUCCESS_WITH_RESTARTS : ModemStatus::SUCCESS);
                    return finishJob(ModemStatus::SUCCESS);
                } else {
                    _subAttempts++;
                    if (_subAttempts < 15) {
                        _timer = millis();
                        _currentStep = Step::INIT_CSQ_DELAY;
                    } else {
                        LOG("Модем: ОШИБКА. Нет сети (таймаут CSQ).");
                        Serial1.end();
                        digitalWrite(_cfg.pwr_pin, LOW);
                        _attempts++;
                        if (_attempts >= ModemCfg::MAX_RETRIES) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);
                        _currentStep = Step::INIT_START;
                    }
                }
            }
            break;

        case Step::INIT_CSQ_DELAY:                                                  
            if (millis() - _timer >= 1000) _currentStep = Step::INIT_CSQ_SEND;
            break;

        default: break;
    }
    return ModemStatus::BUSY;                                                       
}


// -------------------------------------- Поднимает PPP, подключаемся к серверу и отправляем POST --------------------------------------
ModemStatus Sim800LManager::processRequest(const String& payload, String& response) {
    if (_currentJob != JobType::REQUEST) {                                          
        _currentJob = JobType::REQUEST;
        _currentStep = Step::REQ_PPP_BEGIN;
    }

    switch (_currentStep) {
        case Step::REQ_PPP_BEGIN: {                                                  
            LOG("HTTP: Поднятие GPRS/PPP интерфейса (Алгоритм из рабочего скетча)...");
            
            // 1. Закрываем UART с небольшой паузой
            Serial1.flush();
            Serial1.end(); 
            delay(100);
            
            // 2. Настраиваем PPP
            PPP.setApn(ModemCfg::APN);
            PPP.setPins(_cfg.tx_pin, _cfg.rx_pin); 
            
            // Даем драйверу PPP право делать Hard Reset модему!
            // Если после наших ручных AT-команд модем будет в непонятном состоянии,
            // драйвер сам сбросит его и инициализирует как надо.
            PPP.setResetPin(_cfg.rst_pin, true, 200);
            
            esp_task_wdt_delete(NULL); 
            
            // МАГИЯ №2: Возвращаем драйвер SIM800
            bool ppp_ok = PPP.begin(PPP_MODEM_SIM800);
            
            esp_task_wdt_add(NULL); 
            
            if (!ppp_ok) {
                LOG("HTTP: Критическая ошибка. Не удалось запустить драйвер PPP!");
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            
            _isPppActive = true;
            _timer = millis();
            _currentStep = Step::REQ_PPP_ATTACH_WAIT;
            break;
        }

        case Step::REQ_PPP_ATTACH_WAIT:                                             
            if (PPP.attached()) {
                LOG("HTTP: GPRS Attached! Переход в CMUX и ожидание IP...");
                
                // Принудительный перевод в CMUX после подключения к сети
                PPP.mode(ESP_MODEM_MODE_CMUX);
                
                _timer = millis();
                _currentStep = Step::REQ_PPP_CONN_WAIT;
            } else if (millis() - _timer > 40000) {                                 // таймаут attach - 40 сек
                LOG("HTTP: ОШИБКА. Таймаут GPRS Attach.");
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            break;

        case Step::REQ_PPP_CONN_WAIT:                                               
            if (PPP.connected()) {
                LOG("HTTP: PPP Соединение активно! IP-адрес получен.");
                _currentStep = Step::REQ_HTTP_CONNECT;
            } else if (millis() - _timer > 60000) {                                 // таймаут ip - 60 сек
                LOG("HTTP: ОШИБКА. Таймаут получения IP по PPP.");
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            break;
                                              
        case Step::REQ_HTTP_CONNECT: {                                              
            LOG("HTTP: Попытка установки SSL-соединения с сервером ВК...");
            _client.setInsecure();
            _client.setTimeout(15); 
            
            if (!_client.connect(ModemCfg::SERVER_HOST, 443)) {
                _subAttempts++;                                                    // Увеличиваем счетчик попыток
                if (_subAttempts <= 3) {
                    LOG("HTTP: Ошибка SSL. Пауза 3 сек...");
                    _timer = millis();
                    // Переходим в специальный шаг паузы перед новой попыткой
                    _currentStep = Step::REQ_HTTP_RETRY;
                    return ModemStatus::BUSY;
                } else {
                    LOG("HTTP: КРИТИЧЕСКАЯ ОШИБКА. Сервер недоступен после 3 попыток.");
                    return finishJob(ModemStatus::ERR_SERVER_CONNECT);
                }
            }
            _subAttempts = 0; 
            LOG("HTTP: Соединение установлено! Отправка POST Payload...");
            String request = String("POST ") + ModemCfg::SERVER_PATH + " HTTP/1.1\r\n" +
                             "Host: " + ModemCfg::SERVER_HOST + "\r\n" +
                             "Content-Type: application/x-www-form-urlencoded\r\n" +
                             "Content-Length: " + payload.length() + "\r\n" +
                             "Connection: close\r\n\r\n" + 
                             payload;
            _client.print(request);
            
            _timer = millis();
            _uartBuffer = ""; 
            _uartBuffer.reserve(ModemCfg::MAX_HTTP_LEN);                          // Резервируем много памяти под ответ ВК
            
            LOG("HTTP: Запрос отправлен. Ожидание ответа...");
            _currentStep = Step::REQ_HTTP_WAIT_RES;
            break;
        }

        case Step::REQ_HTTP_RETRY:
            if (millis() - _timer >= 3000) {                                      // Между попытками сконнектиться с сервером - 3 секунды
                _currentStep = Step::REQ_HTTP_CONNECT;
            }
            break;

        case Step::REQ_HTTP_WAIT_RES:                                               
            // 1. Читаем всё, что падает в буфер
            while (_client.available() && _uartBuffer.length() < ModemCfg::MAX_HTTP_LEN) {
                _uartBuffer += (char)_client.read();
            }
            
            // 2. Сервер закрыл соединение и всё прочитано
            if (!_client.connected() && _client.available() == 0) {
                _client.stop();
                if (_uartBuffer.indexOf("200 OK") != -1) {
                    LOG("HTTP: УСПЕХ! Ответ 200 OK получен.");
                    response = _uartBuffer; 
                    return finishJob(ModemStatus::SUCCESS);
                } else {
                    LOG("HTTP: ОШИБКА сервера. Код 200 OK не найден в ответе.");
                    Serial.println("<<< СЕРВЕР ОТВЕТИЛ: \n" + _uartBuffer); // Выводим в лог тело ошибки
                    return finishJob(ModemStatus::ERR_HTTP_TIMEOUT);
                }
            } 
            // 3. Защита от бесконечного зависания
            else if (millis() - _timer >= ModemCfg::HTTP_TIMEOUT_MS) { 
                LOG("HTTP: ОШИБКА. Таймаут ожидания ответа от сервера.");
                _client.stop();
                return finishJob(ModemStatus::ERR_HTTP_TIMEOUT);
            }
            break;
    
        default: break;
    }
    return ModemStatus::BUSY;                                                       
}
// -------------------------------------- Выключаем интерфейсы и отключаем модем от питания --------------------------------------
ModemStatus Sim800LManager::processPowerOff() {
    if (_currentJob != JobType::POWER_OFF) {                                        
        _currentJob = JobType::POWER_OFF;
        _currentStep = Step::OFF_START;
    }

    switch (_currentStep) {
        case Step::OFF_START:                                                       
            LOG("Модем: Инициировано штатное выключение...");
            if (_isPppActive) {
                // АНТИ-ПАНИКА: Мягкое закрытие сессии перед уничтожением объекта
                // Заставляем LwIP безопасно закрыть PPP-сессию, пока объект еще жив.
                PPP.mode(ESP_MODEM_MODE_COMMAND);
                
                _timer = millis();
                _currentStep = Step::OFF_DELAY;
            } else {
                Serial1.end(); 
                digitalWrite(_cfg.pwr_pin, LOW); 
                LOG("Модем: Полностью обесточен.");
                return finishJob(ModemStatus::SUCCESS);
            }
            break;

        case Step::OFF_DELAY:                                                       
            // Даем операционной системе 1.5 секунды на переваривание всех асинхронных событий (Phase DEAD) от LwIP перед тем, как удалять объект из памяти
            if (millis() - _timer >= 1500) {
                PPP.end(); // Теперь уничтожать объект абсолютно безопасно
                _isPppActive = false;
                
                Serial1.end();
                digitalWrite(_cfg.pwr_pin, LOW);
                LOG("Модем: Полностью обесточен.");
                return finishJob(ModemStatus::SUCCESS);
            }
            break;

        default: break;
    }
    return ModemStatus::BUSY;                                                       
}