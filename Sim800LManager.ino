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
    _uartBuffer.reserve(ModemCfg::MAX_UART_LEN);
    _timer = millis();
}

WaitResult Sim800LManager::waitAT(String& outResponse) {
    // 1. Вычитываем UART, жестко ограничивая длину сохраненной строки (защита от переполнения кучи)
    int bytesRead = 0;
    while (Serial1.available() && bytesRead < 256) {                                    // вычитываем порциями по 256 байт = защита от спама по Serial
        char c = (char)Serial1.read();
        if (_uartBuffer.length() < ModemCfg::MAX_UART_LEN) {
            _uartBuffer += c;
        }
        bytesRead++;
    }
    
    // Если ожидаемый ответ пришел:
    if (_uartBuffer.indexOf(_expectedAtResponse) != -1) {
        outResponse = _uartBuffer;
        return WaitResult::OK;
    }

    // Если пришла ошибка:
    else if (_uartBuffer.indexOf("ERROR") != -1) {
        outResponse = _uartBuffer;
        return WaitResult::ERROR;
        
    }

    // 3. Проверяем таймаут
    else if (millis() - _timer >= _currentAtTimeout) {
        outResponse = _uartBuffer;
        LOG("Modem: waitAT function timeout!");
        return WaitResult::TIMEOUT;
    }
    return WaitResult::BUSY;
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
        _currentStep = Step::INIT_START;                   // начинаем с INIT_START, INIT_START_DELAY нужен для выдержки таймаут при запуске после перезагрузки
        _attempts = 0;
        _subAttempts = 0;
        // _hardwareRestarted = false; // Раскомментировать, если понадобится в логике
    }

    String res;

    switch (_currentStep) {
        case Step::INIT_START_DELAY:                       // ждем пару секунд перед новой попыткой
            if (millis() - _timer >= 2000)  _currentStep = Step::INIT_START;
            break;

        case Step::INIT_START:                             // стартуем Serial, запускаем модем                                                
            LOG("Модем: Включение питания и инициализация интерфейсов...");
            
            Serial1.begin(9600, SERIAL_8N1, _cfg.rx_pin, _cfg.tx_pin); 
            
            pinMode(_cfg.rst_pin, INPUT);      
            digitalWrite(_cfg.pwr_pin, HIGH);  
            
            _timer = millis();
            _currentStep = Step::INIT_PWR_DELAY; 
            break;

        case Step::INIT_PWR_DELAY:
            // Ждем 3.5 сек загрузки ОС внутри SIM800L
            if (millis() - _timer >= 3500) {
                LOG("Модем: таймаут запуска выждан");
                _currentStep = Step::INIT_AT_SEND;
            }
            break;

        case Step::INIT_AT_SEND:
            // "Будим" UART модема
            for (int i = 0; i < 5; i++) {                   // спамим чтобы убедить его в принятии нашей скорости передачи
                Serial1.println("AT");
                delay(50);
            }
            
            sendAT("AT", "OK", ModemCfg::AT_TIMEOUT_MS);    // теперь отправляем с целью получить ответ
            _currentStep = Step::INIT_AT_WAIT;
            break;

        case Step::INIT_AT_WAIT:                                                    
            if (WaitResult waitStatus = waitAT(res); waitStatus != WaitResult::BUSY) {
                if (waitStatus == WaitResult::OK) {          // если модем ответил "ОК"
                    Serial1.println("AT+IPR=9600");                 // жестко установили скорость            
                    Serial1.println("ATE0");                        // отключили эхо
                    _timer = millis();
                    _currentStep = Step::INIT_SIM_DELAY;
                    LOG("Модем: Успешный ответ на AT. UART синхронизирован!");
                } else {                                            // не успел в таймаут или вернул ERROR
                    _subAttempts++;
                    if (_subAttempts < 5) {
                        _timer = millis();
                        _currentStep = Step::INIT_AT_DELAY;
                    } else {
                        LOG("Модем: КРИТИЧЕСКАЯ ОШИБКА. Нет ответа на AT. Рестарт FSM.");
                        Serial1.end();
                        digitalWrite(_cfg.pwr_pin, LOW);
                        _attempts++;
                        if (_attempts >= ModemCfg::MAX_RETRIES) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);         // если рестары исчерпались - завершаем цикл, выходя из FSM с ошибкой
                        _subAttempts = 0;
                        _timer = millis();
                        _currentStep = Step::INIT_START_DELAY;
                    }
                }
            }
            break;

        case Step::INIT_AT_DELAY:                                                   
            if (millis() - _timer >= 500) _currentStep = Step::INIT_AT_SEND;
            break;

        
        // ----------- Блок проверки SIM -----------
        case Step::INIT_SIM_DELAY:                                                  
            if (millis() - _timer >= 2000) {                                // выжидаем перед проверкой SIM (не знаю, надо ли - возможно позже уберем)
                sendAT("AT+CPIN?", "READY", ModemCfg::SIM_TIMEOUT_MS);
                _currentStep = Step::INIT_SIM_WAIT;
            }
            break;

        case Step::INIT_SIM_WAIT:                                                   
            if (WaitResult waitStatus = waitAT(res); waitStatus != WaitResult::BUSY) {
                if (waitStatus == WaitResult::OK) {         // сим карта готова (дождались в ответе "READY")
                    LOG("Модем: SIM-карта READY.");
                    _subAttempts = 0;
                    _currentStep = Step::INIT_CSQ_SEND;
                } else {                                    // не нашли ожидаемого ответа или вышел таймаут
                    LOG("Модем: Ошибка SIM-КАРТЫ (Не вставлена или заблокирована).");
                    Serial1.end();
                    digitalWrite(_cfg.pwr_pin, LOW);
                    return finishJob(ModemStatus::ERR_NO_SIM);      // состояние SIM не меняется со временем, нет смысла пробовать снова. Связь налажена, а проблема с SIM как правило - статична
                }
            }
            break;

        // --- Блок проверки Сигнала (CSQ) ---
        case Step::INIT_CSQ_SEND:                           // проверяем уровень сигнала                      
            sendAT("AT+CSQ", "OK", ModemCfg::AT_TIMEOUT_MS);
            _currentStep = Step::INIT_CSQ_WAIT;
            break;

        case Step::INIT_CSQ_WAIT:                                                                                                                 
        if (WaitResult waitStatus = waitAT(res); waitStatus != WaitResult::BUSY) {
            bool signal_ok = false;                     

            if (waitStatus != WaitResult::TIMEOUT) {    
                int csqIndex = res.indexOf("+CSQ: "); 
                
                if (csqIndex != -1) {
                    int valueStart = csqIndex + 6; 
                    int commaIndex = res.indexOf(',', valueStart); 
                    
                    if (commaIndex != -1 && commaIndex > valueStart) {
                        int rssi = res.substring(valueStart, commaIndex).toInt();
                        
                        if (rssi != 99 && rssi > 0) {                                           // 99 - нет сигнала/неизвестно. 0 - обычно означает <= -115 dBm (сигнала нет)
                            signal_ok = true;
                            LOG("Модем: Отличный уровень сигнала");
                        } else {                                    
                            LOG("Модем: Сигнал отсутствует или слишком слаб. Ждем...");
                        }
                    } else {
                        LOG("Модем: Получен +CSQ, но формат данных нарушен.");
                    }
                } else {
                    LOG("Модем: Ответ получен, но сигнатура CSQ не найдена.");                 // Если модем прислал "OK" или мусор без "+CSQ: "
                }
            }

            if (signal_ok) {                                    
                return finishJob(ModemStatus::SUCCESS);
            } else {
                _subAttempts++;
                if (_subAttempts < 15) {                        
                    _timer = millis();
                    _currentStep = Step::INIT_CSQ_DELAY;
                } else {
                    LOG("Модем: Ошибка. Нет сети (таймаут CSQ).");
                    Serial1.end();
                    digitalWrite(_cfg.pwr_pin, LOW);
                    _attempts++;
                    if (_attempts >= ModemCfg::MAX_RETRIES) return finishJob(ModemStatus::ERR_BOOT_TIMEOUT);
                    _subAttempts = 0;
                    _timer = millis();
                    _currentStep = Step::INIT_START_DELAY;
                }
            }
        }
        break;

        case Step::INIT_CSQ_DELAY:                                                  
            if (millis() - _timer >= 1000) _currentStep = Step::INIT_CSQ_SEND;
            break;

        default: break;
    }
    return ModemStatus::BUSY_INIT;                                                       
}


// -------------------------------------- Поднимает PPP, подключаемся к серверу и отправляем POST --------------------------------------
ModemStatus Sim800LManager::processRequest(const String& payload, String& response) {
    if (_currentJob != JobType::REQUEST) {                                          
        _currentJob = JobType::REQUEST;
        _currentStep = Step::REQ_PPP_BEGIN;
        _subAttempts = 0;
    }

    switch (_currentStep) {
        case Step::REQ_PPP_BEGIN: {                                                  
            LOG("Модем/PPP: Поднятие GPRS/PPP интерфейса...");
            
            // 1. Закрываем UART с небольшой паузой
            Serial1.flush();
            Serial1.end(); 
            delay(100);
            
            // 2. Настраиваем PPP
            PPP.setApn(ModemCfg::APN);
            PPP.setPins(_cfg.tx_pin, _cfg.rx_pin); 
            
            PPP.setResetPin(_cfg.rst_pin, true, 200);

            esp_task_wdt_delete(NULL); 
            bool ppp_ok = PPP.begin(PPP_MODEM_SIM800);                        // под капотом блокирующая, останавливаем мониторинг WDT, чтобы случайно не инициировать триггер рестарта
            esp_task_wdt_add(NULL); 
            
            if (!ppp_ok) {                                                    // программный перезапуск/повторный вызов PPP.begin() - очень хрупкий и сложный механизм, надежнее перезапустить модем питанием
                LOG("Модем/PPP: Критическая ошибка. Не удалось запустить драйвер PPP!");
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            
            LOG("Модем/PPP: PPP успешно инициализировано!");
            _isPppActive = true;
            _timer = millis();
            _currentStep = Step::REQ_PPP_ATTACH_WAIT;
            break;
        }
        case Step::REQ_PPP_ATTACH_WAIT:                                     // ждем PPP.attach()                    
            if (PPP.attached()) {                                           // PPP стек согласован
                LOG("Модем/HTTP: GPRS Attached! Переход в CMUX и ожидание IP...");
                
                PPP.mode(ESP_MODEM_MODE_CMUX);                                      // CMUX - возможность подкапотно создать несколько интерфейсов (PPP + AT + ...), обычно используется по умолчанию
                
                _timer = millis();
                _currentStep = Step::REQ_PPP_CONN_WAIT;
            } else if (millis() - _timer > ModemCfg::ATTACH_TIMEOUT_MS) {                                 // таймаут attach - 40 сек
                LOG("HTTP: Ошибка. Таймаут GPRS Attach.");
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            break;

        case Step::REQ_PPP_CONN_WAIT:                                       // ждем PPP.connected()                                        
            if (PPP.connected()) {                                          // сетевой стек готов к передаче данных
                LOG("Модем/HTTP: PPP Соединение активно! IP-адрес получен.");
                _currentStep = Step::REQ_HTTP_CONNECT;
            } else if (millis() - _timer > ModemCfg::NETWORK_TIMEOUT_MS) {                                 // таймаут ip - 60 сек
                LOG("Модем/HTTP: Ошибка. Таймаут получения IP по PPP.");
                return finishJob(ModemStatus::ERR_PPP_TIMEOUT);
            }
            break;
                                              
        case Step::REQ_HTTP_CONNECT: {                                              
            LOG("Модем/HTTP: Попытка установки SSL-соединения с сервером VK API...");
            _client.setInsecure();
            _client.setTimeout(5000);                                   // Таймаут внутренних операция Stream
            _client.setHandshakeTimeout(30);                            // Таймаут на криптографическое SSL/TLS-рукопожатие.

            esp_task_wdt_delete(NULL);                                  // WDT может сработать! Отписываемся здесь от него
            if (!_client.connect(ModemCfg::SERVER_HOST, 443, 15000)) {
                esp_task_wdt_add(NULL);                                 // возвращаем мониторинг WDT
                _subAttempts++;                                         // Увеличиваем счетчик попыток
                if (_subAttempts <= 3) {                                // 3 попытки на установку соединения
                    LOG("Модем/HTTP: Ошибка SSL. Пауза 3 сек...");
                    _timer = millis();

                    _currentStep = Step::REQ_HTTP_RETRY;                // пауза перед новой попыткой
                    return ModemStatus::BUSY;
                } else {
                    LOG("Модем/HTTP: Сервер недоступен. Попытки подключения исчерпаны");
                    return finishJob(ModemStatus::ERR_SERVER_CONNECT);
                }
            }
            esp_task_wdt_add(NULL);                                     // возвращаем мониторинг WDT здесь тоже, ведь в if код может не зайти, а задачу нужно добавить обратно обязательно
            _subAttempts = 0; 
            LOG("Модем/HTTP: Соединение установлено! Отправка POST Payload...");
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
            
            LOG("Модем/HTTP: Запрос отправлен. Ожидание ответа...");
            _currentStep = Step::REQ_HTTP_WAIT_RES;
            break;
        }

        case Step::REQ_HTTP_RETRY:
            if (millis() - _timer >= 3000) {                                      // Между попытками сконнектиться с сервером - 3 секунды
                _currentStep = Step::REQ_HTTP_CONNECT;
            }
            break;

        case Step::REQ_HTTP_WAIT_RES: {                                              
            // 1. Читаем всё, что падает в буфер
            int bytesRead = 0;
            while (_client.available() && bytesRead < 1024) {               // порциями по 1024 байта!! Это защита от спама, остальную ее часть - см. в логике
                char c = (char)_client.read();
                if (_uartBuffer.length() < ModemCfg::MAX_HTTP_LEN) {
                    _uartBuffer += c;
                }
                bytesRead++;
            }
            
            // 2. Сервер закрыл соединение И всё прочитано
            if (!_client.connected() && _client.available() == 0) {
                _client.stop();
                if (_uartBuffer.indexOf("200 OK") != -1) {                  // в ответе есть "200 ОК" - ответ от VK API, означающий успешность полученного запроса
                    LOG("Модем/HTTP: Успешность https запроса! Ответ 200 OK получен");
                    response = _uartBuffer; 
                    return finishJob(ModemStatus::SUCCESS);
                } else {
                    LOG("Модем/HTTP: Ошибка https запроса. Код 200 OK не найден в ответе");
                    //Serial.println("<<< СЕРВЕР ОТВЕТИЛ: \n" + _uartBuffer); // Выводим в лог тело ошибки
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
        }
    
        default: break;
    }
    if (_currentStep >= Step::REQ_HTTP_CONNECT) return ModemStatus::BUSY_HTTP;
    else return ModemStatus::BUSY_NET;;                                                       
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
            if (_isPppActive) {                                            // Заставляем LwIP безопасно закрыть PPP-сессию, если PPP в данном цикле работы с модемом был поднят
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
            if (millis() - _timer >= 1500) {                               // Даем системе 1.5 секунды на переваривание всех асинхронных событий (Phase DEAD) от LwIP перед тем, как удалять объект из памяти
                PPP.end();                                                 // Теперь уничтожать объект абсолютно безопасно
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