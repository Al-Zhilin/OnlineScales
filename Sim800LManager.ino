#include "Sim800LManager.h"

Sim800LManager::Sim800LManager() : _isPppActive(false) {    }

void Sim800LManager::begin(Config cfg) {
    _cfg = cfg; // Принимаем уже заполненный конфиг из setup()
    
    pinMode(_cfg.pwr_pin, OUTPUT);
    pinMode(_cfg.rst_pin, OUTPUT);
    digitalWrite(_cfg.pwr_pin, LOW);  // по умолчанию модем выкл
    pinMode(_cfg.rst_pin, INPUT);     // RST "в воздухе", чтобы не держать модем в вечном сбросе. Боюсь, что 3.3В логики не хватит для отпускания RST, попробую подвешенное состояние

    _uartBuffer.reserve(ModemCfg::MAX_HTTP_LEN);                               // Резервируем много памяти под ответ ВК, под буфер общения с SIM хватит и подавно
}

void Sim800LManager::sendAT(const char* cmd, const char* expected, uint32_t timeout) {
    clearUART();
    Serial1.println(cmd);

    _expectedAtResponse = expected;
    _currentAtTimeout = timeout;
    _uartBuffer = "";
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
                    Serial1.println("AT+IPR=115200"); // Приказываем модему перейти на 115200
                    delay(100);                       // Даем модему время применить настройку
                    Serial1.updateBaudRate(115200);   // Переключаем саму ESP32 на 115200           
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

            //esp_task_wdt_delete(NULL); 
            bool ppp_ok = PPP.begin(PPP_MODEM_SIM800);                        // под капотом блокирующая, останавливаем мониторинг WDT, чтобы случайно не инициировать триггер рестарта
            //esp_task_wdt_add(NULL); 
            
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
            LOG("Модем/HTTP: Установка SSL и отправка через HTTPClient...");

            if (!PPP.connected()) return finishJob(ModemStatus::ERR_PPP_TIMEOUT);

            static NetworkClientSecure secureClient; 
            
            secureClient.stop(); // Принудительно очищаем зависшие сокеты от прошлых сеансов
            secureClient.setInsecure();
            secureClient.setTimeout(ModemCfg::HTTP_TIMEOUT_S); 

            HTTPClient http;
            // Передаем наш статичный клиент по ссылке
            http.begin(secureClient, String("https://") + ModemCfg::SERVER_HOST + ModemCfg::SERVER_PATH);
            http.addHeader("Content-Type", "application/x-www-form-urlencoded");

            // Блокирующий вызов - делает всю работу сам
            int httpCode = http.POST(payload);

            ModemStatus finalStatus = ModemStatus::ERR_HTTP_TIMEOUT;

            if (httpCode > 0) {
                if (httpCode == 200) {
                    response = http.getString();
                    LOG("Модем/HTTP: Успешно! Ответ 200 OK получен");
                    finalStatus = ModemStatus::SUCCESS;
                } else {
                    LOG("Модем/HTTP: Ошибка сервера");
                    finalStatus = ModemStatus::ERR_SERVER_CONNECT;
                }
            } else {
                LOG("Модем/HTTP: Ошибка SSL/Сети");
                finalStatus = ModemStatus::ERR_HTTP_TIMEOUT;
            }

            // Мягко закрываем сессию на уровне HTTP
            http.end();
            
            // Даем сетевому стеку 1 секунду на физическую отправку прощальных пакетов TCP FIN
            // в фоновом режиме ДО того, как FSM пойдет выключать модем.
            delay(1000); 
            
            return finishJob(finalStatus);
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
            LOG("Модем: Инициировано штатное выключение. Ожидание очистки сокетов LwIP...");
            
            _timer = millis();
            _currentStep = Step::OFF_DELAY;
            break;

        case Step::OFF_DELAY:                                                       
            // Даем системе 3 секунды на закрытие всех соединений в фоне.
            if (millis() - _timer >= 3000) {                          
                if (_isPppActive) {
                    PPP.end(); // Безопасно удаляем сетевой интерфейс из ОС
                    _isPppActive = false;
                }
                
                Serial1.flush();
                Serial1.end();
                digitalWrite(_cfg.pwr_pin, LOW); // Жестко рубим питание
                LOG("Модем: Полностью обесточен.");
                return finishJob(ModemStatus::SUCCESS);
            }
            break;

        default: break;
    }
    return ModemStatus::BUSY;                                                       
}