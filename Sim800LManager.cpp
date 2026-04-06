#include "Sim800LManager.h"

Sim800LManager::Sim800LManager(Config cfg) : _cfg(cfg), _isPppActive(false) {
    pinMode(_cfg.pwr_pin, OUTPUT);
    pinMode(_cfg.rst_pin, OUTPUT);
    digitalWrite(_cfg.pwr_pin, LOW);        // Обесточено по умолчанию
    digitalWrite(_cfg.rst_pin, HIGH);
}

// --- ЭТАП 1: Инициализация железа ---
ModemStatus Sim800LManager::initHardware(uint8_t max_retries) {
    uint8_t attempts = 0;
    bool hardware_restarted = false;

    // Включаем питание транзистором
    digitalWrite(_cfg.pwr_pin, HIGH);
    delay(1000); 

    while (attempts < max_retries) {
        if (attempts > 0) {
            hardwareReset();
            hardware_restarted = true;
        }

        Serial1.begin(115200, SERIAL_8N1, _cfg.tx_pin, _cfg.rx_pin);

        // 1. Проверка UART
        bool at_ok = false;
        for (int i = 0; i < 5; i++) {
            if (sendATCommand("AT", 1000).indexOf("OK") != -1) {
                at_ok = true;
                break;
            }
            delay(500);
        }
        
        if (!at_ok) {
            attempts++;
            Serial1.end();
            continue; // Идем на следующий круг рестарта
        }

        // 2. Проверка SIM (даем время на инициализацию карты)
        delay(2000); 
        if (sendATCommand("AT+CPIN?", 2000).indexOf("READY") == -1) {
            Serial1.end();
            return ModemStatus::ERR_NO_SIM; // Рестарт тут не поможет, физическая проблема
        }

        // 3. Проверка сигнала (CSQ)
        // В сетях 2G поиск вышки может занять до 15 секунд
        bool signal_ok = false;
        for (int i = 0; i < 15; i++) {
            String csq = sendATCommand("AT+CSQ", 1000);
            int commaIndex = csq.indexOf(',');
            if (commaIndex != -1) {
                int rssi = csq.substring(csq.indexOf(':') + 2, commaIndex).toInt();
                if (rssi >= 7 && rssi != 99) {
                    signal_ok = true;
                    break;
                }
            }
            delay(1000);
        }

        if (!signal_ok) {
            attempts++;
            Serial1.end();
            continue; // Сигнал плохой, пробуем перезагрузить модем
        }

        // Подготовка к PPP
        Serial1.end(); 
        return hardware_restarted ? ModemStatus::SUCCESS_WITH_RESTARTS : ModemStatus::SUCCESS;
    }

    return ModemStatus::ERR_BOOT_TIMEOUT;
}

// --- ЭТАП 2: Сеть и Запрос ---
ModemStatus Sim800LManager::sendRequest(const char* host, const char* path, const String& payload, String& response, uint32_t network_timeout_ms) {
    
    PPP.setApn(_cfg.apn);
    PPP.setPins(_cfg.tx_pin, _cfg.rx_pin);
    
    if (!PPP.begin(PPP_MODEM_SIM800)) {
        return ModemStatus::ERR_PPP_TIMEOUT;
    }
    _isPppActive = true;

    // Ожидание регистрации сети
    unsigned long startAttach = millis();
    while (!PPP.attached() && (millis() - startAttach < network_timeout_ms)) {
        delay(500);
    }

    if (!PPP.attached() || !PPP.waitStatusBits(ESP_NETIF_CONNECTED_BIT, 20000)) {
        return ModemStatus::ERR_PPP_TIMEOUT;
    }

    // HTTPS Запрос
    NetworkClientSecure client;
    client.setInsecure(); 
    client.setTimeout(15); 

    if (!client.connect(host, 443)) {
        return ModemStatus::ERR_SERVER_CONNECT;
    }

    String request = String("POST ") + path + " HTTP/1.1\r\n" +
                     "Host: " + host + "\r\n" +
                     "Content-Type: application/json\r\n" +
                     "Content-Length: " + payload.length() + "\r\n" +
                     "Connection: close\r\n\r\n" + 
                     payload;
                     
    client.print(request);

    // Чтение ответа с таймаутом
    unsigned long http_timeout = millis();
    bool response_received = false;
    response = "";

    while (client.connected() && (millis() - http_timeout < 15000)) {
        if (client.available()) {
            response += client.readString();
            response_received = true;
        }
    }
    client.stop();

    if (!response_received || response.indexOf("200 OK") == -1) {
        return ModemStatus::ERR_HTTP_TIMEOUT;
    }

    return ModemStatus::SUCCESS;
}

// --- ЭТАП 3: Отключение ---
void Sim800LManager::powerOff() {
    if (_isPppActive) {
        PPP.end();
        _isPppActive = false;
        delay(1000); // Время на корректное закрытие сокетов LwIP
    }
    Serial1.end(); // На всякий случай освобождаем UART
    
    // Рубим питание
    digitalWrite(_cfg.pwr_pin, LOW);
}

// --- Приватные методы ---
void Sim800LManager::hardwareReset() {
    digitalWrite(_cfg.rst_pin, LOW);
    delay(300);
    digitalWrite(_cfg.rst_pin, HIGH);
    delay(4000); // Время загрузки RTOS внутри модема
}

String Sim800LManager::sendATCommand(const char* cmd, uint32_t timeout_ms) {
    clearUART();
    Serial1.println(cmd);
    String res = "";
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        while (Serial1.available()) res += (char)Serial1.read();
        if (res.indexOf("OK") != -1 || res.indexOf("ERROR") != -1) break;
    }
    return res;
}

void Sim800LManager::clearUART() {
    while (Serial1.available()) Serial1.read();
}