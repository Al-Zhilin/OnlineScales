#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// --- Настройки ---
const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASS";
const char* functionUrl = "https://functions.yandexcloud.net/id_вашей_функции";

Preferences prefs;
double sumT = 0, sumW = 0, sumT2 = 0, sumTW = 0;
uint32_t count = 0;

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    
    prefs.begin("calib_data", false);
    sumT = prefs.getDouble("sumT", 0);
    sumW = prefs.getDouble("sumW", 0);
    sumT2 = prefs.getDouble("sumT2", 0);
    sumTW = prefs.getDouble("sumTW", 0);
    count = prefs.getUInt("count", 0);
}

// Функция расчета коэффициента k (наклон линии)
double calculateCurrentK() {
    if (count < 2) return 0;
    double denominator = (count * sumT2 - sumT * sumT);
    if (abs(denominator) < 0.000001) return 0; 
    return (count * sumTW - sumT * sumW) / denominator;
}

void sendDataToCloud(float t, float w, double k) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(functionUrl);
        http.addHeader("Content-Type", "application/json");

        StaticJsonDocument<200> doc;
        doc["temp"] = t;
        doc["raw_w"] = w;
        doc["k"] = k;
        doc["cnt"] = count;

        String jsonPayload;
        serializeJson(doc, jsonPayload);

        int httpResponseCode = http.POST(jsonPayload);
        
        if (httpResponseCode > 0) {
            Serial.print("Cloud Response: ");
            Serial.println(httpResponseCode);
        } else {
            Serial.print("Error on sending POST: ");
            Serial.println(httpResponseCode);
        }
        http.end();
    }
}

void loop() {
    // 1. Читаем датчики (эмуляция или реальные вызовы)
    float currentT = readTemp(); 
    float currentW = readWeight();

    // 2. Накапливаем статистику для регрессии
    sumT += currentT;
    sumW += currentW;
    sumT2 += (double)currentT * currentT;
    sumTW += (double)currentT * currentW;
    count++;

    // 3. Считаем текущий коэффициент
    double k = calculateCurrentK();

    // 4. Раз в 10 минут отправляем данные и сохраняем состояние
    static uint32_t sendTimer = 0;
    if (millis() - sendTimer > 600000 || sendTimer == 0) {
        sendTimer = millis();
        
        sendDataToCloud(currentT, currentW, k);
        
        prefs.putDouble("sumT", sumT);
        prefs.putDouble("sumW", sumW);
        prefs.putDouble("sumT2", sumT2);
        prefs.putDouble("sumTW", sumTW);
        prefs.putUInt("count", count);
        
        Serial.printf("Saved: T=%.2f, W=%.2f, K=%.6f, C=%d\n", currentT, currentW, k, count);
    }

    delay(10000); // Проверка раз в 10 секунд
}