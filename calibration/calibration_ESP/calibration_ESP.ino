#include <WiFi.h>
#include <HTTPClient.h>

float scale, temp;
String inputString = "";

bool newData = false;

void setup() {
  Serial.begin(9600);        // Для вывода в монитор
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // адаптируй пины
  ConnectWiFi();
}

void loop() {
  if (Serial2.available()) {  // читаем теперь из Serial2
      inputString = Serial2.readStringUntil('\n'); // до Enter или \n
      inputString.trim(); // убираем пробелы и \r

      int sepIndex = inputString.indexOf('/'); // ищем

      if (sepIndex != -1) {
        String part1 = inputString.substring(0, sepIndex);
        String part2 = inputString.substring(sepIndex + 1);

        scale = part1.toFloat();
        temp = part2.toFloat();

        Serial.print("Первое число: ");
        Serial.println(scale, 4);
        Serial.print("Второе число: ");
        Serial.println(temp, 4);
        newData = true;
      } else {
        Serial.println("Ошибка: нет символа '/' в строке");
      }
  }

  if (newData) {
    HTTPGET();
    newData = false;
  }
}

void ConnectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin("11", "Alexey17");

  uint32_t start_time = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - start_time < 2 * 60 * 1000) {        
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED) {
    ESP.restart();
  }

  Serial.println("Connected!");
}

void HTTPGET() {

  String req = "http://open-monitoring.online/get?cid=4464&key=qitHUa&p1=";      // замените на свою реальную строку
  req += scale;
  req += "&p2=";
  req += temp;
    
  HTTPClient http;
  http.begin(req);
  
  if (http.GET() <= 0) Serial.println("HTTP Error!");
  else Serial.println("HTTP Success!");
  
  http.end();
}
