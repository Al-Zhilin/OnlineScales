#include <WiFi.h>
#include <HTTPClient.h>
#include "../Secrets/Secrets.h"

float param1, param2, param3, param4;
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

      // Считаем количество разделителей '/'
      int slashCount = 0;
      for (int i = 0; i < inputString.length(); i++) {
        if (inputString[i] == '/') slashCount++;
      }
      
      // Ожидаем 3 разделителя для 4 параметров
      if (slashCount == 3) {
        int firstSlash = inputString.indexOf('/');
        int secondSlash = inputString.indexOf('/', firstSlash + 1);
        int thirdSlash = inputString.indexOf('/', secondSlash + 1);
        
        String part1 = inputString.substring(0, firstSlash);
        String part2 = inputString.substring(firstSlash + 1, secondSlash);
        String part3 = inputString.substring(secondSlash + 1, thirdSlash);
        String part4 = inputString.substring(thirdSlash + 1);
        
        param1 = part1.toFloat();
        param2 = part2.toFloat();
        param3 = part3.toFloat();
        param4 = part4.toFloat();
        
        Serial.println("Получены параметры:");
        Serial.print("param1: ");
        Serial.println(param1, 4);
        Serial.print("param2: ");
        Serial.println(param2, 4);
        Serial.print("param3: ");
        Serial.println(param3, 4);
        Serial.print("param4: ");
        Serial.println(param4, 4);
        
        newData = true;
      } else {
        Serial.print("Ошибка: ожидалось 4 параметра (3 разделителя '/'), получено: ");
        Serial.println(slashCount + 1);
        Serial.println("Строка: " + inputString);
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
    Serial.println("Connecting...");
  }

  if (WiFi.status() != WL_CONNECTED) {
    ESP.restart();
  }

  Serial.println("Connected!");
}

void HTTPGET() {

  String req = FUNCTION_ADDRESS;
  req += "?&p1=";
  req += param1;
  req += "&p2=";
  req += param2;
  req += "&p3=";
  req += param3;
  req += "&p4=";
  req += param4;
    
  HTTPClient http;
  http.begin(req);
  
  if (http.GET() <= 0) Serial.println("HTTP Error!");
  else Serial.println("HTTP Success!");
  
  http.end();
}