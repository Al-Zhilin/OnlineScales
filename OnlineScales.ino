#include <GyverDS18.h>
#include <GyverWDT.h>
#include <GyverHX711.h>

// ---------------------------------------------- AT-команды ----------------------------------------------
#define CHECK_MODEM "AT"
#define ECHO_OFF "ATE0"
#define ECHO_ON "ATE1"
#define MODEM_INFO "ATI"
#define SIM_STATUS "AT+CPIN?"
#define REGISTRATION "AT+CREG?"
#define CHECK_OPERATOR "AT+COPS?"
#define SMS_MODE "AT+CMGF=1"
// ---------------------------------------------- AT-команды ----------------------------------------------

// --------------------------------------------- Пины Arduino ---------------------------------------------
#define RST_PIN 9           // пин RST SIM800L
#define DS_PIN 6            // пин DS18b20
#define DT_PIN 7            // DT пин HX711
#define SCL_PIN 8           // SCK пин HX711
// --------------------------------------------- Пины Arduino ---------------------------------------------

GyverHX711 sensor(DT_PIN, SCK_PIN, HX_GAIN128_A);

void setup() {
  Serial.begin(9600);
  sensor.tare();
  sensor.sleepMode(false);
}

void loop() {
  if (sensor.available()) {
    Serial.println(sensor.read());
  }
}
