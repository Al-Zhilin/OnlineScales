#include <GyverDS18.h>
#include <GyverWDT.h>
#include <GyverHX711.h>
#include <EncButton.h>
#include "passwords.h"

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

GyverHX711 sensor(DT_PIN, SCL_PIN, HX_GAIN64_A);
Button tare_button(5);

void setup() {
  Serial.begin(9600);
  sensor.tare();
  sensor.sleepMode(false);
  tare_button.setBtnLevel(1);
}

void loop() {
  if (sensor.available()) {
    Serial.println(expRunningAverageAdaptive(sensor.read()));
  }

  if (Serial.available()) {
    if (Serial.parseInt() == 1)  sensor.tare();
  }
}

int32_t expRunningAverageAdaptive(int32_t newVal) {
  //return newVal;
  static int32_t filVal = 0;
  float k;
  // резкость фильтра зависит от модуля разности значений
  if (abs(newVal - filVal) > 50) k = 0.9;
  else k = 0.05;

  filVal += (newVal - filVal) * k;
  return filVal;
}
