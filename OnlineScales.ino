#include "Enumerations.h"

#define USE_LOG Serial                            // удобный отладчик (закомментируй эту строку чтобы отключить)
void logHelper(const __FlashStringHelper* msg, const char* func, const char* file, int line) {          // функция удобного логирования
    #ifdef USE_LOG
        USE_LOG.print(F("> "));
        USE_LOG.print(msg);
        USE_LOG.print(F(" in "));
        USE_LOG.print(func);
        USE_LOG.print(F("() ["));
        USE_LOG.print(file);
        USE_LOG.print(F(" : "));
        USE_LOG.print(line);
        USE_LOG.println(F("]"));
    #endif
}
#ifdef USE_LOG
  #define LOG(x) logHelper(F(x), __FUNCTION__, __FILE__, __LINE__)
#else
  #define LOG(x)
#endif

#include <GyverWDT.h>
#include <EEManager.h>
#include <uButton.h>
#include "Sensors.h"
#include "Passwords/Secrets.h"
#include "Led_UI.h"
#include "PowerManager.h"
#include "GSM_Handler.h"

#define WORK_MODE 1                               // режим работы: обычная работа (1) / калибровка (0)

#define RST_PIN 9                                 // пин RST на SIM800L
#define SCL_PIN 8                                 // SCL пин HX711
#define DT_PIN 7                                  // DT пин HX711
#define DS_PIN 6                                  // пин DS18b20
#define BUTT_PIN 10                               // пин кнопки
#define RED_PIN 5                                 // красный цвет RGB светодиода
#define GREEN_PIN 4                               // зеленый цвет
#define BLUE_PIN 3                                // пин кнопки


SystemState currentState = (WORK_MODE) ? SystemState::WAKEUP : SystemState::CALIBRATION;
uint32_t stateTimer = 0;
bool pendingTare = false;

AsyncLed led(RED_PIN, GREEN_PIN, BLUE_PIN);
GyverDS18Single ds(DS_PIN);
ScalesManager scales(11.472, DT_PIN, SCL_PIN);
uButton butt(BUTT_PIN);

void changeState(SystemState newState) {                // функция переключения состояния
    currentState = newState;
    stateTimer = millis();
}

void setup() {
    Serial.begin(9600);
    led.begin();
    scales.begin();
    // Инициализация пинов...
    // Чтение логов ошибок из EEPROM...
    stateTimer = millis();
}

void loop() {
    led.tick();
    butt.tick();

    if (butt.hold())    {
        pendingTare = true;                         // подняли флаг. После WAKEUP обработаем его
        LOG("Tare request captured");
    }
    
    switch (currentState) {
        case SystemState::CALIBRATION:
            // здесь только калибровка весов и логирование данных, никакого переключения состояния
            break;

        case SystemState::TARE_PROCESS:
            int8_t tare_state = scales.sensorTare();
            if (tare_state == 1)    led.setMode(LedModes::OK);
            else if (!tare_state)   led.setMode(LedModes::ERROR);
            break;

        case SystemState::WAKEUP:
            break;

        case SystemState::MEASURE:
            break;

        case SystemState::CONNECT_GSM:
            break;

        case SystemState::DATA_SEND:
            break;

        case SystemState::ERROR_HANDLING:
            break;

        case SystemState::SLEEP:
            break;
    }
}