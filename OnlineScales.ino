#include "Enumerations.h"

#define USE_LOG Serial                            // удобный отладчик через Serial (закомментируй эту строку чтобы отключить)
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
#include <EEManager.h>
#include "Sensors.h"
#include "Passwords/Secrets.h"
#include "Led_UI.h"
#include "PowerManager.h"
#include "GSM_Handler.h"
#include "Calibration.h"
#include "InputHandler.h"

#define REF_WEIGHT 20000.0f                             // калибровочная масса для алгоритма подбора компенсации температурного дрейфа

#define RST_PIN 9                                       // пин RST на SIM800L
#define SCL_PIN 8                                       // SCL пин HX711
#define DT_PIN 7                                        // DT пин HX711
#define DS_PIN 6                                        // пин DS18b20
#define BUTT_PIN 10                                     // пин кнопки
#define RED_PIN 5                                       // красный цвет RGB светодиода
#define GREEN_PIN 4                                     // зеленый цвет
#define BLUE_PIN 3                                      // пин кнопки
#define CALIB_SWITCH_PIN 16                             // пин переключателя режимов работы

constexpr uint32_t DATA_SEND_PERIOD = 30*60*1000UL;     // период отправки данных в нормальном режиме работы (первое число - минуты)

SystemState currentState = SystemState::WAKEUP;

uint32_t stateTimer = 0;

AsyncLed led(RED_PIN, GREEN_PIN, BLUE_PIN);
ScalesManager scales(11.472, DT_PIN, SCL_PIN);
ScaleAutoCalibrator calibrator(REF_WEIGHT);
TempManager tempSensor(DS_PIN);
InputHandler inputHanler(BUTT_PIN, CALIB_SWITCH_PIN, currentState, calibrator);

void changeState(SystemState newState) {                // функция переключения состояния FSM. С помощью PREV_STATE помогает "гостить" в определенном состоянии и возвращаться обратно, нужно в некоторых случаях
    static SystemState previousState = SystemState::WAKEUP;

    if (newState == SystemState::PREV_STATE)    currentState = previousState;
    else {
        previousState = currentState;
        currentState = newState;
    }
    stateTimer = millis();
}

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    inputHanler.begin();

    led.begin();
    tempSensor.begin();
    uint16_t next_addr = scales.begin();               // в функции инициализируется обьект EEManager, чтобы 
    calibrator.begin(next_addr);                       // вот тут тоже инициализировался свой объект, по корректному адресу, нам нужно перекинуться инфой об (адресе конца предыдущих данных + 1) = первый адрес для записи новых данных
    stateTimer = millis();
}

void loop() {
    led.tick();
    inputHanler.tick();
    static uint32_t sendState_timer = millis();
    
    switch (currentState) {
        case SystemState::WAKEUP:
            // пробудили весы
            // пробудили ds18

            changeState(SystemState::MEASURE);
            break;

        case SystemState::MEASURE:                          // читаем датчики
            scales.tick();
            tempSensor.tick();

            if (millis() - sendState_timer >= DATA_SEND_PERIOD) {                          // отправляем по заданному периоду
                changeState(SystemState::CONNECT_GSM);
                sendState_timer = millis();
            }
            else changeState(SystemState::ERROR_HANDLING);
            break;

        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------
        case SystemState::CONNECT_GSM:
            // включаем модем
            // подключаем к сети
            // проверяем готовность к передаче данных

            changeState(SystemState::DATA_SEND);
            break;

        case SystemState::DATA_SEND:
            // читаем входящие данные/отправляем на сервер новые
            // проверяем наличие ошибок
            changeState(SystemState::ERROR_HANDLING);
            break;

        case SystemState::SLEEP_MODEM:
            // отключаемся от сети
            // переводим модем в режим сна
            break;
        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------


        case SystemState::ERROR_HANDLING:
            // сюда будем попадать после возникновения ошибок
            // пытаемся их починить, откорректировать работу
            // готовим данные о возникших неполадках/трудностях работы к отправке на сервер
            changeState(SystemState::SLEEP);
            break;

        case SystemState::SLEEP_SENSORS:
            // усыпляем весы и остальную периферию

            changeState(SystemState::WAKEUP);               // ставим состояние, которое начнет выполняться после выхода из сна
            break;


        // ------------------------------------- Особые состояния (не входят в стандартный цикл работы) -------------------------------------
        case SystemState::START_CALIBRATION:

            break;

        case SystemState::END_CALIBRATION:

            break;

        case SystemState::CALIBRATION:
            static uint32_t send_timer = 0, cal_tick_timer = 0, rls_timer = 0;
            
            if (millis() - cal_tick_timer >= 500) {
                scales.tick();
                tempSensor.tick();
                cal_tick_timer = millis();
            }

            if (millis() - rls_timer >= 15000) {
                if (calibrator.isCalibratingMode()) {
                     calibrator.calibrationStep();
                }
                rls_timer = millis();
            }

            if (millis() - send_timer > (uint32_t)(5 * 60 * 1000)) { // отправка данных через Serial
                Serial1.print(sensorData.weightKg, 2);             // неотфильтрованная масса
                Serial1.print("/");
                Serial1.print(sensorData.tempC, 1);                // температура
                Serial1.print("/");
                Serial1.print(calibrator.compensate(sensorData.tempC, sensorData.weightGr) / 1000, 2);                        // отфильтрованная масса
                Serial1.print("/");
                Serial1.println(calibrator.getUncertainty(), 4);                                                              // "неуверенность"
                send_timer = millis();
            }
            break;

        case SystemState::TARE_PROCESS:
        {
            ScalesState tare_state = scales.sensorTare();
            if (tare_state == ScalesState::SUCCESS)    led.setMode(LedModes::OK);
            else if (tare_state == ScalesState::ERROR)   led.setMode(LedModes::ERROR);
            changeState(SystemState::PREV_STATE);                                                // возвращаемся в то состояние, откуда было вызвано тарирование
            break;
        }
        
        case SystemState::PREV_STATE:                 // чтобы компилятор не ругался на этот state
            break;
        // ------------------------------------- Особые состояния (не входят в стандартный цикл работы) -------------------------------------
    }
}