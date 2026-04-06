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

#define TINY_GSM_MODEM_SIM800

#include <GyverWDT.h>
#include <EEManager.h>
#include <uButton.h>
#include <GyverPower.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include "Sensors.h"
#include "Secrets/Secrets.h"
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

constexpr uint32_t DATA_SEND_PERIOD = 5*60*1000UL;     // период отправки данных в нормальном режиме работы (первое число - минуты)

SystemState currentState = SystemState::WAKEUP_SENSORS;
ModificationRequest external_request = ModificationRequest::NONE;

uint32_t stateTimer = 0;

AsyncLed led(RED_PIN, GREEN_PIN, BLUE_PIN);
ScalesManager scales(11.472, DT_PIN, SCL_PIN);
ScaleAutoCalibrator calibrator(REF_WEIGHT);
TempManager tempSensor(DS_PIN);
InputHandler inputHanler(BUTT_PIN, CALIB_SWITCH_PIN, calibrator, external_request, currentState);

void changeFSMState(SystemState newState) {                // функция переключения состояния FSM. С помощью PREV_STATE помогает "гостить" в определенном состоянии и возвращаться обратно, нужно в некоторых случаях
    static SystemState previousState = SystemState::WAKEUP_SENSORS;

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

    if (external_request == ModificationRequest::TARE)  changeFSMState(SystemState::TARE_PROCESS);          // по событию вызываем тарирование, оно потом самостоятельно откатит current_state на состояние до вызова
    
    switch (currentState) {
        case SystemState::WAKEUP_SENSORS:
            //scales.sleepMode(false);

            changeFSMState(SystemState::MEASURE);
            break;

        case SystemState::MEASURE:                          // читаем датчики
            static bool f_send = true;

            scales.tick();
            tempSensor.tick();

            if (external_request == ModificationRequest::START_CALIBRATION) {                   // если запрошена калибровка, переходим на нее
                external_request = ModificationRequest::NONE;
                changeFSMState(SystemState::CALIBRATION);
                calibrator.startCalibration();
                LOG("Calibration started");
            }

            else if (millis() - sendState_timer >= DATA_SEND_PERIOD || f_send) {                          // отправляем по заданному периоду или по force_send
                f_send = false;
                changeFSMState(SystemState::START_MODEM);
                sendState_timer = millis();
                LOG("Starting Modem states...");
            }
            else changeFSMState(SystemState::ERROR_HANDLING);

            break;

        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------
        case SystemState::START_MODEM:
            // включаем модем
            // подключаем к сети
            // проверяем готовность к передаче данных

            changeFSMState(SystemState::DATA_SEND);
            break;

        case SystemState::DATA_SEND:
            // читаем входящие данные/отправляем на сервер новые
            // проверяем наличие ошибок

            Serial1.print(sensorData.weightKg, 2);             // неотфильтрованная масса
            Serial1.print("/");
            Serial1.print(sensorData.tempC, 1);                // температура
            Serial1.print("/");
            Serial1.print(calibrator.compensate(sensorData.tempC, sensorData.weightGr) / 1000, 2);                        // отфильтрованная масса
            Serial1.print("/");
            Serial1.println(calibrator.getUncertainty(), 4);                                                              // "неуверенность"

            changeFSMState(SystemState::SLEEP_MODEM);
            break;

        case SystemState::SLEEP_MODEM:
            // отключаемся от сети
            // переводим модем в режим сна
            changeFSMState(SystemState::ERROR_HANDLING);
            break;
        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------


        case SystemState::ERROR_HANDLING:
            // сюда будем попадать после возникновения ошибок
            // пытаемся их починить, откорректировать работу
            // готовим данные о возникших неполадках/трудностях работы к отправке на сервер
            changeFSMState(SystemState::SLEEP_SENSORS);
            break;

        case SystemState::SLEEP_SENSORS:
            //scales.sleepMode(false);

            changeFSMState(SystemState::WAKEUP_SENSORS);               // ставим состояние, которое начнет выполняться после выхода из сна
            break;


        // ------------------------------------- Особые состояния (не входят в стандартный цикл работы) -------------------------------------

        case SystemState::CALIBRATION:
            static uint32_t cal_tick_timer = 0, rls_timer = 0, send_timer = 0;

            if (external_request == ModificationRequest::END_CALIBRATION) {                                                   // переключатель перевели в режим нормальной работы
                external_request = ModificationRequest::NONE;
                changeFSMState(SystemState::MEASURE);                           // ставим на MEASURE чтобы сразу прогнать стандартный цикл, state пробуждения нам не нужен, датчики у нас не спали
                calibrator.finishCalibration();                                 // при выходе из режима калбировки заканчиваем процесс и сохраняем данные о новой моедил
                LOG("Calibration finished!");
            }
            
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

            if (millis() - send_timer > 5 * 60 * 1000UL) { // отправка данных через Serial
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
            external_request = ModificationRequest::NONE;
            ScalesState tare_state = scales.sensorTare();
            if (tare_state == ScalesState::SUCCESS)    led.setMode(LedModes::OK);
            else if (tare_state == ScalesState::ERROR)   led.setMode(LedModes::ERROR);
            changeFSMState(SystemState::PREV_STATE);                                                // возвращаемся в то состояние, откуда было вызвано тарирование
            break;
        }
        
        case SystemState::PREV_STATE:                 // чтобы компилятор не ругался на этот state
            break;
        // ------------------------------------- Особые состояния (не входят в стандартный цикл работы) -------------------------------------
    }
}