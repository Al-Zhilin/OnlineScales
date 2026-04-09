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

#include <EEManager.h>
#include <uButton.h>
#include "Sensors.h"
#include "Secrets/Secrets.h"
#include "Led_UI.h"
#include "PowerManager.h"
#include "GSM_Handler.h"
#include "Calibration.h"
#include "InputHandler.h"
#include "Sim800LManager.h"
#include <esp_task_wdt.h>
#include <esp_sleep.h>

Config ModemConfig;

#define REF_WEIGHT 20000.0f                             // калибровочная масса для алгоритма подбора компенсации температурного дрейфа

#define SCL_PIN 8                                       // SCL пин HX711
#define DT_PIN 7                                        // DT пин HX711
#define DS_PIN 6                                        // пин DS18b20
#define BUTT_PIN 10                                     // пин кнопки
#define RED_PIN 5                                       // красный цвет RGB светодиода
#define GREEN_PIN 4                                     // зеленый цвет
#define BLUE_PIN 3                                      // пин кнопки
#define CALIB_SWITCH_PIN 16                             // пин переключателя режимов работы
#define WDT_TIMEOUT_MS 10000                            // период для WDT (миллисекунды)
#define SLEEP_TIME_SEC 30                               // время сна между измерениями (секунлы)

constexpr uint32_t DATA_SEND_PERIOD = 15*60*1000UL;     // период отправки данных в нормальном режиме работы (первое число - минуты)

SystemState currentState = SystemState::WAKEUP_SENSORS;
ModificationRequest external_request = ModificationRequest::FORCE_SEND;              // чтобы при запуске модуль сразу сделал запрос, нужно для проверки соединения и мониторинга

uint32_t stateTimer = 0;
uint8_t restart_reason = 0;                                // см. использование ниже
String modemPayload = "";                                  // Буфер для сформированного запроса
String serverResponse = "";                                // Буфер для ответа от VK API
bool hasModemError = false;                                // Флаг для безопасного отключения при ошибках

AsyncLed led(RED_PIN, GREEN_PIN, BLUE_PIN);
ScalesManager scales(11.472, DT_PIN, SCL_PIN);
ScaleAutoCalibrator calibrator(REF_WEIGHT);
TempManager tempSensor(DS_PIN);
InputHandler inputHanler(BUTT_PIN, CALIB_SWITCH_PIN, calibrator, external_request, currentState);
Sim800LManager modemManager(ModemConfig);

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

    modemPayload.reserve(1024);                             // Избегаем излишних проблем со String: заранее резервируем память
    ModemConfig.pwr_pin = 4;                                // пин управления питанием SIM800L
    ModemConfig.rst_pin = 5;                                // пин, управляющий RST модема
    ModemConfig.tx_pin = 16;                                // пин TX от SIM800L
    ModemConfig.rx_pin = 17;                                // пин RS от SIM800L


    led.begin();
    tempSensor.begin();
    uint16_t next_addr = scales.begin();               // в функции инициализируется обьект EEManager, чтобы 
    calibrator.begin(next_addr);                       // вот тут тоже инициализировался свой объект, по корректному адресу, нам нужно перекинуться инфой об (адресе конца предыдущих данных + 1) = первый адрес для записи новых данных
    stateTimer = millis();

    restart_reason = (uint8_t)esp_reset_reason();      // причина завешения предыдущей работы

    esp_task_wdt_config_t twdt_config = {              // настраиваем конфиг для WDT таймера
      .timeout_ms = WDT_TIMEOUT_MS,                         // период допустимого "тупления"
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,      // Мониторим все ядра
      .trigger_panic = true,                                // true = перезагрузка при зависании
    };

    esp_task_wdt_reconfigure(&twdt_config);            // применяем новые настройки конфига
    esp_task_wdt_add(NULL);                            // подписываем loopTask() на мониторинг WDT
}

void loop() {
    led.tick();
    inputHanler.tick();
    esp_task_wdt_reset();
    static uint32_t sendState_timer = millis();

    if (external_request == ModificationRequest::TARE)  changeFSMState(SystemState::TARE_PROCESS);          // по событию вызываем тарирование, оно потом самостоятельно откатит current_state на состояние до вызова
    
    switch (currentState) {
        case SystemState::WAKEUP_SENSORS:
            scales.sleepMode(false);
            changeFSMState(SystemState::MEASURE);
            break;

        case SystemState::MEASURE:                          // читаем датчики
            scales.tick();
            tempSensor.tick();

            if (external_request == ModificationRequest::START_CALIBRATION) {                   // если запрошена калибровка, переходим на нее
                external_request = ModificationRequest::NONE;
                changeFSMState(SystemState::CALIBRATION);
                calibrator.startCalibration();
                LOG("Calibration started");
            }

            else if (millis() - sendState_timer >= DATA_SEND_PERIOD || external_request == ModificationRequest::FORCE_SEND) {                          // отправляем по заданному периоду или по внешнему запросу на немедленную отправку
                if (external_request == ModificationRequest::FORCE_SEND) {
                    external_request = ModificationRequest::NONE;
                }

                hasModemError = false;

                // --- 1. Формируем текст сообщения (с URL-кодированием переноса строки %0A) ---
                String msg = "Отчет с весов:%0A"
                             "Текущий вес: " + String(sensorData.weightKg, 2) + " кг%0A"
                             "Температура: " + String(sensorData.tempC, 1) + " °C%0A"
                             "Вес скомпенсированный: " + String(calibrator.compensate(sensorData.tempC, sensorData.weightGr) / 1000.0f, 2) + " кг%0A"
                             "Неуверенность калибратора: " + String(calibrator.getUncertainty(), 4);

                // --- 2. Упаковываем в формат x-www-form-urlencoded для VK API ---
                modemPayload = "user_id=" + String(VK_USER_ID) + 
                               "&random_id=0" + 
                               "&v=5.199" + 
                               "&access_token=" + String(VK_TOKEN) + 
                               "&message=" + msg;

                changeFSMState(SystemState::START_MODEM);
                sendState_timer = millis();
                LOG("Starting Modem states...");
            }
            else changeFSMState(SystemState::ERROR_HANDLING);

            break;

        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------
        case SystemState::START_MODEM:  {
            ModemStatus status = modemManager.processInit();
            
            if (status == ModemStatus::BUSY) break; // Крутим loop(), пока модем заводится
            
            if (status == ModemStatus::SUCCESS || status == ModemStatus::SUCCESS_WITH_RESTARTS) {
                LOG("Modem initialized successfully.");
                changeFSMState(SystemState::DATA_SEND);
            } else {
                LOG("Modem initialization failed!");
                hasModemError = true;
                restart_reason = static_cast<uint8_t>(status); 
                changeFSMState(SystemState::SLEEP_MODEM); // Обязательно идем обесточивать!
            }
            break;
        }

        case SystemState::DATA_SEND:    {
            ModemStatus status = modemManager.processRequest(modemPayload, serverResponse);
            
            if (status == ModemStatus::BUSY) break; // Крутим loop(), ждем ответа ВК
            
            if (status == ModemStatus::SUCCESS) {
                LOG("Data sent to VK successfully!");
                // LOG(serverResponse.c_str()); // Раскоментируй, если хочешь видеть ответ VK в консоли
            } else {
                LOG("Network/HTTP Error!");
                hasModemError = true;
                restart_reason = static_cast<uint8_t>(status);
            }
            
            // В любом случае (успех или провал) - выключаем модем
            changeFSMState(SystemState::SLEEP_MODEM);
            break;
        }

        case SystemState::SLEEP_MODEM:  {
            ModemStatus status = modemManager.processPowerOff();
            
            if (status == ModemStatus::BUSY) break; // Ждем корректного отключения PPP
            
            LOG("Modem powered off.");
            
            // Маршрутизация после того, как железо безопасно отключено
            if (hasModemError) {
                changeFSMState(SystemState::ERROR_HANDLING);
            } else {
                changeFSMState(SystemState::SLEEP_SENSORS);
            }
            break;
        }
        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------


        case SystemState::ERROR_HANDLING:
            // сюда будем попадать после возникновения ошибок
            // пытаемся их починить, откорректировать работу
            // готовим данные о возникших неполадках/трудностях работы к отправке на сервер
            changeFSMState(SystemState::SLEEP_SENSORS);
            break;

        case SystemState::SLEEP_SENSORS:
            scales.sleepMode(true);
            esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * 1000000ULL);         // настраиваемся на здоровый сон на заданное время
            esp_task_wdt_delete(NULL);                                          // чтобы WDT во сне не проголодался, отписываемся от мониторнга текущей Task
            esp_light_sleep_start();                                            // засыпаем, после пробуждения код начнет выполнятся со следующей строчки и сразу сменит состояние на WAKEUP_SENSORS
            esp_task_wdt_add(NULL);                                             // вновь подписываемся на мониторинг текущей Task

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