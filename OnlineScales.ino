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

#include <LittleFS.h>
#include <FileData.h>
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

#define SCL_PIN 18                                      // SCL пин HX711
#define DT_PIN 8                                        // DT пин HX711
#define DS_PIN 47                                       // пин DS18b20
#define BUTT_PIN 1                                      // пин кнопки
#define RED_PIN 11                                      // красный цвет RGB светодиода
#define GREEN_PIN 12                                    // зеленый цвет
#define BLUE_PIN 13                                     // пин кнопки
#define CALIB_SWITCH_PIN 2                              // пин переключателя режимов работы
#define WDT_TIMEOUT_MS 25000                            // период для WDT (миллисекунды)
#define SLEEP_TIME_SEC 30                               // время сна между измерениями (секунды)
#define BATT_PIN 9                                      // Любой свободный ADC пин (например, GPIO 1, 2, 3...)
#define R1 100000.0f                                    // Резистор от плюса батареи к пину АЦП
#define R2 100000.0f                                    // Резистор от пина АЦП к земле
#define DIVIDER_RATIO ((R1 + R2) / R2)                  // Вычисляем коэффициент делителя
#define SENSOR_ERROR_THRESHOLD 10                       // Сколько ошибочных циклов работы с датчиками должно произойти, чтобы ESP перезагрузилась
#define DATA_RETRY_PERIOD 5*60*1000UL                   // Интервал повторного вызова длинного цикла при возниконовении проблем с выполнением сетевых операцих

constexpr uint32_t DATA_SEND_PERIOD = 15*60*1000UL;     // период отправки данных в нормальном режиме работы (первое число - минуты)

SystemState currentState = SystemState::WAKEUP_SENSORS;
ModificationRequest external_request = ModificationRequest::FORCE_SEND;              // чтобы при запуске модуль сразу сделал запрос, нужно для проверки соединения и мониторинга

float batteryVoltage = 0.0f;                               // Текущее напряжение батареи
uint8_t restart_reason = 0;                                // см. использование ниже
String modemPayload = "";                                  // Буфер для сформированного запроса
String serverResponse = "";                                // Буфер для ответа от VK API
bool hasModemError = false;                                // Флаг для безопасного отключения при ошибках

// --- ПЕРЕМЕННЫЕ, ВЫЖИВАЮЩИЕ ПРИ ПЕРЕЗАГРУЗКЕ И СНЕ (RTC) ---
RTC_DATA_ATTR bool is_retry_mode = false;                 // Флаг режима "повтора" после ошибки связи
RTC_DATA_ATTR uint8_t sensor_error_count = 0;              // Счетчик подряд идущих ошибок датчиков
RTC_DATA_ATTR uint8_t rtc_error_mask = 0;                  // Битовая маска накопленных ошибок текущего цикла
RTC_DATA_ATTR uint8_t consecutive_errors = 0;              // Счетчик циклов подряд, в которых были ошибки
RTC_DATA_ATTR uint8_t reboot_budget = 3;                   // Бюджет перезагрузок (см. соответствующий паттерн поведения)


AsyncLed led(RED_PIN, GREEN_PIN, BLUE_PIN);
ScalesManager scales(11.472, DT_PIN, SCL_PIN);
ScaleAutoCalibrator calibrator(REF_WEIGHT);
TempManager tempSensor(DS_PIN);
InputHandler inputHanler(BUTT_PIN, CALIB_SWITCH_PIN, calibrator, external_request, currentState);
Sim800LManager modemManager;

void changeFSMState(SystemState newState) {                // функция переключения состояния FSM. С помощью PREV_STATE помогает "гостить" в определенном состоянии и возвращаться обратно, нужно в некоторых случаях
    static SystemState previousState = SystemState::WAKEUP_SENSORS;

    if (newState == SystemState::PREV_STATE)    currentState = previousState;
    else {
        previousState = currentState;
        currentState = newState;
    }
}

void setup() {
    Serial.begin(9600);
    // Явно указываем: Скорость, Режим (8 бит, нет четности, 1 стоп-бит), RX-пин, TX-пин
    Serial1.begin(115200, SERIAL_8N1, 16, 17);
    inputHanler.begin();

    modemPayload.reserve(1024);                             // Избегаем излишних проблем со String: заранее резервируем память
    ModemConfig.pwr_pin = 5;                                // пин управления питанием SIM800L
    ModemConfig.rst_pin = 4;                                // пин, управляющий RST модема
    ModemConfig.tx_pin = 17;                                // пин TX от SIM800L
    ModemConfig.rx_pin = 16;                                // пин RS от SIM800L

    modemManager.begin(ModemConfig);

    led.begin();
    tempSensor.begin();

    // --- ИНИЦИАЛИЗАЦИЯ ФАЙЛОВОЙ СИСТЕМЫ ESP32 ---
    // true означает, что при первом запуске (если ФС не найдена) она будет автоматически отформатирована
    if (!LittleFS.begin(true)) {
        LOG("CRITICAL ERROR: SPIFFS Mount Failed!");
    }

    scales.begin();               
    calibrator.begin();

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

       case SystemState::MEASURE: {
            if (!scales.isReady()) break;

            scales.tick();
            tempSensor.tick();

            // --- БЛОК А: Быстрая диагностика датчиков ---
            bool current_sensor_error = false;
            if (sensorData.tempC <= -100.0f || sensorData.tempC == 85.0f) {
                current_sensor_error = true;
                rtc_error_mask |= (1 << 0);
            }
            // Здесь будет проверка HX711 (бит 1)

            if (current_sensor_error) {
                sensor_error_count++;
                if (sensor_error_count >= SENSOR_ERROR_THRESHOLD) {
                    if (reboot_budget > 0) {
                        LOG("Датчики лежат больше допустимого периода. Аппаратный сброс...");
                        reboot_budget--;
                        rtc_error_mask |= (1 << 4); // Бит Hard Reset
                        delay(500);
                        ESP.restart();
                    }
                }
            } else {
                sensor_error_count = 0;
            }

            uint32_t current_period = is_retry_mode ? DATA_RETRY_PERIOD : DATA_SEND_PERIOD;                      // выбор: если ранее были проблемы с модемом - попробуем еще раз запустить его по маленькому таймауту, если все было ок - то по стандартному

            batteryVoltage = filtrateVolts(analogReadMilliVolts(BATT_PIN)) / 1000.0f * DIVIDER_RATIO;            // читаем напряжение с батареи, фильтруем простым EMA

            if (external_request == ModificationRequest::START_CALIBRATION) {                   
                external_request = ModificationRequest::NONE;
                changeFSMState(SystemState::CALIBRATION);
                calibrator.startCalibration();
                LOG("Calibration started");
            }

            else if (millis() - sendState_timer >= current_period || external_request == ModificationRequest::FORCE_SEND) {               // в данном цикле пришло время/нужно принудительно отправлять данные               
                if (external_request == ModificationRequest::FORCE_SEND) {
                    external_request = ModificationRequest::NONE;
                }

                hasModemError = false;

                // --- 1. Формируем компактную строку ошибок из битовой маски ---
                String compact_errors = "";
                compact_errors.reserve(8);
                if (rtc_error_mask != 0) {
                    if (rtc_error_mask & (1 << 0)) compact_errors += "1"; // Ошибка DS18B20
                    if (rtc_error_mask & (1 << 1)) compact_errors += "2"; // Ошибка HX711
                    if (rtc_error_mask & (1 << 2)) compact_errors += "3"; // Ошибка Init Модема
                    if (rtc_error_mask & (1 << 3)) compact_errors += "4"; // Ошибка Сети/HTTP
                    if (rtc_error_mask & (1 << 4)) compact_errors += "5"; // Был Hard Reset
                }

                // --- 2. Формируем текст сообщения (с URL-кодированием переноса строки %0A) ---        
                String msg = "";
                msg.reserve(512);
                msg += "Отчет с весов:%0A";

                msg += "Текущий вес: ";
                msg += (calibrator.isCalibratingMode()) ? String(sensorData.weightKg, 2) : String(calibrator.compensate(sensorData.tempC, sensorData.weightGr) / 1000.0f, 2);
                msg += " кг%0A";

                msg += "Температура: " + String(sensorData.tempC, 1) + " °C%0A";

                if (calibrator.isCalibratingMode()) {
                    msg += "Вес с компенсацией: " + String(calibrator.compensate(sensorData.tempC, sensorData.weightGr) / 1000.0f, 2) + " кг%0A";
                    msg += "Неуверенность калибратора: " + String(calibrator.getUncertainty(), 4) + "%0A";
                }

                msg += "Напряжение батареи: " + String(batteryVoltage) + "В";

                // Вставляем ошибки ТОЛЬКО если они были
                if (compact_errors.length() > 0) {
                     msg += "%0A Errors: " + compact_errors;
                }

                // --- 3. Упаковываем в формат x-www-form-urlencoded для VK API ---
                modemPayload = "user_id=" + String(VK_USER_ID) + 
                               "&random_id=" + String(esp_random()) +  
                               "&v=5.199" + 
                               "&access_token=" + String(VK_TOKEN) + 
                               "&message=" + msg;

                changeFSMState(SystemState::START_MODEM);
                sendState_timer = millis();
                LOG("Starting Modem states...");
            }
            else changeFSMState(SystemState::SLEEP_SENSORS);

            break;
        }

        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------
        case SystemState::START_MODEM:  {
            led.setMode(LedModes::WAIT);             // показываем, что система в процессе работы
            ModemStatus status = modemManager.processInit();
            
            if (status == ModemStatus::BUSY) break;
            
            if (status == ModemStatus::SUCCESS || status == ModemStatus::SUCCESS_WITH_RESTARTS) {
                LOG("Modem initialized successfully.");
                changeFSMState(SystemState::DATA_SEND);
            } else {
                LOG("Modem initialization failed!");
                hasModemError = true;                
                rtc_error_mask |= (1 << 2); 
                changeFSMState(SystemState::SLEEP_MODEM);
            }
            break;
        }

        case SystemState::DATA_SEND:    {
            ModemStatus status = modemManager.processRequest(modemPayload, serverResponse);
            
            if (status == ModemStatus::BUSY) break;
            
            if (status == ModemStatus::SUCCESS) {
                LOG("Data sent to VK successfully!");
                led.setMode(LedModes::OK);
            } else {
                LOG("Network/HTTP Error!");
                led.setMode(LedModes::ERROR);
                hasModemError = true;
                rtc_error_mask |= (1 << 3); 
            }
            changeFSMState(SystemState::SLEEP_MODEM);
            break;
        }

        case SystemState::SLEEP_MODEM:  {
            ModemStatus status = modemManager.processPowerOff();
            
            if (status == ModemStatus::BUSY) break; 
            
            LOG("Modem powered off.");
            if (hasModemError) {
                changeFSMState(SystemState::ERROR_HANDLING);
            } else {
                changeFSMState(SystemState::SLEEP_SENSORS);
            }
            break;
        }
        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------


        case SystemState::ERROR_HANDLING: {
            if (rtc_error_mask != 0) led.setMode(LedModes::ERROR);

            if (rtc_error_mask == 0) {
                // ПОЛНЫЙ УСПЕХ
                is_retry_mode = false;  // Возвращаемся к долгому ожиданию (60 мин)
                consecutive_errors = 0;
                reboot_budget = 3;
                sensor_error_count = 0;
            } 
            else {
                // ЕСТЬ ОШИБКИ
                consecutive_errors++;

                // Если ошибка связана с МОДЕМОМ (биты 2 или 3)
                if (rtc_error_mask & (1 << 2) || rtc_error_mask & (1 << 3)) {
                    is_retry_mode = true; // Включаем режим повтора через 5 минут
                    LOG("Ошибка связи. Следующая попытка через 5 минут.");
                }

                // Логика бюджета перезагрузок (раз в 3 полных провала модема)
                if (consecutive_errors % 3 == 0 && reboot_budget > 0) {
                    reboot_budget--;
                    rtc_error_mask |= (1 << 4);
                    delay(500);
                    ESP.restart();
                }
            }

            rtc_error_mask = 0; // Очищаем для следующего цикла накопления
            changeFSMState(SystemState::SLEEP_SENSORS);
            break;
        }

        case SystemState::SLEEP_SENSORS:
            if (!led.tick()) break;                                             // пока не завершили индикацию - не уходим спать

            scales.sleepMode(true);
            esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * 1000000ULL);         // настраиваемся на здоровый сон на заданное время
            esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTT_PIN, 0);              // пробуждение от нажатия кнопки
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

    vTaskDelay(10 / portTICK_PERIOD_MS);
}