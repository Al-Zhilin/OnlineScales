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
#include "Calibration.h"
#include "InputHandler.h"
#include "Sim800LManager.h"
#include <esp_task_wdt.h>
#include <esp_sleep.h>

#define REF_WEIGHT 20000.0f                             // калибровочная масса для алгоритма подбора компенсации температурного дрейфа

#define SCL_PIN 18                                      // SCL пин HX711
#define DT_PIN 8                                        // DT пин HX711
#define DS_PIN 47                                       // пин DS18b20
#define BUTT_PIN 1                                      // пин кнопки
#define RED_PIN 11                                      // красный цвет RGB светодиода
#define GREEN_PIN 12                                    // зеленый цвет
#define BLUE_PIN 13                                     // пин кнопки
#define CALIB_SWITCH_PIN 2                              // пин переключателя режимов работы
#define MODEM_PWR_PIN 5                                 // пин, управляющий транзистором питания модема
#define MODEM_RST_PIN 4                                 // пин, подключенный к RST SIM800L
#define MODEM_TX_PIN 17                                 // пин, подключенный к TX модема
#define MODEM_RX_PIN 16                                 // пин, подключенный к RX модема
#define WDT_TIMEOUT_MS 25000                            // период для WDT (миллисекунды)
#define SLEEP_TIME_SEC 30                               // время сна между измерениями (секунды)
#define BATT_PIN 9                                      // пин чтения напряжения батареи
#define R1 100000.0f                                    // Резистор от плюса батареи к пину АЦП
#define R2 100000.0f                                    // Резистор от пина АЦП к земле
#define DIVIDER_RATIO ((R1 + R2) / R2)                  // Вычисляем коэффициент делителя
#define SENSOR_ERROR_THRESHOLD 10                       // Сколько ошибочных циклов работы с датчиками должно произойти, чтобы ESP перезагрузилась
#define DATA_RETRY_PERIOD 5*60*1000UL                   // Интервал повторного вызова длинного цикла при возниконовении проблем с выполнением сетевых операцих

constexpr uint32_t DATA_SEND_PERIOD = 15*60*1000UL;     // период отправки данных в нормальном режиме работы (первое число - минуты)

SystemState currentState = SystemState::WAKEUP_SENSORS;                              // текущее состояние FSM
ModificationRequests external_request;                  // внешние вмешательства в FSM

float batteryVoltage = 0.0f;                               // Текущее напряжение батареи
uint8_t restart_reason = 0;                                // см. использование ниже
String modemPayload = "";                                  // Буфер для сформированного запроса
String serverResponse = "";                                // Буфер для ответа от VK API
bool hasModemError = false;                                // Флаг для безопасного отключения при ошибках
SystemState postModemState = SystemState::SLEEP_SENSORS;   // В какое состояние FSM шагать после выполнения цикла работы с модемом (может быть: SLEEP_SENSORS или CALIBRATION) в зависимости от назначения запроса
uint8_t sensor_error_count = 0;                            // Счетчик подряд идущих ошибок датчиков


// --- ПЕРЕМЕННЫЕ, ВЫЖИВАЮЩИЕ ПРИ ПЕРЕЗАГРУЗКЕ И СНЕ (RTC) ---
RTC_DATA_ATTR bool is_retry_mode = false;                  // Флаг режима "повтора" после ошибки связи (управляет периодом между длинными циклами главного FSM)
RTC_DATA_ATTR uint8_t rtc_error_mask = 0;                  // Битовая маска накопленных ошибок
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
    Serial.begin(9600);                                     // чисто для отладки
    Serial1.begin(9600, SERIAL_8N1, 16, 17);                // поднимаем Serial1 дял работы с модемом на пинах 16 и 17
    inputHanler.begin();

    modemPayload.reserve(1024);                             // Избегаем излишних проблем со String: заранее резервируем память

    Config ModemConfig;
    ModemConfig.pwr_pin = MODEM_PWR_PIN;                                // пин управления питанием SIM800L
    ModemConfig.rst_pin = MODEM_RST_PIN;                                // пин, управляющий RST модема
    ModemConfig.tx_pin = MODEM_TX_PIN;                                  // пин TX от SIM800L
    ModemConfig.rx_pin = MODEM_RX_PIN;                                  // пин RX от SIM800L

    modemManager.begin(ModemConfig);

    led.begin();
    tempSensor.begin();

    if (!LittleFS.begin(true)) {                            // инициализируем LittleFS (ляжет под капот FileData)
        LOG("CRITICAL ERROR: SPIFFS Mount Failed!");
    }

    scales.begin();               
    calibrator.begin();

    restart_reason = (uint8_t)esp_reset_reason();      // причина завешения предыдущей работы

    esp_task_wdt_config_t twdt_config = {              // настраиваем конфиг для WDT таймера
      .timeout_ms = WDT_TIMEOUT_MS,                         // период перед резетом
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,      // Мониторим все ядра
      .trigger_panic = true,                                // true = перезагрузка при зависании
    };

    esp_task_wdt_reconfigure(&twdt_config);            // применяем новые настройки конфига
    esp_task_wdt_add(NULL);                            // подписываем loopTask() на мониторинг WDT
}

void loop() {
    led.tick();                                        // тикер светодиода: важен всегда, индикация пользователям не зависит от состояния FSM
    inputHanler.tick();                                // аналогично и с input
    esp_task_wdt_reset();
    static uint32_t sendState_timer = millis();

    if (external_request.tare)  changeFSMState(SystemState::TARE_PROCESS);          // по событию вызываем тарирование, оно потом самостоятельно откатит current_state на состояние до вызова
    
    switch (currentState) {
        case SystemState::WAKEUP_SENSORS:                  // пробуждаем hx711, переходим к измерениям
            scales.sleepMode(false);
            changeFSMState(SystemState::MEASURE);
            break;

        case SystemState::MEASURE: {                       // измерения температуры и веса                  
            ScalesState s_state = scales.tick();           // после пробуждения hx711 еще примерно 400мс настраивается и делает первое измерение - проверяем готовность перед чтением, иначе - мусор/старые значения! (Но обязательно с защитой от зависания!!)
            TempState t_state = tempSensor.tick();         // обновляем и температуру

            if (s_state == ScalesState::BUSY || t_state == TempState::BUSY) break;

            // Быстрая диагностика датчиков на возможные неисправности:
            bool current_sensor_error = false;
            // 1. Temp Sensor
            if (t_state == TempState::ERROR) {
                current_sensor_error = true;
                rtc_error_mask |= (1 << 0);                // Ошибка DS18b20 - бит 0 в маске
            }

            // 2. Scales Sensor
            if (s_state == ScalesState::ERROR) {
                current_sensor_error = true;
                rtc_error_mask |= (1 << 1);                // Записываем ошибку HX711 в бит 1
            }
            
            if (current_sensor_error) {                                             
                sensor_error_count++;
                if (sensor_error_count >= SENSOR_ERROR_THRESHOLD) {
                    changeFSMState(SystemState::ERROR_HANDLING);
                    break;
                }
            } else {
                sensor_error_count = 0;
            }

            batteryVoltage = filtrateVolts(analogReadMilliVolts(BATT_PIN)) / 1000.0f * DIVIDER_RATIO;            // читаем напряжение с батареи, фильтруем простым EMA с адаптивным коэффициентом

            if (external_request.start_calibration) {                                    // пользователь переключил на режим калибровки
                external_request.start_calibration = false;
                calibrator.startCalibration();
                LOG("Calibration started");
                
                String msg = "Переключатель переведен в режим калибровки";
                modemPayload = "peer_ids=";
                modemPayload += VK_PEER_ID; // Макросы подставятся без создания объекта String
                modemPayload += "&random_id=";
                modemPayload += String(esp_random() & 0x7FFFFFFF);
                modemPayload += "&v=5.199&access_token=";
                modemPayload += VK_TOKEN;
                modemPayload += "&message=";
                modemPayload += msg;
                
                postModemState = SystemState::SLEEP_SENSORS;                                                     // После отправки в ВК модем вернет нас в сон!
                changeFSMState(SystemState::START_MODEM);
            }
            else if (external_request.end_calibration) {
                external_request.end_calibration = false;
                calibrator.finishCalibration();
                LOG("Калибровка завершена!");
                
                String msg = "Калибровка завершена:%0A";
                if (calibData.calibrated) {
                    msg += "Выбрана модель: " + String(calibData.modelType) + "%0A";
                    msg += "Данные сохранены в память";
                } else {
                    msg += "Недостаточно данных, модель не сохранена";
                }

                modemPayload = "peer_ids=";
                modemPayload += VK_PEER_ID; // Макросы подставятся без создания объекта String
                modemPayload += "&random_id=";
                modemPayload += String(esp_random() & 0x7FFFFFFF);
                modemPayload += "&v=5.199&access_token=";
                modemPayload += VK_TOKEN;
                modemPayload += "&message=";
                modemPayload += msg;
                
                postModemState = SystemState::SLEEP_SENSORS;
                changeFSMState(SystemState::START_MODEM);
                break;
            }

            if (calibrator.isCalibratingMode()) calibrator.calibrationStep();

            // выбор: если ранее были проблемы с модемом - попробуем еще раз запустить его по маленькому таймауту, если все было ок - то по стандартному
            if (millis() - sendState_timer >= (is_retry_mode ? DATA_RETRY_PERIOD : DATA_SEND_PERIOD) || external_request.force_send) {               // в данном цикле пришло время/нужно принудительно отправлять данные               
                if (external_request.force_send) {
                    external_request.force_send = false;
                }

                hasModemError = false;

                String compact_errors = "";           // строка, содержащая наглядное представление содержания битовой маски ошибок
                compact_errors.reserve(8);
                if (rtc_error_mask != 0) {
                    if (rtc_error_mask & (1 << 0)) compact_errors += "1";       // Ошибка DS18B20
                    if (rtc_error_mask & (1 << 1)) compact_errors += "2";       // Ошибка HX711
                    if (rtc_error_mask & (1 << 2)) compact_errors += "3";       // Ошибка Init Модема
                    if (rtc_error_mask & (1 << 3)) compact_errors += "4";       // Ошибка Сети/HTTP
                    if (rtc_error_mask & (1 << 4)) compact_errors += "5";       // Был Hard Reset
                }

                // Формируем текст сообщения (с URL-кодированием переноса строки %0A)   
                String msg = "";
                msg.reserve(512);
                msg += "Отчет от весов:%0A";

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
                     msg += "%0AErrors: " + compact_errors;            // пользователь увидет только коды ошибок = циферки, которые легко расшифровываются
                }

                // Упаковываем в формат x-www-form-urlencoded для VK API ---
                modemPayload = "peer_ids=";
                modemPayload += VK_PEER_ID; // Макросы подставятся без создания объекта String
                modemPayload += "&random_id=";
                modemPayload += String(esp_random() & 0x7FFFFFFF);
                modemPayload += "&v=5.199&access_token=";
                modemPayload += VK_TOKEN;
                modemPayload += "&message=";
                modemPayload += msg;

                postModemState = SystemState::ERROR_HANDLING;        // Стандартный цикл после отпарвки данных модемом всегда ведет к обработке возможных ошибок
                sendState_timer = millis();
                changeFSMState(SystemState::START_MODEM);
                LOG("Starting Modem states...");
            }
            else changeFSMState(SystemState::SLEEP_SENSORS);
            break;
        }

        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------
        case SystemState::START_MODEM:  {
            led.setMode(LedModes::WAIT);             // показываем, что система в процессе работы
            ModemStatus status = modemManager.processInit();
            
            if (status == ModemStatus::BUSY) break;  // BUSY - показатель процесса работы функции
            
            if (status == ModemStatus::SUCCESS || status == ModemStatus::SUCCESS_WITH_RESTARTS) {               // успех или успех с рестартами (можно использовать для отладки) переводит нас к отправке данных
                LOG("Modem initialized successfully.");
                changeFSMState(SystemState::DATA_SEND);
            } else {                                                                                            // неуспех - к выключению модема
                LOG("Modem initialization failed!");
                hasModemError = true;
                rtc_error_mask |= (1 << 2);                     // бит ошибки, показывающий сбой старта модема
                changeFSMState(SystemState::SLEEP_MODEM);
            }
            break;
        }
        case SystemState::DATA_SEND:    {
            ModemStatus status = modemManager.processRequest(modemPayload, serverResponse);                     // передаем "нагрузку" = наполнение запроса и переменную, куда попадет ответ от сервера (пока не используется)
            
            if (status == ModemStatus::BUSY) break;
            
            if (status == ModemStatus::SUCCESS) {
                LOG("Data sent successfully!");
                led.setMode(LedModes::OK);
            } else {
                LOG("Data send error!");
                led.setMode(LedModes::ERROR);
                hasModemError = true;
                rtc_error_mask |= (1 << 3);                     // бит, показывающий ошибку с сетевым этапом работы модема
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
                changeFSMState(postModemState); 
            }
            
            postModemState = SystemState::SLEEP_SENSORS;
            break;
        }
        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------


        case SystemState::ERROR_HANDLING: {                 // обработка ошибок
            if (rtc_error_mask == 0) {                                    // полный успех! Возвращаем все переменные в нормальное состояние
                is_retry_mode = false;
                consecutive_errors = 0;
                reboot_budget = 3;
                sensor_error_count = 0;
            } 
            else {
                led.setMode(LedModes::ERROR);                             // наличествуют ошибки?? Мигаем!!
                consecutive_errors++;

                // Если ошибка связана с МОДЕМОМ (биты 2 или 3)
                if (rtc_error_mask & (1 << 2) || rtc_error_mask & (1 << 3)) {
                    if (reboot_budget > 0) {                              // пока не исчерпали перезагрузки - пытаемся запуститься каждые 5 минут
                        is_retry_mode = true;                                                 
                        LOG("Ошибка связи. Режим повтора через 5 минут.");
                    } else {                                              // если "жизней" нет - переходим в стандартный режим, более экономичный и ждем чуда
                        is_retry_mode = false;
                        LOG("Бюджет жизней исчерпан. Удрученный режим: ждем чуда раз в 15 минут.");
                    }
                }
                
                // Обработка ошибок датчиков
                if (sensor_error_count >= SENSOR_ERROR_THRESHOLD) {
                    if (reboot_budget > 0) {
                        LOG("Датчики лежат больше допустимого периода. Аппаратный сброс...");
                        reboot_budget--;
                        rtc_error_mask |= (1 << 4);         // Бит Hard Reset
                        delay(500);                         // для спокойствия насчет успеения записи rtc переменной
                        ESP.restart();
                    }
                    else {
                        LOG("Ошибка датчиков, но перезагрузок больше не осталось");
                    }
                }

                // Логика бюджета перезагрузок (раз в 3 полных провала модема)
                if (consecutive_errors % 3 == 0 && reboot_budget > 0) {
                    reboot_budget--;
                    rtc_error_mask |= (1 << 4);                           // записываем соответствующий бит
                    delay(100);                                           // для уверенности, что все успеет сохраниться
                    ESP.restart();
                }
            }

            rtc_error_mask = 0; // Очищаем для следующего цикла накопления
            changeFSMState(SystemState::SLEEP_SENSORS);
            
            postModemState = SystemState::SLEEP_SENSORS; // Сбрасываем маршрут
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



        case SystemState::TARE_PROCESS:
        {   
            external_request.tare = false;
            ScalesState tare_state = scales.sensorTare();
            if (tare_state == ScalesState::SUCCESS)    led.setMode(LedModes::OK);
            else if (tare_state == ScalesState::ERROR)   led.setMode(LedModes::ERROR);
            changeFSMState(SystemState::PREV_STATE);                                                // возвращаемся в то состояние, откуда было вызвано тарирование
            break;
        }
        
        case SystemState::PREV_STATE:                 // чтобы компилятор не ругался на этот state
            break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);              // вряяяяядли нужно, уберем после тестов
}