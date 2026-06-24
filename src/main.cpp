#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include "Enumerations.h"
#include <LittleFS.h>
#include <FileData.h>
#include <uButton.h>
#include "Config.h"
#include "Sensors.h"
#include "Secrets/Secrets.h"
#include "Led_UI.h"
#include "Calibration.h"
#include "InputHandler.h"
#include "Sim800LManager.h"

SensorData sensorData;     // определение глобала, объявленного в Sensors.h

SystemState currentState = SystemState::WAKEUP_SENSORS; // текущее состояние FSM
ModificationRequests external_request;                  // внешние вмешательства в FSM

float batteryVoltage = 0.0f;                               // Текущее напряжение батареи
uint8_t restart_reason = 0;                                // см. использование ниже
String modemPayload = "";                                  // Буфер для сформированного запроса
String serverResponse = "";                                // Буфер для ответа от VK API
bool hasModemError = false;                                // Флаг для безопасного отключения при ошибках
bool modem_cycle_completed = false;                        // Флаг, работал ли модем в этом цикле
SystemState postModemState = SystemState::SLEEP_SENSORS;   // В какое состояние FSM шагать после выполнения цикла работы с модемом (может быть: SLEEP_SENSORS или CALIBRATION) в зависимости от назначения запроса
uint8_t sensor_error_count = 0;                            // Счетчик подряд идущих ошибок датчиков


// --- ПЕРЕМЕННЫЕ, ВЫЖИВАЮЩИЕ ПРИ ПЕРЕЗАГРУЗКЕ И СНЕ (RTC) ---
RTC_DATA_ATTR uint32_t rtc_magic = 0;              // маркер валидности RTC-памяти: 0 = первый старт или повреждение
RTC_DATA_ATTR bool is_retry_mode = false;                  // Флаг режима "повтора" после ошибки связи (управляет периодом между длинными циклами главного FSM)
RTC_DATA_ATTR uint8_t rtc_error_mask = 0;                  // Битовая маска накопленных ошибок
RTC_DATA_ATTR uint8_t consecutive_errors = 0;              // Счетчик циклов подряд, в которых были ошибки
RTC_DATA_ATTR uint8_t reboot_budget = 3;                   // Бюджет перезагрузок (см. соответствующий паттерн поведения)
RTC_DATA_ATTR uint8_t rtc_crash_step = 0;                  // Черный ящик на случай крашев или сбоев модема
RTC_DATA_ATTR uint8_t rtc_main_fsm_state = 0;              // И для главного конечного автомата
RTC_DATA_ATTR uint8_t rtc_reboot_cause = 0;                // Причина последнего ESP.restart() (см. enum RebootCause); печатается в теге как RC:N
RTC_DATA_ATTR uint8_t rtc_problem_mask = 0;                // Накопитель проблем для отчёта: переживает цикл, чтобы ошибки/подавленные ребуты гарантированно попали в тег ВК (биты 0-5 как rtc_error_mask, 6=ребут подавлен в калибровке, 7=сбой LittleFS)
RTC_DATA_ATTR CalibLiveState rtc_calib_snapshot;           // Снимок незавершённой калибровки: позволяет продолжить сессию после программной перезагрузки/сна (теряется только при полном обесточивании)


AsyncLed<LED_PIN> led(LED_SWITCH_PIN);
ScalesManager scales(6.7, DT_PIN, SCL_PIN);                // 11.472
AdaptiveRLS<double, true, 2> compensator;
TempManager tempSensor(DS_PIN);
InputHandler inputHandler(BUTT_PIN, CALIB_SWITCH_PIN, compensator, external_request, currentState);
Sim800LManager modemManager;


void changeFSMState(SystemState newState) {                // функция переключения состояния FSM. С помощью PREV_STATE помогает "гостить" в определенном состоянии и возвращаться обратно, нужно в некоторых случаях
    static SystemState previousState = SystemState::WAKEUP_SENSORS;

    if (newState == SystemState::PREV_STATE)    currentState = previousState;
    else {
        previousState = currentState;
    
        currentState = newState;
    }

    rtc_main_fsm_state = (uint8_t)currentState;
}

// Формирует тело отчёта ВК (вес/температура/калибровка/батарея) + мета-тег диагностики.
// Тег "[SC:N RC:N M:N/N E:HH RB:N]" одноразовый: потребляет restart_reason/rtc_reboot_cause/rtc_problem_mask.
// Вынесено в функцию, чтобы переиспользовать в обычном отчёте и в объединённом стартовом сообщении калибровки.
String buildReportMessage(TempState t_state) {
    String msg = "";
    msg.reserve(512);
    msg += "Отчет от весов:%0A";

    // При ошибке датчика температуры не компенсируем вес некорректным значением — отправляем сырое.
    msg += "Текущий вес: ";
    msg += (t_state != TempState::ERROR && !compensator.isCalibratingMode()) ? String(compensator.compensate(sensorData.weightGr, sensorData.tempC) / 1000.0f, 2) : String(sensorData.weightKg, 2);
    msg += " кг%0A";

    msg += "Температура: " + String(sensorData.tempC, 1) + " °C%0A";

    if (compensator.isCalibratingMode()) {
        msg += "Вес с компенсацией: " + String(compensator.getCompensation(sensorData.weightGr, sensorData.tempC) / 1000.0f, 2) + " кг%0A";
        msg += "Неуверенность модели: " + String(compensator.getUncertainty(), 1) + "%25%0A";
        msg += "Точек: " + String(compensator.getNumSamples()) + ", диапазон: " + String(compensator.getCalibrationDelta(), 1) + " C%0A";
    }

    msg += "Напряжение батареи: " + String(batteryVoltage) + "В";

    // Мета-тег: [SC:N RC:N M:N/N E:HH RB:N]
    // SC=причина старта, RC=причина последнего ребута, M=FSM/шаг модема, E(hex)=маска проблем, RB=бюджет рестартов
    {
        char tag[64]; int n = 0; bool any = false;
        if (restart_reason != 99) {
            bool crash = restart_reason && restart_reason != 1;   // причину 3 (SW) больше НЕ исключаем — печатаем шаги FSM/модема и для неё
            n += snprintf(tag + n, sizeof(tag) - n, "SC:%u", (unsigned)restart_reason);
            if (crash) n += snprintf(tag + n, sizeof(tag) - n, " M:%u/%u", (unsigned)rtc_main_fsm_state, (unsigned)rtc_crash_step);
            if (rtc_reboot_cause) {
                n += snprintf(tag + n, sizeof(tag) - n, " RC:%u", (unsigned)rtc_reboot_cause);
                rtc_reboot_cause = 0;
            }
            restart_reason = 99;
            any = true;
        }
        uint8_t probs = rtc_problem_mask | rtc_error_mask;   // накопленные за прошлый цикл + проблемы текущего цикла
        if (probs) {
            n += snprintf(tag + n, sizeof(tag) - n, any ? " E:%02X" : "E:%02X", (unsigned)probs);
            any = true;
        }
        rtc_problem_mask = 0;   // унесли в сообщение — обнуляем накопитель
        if (any) {
            snprintf(tag + n, sizeof(tag) - n, " RB:%u", (unsigned)reboot_budget);
            msg += "%0A["; msg += tag; msg += ']';
        }
    }

    return msg;
}

void setup() {
    Serial.begin(9600);

    // Проверяем целостность RTC-памяти: при первом включении или повреждении — инициализируем
    if (rtc_magic != 0xDEADBEEF) {
        rtc_magic          = 0xDEADBEEF;
        is_retry_mode      = false;
        rtc_error_mask     = 0;
        consecutive_errors = 0;
        reboot_budget      = 3;
        rtc_crash_step     = 0;
        rtc_main_fsm_state = 0;
        rtc_reboot_cause   = 0;
        rtc_problem_mask   = 0;
        rtc_calib_snapshot.magic = 0;           // снимок калибровки невалиден (холодный старт/обесточивание)
    }

    // Маску ошибок, «зависшую» с прошлой сессии (мы перезагрузились до её очистки в ERROR_HANDLING),
    // уносим в накопитель проблем (попадёт в тег) и обнуляем рабочую маску — иначе первый чистый цикл
    // после ребута был бы ошибочно засчитан как ошибочный (consecutive_errors++ без реальной ошибки).
    rtc_problem_mask |= rtc_error_mask;
    rtc_error_mask = 0;

    inputHandler.begin();

    modemPayload.reserve(1024);
    serverResponse.reserve(512);
    analogSetPinAttenuation(BATT_PIN, ADC_11db);

    Config ModemConfig;
    ModemConfig.pwr_pin = MODEM_PWR_PIN;                                // пин управления питанием SIM800L
    ModemConfig.rst_pin = MODEM_RST_PIN;                                // пин, управляющий RST модема
    ModemConfig.tx_pin = MODEM_TX_PIN;                                  // пин TX от SIM800L
    ModemConfig.rx_pin = MODEM_RX_PIN;                                  // пин RX от SIM800L

    modemManager.begin(ModemConfig);

    led.begin();
    tempSensor.begin();

    // LittleFS: несколько попыток монтирования. НЕ перезагружаемся при провале (иначе boot-loop,
    // который и уничтожает калибровку) — переходим в деградированный режим без долговременного хранения.
    bool fs_ok = false;
    for (int i = 0; i < 3 && !fs_ok; i++) {
        fs_ok = LittleFS.begin(true);                  // true = форматировать при первом сбое
        if (!fs_ok) { Serial.println("LittleFS mount failed, retrying..."); delay(300); }
    }
    if (!fs_ok) {
        rtc_problem_mask |= (1 << 7);                  // флаг сбоя ФС в телеметрию (бит 7)
        Serial.println("WARNING: LittleFS unavailable — running degraded (no persistence)");
    }

    scales.begin();
    compensator.setDebugOut(&Serial);
    compensator.setNormParams(20.0f, 25.0f);
    compensator.setVal2Threshold(0.05f);
    compensator.setInflationParams(1.0f, 999.0f, 25.0f);                  // странный метод, первым параметром 1.0f отключим эту механику пока что
    compensator.setMinDelta(5);
    compensator.setPfloorPercent(0.0004f);
    compensator.setMinSamples(30);
    compensator.setComplexityPenalties(1.10, 1.20);
    const float ema_alphas[2] = {0.97f, 0.997f};
    compensator.setEmaAlphas(ema_alphas, 2);
    compensator.setDriftBoost(200, 100);

    // Поднимаем сохранённую модель из flash на КАЖDOM старте — иначе в обычном режиме (без входа
    // в калибровку) compensate() молча отдаёт сырой вес: begin() раньше звался только при старте
    // калибровки. referenceVal1 здесь не важен (на compensate() не влияет); при входе в калибровку
    // begin(weightGr) перезадаст его, а restoreLiveState (RTC-resume) перезапишет состояние поверх.
    compensator.begin(0.0f);

    restart_reason = (uint8_t)esp_reset_reason();

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
    inputHandler.tick();                               // аналогично и с input
    esp_task_wdt_reset();
    static uint32_t sendState_timer = millis();
    static ScalesState s_state;
    static TempState t_state;

    /*
    String inpt = "";
    while (Serial.available()) {
      inpt += (char)Serial.read();
    }
    if (inpt.startsWith("Тарировать"))  external_request.tare = true;*/

    if (external_request.tare)  changeFSMState(SystemState::TARE_PROCESS);          // по событию вызываем тарирование, оно потом самостоятельно откатит current_state на состояние до вызова

    switch (currentState) {
        case SystemState::WAKEUP_SENSORS:                  // пробуждаем hx711, переходим к измерениям
            scales.sleepMode(false);
            led.begin();
            changeFSMState(SystemState::MEASURE);
            s_state = ScalesState::BUSY;
            t_state = TempState::BUSY;
            break;

        case SystemState::MEASURE: {                       // измерения температуры и веса
            if (s_state == ScalesState::BUSY) s_state = scales.tick();              // после пробуждения hx711 еще примерно 400мс настраивается и делает первое измерение - проверяем готовность перед чтением, иначе - мусор/старые значения! (Но обязательно с защитой от зависания!! и пропуском первых нескольких измерений после запуска проекта - мусорные/нестабильные)
            if (t_state == TempState::BUSY) t_state = tempSensor.tick();            // обновляем и температуру, с обработкой готовности и таймаута

            if (s_state == ScalesState::BUSY || t_state == TempState::BUSY) {
              //Serial.println(String((s_state == ScalesState::BUSY) ? "Ждем весы..." : "Ждем термометр..."));
              break;
            }    

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

            if (!digitalRead(LED_SWITCH_PIN)) {               // Для отладки! Печать в Serial всех параметров (вес, температура, напряжение) при включенном переключателе
              #ifdef USE_LOG
                USE_LOG.print("HX711: ");
                USE_LOG.print(sensorData.weightGr);
                USE_LOG.print(",  DS18b20: ");
                USE_LOG.print(sensorData.tempC);
                USE_LOG.print(",  Volts:");
                USE_LOG.println(batteryVoltage);
              #endif
            }

            if (external_request.start_calibration) {                                    // вход в режим калибровки (переключатель или старт платы)
                if (!sensorData.weightGr) break;                             // ждём первое ненулевое измерение

                // Если калибровку подняли НА СТАРТЕ платы (calib_check_resume) и в RTC лежит валидный
                // снимок незавершённой сессии — продолжаем с того же места. Иначе (ручное переключение
                // в рантайме или нет снимка) — начинаем калибровку с нуля.
                bool resumed = false;
                if (external_request.calib_check_resume && compensator.restoreLiveState(rtc_calib_snapshot)) {
                    resumed = true;
                    LOG("Калибровка восстановлена из RTC-снимка (продолжение после перезагрузки)");
                } else {
                    compensator.begin(sensorData.weightGr);                  // фиксируем референс = текущий вес; при повторном вызове обновляет referenceVal1
                    compensator.startCalibration();
                    compensator.saveLiveState(rtc_calib_snapshot);           // сразу фиксируем свежую пустую сессию, чтобы ребут в первые секунды не воскресил старую
                    LOG("Calibration started");
                }
                external_request.start_calibration = false;
                external_request.calib_check_resume = false;
                external_request.force_send = false;        // гасим force_send: иначе сразу следом ушёл бы второй (дублирующий) отчёт

                // Одно объединённое сообщение: заголовок калибровки + полный отчёт (вместо двух сообщений подряд)
                String msg = resumed
                    ? String("Калибровка продолжена после перезагрузки (RTC)%0A%0A")
                    : (String("Переключатель переведен в режим калибровки%0AЭталонный вес: ") + sensorData.weightGr + "%0A%0A");
                msg += buildReportMessage(t_state);

                modemPayload = "peer_ids=";
                modemPayload += VK_PEER_ID;
                modemPayload += "&random_id=";
                modemPayload += String(esp_random() & 0x7FFFFFFF);
                modemPayload += "&v=5.199&access_token=";
                modemPayload += VK_TOKEN;
                modemPayload += "&message=";
                modemPayload += msg;

                postModemState = SystemState::ERROR_HANDLING;                           // После отправки в ВК модем вернет нас в сон!
                changeFSMState(SystemState::START_MODEM);
                break;
            }
            else if (external_request.end_calibration) {
                bool calib_saved = compensator.finishCalibration();
                rtc_calib_snapshot.magic = 0;               // калибровка завершена — RTC-снимок больше не валиден (повторное включение начнёт с нуля)
                external_request.end_calibration = false;
                external_request.force_send = false;        // не допускаем случайную отправку данных сразу после завершения калибровки
                LOG("Калибровка завершена!");

                String msg = "Калибровка завершена:%0A";
                if (calib_saved) {
                    msg += compensator.getPolynomialString(true);
                    msg += "%0AДанные сохранены в память";
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

                postModemState = SystemState::ERROR_HANDLING;
                changeFSMState(SystemState::START_MODEM);
                break;
            }

            if (compensator.isCalibratingMode()) {
                compensator.calibrationStep(sensorData.weightGr, sensorData.tempC);
                compensator.saveLiveState(rtc_calib_snapshot);   // RTC-чекпоинт: переживёт ESP.restart()/сон/WDT, калибровка продолжится после перезагрузки
            }
            // выбор: если ранее были проблемы с модемом - попробуем еще раз запустить его по маленькому таймауту, если все было ок - то по стандартному
            if (millis() - sendState_timer >= (is_retry_mode ? DATA_RETRY_PERIOD : ((compensator.isCalibratingMode()) ? DATA_SEND_PERIOD_CALIB : DATA_SEND_PERIOD)) || external_request.force_send) {               // в данном цикле пришло время/нужно принудительно отправлять данные
                hasModemError = false;

                if (external_request.force_send) {
                    external_request.force_send = false;
                }

                // Формируем текст отчёта + мета-тег диагностики (общий хелпер, см. buildReportMessage)
                String msg = buildReportMessage(t_state);

                // Упаковываем в формат x-www-form-urlencoded для VK API ---
                modemPayload = "peer_ids=";
                modemPayload += VK_PEER_ID; // Макросы подставятся без создания объекта String
                modemPayload += "&random_id=";
                modemPayload += String(esp_random() & 0x7FFFFFFF);
                modemPayload += "&v=5.199&access_token=";
                modemPayload += VK_TOKEN;

                modemPayload += "&message=";
                modemPayload += msg;

                postModemState = SystemState::ERROR_HANDLING;        // Стандартный цикл после отправки данных модемом всегда ведет к обработке возможных ошибок
                sendState_timer = millis();
                changeFSMState(SystemState::START_MODEM);
                LOG("Starting Modem states...");
            }
            else {
                if (rtc_error_mask & 3) led.pushReport(LedModes::REP_SENS_ERR);
                else led.pushReport(LedModes::REP_SENS_OK);

                changeFSMState(SystemState::SLEEP_SENSORS);
            }
            break;
        }

        // ------------------- Особая часть цикла работы, вызывается по таймеру -------------------
        case SystemState::START_MODEM:  {
            ModemStatus status = modemManager.processInit();

            modem_cycle_completed = true;
            if (status == ModemStatus::BUSY_INIT) { led.setMode(LedModes::BREATH_INIT); break; }
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

            if (status == ModemStatus::BUSY_NET) { led.setMode(LedModes::BREATH_NET); break; }
            if (status == ModemStatus::BUSY_HTTP) { led.setMode(LedModes::BREATH_HTTP); break; }
            if (status == ModemStatus::BUSY) break;

            if (status == ModemStatus::SUCCESS) {
                LOG("Data sent successfully!");
            } else {
                LOG("Data send error!");
                hasModemError = true;
                // Парсинг ошибок сети:
                if (status == ModemStatus::ERR_PPP_TIMEOUT) rtc_error_mask |= (1 << 3);         // Сеть
                else rtc_error_mask |= (1 << 5);                                                // Сервер (API)
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

        case SystemState::ERROR_HANDLING: {
            // Во время калибровки перезагрузки ПОЛНОСТЬЮ запрещены: ESP.restart() уничтожил бы прогресс.
            // Проблемы при этом не замалчиваем — помечаем их в накопителе (бит 6) → уедут в тег ВК.
            bool calibrating = compensator.isCalibratingMode();

            if (rtc_error_mask == 0) {
                is_retry_mode = false;
                consecutive_errors = 0;
                reboot_budget = 3;
                sensor_error_count = 0;
            }
            else {
                consecutive_errors++;

                // Логика таймаутов: короткий цикл повтора при проблемах связи (петлю сохраняем как есть)
                if (rtc_error_mask & (1 << 2) || rtc_error_mask & (1 << 3) || rtc_error_mask & (1 << 5)) {
                    if (reboot_budget > 0) {
                        is_retry_mode = true;
                        LOG("Ошибка связи. Режим повтора по короткому таймауту");
                    } else {
                        is_retry_mode = false;
                        LOG("Бюджет перезегрузок исчерпан. Ждем чуда по стандартному таймауту");
                    }
                }

                if (sensor_error_count >= SENSOR_ERROR_THRESHOLD) {
                    if (calibrating) {
                        rtc_problem_mask |= (1 << 6);   // ребут подавлен (идёт калибровка) — только уведомляем тегом
                        LOG("Датчики лежат, но идёт калибровка — перезагрузка запрещена");
                    } else if (reboot_budget > 0) {
                        LOG("Датчики лежат больше допустимого периода. Аппаратный сброс...");
                        reboot_budget--;
                        rtc_error_mask |= (1 << 4);
                        rtc_problem_mask |= rtc_error_mask;                 // сохранить причину для тега после ребута
                        rtc_reboot_cause = (uint8_t)RebootCause::SENSORS_DOWN;
                        esp_task_wdt_reset();
                        delay(500);
                        ESP.restart();
                    }
                    else {
                        LOG("Ошибка датчиков, но перезагрузок больше не осталось");
                    }
                }

                if (consecutive_errors % 3 == 0 && reboot_budget > 0) {
                    if (calibrating) {
                        rtc_problem_mask |= (1 << 6);   // ребут подавлен (идёт калибровка) — только уведомляем тегом
                        LOG("Подряд идущие ошибки, но идёт калибровка — перезагрузка запрещена");
                    } else {
                        reboot_budget--;
                        rtc_error_mask |= (1 << 4);
                        rtc_problem_mask |= rtc_error_mask;                 // сохранить причину для тега после ребута
                        rtc_reboot_cause = (uint8_t)RebootCause::COMM_ERRORS;
                        esp_task_wdt_reset();
                        delay(100);
                        ESP.restart();
                    }
                }
            }

            if (rtc_error_mask & 3) led.pushReport(LedModes::REP_SENS_ERR);
            else led.pushReport(LedModes::REP_SENS_OK);

            if (modem_cycle_completed) {                                                                  // если в этом цикле модем работал - запускаем индикацию результатов
                if (rtc_error_mask & (1 << 5))      led.pushReport(LedModes::REP_MOD_ERR_SERV);
                else if (rtc_error_mask & (1 << 3)) led.pushReport(LedModes::REP_MOD_ERR_NET);
                else if (rtc_error_mask & (1 << 2)) led.pushReport(LedModes::REP_MOD_ERR_HW);
                else                                led.pushReport(LedModes::REP_MOD_OK);

                modem_cycle_completed = false; // Сбрасываем для следующего прохода
            }

            rtc_problem_mask |= rtc_error_mask;   // унести проблемы цикла в накопитель, чтобы они попали в следующий тег (особенно когда ребут подавлен в калибровке)
            rtc_error_mask = 0;
            changeFSMState(SystemState::SLEEP_SENSORS);
            postModemState = SystemState::SLEEP_SENSORS;
            break;
        }

        case SystemState::SLEEP_SENSORS:
            if (!led.tick()) break;
            led.powerOff();

            scales.sleepMode(true);

            // ESP_SLEEP_WAKEUP_ALL безусловно сбрасывает все источники пробуждения без проверки
            // предыдущего состояния — в отличие от конкретного источника (GPIO, TIMER и т.д.),
            // которые падают с ошибкой "Incorrect wakeup source to disable", если ещё не были включены.
            // После сброса заново включаем всё нужное.
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * 1000000ULL);

            // Для кнопки: регистрируем только если HIGH (иначе — немедленный wake и busy-loop).
            // Для переключателей: регистрируем всегда, но полярность по текущему состоянию:
            //   HIGH → LOW_LEVEL (ловим включение), LOW → HIGH_LEVEL (ловим выключение).
            {
                bool any_gpio = false;

                if (digitalRead(BUTT_PIN)) {
                    gpio_wakeup_enable((gpio_num_t)BUTT_PIN, GPIO_INTR_LOW_LEVEL);
                    any_gpio = true;
                }

                // CALIB_SWITCH: реагируем на оба направления переключения
                if (digitalRead(CALIB_SWITCH_PIN)) {
                    gpio_wakeup_enable((gpio_num_t)CALIB_SWITCH_PIN, GPIO_INTR_LOW_LEVEL);
                } else {
                    gpio_wakeup_enable((gpio_num_t)CALIB_SWITCH_PIN, GPIO_INTR_HIGH_LEVEL);
                }
                any_gpio = true;

                // LED_SWITCH: аналогично, реагируем на оба направления
                if (digitalRead(LED_SWITCH_PIN)) {
                    gpio_wakeup_enable((gpio_num_t)LED_SWITCH_PIN, GPIO_INTR_LOW_LEVEL);
                } else {
                    gpio_wakeup_enable((gpio_num_t)LED_SWITCH_PIN, GPIO_INTR_HIGH_LEVEL);
                }
                any_gpio = true;

                if (any_gpio) esp_sleep_enable_gpio_wakeup();
            }

            esp_task_wdt_delete(NULL);
            esp_light_sleep_start();
            esp_task_wdt_add(NULL);

            changeFSMState(SystemState::WAKEUP_SENSORS);
            break;

        case SystemState::TARE_PROCESS:
        {
            external_request.tare = false;
            ScalesState tare_state = scales.sensorTare();

            if (tare_state == ScalesState::SUCCESS)      led.pushReport(LedModes::ACT_TARE_OK);
            else if (tare_state == ScalesState::ERROR)   led.pushReport(LedModes::ACT_TARE_ERR);

            changeFSMState(SystemState::PREV_STATE);
            break;
        }

        case SystemState::PREV_STATE:
            break;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}
