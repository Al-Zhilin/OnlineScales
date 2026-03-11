#include "Enumerations.h"

class InputHandler {
  private:
    uButton *butt = nullptr;
    uint8_t switch_pins, butt_pin;
    bool switch_state = false;                        // храним предыдущее состояние переключателей
    uint32_t switch_timer = millis();                 // таймер для игнорирования возможного дребезга
    SystemState& currentState;
    ScaleAutoCalibrator& calibrator;

  public:
    InputHandler(uint8_t button, uint8_t sw1_pin, SystemState& state, ScaleAutoCalibrator& cal) : switch_pins(sw1_pin), butt_pin(button), currentState(state), calibrator(cal) {}

    void begin() {
      pinMode(switch_pins, INPUT_PULLUP);
      butt = new uButton(butt_pin);
    }

    void tick() {
      butt->tick();
      bool switch_val = !digitalRead(switch_pins);     // ручное чтение состояний переключателя. true - вкл режим калибровки

      if (switch_val != switch_state && millis() - switch_timer >= 500) {                   // дернули переключатель режима работы
        if (switch_val) changeState(SystemState::START_CALIBRATION);        // пользователь включил режим калибровки
        else changeState(SystemState::END_CALIBRATION);      // пользователь выключил режим калбировки
      }


      // ---------------------------------------- Универсальные коды ввода ----------------------------------------
      if (butt->hold(0))    {                   // кнопка удержана без предшествующих нажатий - поднимаем флаг тарирования
        changeMode(SystemState::TARE_PROCESS);
        LOG("Tare request captured");
      }

      else if (butt->hold(4)) {                 // удержание после 4 кликов - вывести сохраненную модель калибровки в Serial, для отладки и интереса
          LOG("Requesting saved model data...");
          calibrator.printSavedModelData();
      }
      // ---------------------------------------- Универсальные коды ввода ----------------------------------------



      // ---------------------------------------- Коды ввода в режиме Калибровки ----------------------------------------
      if (currentState == SystemState::CALIBRATION) {
        if (butt->hold(1)) {              // удержание после одного клика - начать калибровку
          calibrator.startCalibration();
        }

        else if (butt->hold(2)) {                // удержание после 2 кликов - закончить калибровку
          calibrator.finishCalibration();
        }

        else if (butt->hold(3)) {                // удержание после 3 кликов - сбросить данные о предыдущей калибровке
          calibrator.resetCalibration();
        }

      }
      // ---------------------------------------- Коды ввода в режиме Калибровки ----------------------------------------



      // ---------------------------------------- Коды ввода в остальных режимах ----------------------------------------
      else {
        delay(1); 
      }
      // ---------------------------------------- Коды ввода в остальных режимах ----------------------------------------

    }
};