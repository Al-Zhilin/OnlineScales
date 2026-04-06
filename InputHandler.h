#include "Enumerations.h"

class InputHandler {
  private:
    uButton *butt = nullptr;
    uint8_t switch_pins, butt_pin;
    bool switch_state = false;                        // храним предыдущее состояние переключателей
    uint32_t switch_timer = millis();                 // таймер для игнорирования возможного дребезга
    ScaleAutoCalibrator& calibrator;
    ModificationRequest& external_request;
    SystemState& current_state;

  public:
    InputHandler(uint8_t button, uint8_t sw1_pin, ScaleAutoCalibrator& cal, ModificationRequest& ext, SystemState& curr_state) : switch_pins(sw1_pin), butt_pin(button), calibrator(cal), external_request(ext), 
                                                                                                                                 current_state(curr_state) {}

    void begin() {
      pinMode(switch_pins, INPUT_PULLUP);
      butt = new uButton(butt_pin);
      switch_state = !digitalRead(switch_pins);        // начальное положение переключателя режима работы
    }

    void tick() {
      butt->tick();
      bool switch_val = !digitalRead(switch_pins);     // ручное чтение состояний переключателя. true - вкл режим калибровки

      if (switch_val != switch_state && millis() - switch_timer >= 500) {                   // дернули переключатель режима работы
        switch_state = switch_val;
        if (switch_val) external_request = ModificationRequest::START_CALIBRATION;        // пользователь включил режим калибровки
        else external_request = ModificationRequest::END_CALIBRATION;      // пользователь выключил режим калбировки
      }


      // ---------------------------------------- Универсальные коды ввода ----------------------------------------
      if (butt->hold(0))    {                   // кнопка удержана без предшествующих нажатий - поднимаем флаг тарирования
        external_request = ModificationRequest::TARE;
        LOG("Tare request captured");
      }

      else if (butt->hold(4)) {                 // удержание после 4 кликов - вывести сохраненную модель калибровки в Serial, для отладки и интереса
          LOG("Requesting saved model data...");
          calibrator.printSavedModelData();
      }
      // ---------------------------------------- Универсальные коды ввода ----------------------------------------



      /*// ---------------------------------------- Коды ввода в режиме Калибровки ----------------------------------------
      if (current_state == SystemState::CALIBRATION) {
        else if (butt->hold(3)) {                // удержание после 3 кликов - сбросить данные о предыдущей калибровке
          calibrator.resetCalibration();
        }
      }
      // ---------------------------------------- Коды ввода в режиме Калибровки ----------------------------------------*/



      // ---------------------------------------- Коды ввода в остальных режимах ----------------------------------------
      else {
        // if (butt->hold(1))  force_send = true;      // принудительный цикл измерений и отправки значений сейчас же
        // выйти из сна
      }
      // ---------------------------------------- Коды ввода в остальных режимах ----------------------------------------

    }
};