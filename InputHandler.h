#include "Enumerations.h"

class InputHandler {
  private:
    uButton *butt = nullptr;
    uint8_t switch_pins, butt_pin;
    bool switch_state = false;                        // храним предыдущее состояние переключателей
    uint32_t switch_timer = millis();                 // таймер для игнорирования возможного дребезга
    ScaleAutoCalibrator& calibrator;
    ModificationRequests& external_request;
    SystemState& current_state;

  public:
    InputHandler(uint8_t button, uint8_t sw1_pin, ScaleAutoCalibrator& cal, ModificationRequests& ext, SystemState& curr_state) : switch_pins(sw1_pin), butt_pin(button), calibrator(cal), external_request(ext), current_state(curr_state) {}

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
        _switch_timer = millis();
        
        if (switch_val) external_request.start_calibration = true;        // пользователь включил режим калибровки
        else external_request.end_calibration = true;                     // пользователь выключил режим калбировки
      }

      if (Serial.available() > 0) {
        String input = Serial.readString();           // Читаем строку
        input.trim();                                 // Убираем пробелы и переводы строк
        
        int number = input.toInt();                   // Преобразуем в число
        
        Serial.println("Parsed number: " + String(number));
        if (number == 1) external_request.start_calibration = true;
        else if (number == 0) external_request.end_calibration = true;
      }


      // ---------------------------------------- Универсальные коды ввода ----------------------------------------
      if (butt->hold(0))    {                   // кнопка удержана без предшествующих нажатий - поднимаем флаг тарирования
        external_request.tare = true;
        LOG("Tare request captured");
      }

      else if (butt->hold(1))  {                // принудительный цикл измерений и отправки значений сейчас же
        LOG("");
        external_request.force_send = true;;
      }

      else if (butt->hold(2)) {                 // вывести сохраненную модель калибровки в Serial, для отладки и интереса
          LOG("Input Handler: print saved");
          calibrator.printSavedModelData();
      }
      // ---------------------------------------- Универсальные коды ввода ----------------------------------------



      /*// ---------------------------------------- Коды ввода в режиме Калибровки ----------------------------------------
      if (current_state == SystemState::CALIBRATION) {
        // здесь коды ввода только для режима калибровки
      }
      // ---------------------------------------- Коды ввода в режиме Калибровки ----------------------------------------*/



      // ---------------------------------------- Коды ввода в остальных режимах ----------------------------------------
      /*else {
        // здесь коды ввода во всех режимах, кроме калибровки
      }*/
      // ---------------------------------------- Коды ввода в остальных режимах ----------------------------------------

    }
};