#pragma once
// LEDs de estado externos (1 verde + 1 vermelho, ou um LED bicolor).
//
//   Verde fixo           AUTO ou SEGUIR a controlar
//   Verde a piscar       SEGUIR à espera de dados do telemóvel
//   Verde + vermelho     MANUAL ou a calibrar (no bicolor: laranja)
//   Vermelho fixo        OFF
//   Vermelho a piscar    erro: IMU em falha ou corte de segurança (5 s)

#include <Arduino.h>
#include "pins.h"

enum class LedState : uint8_t { IDLE, ERROR, MANUAL, ACTIVE, WAITING, CALIB };

class StatusLed {
public:
  void begin() {
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    set(false, false);
  }

  void set(bool green, bool red) {
    write(LED_GREEN_PIN, green);
    write(LED_RED_PIN, red);
  }

  void update(LedState s) {
    uint32_t t = millis();
    bool fast = (t / 125) % 2;   // 4 Hz
    bool slow = (t / 500) % 2;   // 1 Hz
    switch (s) {
      case LedState::IDLE:    set(false, true);  break;
      case LedState::ERROR:   set(false, fast);  break;
      case LedState::MANUAL:  set(true,  true);  break;
      case LedState::ACTIVE:  set(true,  false); break;
      case LedState::WAITING: set(slow,  false); break;
      case LedState::CALIB:   set(true,  true);  break;
    }
  }

private:
  static void write(uint32_t pin, bool on) {
#if STATUS_LED_ACTIVE_LOW
    digitalWrite(pin, on ? LOW : HIGH);    // LED bicolor de ânodo comum
#else
    digitalWrite(pin, on ? HIGH : LOW);    // LEDs normais / cátodo comum
#endif
  }
};
