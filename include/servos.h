#pragma once
// Dois servos MG996R com PWM por hardware (mesmo timer, 50 Hz).
// Ângulo "nominal" 0..180 graus mapeado em SERVO_PULSE_MIN_US..MAX_US.

#include <Arduino.h>
#include "pins.h"

class ServoPair {
public:
  enum Axis : uint8_t { ROLL = 0, PITCH = 1 };

  // Devolve false se os pinos não estiverem no mesmo timer
  bool begin() {
    PinName pR = digitalPinToPinName(SERVO_ROLL_PIN);
    PinName pP = digitalPinToPinName(SERVO_PITCH_PIN);
    TIM_TypeDef *tR = (TIM_TypeDef *)pinmap_peripheral(pR, PinMap_PWM);
    TIM_TypeDef *tP = (TIM_TypeDef *)pinmap_peripheral(pP, PinMap_PWM);
    if (tR == nullptr || tR != tP) return false;

    _ch[ROLL]  = STM_PIN_CHANNEL(pinmap_function(pR, PinMap_PWM));
    _ch[PITCH] = STM_PIN_CHANNEL(pinmap_function(pP, PinMap_PWM));

    _tim = new HardwareTimer(tR);
    _tim->setMode(_ch[ROLL],  TIMER_OUTPUT_COMPARE_PWM1, SERVO_ROLL_PIN);
    _tim->setMode(_ch[PITCH], TIMER_OUTPUT_COMPARE_PWM1, SERVO_PITCH_PIN);
    _tim->setOverflow(SERVO_PERIOD_US, MICROSEC_FORMAT);
    writeDeg(ROLL, 90);
    writeDeg(PITCH, 90);
    _tim->resume();
    return true;
  }

  void writeDeg(uint8_t axis, float deg) {
    deg = constrain(deg, 0.0f, 180.0f);
    _deg[axis] = deg;
    _us[axis] = SERVO_PULSE_MIN_US +
                (uint32_t)((SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * deg / 180.0f);
    if (_tim) _tim->setCaptureCompare(_ch[axis], _us[axis], MICROSEC_COMPARE_FORMAT);
  }

  float    deg(uint8_t axis) const { return _deg[axis]; }
  uint32_t us(uint8_t axis)  const { return _us[axis]; }

private:
  HardwareTimer *_tim = nullptr;
  uint32_t _ch[2] = {0, 0};
  uint32_t _us[2] = {1500, 1500};
  float    _deg[2] = {90, 90};
};
