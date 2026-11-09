#pragma once
// PID simples para um eixo da plataforma.
// - Derivada sobre a MEDIDA (usa a velocidade angular do giroscópio):
//   evita "pontapés" na saída quando muda o setpoint e é menos ruidosa.
// - Anti-windup por clamping: o integral só cresce se a saída não estiver
//   saturada no mesmo sentido do erro.

#include <Arduino.h>

struct PID {
  float kp = 1.0f, ki = 0.0f, kd = 0.05f;
  float outMin = -45.0f, outMax = 45.0f;   // graus de servo
  float iTerm = 0.0f;
  float p = 0, d = 0;                      // últimas parcelas (telemetria)

  void reset() { iTerm = 0.0f; }

  // err  = setpoint - medida   [graus]
  // rate = d(medida)/dt        [graus/s] (giroscópio)
  float update(float err, float rate, float dt) {
    p = kp * err;
    d = -kd * rate;
    float iNew = iTerm + ki * err * dt;
    float out  = p + iNew + d;

    bool satHigh = out > outMax && err > 0;
    bool satLow  = out < outMin && err < 0;
    if (!satHigh && !satLow) iTerm = constrain(iNew, outMin, outMax);

    out = p + iTerm + d;
    return constrain(out, outMin, outMax);
  }
};
