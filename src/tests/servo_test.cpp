// Teste 1 - Servos MG996R com PWM por hardware (TIM4, 50 Hz)
//
// Comandos no monitor série (115200, terminar com Enter):
//   r <graus>   -> servo ROLL  para 0..180 graus      ex: r 90
//   p <graus>   -> servo PITCH para 0..180 graus      ex: p 45
//   R <us>      -> ROLL  com impulso bruto em us      ex: R 1500  (calibração)
//   P <us>      -> PITCH com impulso bruto em us      ex: P 1500
//   c           -> centrar os dois (90 graus)
//   s           -> varrimento lento dos dois servos
//
// ATENÇÃO: primeiro teste sem a plataforma montada, para descobrir os
// limites mecânicos reais de cada eixo antes de os acoplar.

#include <Arduino.h>
#include "pins.h"

static HardwareTimer *servoTim = nullptr;
static uint32_t chRoll, chPitch;

static uint32_t angleToPulse(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return SERVO_PULSE_MIN_US +
         (uint32_t)((SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * deg / 180.0f);
}

static void setPulse(uint32_t ch, uint32_t us) {
  us = constrain(us, (uint32_t)SERVO_RAW_MIN_US, (uint32_t)SERVO_RAW_MAX_US);
  servoTim->setCaptureCompare(ch, us, MICROSEC_COMPARE_FORMAT);
}

static void setAngle(uint32_t ch, float deg) { setPulse(ch, angleToPulse(deg)); }

static void printHelp() {
  Serial.println(F("\n=== Teste de servos ==="));
  Serial.println(F("r <graus> | p <graus> | R <us> | P <us> | c | s"));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Obter timer e canal a partir dos pinos (evita erros de mapeamento)
  PinName pRoll  = digitalPinToPinName(SERVO_ROLL_PIN);
  PinName pPitch = digitalPinToPinName(SERVO_PITCH_PIN);
  TIM_TypeDef *timRoll  = (TIM_TypeDef *)pinmap_peripheral(pRoll,  PinMap_PWM);
  TIM_TypeDef *timPitch = (TIM_TypeDef *)pinmap_peripheral(pPitch, PinMap_PWM);
  chRoll  = STM_PIN_CHANNEL(pinmap_function(pRoll,  PinMap_PWM));
  chPitch = STM_PIN_CHANNEL(pinmap_function(pPitch, PinMap_PWM));

  if (timRoll == nullptr || timRoll != timPitch) {
    Serial.println(F("ERRO: os pinos dos servos não estão no mesmo timer."));
    while (true) {}
  }

  servoTim = new HardwareTimer(timRoll);
  servoTim->setMode(chRoll,  TIMER_OUTPUT_COMPARE_PWM1, SERVO_ROLL_PIN);
  servoTim->setMode(chPitch, TIMER_OUTPUT_COMPARE_PWM1, SERVO_PITCH_PIN);
  servoTim->setOverflow(SERVO_PERIOD_US, MICROSEC_FORMAT);
  setAngle(chRoll, 90);
  setAngle(chPitch, 90);
  servoTim->resume();

  printHelp();
}

static void sweep() {
  Serial.println(F("Varrimento 0 -> 180 -> 90"));
  for (int a = 90; a >= 0; a--)   { setAngle(chRoll, a); setAngle(chPitch, a); delay(15); }
  for (int a = 0; a <= 180; a++)  { setAngle(chRoll, a); setAngle(chPitch, a); delay(15); }
  for (int a = 180; a >= 90; a--) { setAngle(chRoll, a); setAngle(chPitch, a); delay(15); }
  Serial.println(F("Fim"));
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  float val = line.substring(1).toFloat();

  switch (cmd) {
    case 'r': setAngle(chRoll, val);  Serial.printf("ROLL  = %.1f graus (%lu us)\n", val, angleToPulse(val)); break;
    case 'p': setAngle(chPitch, val); Serial.printf("PITCH = %.1f graus (%lu us)\n", val, angleToPulse(val)); break;
    case 'R': setPulse(chRoll,  (uint32_t)val); Serial.printf("ROLL  = %lu us\n", (uint32_t)val); break;
    case 'P': setPulse(chPitch, (uint32_t)val); Serial.printf("PITCH = %lu us\n", (uint32_t)val); break;
    case 'c': setAngle(chRoll, 90); setAngle(chPitch, 90); Serial.println(F("Centrados")); break;
    case 's': sweep(); break;
    default:  printHelp(); break;
  }
}
