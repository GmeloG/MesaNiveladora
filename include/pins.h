#pragma once
#include <Arduino.h>

// =====================================================================
//  Mapa de pinos - NUCLEO-F767ZI (conector Arduino)
// =====================================================================

// --- Servos MG996R (ambos no TIM4, PWM 50 Hz) ---
#define SERVO_ROLL_PIN    PD15   // D9  - TIM4_CH4
#define SERVO_PITCH_PIN   PD14   // D10 - TIM4_CH3

// --- Módulo Bluetooth (USART6) ---
#define BT_RX_PIN         PG9    // D0  <- TXD do módulo
#define BT_TX_PIN         PG14   // D1  -> RXD do módulo
#define BT_BAUD           9600   // default habitual destes módulos

// --- IMU MPU6050 (I2C1) ---
#define IMU_SDA_PIN       PB9    // D14
#define IMU_SCL_PIN       PB8    // D15

// --- LEDs de estado externos (conector CN10) ---
// Cada LED: pino -> resistência 220-330 ohm -> LED -> GND
#define LED_GREEN_PIN     PE9    // D6
#define LED_RED_PIN       PE11   // D5
#define STATUS_LED_ACTIVE_LOW 0  // 1 se for LED bicolor de ânodo comum (comum ao 3V3)

// --- Parâmetros dos servos ---
#define SERVO_PERIOD_US     20000  // 50 Hz
// Gama conservadora para começar; alargar depois de medir (até ~500-2500 us)
#define SERVO_PULSE_MIN_US  1000
#define SERVO_PULSE_MAX_US  2000
#define SERVO_RAW_MIN_US    500    // limites absolutos para o comando de calibração
#define SERVO_RAW_MAX_US    2500
