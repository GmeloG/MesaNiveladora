# Ligações — NUCLEO-F767ZI

| Componente | Pino do componente | Nucleo (Arduino) | Pino STM32 | Notas |
|---|---|---|---|---|
| Servo ROLL (MG996R) | Sinal (laranja) | D9 | PD15 | TIM4_CH4 |
| Servo PITCH (MG996R) | Sinal (laranja) | D10 | PD14 | TIM4_CH3 |
| Servos | V+ (vermelho) | — | — | **Fonte externa 5–6 V, ≥ 3 A** |
| Servos | GND (castanho) | GND | — | GND da fonte comum ao da Nucleo |
| Bluetooth | TXD | D0 | PG9 | USART6_RX |
| Bluetooth | RXD | D1 | PG14 | USART6_TX |
| Bluetooth | VCC | 5V ou 3V3 | — | Confirmar a tensão do módulo |
| Bluetooth | GND | GND | — | |
| MPU6050 | SDA | D14 | PB9 | I2C1 |
| MPU6050 | SCL | D15 | PB8 | I2C1 |
| MPU6050 | VCC | 3V3 | — | (ou 5V se o módulo tiver regulador) |
| MPU6050 | GND | GND | — | AD0 a GND → endereço 0x68 |

O monitor série (`Serial`) sai pelo USB do ST-Link (USART3, virtual COM).

## Testes
1. `env:servo_test` — mexer servos, descobrir limites (comandos `R`/`P` em µs).
2. `env:bt_echo` — ponte USB ↔ Bluetooth; testar com LightBlue no iPhone.
3. `env:imu_test` — MPU6050, filtro complementar, CSV a 50 Hz.
4. `env:sensors_bt_test` — IMU + Bluetooth sem servos; dados no PC e no iPhone, comandos dos dois lados.
