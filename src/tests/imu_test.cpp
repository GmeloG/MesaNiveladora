// Teste 3 - MPU6050 + filtro complementar
//
// - Faz scan I2C no arranque e confirma o sensor.
// - Calibra o giroscópio: deixar a placa PARADA nos primeiros ~2 s.
// - Ciclo de amostragem fixo a 200 Hz (dt = 5 ms).
// - Envia CSV a 50 Hz: t_ms,roll_acc,pitch_acc,roll,pitch
//   (usar o Serial Plotter / guardar para gráficos do relatório)
//
// Comandos: 'c' recalibra o giroscópio | 'a <valor>' muda alpha (ex: a 0.96)

#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "imu.h"

static constexpr uint32_t LOOP_US  = 5000;  // 200 Hz
static constexpr uint8_t  PRINT_DIV = 4;    // 200/4 = 50 Hz de saída

MPU6050 imu;
ComplementaryFilter filt(0.98f);

static void i2cScan() {
  Serial.println(F("Scan I2C:"));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("  dispositivo em 0x%02X\n", a);
  }
}

static void calibrate() {
  Serial.println(F("A calibrar giroscópio - NÃO mexer..."));
  imu.calibrateGyro(1000);
  Serial.printf("Offsets gyro [graus/s]: x=%.3f y=%.3f z=%.3f\n",
                imu.gyroOffsetX(), imu.gyroOffsetY(), imu.gyroOffsetZ());
  imu.update();
  filt.init(imu);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== Teste MPU6050 ==="));

  Wire.setSDA(IMU_SDA_PIN);
  Wire.setSCL(IMU_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  i2cScan();

  if (!imu.begin()) {
    Serial.println(F("ERRO: MPU6050 não responde em 0x68. Verificar ligações/AD0."));
    while (true) {}
  }
  // 0x68 = original; clones podem devolver 0x70, 0x72 ou 0x98
  Serial.printf("WHO_AM_I = 0x%02X\n", imu.whoAmI());

  calibrate();
  Serial.println(F("t_ms,roll_acc,pitch_acc,roll,pitch"));
}

void loop() {
  static uint32_t next = micros();
  static uint32_t last = micros();
  static uint8_t  div  = 0;

  // Comandos
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "c") calibrate();
    else if (line.startsWith("a")) {
      filt.alpha = constrain(line.substring(1).toFloat(), 0.0f, 1.0f);
      Serial.printf("alpha = %.3f\n", filt.alpha);
    }
    next = micros();
  }

  // Período fixo
  if ((int32_t)(micros() - next) < 0) return;
  next += LOOP_US;

  uint32_t now = micros();
  float dt = (now - last) * 1e-6f;
  last = now;

  if (!imu.update()) { Serial.println(F("Falha leitura I2C")); return; }
  filt.update(imu, dt);

  if (++div >= PRINT_DIV) {
    div = 0;
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f\n", millis(),
                  filt.rollAcc, filt.pitchAcc, filt.roll, filt.pitch);
  }
}
