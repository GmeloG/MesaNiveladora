#pragma once
// Driver mínimo MPU6050 (I2C, registos diretos) + filtro complementar.
// Header-only para reutilizar nos testes e na aplicação final.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MPU6050 {
public:
  static constexpr uint8_t ADDR = 0x68;          // AD0 = GND (0x69 se AD0 = VCC)
  static constexpr float ACC_LSB  = 16384.0f;    // +-2 g
  static constexpr float GYRO_LSB = 65.5f;       // +-500 graus/s

  float ax, ay, az;   // g
  float gx, gy, gz;   // graus/s (já sem offset)

  // Devolve false se o sensor não responder
  bool begin(uint8_t addr = ADDR) {
    _addr = addr;
    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) return false;
    write(0x6B, 0x01);   // PWR_MGMT_1: acorda, relógio = PLL gyro X
    delay(50);
    write(0x19, 0x00);   // SMPLRT_DIV: 1 kHz (com DLPF ativo)
    write(0x1A, 0x03);   // CONFIG: DLPF ~44 Hz
    write(0x1B, 0x08);   // GYRO_CONFIG: +-500 graus/s
    write(0x1C, 0x00);   // ACCEL_CONFIG: +-2 g
    delay(50);
    return true;
  }

  uint8_t whoAmI() { return read8(0x75); }

  // Média do giroscópio com o sensor parado -> offsets
  void calibrateGyro(uint16_t samples = 1000) {
    float sx = 0, sy = 0, sz = 0;
    _gox = _goy = _goz = 0;
    for (uint16_t i = 0; i < samples; i++) {
      update();
      sx += gx; sy += gy; sz += gz;
      delay(2);
    }
    _gox = sx / samples; _goy = sy / samples; _goz = sz / samples;
  }

  bool update() {
    uint8_t b[14];
    Wire.beginTransmission(_addr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(_addr, (uint8_t)14) != 14) return false;
    for (int i = 0; i < 14; i++) b[i] = Wire.read();

    ax = (int16_t)(b[0]  << 8 | b[1])  / ACC_LSB;
    ay = (int16_t)(b[2]  << 8 | b[3])  / ACC_LSB;
    az = (int16_t)(b[4]  << 8 | b[5])  / ACC_LSB;
    gx = (int16_t)(b[8]  << 8 | b[9])  / GYRO_LSB - _gox;
    gy = (int16_t)(b[10] << 8 | b[11]) / GYRO_LSB - _goy;
    gz = (int16_t)(b[12] << 8 | b[13]) / GYRO_LSB - _goz;
    return true;
  }

  float gyroOffsetX() const { return _gox; }
  float gyroOffsetY() const { return _goy; }
  float gyroOffsetZ() const { return _goz; }

private:
  uint8_t _addr = ADDR;
  float _gox = 0, _goy = 0, _goz = 0;

  void write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
  }
  uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
  }
};

// Filtro complementar para roll/pitch (graus)
class ComplementaryFilter {
public:
  float roll = 0, pitch = 0;          // estimativa filtrada
  float rollAcc = 0, pitchAcc = 0;    // só acelerómetro (para comparar)
  float alpha;

  explicit ComplementaryFilter(float a = 0.98f) : alpha(a) {}

  void init(const MPU6050 &imu) {
    accAngles(imu);
    roll = rollAcc; pitch = pitchAcc;
  }

  void update(const MPU6050 &imu, float dt) {
    accAngles(imu);
    roll  = alpha * (roll  + imu.gx * dt) + (1.0f - alpha) * rollAcc;
    pitch = alpha * (pitch + imu.gy * dt) + (1.0f - alpha) * pitchAcc;
  }

private:
  void accAngles(const MPU6050 &imu) {
    rollAcc  = atan2f(imu.ay, imu.az) * RAD_TO_DEG;
    pitchAcc = atan2f(-imu.ax, sqrtf(imu.ay * imu.ay + imu.az * imu.az)) * RAD_TO_DEG;
  }
};
