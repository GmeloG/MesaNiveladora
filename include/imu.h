#pragma once
// Drivers de IMU (I2C, registos diretos) + filtro complementar.
//
// O módulo usado no projeto é um GY-85:
//   0x53 ADXL345   acelerómetro
//   0x68 ITG-3205  giroscópio
//   0x1E HMC5883L  magnetómetro (não usado)
// O driver MPU6050 fica disponível caso se troque de sensor.
//
// Para escolher o sensor basta mudar o typedef ImuSensor no fim do ficheiro.

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ---------------------------------------------------------------------
// Funções I2C comuns
// ---------------------------------------------------------------------
namespace imu_i2c {
inline bool write8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
inline bool readN(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}
inline uint8_t read8(uint8_t addr, uint8_t reg) {
  uint8_t v = 0xFF;
  readN(addr, reg, &v, 1);
  return v;
}
inline bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}
}  // namespace imu_i2c

// ---------------------------------------------------------------------
// Base comum: valores em unidades físicas + calibração do giroscópio
// ---------------------------------------------------------------------
struct ImuData {
  float ax = 0, ay = 0, az = 0;   // g
  float gx = 0, gy = 0, gz = 0;   // graus/s (já sem offset)
};

// ---------------------------------------------------------------------
// GY-85: ADXL345 (acelerómetro) + ITG-3205 (giroscópio)
// ---------------------------------------------------------------------
class GY85 : public ImuData {
public:
  static constexpr uint8_t ACC_ADDR  = 0x53;   // ADXL345 (SDO a GND)
  static constexpr uint8_t GYRO_ADDR = 0x68;   // ITG-3205 (AD0 a GND)
  static constexpr float   ACC_LSB   = 256.0f; // FULL_RES: 3,9 mg/LSB
  static constexpr float   GYRO_LSB  = 14.375f;// LSB por grau/s

  // Sinais dos eixos do giroscópio em relação ao acelerómetro.
  // Se ao rodar a placa o ângulo filtrado "fugir" do ângulo do
  // acelerómetro, trocar o sinal do eixo correspondente.
  float gyroSignX = 1.0f, gyroSignY = 1.0f, gyroSignZ = 1.0f;

  // Sensor montado ao contrário (chips virados para baixo):
  // equivale a rodar 180 graus em torno do eixo X -> Y e Z trocam de sinal.
  bool upsideDown = true;   // GY-85 montado com os chips virados para baixo

  bool begin() {
    if (!imu_i2c::present(ACC_ADDR) || !imu_i2c::present(GYRO_ADDR)) return false;

    // ADXL345
    imu_i2c::write8(ACC_ADDR, 0x2D, 0x00);   // POWER_CTL: standby para configurar
    imu_i2c::write8(ACC_ADDR, 0x31, 0x08);   // DATA_FORMAT: FULL_RES, +-2 g
    imu_i2c::write8(ACC_ADDR, 0x2C, 0x0B);   // BW_RATE: 200 Hz
    imu_i2c::write8(ACC_ADDR, 0x2D, 0x08);   // POWER_CTL: modo de medição

    // ITG-3205
    imu_i2c::write8(GYRO_ADDR, 0x3E, 0x80);  // PWR_MGM: reset
    delay(10);
    imu_i2c::write8(GYRO_ADDR, 0x3E, 0x01);  // PWR_MGM: relógio = PLL gyro X
    imu_i2c::write8(GYRO_ADDR, 0x15, 0x04);  // SMPLRT_DIV: 1 kHz / 5 = 200 Hz
    imu_i2c::write8(GYRO_ADDR, 0x16, 0x1B);  // DLPF_FS: +-2000 graus/s, DLPF 42 Hz
    delay(50);

    return accId() == 0xE5;              // DEVID do ADXL345
  }

  uint8_t accId()  { return imu_i2c::read8(ACC_ADDR, 0x00); }   // esperado 0xE5
  uint8_t gyroId() { return imu_i2c::read8(GYRO_ADDR, 0x00); }  // esperado 0x68/0x69
  uint8_t whoAmI() { return gyroId(); }

  bool update() {
    uint8_t a[6], g[6];
    if (!imu_i2c::readN(ACC_ADDR, 0x32, a, 6)) return false;    // little-endian
    if (!imu_i2c::readN(GYRO_ADDR, 0x1D, g, 6)) return false;   // big-endian

    ax = (int16_t)(a[1] << 8 | a[0]) / ACC_LSB;
    ay = (int16_t)(a[3] << 8 | a[2]) / ACC_LSB;
    az = (int16_t)(a[5] << 8 | a[4]) / ACC_LSB;

    gx = gyroSignX * (int16_t)(g[0] << 8 | g[1]) / GYRO_LSB - _gox;
    gy = gyroSignY * (int16_t)(g[2] << 8 | g[3]) / GYRO_LSB - _goy;
    gz = gyroSignZ * (int16_t)(g[4] << 8 | g[5]) / GYRO_LSB - _goz;

    if (upsideDown) {
      ay = -ay; az = -az;
      gy = -gy; gz = -gz;   // offsets já foram retirados antes da inversão
    }
    return true;
  }

  void calibrateGyro(uint16_t samples = 1000) {
    bool flip = upsideDown;
    upsideDown = false;          // offsets medidos nos eixos do próprio chip
    float sx = 0, sy = 0, sz = 0;
    _gox = _goy = _goz = 0;
    for (uint16_t i = 0; i < samples; i++) {
      update();
      sx += gx; sy += gy; sz += gz;
      delay(2);
    }
    _gox = sx / samples; _goy = sy / samples; _goz = sz / samples;
    upsideDown = flip;
  }

  float gyroOffsetX() const { return _gox; }
  float gyroOffsetY() const { return _goy; }
  float gyroOffsetZ() const { return _goz; }

private:
  float _gox = 0, _goy = 0, _goz = 0;
};

// ---------------------------------------------------------------------
// MPU6050 (alternativa)
// ---------------------------------------------------------------------
class MPU6050 : public ImuData {
public:
  static constexpr uint8_t ADDR = 0x68;
  static constexpr float ACC_LSB  = 16384.0f;  // +-2 g
  static constexpr float GYRO_LSB = 65.5f;     // +-500 graus/s

  bool begin() {
    if (!imu_i2c::present(ADDR)) return false;
    imu_i2c::write8(ADDR, 0x6B, 0x01);
    delay(50);
    imu_i2c::write8(ADDR, 0x19, 0x00);
    imu_i2c::write8(ADDR, 0x1A, 0x03);
    imu_i2c::write8(ADDR, 0x1B, 0x08);
    imu_i2c::write8(ADDR, 0x1C, 0x00);
    delay(50);
    return true;
  }

  uint8_t whoAmI() { return imu_i2c::read8(ADDR, 0x75); }

  bool update() {
    uint8_t b[14];
    if (!imu_i2c::readN(ADDR, 0x3B, b, 14)) return false;
    ax = (int16_t)(b[0]  << 8 | b[1])  / ACC_LSB;
    ay = (int16_t)(b[2]  << 8 | b[3])  / ACC_LSB;
    az = (int16_t)(b[4]  << 8 | b[5])  / ACC_LSB;
    gx = (int16_t)(b[8]  << 8 | b[9])  / GYRO_LSB - _gox;
    gy = (int16_t)(b[10] << 8 | b[11]) / GYRO_LSB - _goy;
    gz = (int16_t)(b[12] << 8 | b[13]) / GYRO_LSB - _goz;
    return true;
  }

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

  float gyroOffsetX() const { return _gox; }
  float gyroOffsetY() const { return _goy; }
  float gyroOffsetZ() const { return _goz; }

private:
  float _gox = 0, _goy = 0, _goz = 0;
};

// ---------------------------------------------------------------------
// Sensor usado no projeto
// ---------------------------------------------------------------------
typedef GY85 ImuSensor;

// ---------------------------------------------------------------------
// Filtro complementar para roll/pitch (graus)
// ---------------------------------------------------------------------
class ComplementaryFilter {
public:
  float roll = 0, pitch = 0;          // estimativa filtrada
  float rollAcc = 0, pitchAcc = 0;    // só acelerómetro (para comparar)
  float alpha;

  explicit ComplementaryFilter(float a = 0.98f) : alpha(a) {}

  void init(const ImuData &imu) {
    accAngles(imu);
    roll = rollAcc; pitch = pitchAcc;
  }

  void update(const ImuData &imu, float dt) {
    accAngles(imu);
    roll  = alpha * (roll  + imu.gx * dt) + (1.0f - alpha) * rollAcc;
    pitch = alpha * (pitch + imu.gy * dt) + (1.0f - alpha) * pitchAcc;
  }

private:
  void accAngles(const ImuData &imu) {
    rollAcc  = atan2f(imu.ay, imu.az) * RAD_TO_DEG;
    pitchAcc = atan2f(-imu.ax, sqrtf(imu.ay * imu.ay + imu.az * imu.az)) * RAD_TO_DEG;
  }
};
