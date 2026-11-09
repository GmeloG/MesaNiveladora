// Teste 4 - Integração SEM servos: MPU6050 + Bluetooth + USB
//
// - IMU lido a 200 Hz (período fixo), filtro complementar.
// - USB (monitor série): CSV a 50 Hz  -> t_ms,roll_acc,pitch_acc,roll,pitch
// - Bluetooth (iPhone/LightBlue, Notify em FFE1): "R:x.x P:y.y" a 10 Hz
// - Comandos aceites pelos DOIS lados (terminar com Enter / \n):
//     ?        estado (IMU, alpha, frequência real do ciclo, overruns)
//     c        recalibrar giroscópio (placa parada!)
//     a <x>    alterar alpha do filtro (ex: a 0.96)
//     s        ligar/desligar streaming de dados
//     l        alternar LED verde da Nucleo (confirma receção pelo BT)
//
// Se o IMU falhar, o teste continua a correr para se poder testar o BT.

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include "pins.h"
#include "imu.h"

static constexpr uint32_t LOOP_US       = 5000;  // 200 Hz
static constexpr uint8_t  USB_DIV       = 4;     // 50 Hz
static constexpr uint8_t  BT_DIV        = 20;    // 10 Hz (9600 baud chega)

HardwareSerial BT(BT_RX_PIN, BT_TX_PIN);  // STM32duino 2.x
ImuSensor imu;   // GY-85 (ver include/imu.h)
ComplementaryFilter filt(0.98f);

static bool     imuOk     = false;
static bool     streaming = true;
static uint32_t overruns  = 0;
static uint32_t loopCount = 0;

// ---------- saída para os dois lados ----------
static void both(const char *fmt, ...) {
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  BT.print(buf);
}

// ---------- IMU ----------
static void i2cScan() {
  both("Scan I2C:\r\n");
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { both("  0x%02X\r\n", a); n++; }
  }
  if (n == 0) both("  nenhum dispositivo!\r\n");
}

static void calibrate() {
  if (!imuOk) { both("IMU indisponivel\r\n"); return; }
  both("A calibrar giroscopio - NAO mexer...\r\n");
  imu.calibrateGyro(1000);
  both("Offsets gyro: x=%.3f y=%.3f z=%.3f\r\n",
       imu.gyroOffsetX(), imu.gyroOffsetY(), imu.gyroOffsetZ());
  imu.update();
  filt.init(imu);
}

static void status() {
  static uint32_t lastT = 0, lastCount = 0;
  uint32_t now = millis();
  float hz = (now != lastT) ? (loopCount - lastCount) * 1000.0f / (now - lastT) : 0;
  lastT = now; lastCount = loopCount;
  both("--- Estado ---\r\n");
  both("uptime: %lu s | IMU: %s\r\n", now / 1000, imuOk ? "OK" : "FALHA");
  both("alpha: %.3f | ciclo: %.1f Hz | overruns: %lu\r\n", filt.alpha, hz, overruns);
  both("streaming: %s\r\n", streaming ? "ON" : "OFF");
}

static void help() {
  both("Comandos: ? | c | a <x> | s | l | r | i | h\r\n");
}

// ---------- comandos (não bloqueante, um buffer por porta) ----------
static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  // Comandos de uma letra só são aceites se a mensagem tiver EXATAMENTE
  // essa letra (evita que mensagens do módulo sejam lidas como comandos).
  if (line.length() == 1) {
    switch (line.charAt(0)) {
      case '?': status(); break;
      case 'c': calibrate(); break;
      case 's':
        streaming = !streaming;
        both("streaming %s\r\n", streaming ? "ON" : "OFF");
        break;
      case 'l':
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        both("LED %s\r\n", digitalRead(LED_BUILTIN) ? "ON" : "OFF");
        break;
      case 'h': help(); break;
      case 'r':   // valores em bruto do sensor (diagnóstico)
        if (imu.update()) {
          both("ax=%.3f ay=%.3f az=%.3f g\r\n", imu.ax, imu.ay, imu.az);
          both("gx=%.2f gy=%.2f gz=%.2f dps\r\n", imu.gx, imu.gy, imu.gz);
        } else {
          both("leitura I2C falhou\r\n");
        }
        both("ADXL345 id=0x%02X (esperado E5) | ITG-3205 id=0x%02X (esperado 68/69)\r\n", imu.accId(), imu.gyroId());
        break;
      case 'i':   // reinicia o IMU (scan + configuração + calibração)
        i2cScan();
        imuOk = imu.begin();
        both("IMU %s\r\n", imuOk ? "reiniciado OK" : "nao responde");
        if (imuOk) calibrate();
        break;
      default: break;
    }
    return;
  }

  // "a <valor>": só aceita um número válido entre 0.50 e 0.999
  if (line.startsWith("a ")) {
    String v = line.substring(2);
    v.trim();
    float a = v.toFloat();
    if (v.length() && (isDigit(v.charAt(0)) || v.charAt(0) == '.') && a >= 0.5f && a < 1.0f) {
      filt.alpha = a;
      both("alpha = %.3f\r\n", filt.alpha);
    }  // valor inválido: ignora em silêncio
    return;
  }

  // Tudo o resto é ignorado (ruído / mensagens de estado do módulo)
}

// Um comando termina com Enter (\n/\r) OU após 50 ms sem chegar nada.
// (as apps BLE, como o LightBlue, enviam o texto SEM Enter no fim)
static void pollPort(Stream &port, String &buf, uint32_t &lastRx) {
  while (port.available()) {
    char c = port.read();
    lastRx = millis();
    if (c == '\n' || c == '\r') {
      if (buf.length()) { handleCommand(buf); buf = ""; }
    } else if (c < 0x20 || c > 0x7E) {
      // ignora bytes não imprimíveis (ruído na linha)
    } else if (buf.length() < 32) {
      buf += c;
    }
  }
  if (buf.length() && millis() - lastRx > 50) {
    handleCommand(buf);
    buf = "";
  }
}

// ---------- setup / loop ----------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  BT.begin(BT_BAUD);
  delay(500);

  both("\r\n=== Teste IMU + Bluetooth (sem servos) ===\r\n");

  Wire.setSDA(IMU_SDA_PIN);
  Wire.setSCL(IMU_SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);   // 100 kHz: mais tolerante a fios compridos
  i2cScan();

  imuOk = imu.begin();
  if (imuOk) {
    both("GY-85 OK (ADXL345 0x%02X, ITG-3205 0x%02X)\r\n", imu.accId(), imu.gyroId());
    calibrate();
  } else {
    both("ERRO: GY-85 nao responde (0x68). BT continua ativo.\r\n");
  }

  help();
  Serial.println(F("t_ms,roll_acc,pitch_acc,roll,pitch"));
}

void loop() {
  static String usbBuf, btBuf;
  static uint32_t usbRx = 0, btRx = 0, tBeat = 0;
  static uint32_t next = micros();
  static uint32_t last = micros();
  static uint8_t usbDiv = 0, btDiv = 0;

  pollPort(Serial, usbBuf, usbRx);
  pollPort(BT, btBuf, btRx);

  // Sinal de vida a cada 2 s quando não há dados do IMU para enviar
  if ((!imuOk || !streaming) && millis() - tBeat >= 2000) {
    tBeat = millis();
    BT.printf("STM32 ok t=%lus IMU:%s\r\n", millis() / 1000, imuOk ? "OK" : "FALHA");
  }

  if ((int32_t)(micros() - next) < 0) return;
  next += LOOP_US;
  if ((int32_t)(micros() - next) > 0) {   // atrasou mais de um período
    overruns++;
    next = micros() + LOOP_US;
  }
  loopCount++;

  uint32_t now = micros();
  float dt = (now - last) * 1e-6f;
  last = now;

  if (!imuOk) return;
  if (!imu.update()) return;
  filt.update(imu, dt);

  if (!streaming) return;

  if (++usbDiv >= USB_DIV) {
    usbDiv = 0;
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f\r\n", millis(),
                  filt.rollAcc, filt.pitchAcc, filt.roll, filt.pitch);
  }
  if (++btDiv >= BT_DIV) {
    btDiv = 0;
    BT.printf("R:%.1f P:%.1f\r\n", filt.roll, filt.pitch);
  }
}
