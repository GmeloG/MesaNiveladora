// =====================================================================
//  Mesa Niveladora - aplicação principal (LAMEC)
//  NUCLEO-F767ZI + MPU6050 + 2x MG996R + módulo Bluetooth
// =====================================================================
//
//  Ciclo de controlo a 200 Hz (dt = 5 ms):
//    IMU -> filtro complementar -> PID por eixo -> servos
//
//  Modos:
//    0 OFF     servos ao centro (+trim), controlo desligado   [arranque]
//    1 MANUAL  servos comandados diretamente (comando "u")
//    2 AUTO    PID mantém a plataforma no setpoint (nivelada por defeito)
//    3 SEGUIR  a plataforma imita a inclinação do telemóvel (comando "f")
//              - referência suavizada (filtro de 1.ª ordem)
//              - sem "f" durante 1 s -> volta ao nível
//
//  Segurança: em AUTO, se |roll| ou |pitch| > ANG_LIMIT -> passa a OFF.
//             Se o IMU falhar -> OFF.
//
//  Comandos (Bluetooth ou USB; com ou sem Enter):
//    ?                 estado completo
//    h                 ajuda
//    m <0|1|2|3>       modo OFF / MANUAL / AUTO / SEGUIR
//    f <roll> <pitch>  referência vinda do telemóvel (modo SEGUIR, sem resposta)
//    sp <roll> <pitch> setpoints em graus (AUTO)          ex: sp 0 5
//    g <r|p|b> <kp> <ki> <kd>  ganhos do PID (b = ambos)    ex: g b 1.2 0.4 0.05
//    u <r|p> <graus>   desvio manual do servo (MANUAL)     ex: u r 10
//    t <r|p> <graus>   trim (centro mecânico do servo)     ex: t p -3
//    inv <r|p>         inverte o sentido de atuação desse eixo
//    a <0.5..0.999>    alpha do filtro complementar
//    c                 recalibrar giroscópio (passa a OFF)
//    s                 ligar/desligar telemetria
//    r                 valores em bruto do IMU
//    flip              sensor montado ao contrário (liga/desliga)
//    gs <x|y>          inverte o sinal do giroscópio nesse eixo (diagnóstico)
//    l                 alternar LED verde
//
//  Telemetria BT (10 Hz):  M<modo> R:<roll> P:<pitch> uR:<cmd> uP:<cmd>
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include "pins.h"
#include "imu.h"
#include "pid.h"
#include "servos.h"
#include "status_led.h"

// ---------------- parâmetros ----------------
static constexpr uint32_t LOOP_US        = 5000;   // 200 Hz
static constexpr uint8_t  TELEM_DIV      = 20;     // 10 Hz
// Desvio máximo de cada servo em relação ao centro [graus], por eixo,
// para proteger a mecânica: roll ±50, pitch ±40
static constexpr float    SERVO_LIMIT[2] = {50.0f, 40.0f};   // {roll, pitch}
static constexpr float    ANG_LIMIT      = 40.0f;  // corte de segurança em AUTO
static constexpr float    SP_LIMIT       = 20.0f;  // setpoint máximo
static constexpr uint32_t FOLLOW_TIMEOUT = 1000;   // ms sem "f" -> nível
static constexpr float    FOLLOW_TAU     = 0.15f;  // s, suavização da referência

enum Mode : uint8_t { OFF = 0, MANUAL = 1, AUTO = 2, FOLLOW = 3 };
static const char *MODE_NAME[] = {"OFF", "MANUAL", "AUTO", "SEGUIR"};

// ---------------- objetos ----------------
HardwareSerial BT(BT_RX_PIN, BT_TX_PIN);
ImuSensor imu;   // GY-85 (ver include/imu.h)
ComplementaryFilter filt(0.98f);
ServoPair servos;
PID pid[2];

// ---------------- estado ----------------
static Mode     mode      = OFF;
static bool     imuOk     = false;
static bool     servoOk   = false;
static bool     telemetry = true;
static float    sp[2]     = {0, 0};     // setpoints
static float    manual[2] = {0, 0};     // desvios em MANUAL
static float    trim[2]   = {0, 0};     // centro mecânico
static float    dir[2]    = {1, 1};     // sentido de atuação
static float    u[2]      = {0, 0};     // comando aplicado (desvio do centro)
static float    follow[2] = {0, 0};     // referência pedida pelo telemóvel
static float    followF[2]= {0, 0};     // referência suavizada
static uint32_t lastFollowMs = 0;
static uint32_t overruns  = 0, loopCount = 0;
static uint32_t faultUntil = 0;           // LED vermelho a piscar até este instante
StatusLed sled;

// ---------------- saída para os dois lados ----------------
static void both(const char *fmt, ...) {
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  BT.print(buf);
}

// ---------------- auxiliares ----------------
static void setMode(Mode m, const char *why = nullptr) {
  if ((m == AUTO || m == FOLLOW) && !imuOk) {
    both("%s recusado: IMU indisponivel\r\n", MODE_NAME[m]); return;
  }
  mode = m;
  pid[0].reset(); pid[1].reset();
  if (m != MANUAL) { manual[0] = manual[1] = 0; }
  if (m == FOLLOW) {
    follow[0] = follow[1] = 0;
    followF[0] = filt.roll; followF[1] = filt.pitch;   // parte de onde está
    lastFollowMs = millis();
  }
  if (m == OFF && why && (strstr(why, "angulo") || strstr(why, "IMU")))
    faultUntil = millis() + 5000;          // sinaliza o erro durante 5 s
  both("modo %s%s%s\r\n", MODE_NAME[m], why ? " - " : "", why ? why : "");
}

static void calibrate() {
  if (!imuOk) { both("IMU indisponivel\r\n"); return; }
  if (mode != OFF) setMode(OFF, "calibracao");
  both("A calibrar giroscopio - NAO mexer...\r\n");
  sled.update(LedState::CALIB);
  imu.calibrateGyro(1000);
  imu.update();
  filt.init(imu);
  both("Calibrado. Offsets: %.2f %.2f %.2f\r\n",
       imu.gyroOffsetX(), imu.gyroOffsetY(), imu.gyroOffsetZ());
}

static void status() {
  static uint32_t lastT = 0, lastCount = 0;
  uint32_t now = millis();
  float hz = (now != lastT) ? (loopCount - lastCount) * 1000.0f / (now - lastT) : 0;
  lastT = now; lastCount = loopCount;
  both("--- Estado ---\r\n");
  both("modo %s | IMU %s | servos %s\r\n", MODE_NAME[mode],
       imuOk ? "OK" : "FALHA", servoOk ? "OK" : "FALHA");
  both("ciclo %.1f Hz | overruns %lu | alpha %.3f\r\n", hz, overruns, filt.alpha);
  both("roll %.2f  pitch %.2f | sensor %s | gyro x%+.0f y%+.0f\r\n", filt.roll, filt.pitch,
       imu.upsideDown ? "ao contrario" : "normal", imu.gyroSignX, imu.gyroSignY);
  both("acel: roll %.2f  pitch %.2f\r\n", filt.rollAcc, filt.pitchAcc);
  both("sp   %.1f  %.1f\r\n", sp[0], sp[1]);
  both("PID r: %.3f %.3f %.3f\r\n", pid[0].kp, pid[0].ki, pid[0].kd);
  both("PID p: %.3f %.3f %.3f\r\n", pid[1].kp, pid[1].ki, pid[1].kd);
  both("trim %.1f %.1f | dir %+.0f %+.0f\r\n", trim[0], trim[1], dir[0], dir[1]);
  both("servo r %lu us  p %lu us\r\n", servos.us(0), servos.us(1));
}

static void help() {
  both("? h | m 0/1/2/3 | sp r p | f r p | g r/p/b kp ki kd | u r/p deg\r\n");
  both("t r/p deg | inv r/p | a x | c | s | r | l | flip | gs x/y\r\n");
}

// ---------------- parser de comandos ----------------
static int split(const String &s, String out[], int maxTok) {
  int n = 0, i = 0, len = s.length();
  while (i < len && n < maxTok) {
    while (i < len && s[i] == ' ') i++;
    if (i >= len) break;
    int j = i;
    while (j < len && s[j] != ' ') j++;
    out[n++] = s.substring(i, j);
    i = j;
  }
  return n;
}

static bool num(const String &s, float &v) {
  if (!s.length()) return false;
  char c = s[0];
  if (!(isDigit(c) || c == '-' || c == '+' || c == '.')) return false;
  v = s.toFloat();
  return true;
}

// devolve 0 (roll), 1 (pitch), 2 (ambos) ou -1
static int axisOf(const String &s) {
  if (s == "r") return 0;
  if (s == "p") return 1;
  if (s == "b") return 2;
  return -1;
}

static void handleCommand(String line) {
  line.trim();
  if (!line.length()) return;

  // comandos de uma letra: só se a mensagem for EXATAMENTE essa letra
  if (line.length() == 1) {
    switch (line[0]) {
      case '?': status(); break;
      case 'h': help(); break;
      case 'c': calibrate(); break;
      case 's': telemetry = !telemetry;
                both("telemetria %s\r\n", telemetry ? "ON" : "OFF"); break;
      case 'l': digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                both("LED %s\r\n", digitalRead(LED_BUILTIN) ? "ON" : "OFF"); break;
      case 'r':
        if (imu.update()) {
          both("ax=%.3f ay=%.3f az=%.3f\r\n", imu.ax, imu.ay, imu.az);
          both("gx=%.2f gy=%.2f gz=%.2f\r\n", imu.gx, imu.gy, imu.gz);
        } else both("leitura I2C falhou\r\n");
        both("ADXL345 id=0x%02X (esperado E5) | ITG-3205 id=0x%02X (esperado 68/69)\r\n", imu.accId(), imu.gyroId());
        break;
      default: break;
    }
    return;
  }

  String t[5];
  int n = split(line, t, 5);
  float v1, v2, v3;

  if (t[0] == "m" && n == 2 && num(t[1], v1)) {
    int m = (int)v1;
    if (m >= 0 && m <= 3) setMode((Mode)m);
  }
  else if (t[0] == "f" && n == 3 && num(t[1], v1) && num(t[2], v2)) {
    // referência do telemóvel: só conta em SEGUIR; sem resposta (vem a 15 Hz)
    if (mode == FOLLOW) {
      follow[0] = constrain(v1, -SP_LIMIT, SP_LIMIT);
      follow[1] = constrain(v2, -SP_LIMIT, SP_LIMIT);
      lastFollowMs = millis();
    }
  }
  else if (t[0] == "sp" && n == 3 && num(t[1], v1) && num(t[2], v2)) {
    sp[0] = constrain(v1, -SP_LIMIT, SP_LIMIT);
    sp[1] = constrain(v2, -SP_LIMIT, SP_LIMIT);
    both("setpoint r=%.1f p=%.1f\r\n", sp[0], sp[1]);
  }
  else if (t[0] == "g" && n == 5 && axisOf(t[1]) >= 0 &&
           num(t[2], v1) && num(t[3], v2) && num(t[4], v3) &&
           v1 >= 0 && v2 >= 0 && v3 >= 0) {
    int a = axisOf(t[1]);
    for (int i = 0; i < 2; i++) {
      if (a == 2 || a == i) {
        pid[i].kp = v1; pid[i].ki = v2; pid[i].kd = v3; pid[i].reset();
      }
    }
    both("ganhos %s: %.3f %.3f %.3f\r\n", t[1].c_str(), v1, v2, v3);
  }
  else if (t[0] == "u" && n == 3 && axisOf(t[1]) >= 0 && axisOf(t[1]) < 2 && num(t[2], v1)) {
    if (mode != MANUAL) { both("usar primeiro: m 1\r\n"); return; }
    int a = axisOf(t[1]);
    manual[a] = constrain(v1, -SERVO_LIMIT[a], SERVO_LIMIT[a]);
    both("manual %s = %.1f\r\n", t[1].c_str(), manual[a]);
  }
  else if (t[0] == "t" && n == 3 && axisOf(t[1]) >= 0 && axisOf(t[1]) < 2 && num(t[2], v1)) {
    int a = axisOf(t[1]);
    trim[a] = constrain(v1, -20.0f, 20.0f);
    both("trim %s = %.1f\r\n", t[1].c_str(), trim[a]);
  }
  else if (t[0] == "inv" && n == 2 && axisOf(t[1]) >= 0 && axisOf(t[1]) < 2) {
    int a = axisOf(t[1]);
    dir[a] = -dir[a];
    pid[a].reset();
    both("dir %s = %+.0f\r\n", t[1].c_str(), dir[a]);
  }
  else if (t[0] == "flip" && n == 1) {
    imu.upsideDown = !imu.upsideDown;
    if (mode != OFF) setMode(OFF, "orientacao do sensor mudou");
    calibrate();   // recalibra o giroscópio e reinicia o filtro
    both("sensor %s\r\n", imu.upsideDown ? "AO CONTRARIO" : "normal");
  }
  else if (t[0] == "gs" && n == 2 && (t[1] == "x" || t[1] == "y")) {
    float &sg = (t[1] == "x") ? imu.gyroSignX : imu.gyroSignY;
    sg = -sg;
    if (mode != OFF) setMode(OFF, "sinal do giroscopio mudou");
    calibrate();
    both("sinal gyro %s = %+.0f\r\n", t[1].c_str(), sg);
  }
  else if (t[0] == "a" && n == 2 && num(t[1], v1) && v1 >= 0.5f && v1 < 1.0f) {
    filt.alpha = v1;
    both("alpha = %.3f\r\n", filt.alpha);
  }
  // tudo o resto é ignorado (ruído / mensagens do módulo)
}

// Um comando termina com Enter OU após 50 ms sem chegar nada (apps BLE)
static void pollPort(Stream &port, String &buf, uint32_t &lastRx) {
  while (port.available()) {
    char c = port.read();
    lastRx = millis();
    if (c == '\n' || c == '\r') {
      if (buf.length()) { handleCommand(buf); buf = ""; }
    } else if (c >= 0x20 && c <= 0x7E && buf.length() < 40) {
      buf += c;
    }
  }
  if (buf.length() && millis() - lastRx > 50) { handleCommand(buf); buf = ""; }
}

// ---------------- setup ----------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  sled.begin();
  Serial.begin(115200);
  BT.begin(BT_BAUD);

  // Servos primeiro, para ficarem logo ao centro
  servoOk = servos.begin();
  for (int i = 0; i < 2; i++) { pid[i].outMin = -SERVO_LIMIT[i]; pid[i].outMax = SERVO_LIMIT[i]; }

  delay(300);
  both("\r\n=== Mesa Niveladora ===\r\n");
  if (!servoOk) both("ERRO: pinos dos servos nao estao no mesmo timer\r\n");

  Wire.setSDA(IMU_SDA_PIN);
  Wire.setSCL(IMU_SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);   // 100 kHz: mais tolerante a fios compridos

  imuOk = imu.begin();
  if (imuOk) {
    both("GY-85 OK (ADXL345 0x%02X, ITG-3205 0x%02X)\r\n", imu.accId(), imu.gyroId());
    calibrate();
  } else {
    both("ERRO: GY-85 nao responde\r\n");
  }
  both("modo OFF. Enviar 'h' para ajuda.\r\n");
}

// ---------------- loop ----------------
void loop() {
  static String usbBuf, btBuf;
  static uint32_t usbRx = 0, btRx = 0, tBeat = 0;
  static uint32_t next = micros(), last = micros();
  static uint8_t telDiv = 0;

  // ---- LEDs de estado ----
  {
    LedState ls;
    if (!imuOk || millis() < faultUntil)  ls = LedState::ERROR;
    else if (mode == MANUAL)               ls = LedState::MANUAL;
    else if (mode == AUTO)                 ls = LedState::ACTIVE;
    else if (mode == FOLLOW)
      ls = (millis() - lastFollowMs > FOLLOW_TIMEOUT) ? LedState::WAITING : LedState::ACTIVE;
    else                                   ls = LedState::IDLE;
    sled.update(ls);
  }

  pollPort(Serial, usbBuf, usbRx);
  pollPort(BT, btBuf, btRx);

  // sinal de vida quando não há telemetria útil
  if ((!imuOk || !telemetry) && millis() - tBeat >= 2000) {
    tBeat = millis();
    BT.printf("STM32 ok t=%lus modo %s IMU:%s\r\n",
              millis() / 1000, MODE_NAME[mode], imuOk ? "OK" : "FALHA");
  }

  // período fixo
  if ((int32_t)(micros() - next) < 0) return;
  next += LOOP_US;
  if ((int32_t)(micros() - next) > 0) { overruns++; next = micros() + LOOP_US; }
  loopCount++;

  uint32_t now = micros();
  float dt = (now - last) * 1e-6f;
  last = now;

  // ---- medida ----
  if (imuOk) {
    if (imu.update()) {
      filt.update(imu, dt);
    } else if (mode != OFF) {
      setMode(OFF, "falha de leitura do IMU");
    }
  }
  const float ang[2]  = {filt.roll, filt.pitch};
  const float rate[2] = {imu.gx, imu.gy};

  // ---- segurança ----
  if ((mode == AUTO || mode == FOLLOW) && (fabsf(ang[0]) > ANG_LIMIT || fabsf(ang[1]) > ANG_LIMIT)) {
    setMode(OFF, "angulo fora do limite");
  }

  // ---- controlo ----
  for (int i = 0; i < 2; i++) {
    switch (mode) {
      case OFF:    u[i] = 0; break;
      case MANUAL: u[i] = manual[i]; break;
      case AUTO:   u[i] = pid[i].update(sp[i] - ang[i], rate[i], dt); break;
      case FOLLOW: {
        float target = (millis() - lastFollowMs > FOLLOW_TIMEOUT) ? 0.0f : follow[i];
        followF[i] += (target - followF[i]) * dt / (FOLLOW_TAU + dt);
        u[i] = pid[i].update(followF[i] - ang[i], rate[i], dt);
        break;
      }
    }
    float cmd = 90.0f + trim[i] + dir[i] * u[i];
    cmd = constrain(cmd, 90.0f - SERVO_LIMIT[i], 90.0f + SERVO_LIMIT[i]);   // limite final, inclui o trim
    servos.writeDeg(i, cmd);
  }

  // ---- telemetria ----
  if (telemetry && imuOk && ++telDiv >= TELEM_DIV) {
    telDiv = 0;
    BT.printf("M%d R:%.1f P:%.1f uR:%.1f uP:%.1f\r\n",
              mode, ang[0], ang[1], u[0], u[1]);
  }
}
