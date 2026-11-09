// Teste só com a placa + USB (sem módulos ligados)
// - LD1 -> LD2 -> LD3 acendem em sequência
// - A cada segundo envia "vivo N" para o PC
// - Comandos: 1/2/3 alterna LD1/LD2/LD3 | s retoma a sequência
#include <Arduino.h>

static const uint32_t LEDS[3] = {PB0, PB7, PB14}; // LD1, LD2, LD3
static bool sequencia = true;

static void apagaTodos()
{
  for (uint32_t p : LEDS)
    digitalWrite(p, LOW);
}

void setup()
{
  for (uint32_t p : LEDS)
    pinMode(p, OUTPUT);
  Serial.begin(115200);
}

void loop()
{
  static uint32_t tLed = 0, tMsg = 0, n = 0;
  static uint8_t idx = 0;

  if (sequencia && millis() - tLed >= 300)
  {
    tLed = millis();
    apagaTodos();
    digitalWrite(LEDS[idx], HIGH);
    idx = (idx + 1) % 3;
  }

  if (millis() - tMsg >= 1000)
  {
    tMsg = millis();
    Serial.print("vivo ");
    Serial.println(n++);
  }

  while (Serial.available())
  {
    char c = Serial.read();
    if (c >= '1' && c <= '3')
    {
      if (sequencia)
      {
        sequencia = false;
        apagaTodos();
      }
      uint32_t p = LEDS[c - '1'];
      digitalWrite(p, !digitalRead(p));
      Serial.print("LD");
      Serial.print(c);
      Serial.println(" alternado");
    }
    else if (c == 's')
    {
      sequencia = true;
      Serial.println("sequencia retomada");
    }
  }
}