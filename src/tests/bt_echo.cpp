// Teste 2 - Ponte entre o monitor série (USB/ST-Link) e o módulo Bluetooth
//
// - O que escreves no monitor série segue para o módulo (dá para comandos AT).
// - O que o iPhone envia (LightBlue -> FFE1 -> Write) aparece no monitor.
// - A cada 2 s o STM32 envia "heartbeat" para o iPhone (ativar Notify em FFE1).

#include <Arduino.h>
#include "pins.h"

HardwareSerial BT(BT_RX_PIN, BT_TX_PIN);  // STM32duino 2.x

void setup() {
  Serial.begin(115200);
  BT.begin(BT_BAUD);
  delay(500);
  Serial.println(F("\n=== Teste Bluetooth (ponte USB <-> BT) ==="));
  Serial.printf("BT em USART6, %d baud\n", BT_BAUD);
}

void loop() {
  while (Serial.available()) BT.write(Serial.read());
  while (BT.available())     Serial.write(BT.read());

  static uint32_t last = 0;
  if (millis() - last >= 2000) {
    last = millis();
    BT.printf("STM32 ok t=%lu s\r\n", millis() / 1000);
  }
}
