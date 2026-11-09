// Teste 0 - Diagnóstico mínimo: a placa corre código e o USB série funciona?
// - LED verde (LD1) pisca a cada 0,5 s  -> o programa está a correr
// - "vivo N" no monitor a cada 0,5 s    -> a porta série (COM) funciona
// - O que escreveres é devolvido como "recebi: x"
//
// Usa a USART3 (PD9/PD8) EXPLICITAMENTE - é a que está ligada ao ST-Link
// (Virtual COM) na Nucleo-144 - em vez de confiar no mapeamento do "Serial".
//
// LEDs de diagnóstico (fixos):
//   LD2 azul     aceso -> o framework mapeou "Serial" para USB nativo (CN13)
//   LD3 vermelho aceso -> "Serial" NÃO está na USART3

#include <Arduino.h>
// LEDs da Nucleo-144: LD1 verde = PB0, LD2 azul = PB7, LD3 vermelho = PB14

Uart PC(PD9, PD8);   // RX = PD9, TX = PD8 (USART3 -> ST-Link VCP)

void setup() {
  pinMode(PB0, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB14, OUTPUT);

#if defined(USBCON) && defined(USBD_USE_CDC)
  digitalWrite(PB7, HIGH);
#endif
#if !defined(SERIAL_UART_INSTANCE) || (SERIAL_UART_INSTANCE != 3)
  digitalWrite(PB14, HIGH);
#endif

  PC.begin(115200);
}

void loop() {
  static uint32_t last = 0, n = 0;
  if (millis() - last >= 500) {
    last = millis();
    digitalWrite(PB0, !digitalRead(PB0));
    PC.print("vivo ");
    PC.println(n++);
  }
  while (PC.available()) {
    PC.print("recebi: ");
    PC.println((char)PC.read());
  }
}
