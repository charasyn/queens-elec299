#include "QSerial.h"

#define PIN_IR_TX (12)

QSerial irSerial;

void setup() {
  irSerial.attach(-1, PIN_IR_TX);
}

void loop() {
  irSerial.transmit('Q');
  delay(100);
}
