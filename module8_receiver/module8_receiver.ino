#include "QSerial.h"

#define PIN_IR_RX (12)

QSerial irSerial;

void setup() {
  Serial.begin(115200);
  irSerial.attach(PIN_IR_RX, -1);
}

void loop() {
  // why 200? timeout in ms? (yep, source says so)
  int received = irSerial.receive(200);
  if(received < 0) {
    Serial.print("Error ");
    Serial.println(received);
  }
  else {
    Serial.println((char)received);
  }
}
