#include "QSerial.h"

#define PIN_IR_RX (A5)
#define PIN_LED   (13)

QSerial irSerial;

char buf[100];
byte index;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_IR_RX, INPUT);
  pinMode(PIN_LED, OUTPUT);
  irSerial.attach(PIN_IR_RX, -1);
  index = 0;
}

void loop() {
  // why 200? timeout in ms? (yep, source says so)
  int received = irSerial.receive(2000);
  if(received < 0) {
    Serial.print("Error ");
    Serial.println(received);
    digitalWrite(13,LOW);
  }
  else if (received == 0) {
    
  }
  else{
    digitalWrite(13,HIGH);
    Serial.print("Received ");
    Serial.print((char)received);
    Serial.print("   Str: ");
    buf[index++] = received;
    buf[index] = 0;
    Serial.println(buf);
    if(received == '\n' || index >= 100){
      index = 0;
    }
  }
}
