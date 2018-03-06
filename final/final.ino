#include "QSerial.h"
#include "MovingAverage.hpp"
#define PIN_IR_RX (A5)
#define PIN_LED   (13)

#define APIN_DIST      (A3)

QSerial irSerial;
MovingAverage distAvg(16);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_IR_RX,   INPUT);
  pinMode(PIN_LED,    OUTPUT);
  pinMode(APIN_DIST,   INPUT);
  //irSerial.attach(PIN_IR_RX, -1);
}

void loop() {
  Serial.println(distAvg.AddSample(analogRead(APIN_DIST)));
  delay(20);
  /*
  // put your main code here, to run repeatedly:
  int received = irSerial.receive(200);
  if(received < 0) {
    Serial.print("Error ");
    Serial.println(received);
    digitalWrite(PIN_LED,LOW);
  }
  else if (received == 0) {
    
  }
  else{
    digitalWrite(PIN_LED,HIGH);
    Serial.print("Received ");
    Serial.println((char)received);
  }
  */
}
