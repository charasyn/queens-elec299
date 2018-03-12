#include <Servo.h>

#define PIN_SERVO_PAN   (9)
#define PIN_SERVO_TILT (10)
#define PIN_SERVO_GRIP (11)
#define PIN_LED        (13)

#define APIN_GRIPFORCE (5)

Servo sPan, sTilt, sGrip;

void setup() {
  Serial.begin(115200);
  
  sPan.attach(PIN_SERVO_PAN);
  sTilt.attach(PIN_SERVO_TILT);
  sGrip.attach(PIN_SERVO_GRIP);

  sPan.write(109);
  sTilt.write(70);
  sGrip.write(40);
}

void loop() {
  while(Serial.available() <= 0);
  char c = Serial.read();
  int pos = Serial.parseInt();
  switch (c) {
    case 'p':
      sPan.write(pos);
      break;
    case 't':
      sTilt.write(pos);
      break;
    case 'g':
      sGrip.write(pos);
      break;
  }
  
  // grip = 93, tilt = 60 to grab ball
  // grip = 50, tilt = 110 to drop ball
  // p = 109 for straight ahead
  // p = 20 for left
  // p = 180 for right
  
}
