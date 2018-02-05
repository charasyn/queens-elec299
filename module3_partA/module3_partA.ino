#include <Servo.h>

#define PIN_SERVO_PAN   (9)
#define PIN_SERVO_TILT (10)
#define PIN_SERVO_GRIP (11)
#define PIN_LED        (13)

#define APIN_GRIPFORCE (5)

Servo sPan, sTilt, sGrip;

void setup() {
  pinMode(PIN_LED,OUTPUT);
  sPan.attach(PIN_SERVO_PAN);
  sTilt.attach(PIN_SERVO_TILT);
  sGrip.attach(PIN_SERVO_GRIP);

  sPan.write(109);
  sTilt.write(70);

  digitalWrite(PIN_LED,0);
}

void loop() {
  int curGrip = 40;
  for(;;){
    sGrip.write(curGrip);

    if(analogRead(APIN_GRIPFORCE) > 40)
      break;

    curGrip++;
    delay(50);
  }
  digitalWrite(PIN_LED,1);
  for(;;);

}
