#include <Servo.h>

#define PIN_ENC_L (3)
#define PIN_ENC_R (2)
#define PIN_R_SPEED (6)
#define PIN_R_DIRN  (7)
#define PIN_L_SPEED (5)
#define PIN_L_DIRN  (4)

#define PIN_SERVO_PAN   (9)
#define PIN_SERVO_TILT (10)
#define PIN_SERVO_GRIP (11)

#define PIN_IR_RX (A5)
#define PIN_LED   (13)

#define APIN_DIST      (A3)

#define SPEED (255)

Servo sPan, sTilt, sGrip;

void SetMotorSpeeds(int l, int r) {
  digitalWrite(PIN_L_DIRN,l>0);
  digitalWrite(PIN_R_DIRN,r>0);
  analogWrite(PIN_L_SPEED,abs(l));
  analogWrite(PIN_R_SPEED,abs(r));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PIN_L_DIRN,OUTPUT);
  pinMode(PIN_R_DIRN,OUTPUT);
  pinMode(PIN_L_SPEED,OUTPUT);
  pinMode(PIN_R_SPEED,OUTPUT);
  pinMode(PIN_ENC_L,INPUT);
  pinMode(PIN_ENC_R,INPUT);

  sPan.attach(PIN_SERVO_PAN);
  sTilt.attach(PIN_SERVO_TILT);
  sGrip.attach(PIN_SERVO_GRIP);

  sPan.write(109);
  sTilt.write(140);
  sGrip.write(40);
}

void loop() {
  int lc = 0, rc = 0;
  int lp = digitalRead(PIN_ENC_L), rp = digitalRead(PIN_ENC_R);
  for(int i = 0; i < 200; i++) {
    if(Serial.available() > 0)
      for(;;) SetMotorSpeeds(0,0);
    if(digitalRead(PIN_ENC_L) != lp) lc++;
    if(digitalRead(PIN_ENC_R) != rp) rc++;
    lp = digitalRead(PIN_ENC_L), rp = digitalRead(PIN_ENC_R);
    if(analogRead(APIN_DIST) >= 250) SetMotorSpeeds(0,0);
    else SetMotorSpeeds(SPEED,SPEED);
    delay(5);
  }
  Serial.print("Left count: ");
  Serial.print(lc);
  Serial.print("\tRight count: ");
  Serial.print(rc);
  Serial.println();
}
