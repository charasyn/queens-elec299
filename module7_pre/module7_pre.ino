#include <Servo.h>
#include <EEPROM.h>

#define PIN_SERVO_PAN   (9)
#define PIN_SERVO_TILT (10)
#define PIN_SERVO_GRIP (11)

#define PIN_ENC_L (3)
#define PIN_ENC_R (2)

#define PIN_R_SPEED (6)
#define PIN_R_DIRN  (7)
#define PIN_L_SPEED (5)
#define PIN_L_DIRN  (4)

#define PIN_L_BUMPER ( 8)
#define PIN_R_BUMPER (12)

#define APIN_GRIPFORCE (5)
#define APIN_DIST      (3)

#define FAST_SPEED (130)
#define SLOW_SPEED (75)

#define BACK_DIST (6)

#define M_L_SPEED(x) (x)
#define M_R_SPEED(x) (x*12/10)

#define ADD_DELTA (2)
#define SUB_DELTA (2)

Servo sPan, sTilt, sGrip;

void setSpeedsRaw(int l, int r){
  digitalWrite(PIN_L_DIRN,l>0);
  digitalWrite(PIN_R_DIRN,r>0);
  analogWrite(PIN_L_SPEED,abs(l));
  analogWrite(PIN_R_SPEED,abs(r));
}

void setup() {
  sPan.attach(PIN_SERVO_PAN);
  sTilt.attach(PIN_SERVO_TILT);
  sGrip.attach(PIN_SERVO_GRIP);
  
  sPan.write(109);
  sTilt.write(160);
  sGrip.write(40);
  
  pinMode(PIN_L_BUMPER,INPUT);
  pinMode(PIN_L_BUMPER,INPUT);
  pinMode(PIN_L_DIRN,OUTPUT);
  pinMode(PIN_R_DIRN,OUTPUT);
  pinMode(PIN_L_SPEED,OUTPUT);
  pinMode(PIN_R_SPEED,OUTPUT);
  pinMode(PIN_ENC_L,INPUT);
  pinMode(PIN_ENC_R,INPUT);

  Serial.begin(115200);
}

void loop() {
  // drive forward
  setSpeedsRaw(0,0);
  Serial.print("Raw value: ");
  Serial.println(analogRead(APIN_DIST));
  delay(100);
}
