#include <Servo.h>

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

#define FAST_SPEED (130)
#define SLOW_SPEED (75)

#define BACK_DIST (6)

#define M_L_SPEED(x) (x)
#define M_R_SPEED(x) (x*12/10)


Servo sPan, sTilt, sGrip;

int l_count = 0;
int r_count = 0;
int l_stat;
int r_stat;

void countEncoder(void){
  int l_cur, r_cur;
  l_cur = digitalRead(PIN_ENC_L);
  r_cur = digitalRead(PIN_ENC_R);
  if(l_cur != l_stat) {
    l_count ++;
    l_stat = l_cur;
  }
  if(r_cur != r_stat) {
    r_count ++;
    r_stat = r_cur;
  }
}

void driveDist(int counts){
  int l_start_count = l_count;
  int r_start_count = r_count;
  for(;;){
    // detect encoder
    countEncoder();
    // adjust speed
    if((l_count - l_start_count) >= counts - 1)
      analogWrite(PIN_L_SPEED, SLOW_SPEED);
    if((r_count - r_start_count) >= counts - 1)
      analogWrite(PIN_R_SPEED, SLOW_SPEED);
    // stop
    if(((l_count - l_start_count) >= counts) &&
       ((r_count - r_start_count) >= counts))
    {
      break;
    }
  }
}

void setSpeeds(int l, int r){
  digitalWrite(PIN_L_DIRN,l>0);
  digitalWrite(PIN_R_DIRN,r>0);
  analogWrite(PIN_L_SPEED,M_L_SPEED(abs(l)));
  analogWrite(PIN_R_SPEED,M_R_SPEED(abs(r)));
}

void reactBumpers(){
  int lb, rb;
  lb = !digitalRead(PIN_L_BUMPER);
  rb = !digitalRead(PIN_R_BUMPER);

  if(lb && rb){
    setSpeeds(-SLOW_SPEED,-SLOW_SPEED);
    driveDist(BACK_DIST);
    setSpeeds(-SLOW_SPEED,SLOW_SPEED);
    driveDist(20); // determined through trial and error
  }
  else if(lb) {
    setSpeeds(-SLOW_SPEED,-SLOW_SPEED);
    driveDist(BACK_DIST);
    setSpeeds(SLOW_SPEED,-SLOW_SPEED);
    driveDist(10); // determined through trial and error
  }
  else if(rb) {
    setSpeeds(-SLOW_SPEED,-SLOW_SPEED);
    driveDist(BACK_DIST);
    setSpeeds(-SLOW_SPEED,SLOW_SPEED);
    driveDist(10); // determined through trial and error
  }
}

void setup() {
  // put your setup code here, to run once:
  // pin configs
  
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
  
  l_stat = digitalRead(PIN_ENC_L);
  r_stat = digitalRead(PIN_ENC_R);
}

void loop() {
  // drive forward
  setSpeeds(FAST_SPEED,FAST_SPEED);
  // handle bumpers
  reactBumpers();

  // make it not react super instantly
  // if this is too low, it will never turn 180
  delay(50);
}
