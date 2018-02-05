#define PIN_ENC_L (3)
#define PIN_ENC_R (2)

#define PIN_R_SPEED (6)
#define PIN_R_DIRN (7)
#define PIN_L_SPEED (5)
#define PIN_L_DIRN (4)

#define FAST_SPEED (127)
#define SLOW_SPEED (63)

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

void setup() {
  // put your setup code here, to run once:
  // pin configs
  pinMode(PIN_ENC_L,INPUT);
  pinMode(PIN_ENC_R,INPUT);
  pinMode(PIN_L_DIRN,OUTPUT);
  pinMode(PIN_R_DIRN,OUTPUT);
  pinMode(PIN_L_SPEED,OUTPUT);
  pinMode(PIN_R_SPEED,OUTPUT);
  
  l_stat = digitalRead(PIN_ENC_L);
  r_stat = digitalRead(PIN_ENC_R);
}

void loop() {
  // put your main code here, to run repeatedly:
  // go straight
  digitalWrite(PIN_L_DIRN,1);
  digitalWrite(PIN_R_DIRN,1);
  analogWrite(PIN_L_SPEED,FAST_SPEED*10/13);
  analogWrite(PIN_R_SPEED,FAST_SPEED);
  driveDist(16 * 3);
  
  // go back
  digitalWrite(PIN_L_DIRN,0);
  digitalWrite(PIN_R_DIRN,0);
  analogWrite(PIN_L_SPEED,FAST_SPEED*10/13);
  analogWrite(PIN_R_SPEED,FAST_SPEED);
  driveDist(16 * 1);
  
  // 90 deg. turn
  digitalWrite(PIN_L_DIRN,1);
  digitalWrite(PIN_R_DIRN,0);
  analogWrite(PIN_L_SPEED,FAST_SPEED*10/13);
  analogWrite(PIN_R_SPEED,FAST_SPEED);
  driveDist(10);

  analogWrite(PIN_L_SPEED,0);
  analogWrite(PIN_R_SPEED,0);
  for(;;);
}
