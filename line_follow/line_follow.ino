#define PIN_R_SPEED (6)
#define PIN_R_DIRN (7)
#define PIN_L_SPEED (5)
#define PIN_L_DIRN (4)

#define PIN_LINE_L (0)
#define PIN_LINE_R (1)
#define PIN_LINE_C (2)

#define PIN_LED_L (8)
#define PIN_LED_R (9)

#define BUFAVG_SIZE (8)

int bufLeft[BUFAVG_SIZE], bufCent[BUFAVG_SIZE], bufRight[BUFAVG_SIZE];
int bufPos;

int bufAvg(int *buf){
  int accum = 0;
  for(int i=0;i<BUFAVG_SIZE;i++)
    accum += buf[i];
  return accum / BUFAVG_SIZE;
}

void setup() {
  pinMode(PIN_L_DIRN,OUTPUT);
  pinMode(PIN_R_DIRN,OUTPUT);
  pinMode(PIN_L_SPEED,OUTPUT);
  pinMode(PIN_R_SPEED,OUTPUT);
  for(;;){
    bufLeft[bufPos] = analogRead(PIN_LINE_L);
    bufCent[bufPos] = analogRead(PIN_LINE_C);
    bufRight[bufPos] = analogRead(PIN_LINE_R);
    bufPos++;
    int l,r,c;
    int ls = 85, rs = 85;
    int ll = 0 , rl = 0;
    
    if(bufPos>=BUFAVG_SIZE) bufPos=0;
    (l = bufAvg(bufLeft));
    (c = bufAvg(bufCent));
    (r = bufAvg(bufRight));
    if(r > c && r > l){
      rl = 1;
      rs -= 30;
    }
    else if(l > c && l > r){
      ll = 1;
      ls -= 30;
    }
      digitalWrite(PIN_LED_R,rl);
      digitalWrite(PIN_LED_L,ll);

  digitalWrite(PIN_L_DIRN,1);
  digitalWrite(PIN_R_DIRN,1);
  analogWrite(PIN_L_SPEED,ls);
  analogWrite(PIN_R_SPEED,rs);
    
    delay(5);
  }
}

/*
void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_L_DIRN,OUTPUT);
  pinMode(PIN_R_DIRN,OUTPUT);
  pinMode(PIN_L_SPEED,OUTPUT);
  pinMode(PIN_R_SPEED,OUTPUT);
  digitalWrite(PIN_L_DIRN,0);
  digitalWrite(PIN_R_DIRN,1);
  analogWrite(PIN_L_SPEED,255);
  analogWrite(PIN_R_SPEED,255);
  delay(2000);
  
  analogWrite(PIN_L_SPEED,0);
  analogWrite(PIN_R_SPEED,0);
}
*/
void loop() {
  // put your main code here, to run repeatedly:
  
}
