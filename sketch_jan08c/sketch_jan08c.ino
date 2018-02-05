#define PIN_R_SPEED (6)
#define PIN_R_DIRN (7)
#define PIN_L_SPEED (5)
#define PIN_L_DIRN (4)

#define PIN_LINE_L (0)
#define PIN_LINE_R (1)
#define PIN_LINE_C (2)

#define PIN_LED_L (8)
#define PIN_LED_R (9)

int bufLeft[16], bufCent[16], bufRight[16];
int bufPos;

int bufAvg(int *buf){
  int accum = 0;
  for(int i=0;i<16;i++)
    accum += buf[i];
  return accum / 16;
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
    int ls = 50, rs = 65;
    
    if(bufPos>=16) bufPos=0;
    Serial.print("L: ");
    Serial.print(l = bufAvg(bufLeft));
    Serial.print("\tC: ");
    Serial.print(c = bufAvg(bufCent));
    Serial.print("\tR: ");
    Serial.print(r = bufAvg(bufRight));
    if(r > c && r > l){
      Serial.print(" rotate CW");
      rs -= 10;
    }
    else if(l > c && l > r){
      Serial.print(" rotate CCW");
      ls -= 10;
    }
    Serial.println();

  digitalWrite(PIN_L_DIRN,1);
  digitalWrite(PIN_R_DIRN,1);
  analogWrite(PIN_L_SPEED,ls);
  analogWrite(PIN_R_SPEED,rs);
    
    delay(10);
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
