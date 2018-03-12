#include <Servo.h>
#include "QSerial.h"
#include "MovingAverage.hpp"

#define PIN_R_SPEED (6)
#define PIN_R_DIRN  (7)
#define PIN_L_SPEED (5)
#define PIN_L_DIRN  (4)

#define PIN_IR_RX (A5)
#define PIN_LED   (13)

#define PIN_SERVO_PAN   (9)
#define PIN_SERVO_TILT (10)
#define PIN_SERVO_GRIP (11)

#define APIN_LINE_L (A0)
#define APIN_LINE_R (A1)
#define APIN_LINE_C (A2)
#define APIN_DIST   (A3)

Servo sPan, sTilt, sGrip;
QSerial irSerial;
MovingAverage distAvg(8);
MovingAverage lAvg(4);
MovingAverage cAvg(4);
MovingAverage rAvg(4);

void SetMotorSpeeds(int l, int r) {
  digitalWrite(PIN_L_DIRN,l>0);
  digitalWrite(PIN_R_DIRN,r>0);
  analogWrite(PIN_L_SPEED,abs(l));
  analogWrite(PIN_R_SPEED,abs(r));
}

// returns:
//   2 if we need to turn counterclockwise
//   1 if we need to turn clockwise
//   0 if we're good (following the line)
//  -1 if we need to stop
int TurnFromLineFollow(void) {
  // Take average of past 16 readings
  int l = lAvg.AddSample(analogRead(APIN_LINE_L));
  int c = cAvg.AddSample(analogRead(APIN_LINE_C));
  int r = rAvg.AddSample(analogRead(APIN_LINE_R));
  if(r > 900 && c > 900 && l > 900){
    return -1;
  }
  // Taken from line_follow (for now)
  if(r > c && r > l){
    return 1;
  }
  if(l > c && l > r){
    return 2;
  }
  return 0;
}

void DriveAlongLine(void) {
  bool distPastPeak = false;
  uint16_t distVal;
  int baseSpeed;
  int turnResult;
  lAvg.ResetToValue(analogRead(APIN_LINE_L));
  cAvg.ResetToValue(analogRead(APIN_LINE_C));
  rAvg.ResetToValue(analogRead(APIN_LINE_R));
  distVal = distAvg.ResetToValue(analogRead(APIN_DIST));
  do {
    // Calculate base speed
    distVal = distAvg.AddSample(analogRead(APIN_DIST));
    if(distPastPeak) {
           if(distVal < 380) break;
      else if(distVal < 500) baseSpeed = 80;
      else                   baseSpeed = 90;
    }
    else {
           if(distVal < 300) baseSpeed = 200; // FULL SPEED AHEAD
      else if(distVal < 400) baseSpeed = 120;
      else if(distVal < 500) baseSpeed = 100;
      else                 { baseSpeed =  90; distPastPeak = true;}
    }

    // Set motor speeds based on need to turn
    turnResult = TurnFromLineFollow();
    if(turnResult < 0)
      break;
    switch(turnResult) {
      case 0:
        SetMotorSpeeds(baseSpeed, baseSpeed);
        break;
      case 1:
        SetMotorSpeeds(baseSpeed, baseSpeed - 100);
        break;
      case 2:
        SetMotorSpeeds(baseSpeed - 100, baseSpeed);
        break;
        
    }

    // Delay to allow time for robot to move
    delay(20);
  } while (1);
  SetMotorSpeeds(0,0);
}
void DriveBackwardsAlongLine(void) {
  bool distPastPeak = false;
  uint16_t distVal;
  int baseSpeed;
  int turnResult;
  lAvg.ResetToValue(analogRead(APIN_LINE_L));
  cAvg.ResetToValue(analogRead(APIN_LINE_C));
  rAvg.ResetToValue(analogRead(APIN_LINE_R));
  distVal = distAvg.ResetToValue(analogRead(APIN_DIST));
  do {
    // Calculate base speed
    baseSpeed = 200;

    // Set motor speeds based on need to turn
    turnResult = TurnFromLineFollow();
    if(turnResult < 0)
      break;
    switch(turnResult) {
      case 0:
        SetMotorSpeeds(-baseSpeed, -baseSpeed);
        break;
      case 1:
        SetMotorSpeeds(-baseSpeed, -(baseSpeed - 100));
        break;
      case 2:
        SetMotorSpeeds(-(baseSpeed - 100), -baseSpeed);
        break;
        
    }

    // Delay to allow time for robot to move
    delay(20);
  } while (1);
  SetMotorSpeeds(0,0);
}


void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  pinMode(PIN_L_DIRN,  OUTPUT);
  pinMode(PIN_R_DIRN,  OUTPUT);
  pinMode(PIN_L_SPEED, OUTPUT);
  pinMode(PIN_R_SPEED, OUTPUT);
  pinMode(PIN_IR_RX,   INPUT);
  pinMode(PIN_LED,    OUTPUT);
  pinMode(APIN_DIST,   INPUT);

  sPan.attach(PIN_SERVO_PAN);
  sTilt.attach(PIN_SERVO_TILT);
  sGrip.attach(PIN_SERVO_GRIP);
  
  sPan.write(109);
  sTilt.write(70);
  sGrip.write(50);
  
  //irSerial.attach(PIN_IR_RX, -1);
  distAvg.ResetToValue(analogRead(APIN_DIST));
}

void GrabBall(void) {
  sTilt.write(60);
  delay(200);
  sGrip.write(93);
  delay(300);
  sTilt.write(110);
  delay(100);
}

void DropBall(void) {
  sGrip.write(50);
  delay(300);
}

void TurnCW(int deg) {
  if(deg == 90) {
    SetMotorSpeeds(200, -240);
    delay(440);
    SetMotorSpeeds(0,0);
    return;
  }
  if(deg == 180) {
    SetMotorSpeeds(200, -240);
    delay(870);
    SetMotorSpeeds(0,0);
    return;
  }
}

void GoForwardABit(void) {
  SetMotorSpeeds(240, 240); // balanced straightness???
  delay(130);
  SetMotorSpeeds(0,0);
}

void loop() {
  delay(1000);
  DriveAlongLine();
  GrabBall();
  TurnCW(180);
  DriveAlongLine();
  GoForwardABit();
  TurnCW(90);
  DriveAlongLine();
  DropBall();
  for(;;){}
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
