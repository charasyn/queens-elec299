#include "secret.h"

Servo sPan, sTilt, sGrip;
QSerial irSerial;
MovingAverage distAvg(8);
MovingAverage lAvg(4);
MovingAverage cAvg(4);
MovingAverage rAvg(4);
int swag = 0;
int speed_L, speed_R;

void SetMotorSpeeds(int l, int r) {
  digitalWrite(PIN_L_DIRN,l>0);
  digitalWrite(PIN_R_DIRN,r>0);
  analogWrite(PIN_L_SPEED,abs(l));
  analogWrite(PIN_R_SPEED,abs(r));
  speed_L = l;
  speed_R = r;
}

void ReduceMotorSpeeds(int nTimes16) {
  int l = speed_L * nTimes16 / 16;
  int r = speed_R * nTimes16 / 16;
  digitalWrite(PIN_L_DIRN,l>0);
  digitalWrite(PIN_R_DIRN,r>0);
  analogWrite(PIN_L_SPEED,abs(l));
  analogWrite(PIN_R_SPEED,abs(r));
}

void PANIC() {
  for(;;){
    sTilt.write(180);
    delay(400);
    sTilt.write(90);
    delay(400);
  }
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
  if(r > LF_STOP_THRESH
    && c > LF_STOP_THRESH
    && l > LF_STOP_THRESH){
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
// returns:
//  +'ve > 1 if we need to turn clockwise
//   1 for all white
//   0 if we're mostly on the center
//  -1 for all black
//  -'ve < -1 if we need to turn counter-clockwise
int TurnFromLineFollow_Better(void) {
  // Take average of past 16 readings
  int l = lAvg.AddSample(analogRead(APIN_LINE_L));
  int c = cAvg.AddSample(analogRead(APIN_LINE_C));
  int r = rAvg.AddSample(analogRead(APIN_LINE_R));
  int l_adj = max(min((l - 676) * 64 / 302 , 63), 0);
  int c_adj = max(min((c - 756) * 64 / 219 , 63), 0);
  int r_adj = max(min((r - 779) * 64 / 208 , 63), 0);
  if(l_adj < LF_WHITE_THRESH
    && c_adj < LF_WHITE_THRESH
    && r_adj < LF_WHITE_THRESH) {
    return 1;
  }
  if(l_adj > LF_BLACK_THRESH
    && c_adj > LF_BLACK_THRESH
    && r_adj > LF_BLACK_THRESH) {
    return -1;
  }
  // Taken from line_follow (for now)
  if(r_adj > c_adj){
    return max(r_adj - c_adj, 2);
  }
  if(l_adj > c_adj){
    return -(max(l_adj - c_adj, 2));
  }
  return 0;
}
void StartLineFollowFromDiagonal(void) {
  bool distPastPeak = false, backToLine = false, keepGoing = true;
  int loopCount = 0;
  uint16_t distVal;
  uint16_t baseSpeed;
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

    if(backToLine) {
           if(loopCount < 10) baseSpeed = min(baseSpeed, 150);
      else if(loopCount < 40) baseSpeed = min(baseSpeed, 150);
      else if(loopCount < 50) baseSpeed = min(baseSpeed, 200);
    }
    else {
           if(loopCount < 30) baseSpeed = min(baseSpeed, 200);
      else if(loopCount < 50) baseSpeed = min(baseSpeed, 150);
      else if(loopCount < 70) baseSpeed = min(baseSpeed, 100);
      else                    baseSpeed = min(baseSpeed,  80);
    }

    // Set motor speeds based on need to turn
    turnResult = TurnFromLineFollow_Better();
    switch(turnResult) {
      case -1: // all black
        keepGoing = false;
        break;
      case 0: // balanced
        if(!backToLine) {
          backToLine = true;
          loopCount = 0;
          SetMotorSpeeds(200,100);
          delay(200);
          break;
        }
      case 1: // all white
        // go straight
        SetMotorSpeeds(baseSpeed, baseSpeed * 17 / 16);
        break;
      default:
        if(!backToLine) break;
        if(turnResult > 0) {
          SetMotorSpeeds(baseSpeed, max((baseSpeed - turnResult * 3),0));
        }
        else {
          SetMotorSpeeds(max((baseSpeed - (-turnResult) * 3),0), baseSpeed);
        }
        
    }

    // Delay to allow time for robot to move
    delay(20);
    loopCount ++;
  } while (keepGoing);
  SetMotorSpeeds(0,0);
}

void DriveAlongLine(bool towardsBall) {
  bool distPastPeak = false, keepGoing = true;
  uint16_t distVal;
  uint16_t baseSpeed;
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
           if(distVal < 300) baseSpeed = 150; // FULL SPEED AHEAD
      else if(distVal < 400) baseSpeed = 120;
      else if(distVal < 450) baseSpeed = 100;
      else                 { baseSpeed =  90; distPastPeak = true;}
    }

    if(distPastPeak && towardsBall) {
        sPan.write(109);
        sGrip.write(50);
        sTilt.write(60);
    }

    // Set motor speeds based on need to turn
    turnResult = TurnFromLineFollow_Better();
    switch(turnResult) {
      case -1: // all black
        keepGoing = false;
        break;
      case 0: // balanced
        SetMotorSpeeds(baseSpeed, baseSpeed * 17 / 16);
        break;
      case 1: // all white
        // not supposed to happen
        PANIC();
        break;
      default:
        if(turnResult > 0) {
          SetMotorSpeeds(baseSpeed, max((baseSpeed - turnResult * 3),0));
        }
        else {
          SetMotorSpeeds(max((baseSpeed - (-turnResult) * 3),0), baseSpeed);
        }
        
    }

    // Delay to allow time for robot to move
    delay(20);
  } while (keepGoing);
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

void WaitNPulses(int nPulses) {
  int lc = 0, rc = 0;
  int ls = digitalRead(PIN_L_ENC), rs = digitalRead(PIN_R_ENC);
  lAvg.ResetToValue(analogRead(APIN_LINE_L));
  cAvg.ResetToValue(analogRead(APIN_LINE_C));
  rAvg.ResetToValue(analogRead(APIN_LINE_R));
  while(lc < nPulses && rc < nPulses) {
    int lt = digitalRead(PIN_L_ENC), rt = digitalRead(PIN_R_ENC);
    if(lt != ls) {
      ls = lt;
      lc++;
    }
    if(rt != rs) {
      rs = rt;
      rc++;
    }
    int closeness = min(nPulses - lc, nPulses - rc);
         if (closeness > 10) ;
    else if (closeness > 5) ReduceMotorSpeeds(10); // out of 16
    else if (closeness > 2) ReduceMotorSpeeds(7); // out of 16
    else if (closeness > 0) ReduceMotorSpeeds(4); // out of 16
    else return;
    if(TurnFromLineFollow_Better() == 0 && closeness < 6) {
      // HALT IMMEDIATELY
      SetMotorSpeeds(0,0);
      return;
    }
    delay(20);
  }
}

void TurnCW(int deg) {
  if(deg == 90) {
    SetMotorSpeeds(200, -240);
    WaitNPulses(12);
    //delay(440);
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

void TurnCCW(int deg) {
  if(deg == 90) {
    SetMotorSpeeds(-200, 240);
    delay(440);
    SetMotorSpeeds(0,0);
    return;
  }
  if(deg == 180) {
    SetMotorSpeeds(-200, 240);
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

void DUNK(void) {
  sTilt.write(180);
  delay(400);
  sTilt.write(90);
  delay(200);
  sGrip.write(50);
  delay(300);
}

int CheckIRPosition(void) {
   // for testing w/o emitter
  delay(500);
  return 0;
}

void TurnFromCenterTowardsBallBasedOnIRPosition(int irPosition) {
  switch(irPosition) {
    case 0:
      TurnCW(90);
    case 1:
      break;
    case 2:
      TurnCCW(90);
      break;
  }
  delay(2000);
}

void TurnTowardsCenterBasedOnIRPosition(int irPosition) {
  TurnCW(180);
  delay(2000);
}

void TurnFromCenterTowardsGoalBasedOnIRPosition(int irPosition) {
  switch(irPosition) {
    case 0:
      TurnCCW(90);
    case 1:
      break;
    case 2:
      TurnCW(90);
      break;
  }
  delay(2000);
}

void ReturnToCenterFromGoal(void) {
  TurnCW(180);
  DriveAlongLine(false);
  GoForwardABit();
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  pinMode(PIN_L_DIRN,  OUTPUT);
  pinMode(PIN_R_DIRN,  OUTPUT);
  pinMode(PIN_L_SPEED, OUTPUT);
  pinMode(PIN_R_SPEED, OUTPUT);
  pinMode(PIN_L_ENC,    INPUT);
  pinMode(PIN_R_ENC,    INPUT);
  pinMode(PIN_IR_RX,    INPUT);
  pinMode(PIN_LED,     OUTPUT);
  pinMode(APIN_DIST,    INPUT);

  sPan.attach(PIN_SERVO_PAN);
  sTilt.attach(PIN_SERVO_TILT);
  sGrip.attach(PIN_SERVO_GRIP);
  
  sPan.write(109);
  sTilt.write(130);
  sGrip.write(50);
  
  //irSerial.attach(PIN_IR_RX, -1);
  distAvg.ResetToValue(analogRead(APIN_DIST));
  lAvg.ResetToValue(analogRead(APIN_LINE_L));
  cAvg.ResetToValue(analogRead(APIN_LINE_C));
  rAvg.ResetToValue(analogRead(APIN_LINE_R));
}

void loop() {
  int irPosition = CheckIRPosition();
  TurnFromCenterTowardsBallBasedOnIRPosition(irPosition);
  DriveAlongLine(true);
  GrabBall();
  TurnTowardsCenterBasedOnIRPosition(irPosition);
  if(swag) {
    StartLineFollowFromDiagonal();
  }
  else {
    DriveAlongLine(false);
    GoForwardABit();
    TurnFromCenterTowardsGoalBasedOnIRPosition(irPosition);
    DriveAlongLine(false);
  }
  DUNK();
  ReturnToCenter();
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
