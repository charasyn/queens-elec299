#include "secret.h"

//#define CALIBRATION

Servo sPan, sTilt, sGrip;
QSerial irSerial;
MovingAverage distAvg(8);
MovingAverage lAvg(2);
MovingAverage cAvg(2);
MovingAverage rAvg(2);

////////////////////////////////////////////////////////////
// Function prototypes
static void PANIC(void);
static void loadCalibrationData(void);
static void saveCalibrationData(void);
static void setMotorSpeeds(int, int);
static void setMotorSpeeds_raw(int, int);
static void waitForInput(void);
static void calibrateLineSensors(void);
static void calibrateDistanceSensor(void);
static void calibrateDrivingSpeeds(void);
static void runCalibrationSuite(void);
static void driveAlongLine(bool);
static void turnCW(int);
static void turnCCW(int);
static void goForwardABit(void);
static void turnFromCenterTowardsBallBasedOnIRPosition(int);
static void turnTowardsCenterBasedOnIRPosition(int);
static void turnFromCenterTowardsGoalBasedOnIRPosition(int);
static void returnToCenterFromGoal(void);
static void grabBall(void);
static void dropBall(void);

static void DUNK(void);

static int getCharInput(void);
static int checkIRPosition(void);
static int turnFromLineFollow(void);

////////////////////////////////////////////////////////////
// Most important function
static void PANIC() {
    setMotorSpeeds_raw(0,0);
    for(;;){
        sTilt.write(180);
        delay(400);
        sTilt.write(90);
        delay(400);
    }
}

////////////////////////////////////////////////////////////
// Calibration init code
#define CAL_VER 2

struct Calibration {
    int16_t version;
    struct LineSensorVals {
        int16_t intercept, slope;
    } lineL, lineC, lineR;
    int16_t distStop, distPeak, distSlow;
    uint8_t motorSpeedsL[3], motorSpeedsR[3];
    int16_t t90, t90post;
    int16_t t180, t180post;
} cal;

static void loadCalibrationData(void) {
    EEPROM.get(0, cal);
    if(cal.version != CAL_VER) {
        // Load old values because.
        cal.lineL.intercept = 676;
        cal.lineL.slope = 8192 / 302;
        cal.lineC.intercept = 756;
        cal.lineC.slope = 8192 / 219;
        cal.lineR.intercept = 779;
        cal.lineR.slope = 8192 / 208;
        
        cal.distStop = 400;
        cal.distPeak = 500;
        cal.distSlow = 300;

        cal.motorSpeedsL[0] = 80;
        cal.motorSpeedsR[0] = 96;
        cal.motorSpeedsL[1] = 150;
        cal.motorSpeedsR[1] = 180;
        cal.motorSpeedsL[2] = 200;
        cal.motorSpeedsR[2] = 240;

        cal.t90 = 440;
        cal.t90post = 200;

        cal.t180 = 880;
        cal.t180post = 200;
    }
}
static void saveCalibrationData(void) {
    cal.version = CAL_VER;
    EEPROM.put(0, cal);
}

////////////////////////////////////////////////////////////
// Calibration functions
#ifdef CALIBRATION
#define PRINTVAL(s,v) Serial.print(s); Serial.println(v);
#define PRINTSAME(x) Serial.print(F(#x ": ")); Serial.println(x);

static void waitForInput(void) {
    Serial.println(F("Let go of the robot, then press enter."));
    getCharInput();
}

static int getCharInput(void) {
    int ret = 0;
    while(Serial.available() > 0) { Serial.read(); }
    while (1) {
        int c = Serial.read();
        if (c == '\n') break;
        ret = c;
        if (c < 0) delay(20);
    }
    while(Serial.available() > 0) { Serial.read(); }
    return ret;
}

static void calibrateLineSensors(void) {
    Serial.println(F("Calibrating line sensors."));
    Serial.println(F("Please place me with all sensors on white."));
    waitForInput();

    lAvg.ResetToValue(analogRead(APIN_LINE_L));
    cAvg.ResetToValue(analogRead(APIN_LINE_C));
    rAvg.ResetToValue(analogRead(APIN_LINE_R));

    int lw = lAvg.GetCurrentAvg();
    int cw = cAvg.GetCurrentAvg();
    int rw = rAvg.GetCurrentAvg();
    for(int i = 0; i < (1000/20); i++) {
        lw = max(lAvg.AddSample(analogRead(APIN_LINE_L)), lw);
        cw = max(cAvg.AddSample(analogRead(APIN_LINE_C)), cw);
        rw = max(rAvg.AddSample(analogRead(APIN_LINE_R)), rw);
        delay(20);
    }

    Serial.println(F("Readings done.\nPlease place me with all sensors on black."));
    waitForInput();

    lAvg.ResetToValue(analogRead(APIN_LINE_L));
    cAvg.ResetToValue(analogRead(APIN_LINE_C));
    rAvg.ResetToValue(analogRead(APIN_LINE_R));

    int lb = lAvg.GetCurrentAvg();
    int cb = cAvg.GetCurrentAvg();
    int rb = rAvg.GetCurrentAvg();
    for(int i = 0; i < (1000/20); i++) {
        lb = min(lAvg.AddSample(analogRead(APIN_LINE_L)), lb);
        cb = min(cAvg.AddSample(analogRead(APIN_LINE_C)), cb);
        rb = min(rAvg.AddSample(analogRead(APIN_LINE_R)), rb);
        delay(20);
    }

    cal.lineL.intercept = lw;
    cal.lineL.slope = 8192 / (lb - lw);
    cal.lineC.intercept = cw;
    cal.lineC.slope = 8192 / (cb - cw);
    cal.lineR.intercept = rw;
    cal.lineR.slope = 8192 / (rb - rw);

    Serial.println("Readings done.\nCalibrated values have been recorded.");
    Serial.println("Calibration of line sensors successful.");
    Serial.println("Values:");
    PRINTVAL("L intercept: ", cal.lineL.intercept)
    PRINTVAL("L slope:     ", cal.lineL.slope)
    PRINTVAL("C intercept: ", cal.lineC.intercept)
    PRINTVAL("C slope:     ", cal.lineC.slope)
    PRINTVAL("R intercept: ", cal.lineR.intercept)
    PRINTVAL("R slope:     ", cal.lineR.slope)
}

static void calibrateDistanceSensor(void) {
    Serial.println("Calibrating distance sensor.");
    Serial.println("Please place me right up against the arena wall with my arm against the wall.");
    waitForInput();

    distAvg.ResetToValue(analogRead(APIN_DIST));

    int stopdist = distAvg.GetCurrentAvg();
    for(int i = 0; i < (1000/20); i++) {
        stopdist = max(distAvg.AddSample(analogRead(APIN_DIST)), stopdist);
        delay(20);
    }

    setMotorSpeeds(-1,-1);

    int peakdist = distAvg.GetCurrentAvg();
    for(int i = 0; i < 10; i++) {
        int read = distAvg.AddSample(analogRead(APIN_DIST));
        peakdist = max(read, peakdist);
        if (read >= peakdist) i = 0;
        delay(20);
    }
    peakdist -= 20; // to make it more lenient

    setMotorSpeeds(0,0);

    Serial.println("Readings done.\nPlease place me where you would like me to slow down.");
    waitForInput();


    distAvg.ResetToValue(analogRead(APIN_DIST));

    int slowdist = distAvg.GetCurrentAvg();
    for(int i = 0; i < (1000/20); i++) {
        slowdist = min(distAvg.AddSample(analogRead(APIN_DIST)), slowdist);
        delay(20);
    }

    cal.distStop = stopdist;
    cal.distSlow = slowdist;
    cal.distPeak = peakdist;

    Serial.println("Readings done.\nCalibrated values have been recorded.");
    Serial.println("Calibration of line sensors successful.");
    Serial.println("Values:");
    PRINTVAL("Stop dist: ", stopdist)
    PRINTVAL("Peak dist: ", peakdist)
    PRINTVAL("Slow dist: ", slowdist)
}

static void getEncoderPulses(int * l, int * r, int len) {
    int lc = 0, rc = 0;
    int ls = digitalRead(PIN_L_ENC), rs = digitalRead(PIN_R_ENC);
    for (int i = 0; i < len; i++) {
        int lt = digitalRead(PIN_L_ENC), rt = digitalRead(PIN_R_ENC);
        if(lt != ls) {
            ls = lt;
            lc++;
        }
        if(rt != rs) {
            rs = rt;
            rc++;
        }
        delay(20);
    }
    *l = lc;
    *r = rc;
}

static void calibrateDrivingSpeeds(void) {
    Serial.println("Calibrating driving speeds and turn times.");
    Serial.println("Please place me in an open spot.");
    waitForInput();

    int lc, rc;
    bool keepGoing = true;;

    int slowL = cal.motorSpeedsL[0], slowR = cal.motorSpeedsR[0];
    for (keepGoing=true; keepGoing; ) {
        Serial.println("Calibrating slow speed.");
        setMotorSpeeds_raw(slowL,slowR);
        getEncoderPulses(&lc, &rc, 4000/20);
        setMotorSpeeds(0,0);
        int nL = slowL, nR = slowR;
        bool otherKeepGoing = true;
        while(otherKeepGoing) {
            Serial.print("Prev Vals: ");
            Serial.print(slowL);
            Serial.print(" ");
            Serial.print(slowR);
            Serial.print("\nNext Vals: ");
            Serial.print(nL);
            Serial.print(" ");
            Serial.print(nR);
            Serial.print("\nEnc. Vals: ");
            Serial.print(lc);
            Serial.print(" ");
            Serial.print(rc);
            Serial.println("\nPlease enter a command...");
            switch(getCharInput()) {
            case 'a': nL += 1; break;
            case 's': nR += 1; break;
            case 'z': nL -= 1; break;
            case 'x': nR -= 1; break;
            case 'A': nL += 10; break;
            case 'S': nR += 10; break;
            case 'Z': nL -= 10; break;
            case 'X': nR -= 10; break;
            
            case 'q': keepGoing = false;
            case 'n': otherKeepGoing = false; break;
            }
        }
        slowL = nL;
        slowR = nR;
    }

    int medL = cal.motorSpeedsL[1], medR = cal.motorSpeedsR[1];
    for (keepGoing=true; keepGoing; ) {
        Serial.println("Calibrating medium speed.");
        setMotorSpeeds_raw(medL,medR);
        getEncoderPulses(&lc, &rc, 2000/20);
        setMotorSpeeds(0,0);
        int nL = medL, nR = medR;
        bool otherKeepGoing = true;
        while(otherKeepGoing) {
            Serial.print("Prev Vals: ");
            Serial.print(medL);
            Serial.print(" ");
            Serial.print(medR);
            Serial.print("\nNext Vals: ");
            Serial.print(nL);
            Serial.print(" ");
            Serial.print(nR);
            Serial.print("\nEnc. Vals: ");
            Serial.print(lc);
            Serial.print(" ");
            Serial.print(rc);
            Serial.println("\nPlease enter a command...");
            switch(getCharInput()) {
            case 'a': nL += 1; break;
            case 's': nR += 1; break;
            case 'z': nL -= 1; break;
            case 'x': nR -= 1; break;
            case 'A': nL += 10; break;
            case 'S': nR += 10; break;
            case 'Z': nL -= 10; break;
            case 'X': nR -= 10; break;
            
            case 'q': keepGoing = false;
            case 'n': otherKeepGoing = false; break;
            }
        }
        medL = nL;
        medR = nR;
    }

    int fastL = cal.motorSpeedsL[2], fastR = cal.motorSpeedsR[2];
    for (keepGoing=true; keepGoing; ) {
        Serial.println("Calibrating fastium speed.");
        setMotorSpeeds_raw(fastL,fastR);
        getEncoderPulses(&lc, &rc, 1000/20);
        setMotorSpeeds(0,0);
        int nL = fastL, nR = fastR;
        bool otherKeepGoing = true;
        while(otherKeepGoing) {
            Serial.print("Prev Vals: ");
            Serial.print(fastL);
            Serial.print(" ");
            Serial.print(fastR);
            Serial.print("\nNext Vals: ");
            Serial.print(nL);
            Serial.print(" ");
            Serial.print(nR);
            Serial.print("\nEnc. Vals: ");
            Serial.print(lc);
            Serial.print(" ");
            Serial.print(rc);
            Serial.println("\nPlease enter a command...");
            switch(getCharInput()) {
            case 'a': nL += 1; break;
            case 's': nR += 1; break;
            case 'z': nL -= 1; break;
            case 'x': nR -= 1; break;
            case 'A': nL += 10; break;
            case 'S': nR += 10; break;
            case 'Z': nL -= 10; break;
            case 'X': nR -= 10; break;
            
            case 'q': keepGoing = false;
            case 'n': otherKeepGoing = false; break;
            }
        }
        fastL = nL;
        fastR = nR;
    }

    cal.motorSpeedsL[0] = slowL;
    cal.motorSpeedsR[0] = slowR;
    cal.motorSpeedsL[1] = medL;
    cal.motorSpeedsR[1] = medR;
    cal.motorSpeedsL[2] = fastL;
    cal.motorSpeedsR[2] = fastR;

    Serial.println("Readings done.\nCalibrated values have been recorded.");
    Serial.println("Calibration of motor speeds successful.");
    Serial.println("Values:");
    PRINTSAME(slowL)
    PRINTSAME(slowR)
    PRINTSAME(medL)
    PRINTSAME(medR)
    PRINTSAME(fastL)
    PRINTSAME(fastR)
    
    Serial.println("Next up: turning.");
    int t90 = cal.t90, t90post = cal.t90post;
    for (keepGoing=true; keepGoing; ) {
        Serial.println("Calibrating 90 deg time.");
        setMotorSpeeds(2,-2);
        delay(t90);
        setMotorSpeeds(0,0);
        delay(t90post);
        int nL = t90, nR = t90post;
        bool otherKeepGoing = true;
        while(otherKeepGoing) {
            Serial.print("Prev Vals: ");
            Serial.print(t90);
            Serial.print(" ");
            Serial.print(t90post);
            Serial.print("\nNext Vals: ");
            Serial.print(nL);
            Serial.print(" ");
            Serial.print(nR);
            Serial.print("\nEnc. Vals: ");
            Serial.print(lc);
            Serial.print(" ");
            Serial.print(rc);
            Serial.println("\nPlease enter a command...");
            switch(getCharInput()) {
            case 'a': nL += 10; break;
            case 's': nR += 10; break;
            case 'z': nL -= 10; break;
            case 'x': nR -= 10; break;
            case 'A': nL += 100; break;
            case 'S': nR += 100; break;
            case 'Z': nL -= 100; break;
            case 'X': nR -= 100; break;
            
            case 'q': keepGoing = false;
            case 'n': otherKeepGoing = false; break;
            }
        }
        t90 = nL;
        t90post = nR;
    }

    int t180 = cal.t180, t180post = cal.t180post;
    for (keepGoing=true; keepGoing; ) {
        Serial.println("Calibrating 180 deg time.");
        setMotorSpeeds(2,-2);
        delay(t180);
        setMotorSpeeds(0,0);
        delay(t180post);
        int nL = t180, nR = t180post;
        bool otherKeepGoing = true;
        while(otherKeepGoing) {
            Serial.print("Prev Vals: ");
            Serial.print(t180);
            Serial.print(" ");
            Serial.print(t180post);
            Serial.print("\nNext Vals: ");
            Serial.print(nL);
            Serial.print(" ");
            Serial.print(nR);
            Serial.print("\nEnc. Vals: ");
            Serial.print(lc);
            Serial.print(" ");
            Serial.print(rc);
            Serial.println("\nPlease enter a command...");
            switch(getCharInput()) {
            case 'a': nL += 10; break;
            case 's': nR += 10; break;
            case 'z': nL -= 10; break;
            case 'x': nR -= 10; break;
            case 'A': nL += 100; break;
            case 'S': nR += 100; break;
            case 'Z': nL -= 100; break;
            case 'X': nR -= 100; break;
            
            case 'q': keepGoing = false;
            case 'n': otherKeepGoing = false; break;
            }
        }
        t180 = nL;
        t180post = nR;
    }

    cal.t90 = t90;
    cal.t90post = t90post;
    cal.t180 = t180;
    cal.t180post = t180post;

    Serial.println("Readings done.\nCalibrated values have been recorded.");
    Serial.println("Calibration of motor speeds and turn times successful.");
    Serial.println("Values:");
    PRINTSAME(t90)
    PRINTSAME(t90post)
    PRINTSAME(t180)
    PRINTSAME(t180post)
}

static void testIRCode(void) {
    Serial.println("Testing IR.");
    waitForInput();
    Serial.println(checkIRPosition());
}

static void printVals(void) {
    PRINTSAME(cal.version)
    Serial.println("###################");
    PRINTSAME(cal.lineL.intercept)
    PRINTSAME(cal.lineL.slope)
    PRINTSAME(cal.lineC.intercept)
    PRINTSAME(cal.lineC.slope)
    PRINTSAME(cal.lineR.intercept)
    PRINTSAME(cal.lineR.slope)
    Serial.println("###################");
    PRINTSAME(cal.distStop)
    PRINTSAME(cal.distPeak)
    PRINTSAME(cal.distSlow)
    Serial.println("###################");
    PRINTSAME(cal.motorSpeedsL[0])
    PRINTSAME(cal.motorSpeedsR[0])
    PRINTSAME(cal.motorSpeedsL[1])
    PRINTSAME(cal.motorSpeedsR[1])
    PRINTSAME(cal.motorSpeedsL[2])
    PRINTSAME(cal.motorSpeedsR[2])
    Serial.println("###################");
    PRINTSAME(cal.t90)
    PRINTSAME(cal.t90post)
    PRINTSAME(cal.t180)
    PRINTSAME(cal.t180post)

}

static void runCalibrationSuite(void) {
    while(1) {
        Serial.println("\n*****************************");
        Serial.println("* Calibration Suite for 299 *");
        Serial.println("*****************************");
        Serial.println("");
        Serial.println("1) Calibrate line sensors");
        Serial.println("2) Calibrate distance sensor");
        Serial.println("3) Calibrate driving speeds");
        Serial.println("i) Test IR code");
        Serial.println("s) Save the calibration data");
        Serial.println("p) Print the calibration data");
        Serial.println("q) Quit");
        Serial.println("");
        Serial.println("Make your selection...");
        while(Serial.available() <= 0) {}
        int c = Serial.read();
        while(Serial.available() > 0) { Serial.read(); }
        switch(c) {
            case '1':
                calibrateLineSensors();
                break;
            case '2':
                calibrateDistanceSensor();
                break;
            case '3':
                calibrateDrivingSpeeds();
                break;
            case 's':
            case 'S':
                saveCalibrationData();
                break;
            case 'p':
                printVals();
                break;
            case 'i':
                testIRCode();
                break;
            case 'q':
                return;
        }
    }
}
#endif

////////////////////////////////////////////////////////////
// Robot functions
static void setMotorSpeeds(int l, int r) {
    digitalWrite(PIN_L_DIRN, l > 0);
    digitalWrite(PIN_R_DIRN, r > 0);
    analogWrite(PIN_L_SPEED, l == 0 ? 0 : cal.motorSpeedsL[abs(l) - 1]);
    analogWrite(PIN_R_SPEED, r == 0 ? 0 : cal.motorSpeedsR[abs(r) - 1]);
}

static void setMotorSpeeds_raw(int l, int r) {
    digitalWrite(PIN_L_DIRN, l > 0);
    digitalWrite(PIN_R_DIRN, r > 0);
    analogWrite(PIN_L_SPEED, abs(l));
    analogWrite(PIN_R_SPEED, abs(r));
}

// returns:
//  +'ve > 1 if we need to turn clockwise
//   1 for all white
//   0 if we're mostly on the center
//  -1 for all black
//  -'ve < -1 if we need to turn counter-clockwise
int turnFromLineFollow(void) {
    // Take average of past 16 readings
    int l = lAvg.AddSample(analogRead(APIN_LINE_L));
    int c = cAvg.AddSample(analogRead(APIN_LINE_C));
    int r = rAvg.AddSample(analogRead(APIN_LINE_R));
    int l_adj = min((uint16_t)max(l - cal.lineL.intercept, 0) * cal.lineL.slope / (8192 / 64) , 63);
    int c_adj = min((uint16_t)max(c - cal.lineC.intercept, 0) * cal.lineC.slope / (8192 / 64) , 63);
    int r_adj = min((uint16_t)max(r - cal.lineR.intercept, 0) * cal.lineR.slope / (8192 / 64) , 63);
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

static void driveAlongLine(bool towardsBall) {
    bool distPastPeak = false, keepGoing = true;
    uint16_t distVal;
    uint8_t baseSpeed = 3;
    int turnResult;
    lAvg.ResetToValue(analogRead(APIN_LINE_L));
    cAvg.ResetToValue(analogRead(APIN_LINE_C));
    rAvg.ResetToValue(analogRead(APIN_LINE_R));
    distVal = distAvg.ResetToValue(analogRead(APIN_DIST));
    do {
        // Calculate base speed
        distVal = distAvg.AddSample(analogRead(APIN_DIST));

        if (distPastPeak) {
                 if (distVal <= cal.distStop) break;
            else                              baseSpeed = 1; // slow speed
        } else {
                 if (distVal >= cal.distPeak) distPastPeak = true;
            else if (distVal >= cal.distSlow) baseSpeed = 2; // medium speed
            else                              baseSpeed = 3; // full speed ahead
        }

        if(distPastPeak && towardsBall) {
            sPan.write(109);
            sGrip.write(50);
            sTilt.write(60);
        }

        // Set motor speeds based on need to turn
        turnResult = turnFromLineFollow();
        switch(turnResult) {
        case -1: // all black
            keepGoing = false;
            break;
        case  0: // balanced
            setMotorSpeeds(baseSpeed, baseSpeed);
            break;
        case  1: // all white
            // not supposed to happen
            PANIC();
            break;
        default:
            if(turnResult > 0) {
                setMotorSpeeds(baseSpeed, baseSpeed - 1);
            }
            else {
                setMotorSpeeds(baseSpeed - 1, baseSpeed);
            }
        }

        // Delay to allow time for robot to move
        delay(20);
    } while (keepGoing);
    setMotorSpeeds(0,0);
}


void turnCW(int deg) {
    //lAvg.ResetToValue(analogRead(APIN_LINE_L));
    cAvg.ResetToValue(analogRead(APIN_LINE_C));
    rAvg.ResetToValue(analogRead(APIN_LINE_R));
    int time, timePost, ogTimeDiv;
    if(deg == 90) {
        time = cal.t90;
        timePost = cal.t90post;
    }
    else if(deg == 180) {
        time = cal.t180;
        timePost = cal.t180post;
    }
    else {
        PANIC();
        return;
    }
    ogTimeDiv=time/3;
    setMotorSpeeds(2,-2);
    for(; time > 0; time -= 10) {
        //int l = lAvg.AddSample(analogRead(APIN_LINE_L));
        int c = cAvg.AddSample(analogRead(APIN_LINE_C));
        int r = rAvg.AddSample(analogRead(APIN_LINE_R));
        //int l_adj = min((uint16_t)max(l - cal.lineL.intercept, 0) * cal.lineL.slope / (8192 / 64) , 63);
        int c_adj = min((uint16_t)max(c - cal.lineC.intercept, 0) * cal.lineC.slope / (8192 / 64) , 63);
        int r_adj = min((uint16_t)max(r - cal.lineR.intercept, 0) * cal.lineR.slope / (8192 / 64) , 63);
        if (time <= ogTimeDiv) {
            if (r_adj > LF_BLACK_THRESH) {
                setMotorSpeeds(1,-1);
            }
            if (c_adj > LF_BLACK_THRESH) {
                setMotorSpeeds(0,0);
            }
        }
        delay(10);
    }
    setMotorSpeeds(0,0);
    delay(timePost);
}

void turnCCW(int deg) {
    lAvg.ResetToValue(analogRead(APIN_LINE_L));
    cAvg.ResetToValue(analogRead(APIN_LINE_C));
    //rAvg.ResetToValue(analogRead(APIN_LINE_R));
    int time, timePost, ogTimeDiv;
    if(deg == 90) {
        time = cal.t90;
        timePost = cal.t90post;
    }
    else if(deg == 180) {
        time = cal.t180;
        timePost = cal.t180post;
    }
    else {
        PANIC();
        return;
    }
    ogTimeDiv=time/3;
    setMotorSpeeds(-2,2);
    for(; time > 0; time -= 10) {
        int l = lAvg.AddSample(analogRead(APIN_LINE_L));
        int c = cAvg.AddSample(analogRead(APIN_LINE_C));
        //int r = rAvg.AddSample(analogRead(APIN_LINE_R));
        int l_adj = min((uint16_t)max(l - cal.lineL.intercept, 0) * cal.lineL.slope / (8192 / 64) , 63);
        int c_adj = min((uint16_t)max(c - cal.lineC.intercept, 0) * cal.lineC.slope / (8192 / 64) , 63);
        //int r_adj = min((uint16_t)max(r - cal.lineR.intercept, 0) * cal.lineR.slope / (8192 / 64) , 63);
        if (time <= ogTimeDiv) {
            if (l_adj > LF_BLACK_THRESH) {
                setMotorSpeeds(-1,1);
            }
            if (c_adj > LF_BLACK_THRESH) {
                setMotorSpeeds(0,0);
            }
        }
        delay(10);
    }
    setMotorSpeeds(0,0);
    delay(timePost);
}

void goForwardABit(void) {
    setMotorSpeeds(3,3);
    delay(130);
    setMotorSpeeds(0,0);
}

void DUNK(void) {
    sTilt.write(180);
    delay(400);
    sTilt.write(90);
    delay(200);
    sGrip.write(50);
    delay(300);
}

void grabBall(void) {
    sTilt.write(40);
    delay(200);
    sGrip.write(93);
    delay(300);
    sTilt.write(110);
    delay(100);
}

void dropBall(void) {
    sGrip.write(50);
    delay(300);
}


int checkIRPosition(void) {
    sTilt.write(80);
    int received = 0, armPos = 10;
    while (1) {
        received = irSerial.receive(1000);
        if (received >= '0' && received <= '2') {
            break;
        }
        else {
            if(armPos < 40 || (armPos >= 80 && armPos < 120) || armPos >= 160)
                armPos += 5;
            else
                armPos += 40;
            if(armPos >= 190) {
                armPos = 10;
                sPan.write(armPos);
                delay(200);
            }
            sPan.write(armPos);
            delay(200);
        }
    }
    sPan.write(109);
    sTilt.write(60);
    return received - '0';
}

void turnFromCenterTowardsBallBasedOnIRPosition(int irPosition) {
    switch(irPosition) {
    case 0:
        turnCW(90);
        break;
    case 1:
        break;
    case 2:
        turnCCW(90);
        break;
    }
}

void turnTowardsCenterBasedOnIRPosition(int irPosition) {
    turnCW(180);
}

void turnFromCenterTowardsGoalBasedOnIRPosition(int irPosition) {
    switch(irPosition) {
    case 0:
        turnCCW(90);
    case 1:
        break;
    case 2:
        turnCW(90);
        break;
    }
}

void returnToCenterFromGoal(void) {
    turnCW(180);
    driveAlongLine(false);
    goForwardABit();
}

void setup() {
    loadCalibrationData();
    
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
    sTilt.write(60);
    sGrip.write(50);
    
    irSerial.attach(PIN_IR_RX, -1);
    distAvg.ResetToValue(analogRead(APIN_DIST));
    lAvg.ResetToValue(analogRead(APIN_LINE_L));
    cAvg.ResetToValue(analogRead(APIN_LINE_C));
    rAvg.ResetToValue(analogRead(APIN_LINE_R));

#ifdef CALIBRATION
    Serial.begin(115200);
    runCalibrationSuite();
    for(;;);
#endif
}

void loop() {
    int irPosition = checkIRPosition();
    turnFromCenterTowardsBallBasedOnIRPosition(irPosition);
    driveAlongLine(true);
    grabBall();
    turnTowardsCenterBasedOnIRPosition(irPosition);
    driveAlongLine(false);
    goForwardABit();
    turnFromCenterTowardsGoalBasedOnIRPosition(irPosition);
    driveAlongLine(false);
    DUNK();
    returnToCenterFromGoal();
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
