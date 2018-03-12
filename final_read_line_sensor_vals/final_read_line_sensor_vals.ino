
#define APIN_LINE_L (A0)
#define APIN_LINE_R (A1)
#define APIN_LINE_C (A2)
#define APIN_DIST   (A3)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {

    int l = analogRead(APIN_LINE_L);
    int c = analogRead(APIN_LINE_C);
    int r = analogRead(APIN_LINE_R);
    int d = analogRead(APIN_DIST);

    
    Serial.print("L: ");
    Serial.print(l);
    Serial.print("\tC: ");
    Serial.print(c);
    Serial.print("\tR: ");
    Serial.print(r);
    
    Serial.print("\tD: ");
    Serial.print(d);
    Serial.println();
    delay(200);
}
