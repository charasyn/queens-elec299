void setup() {
  // put your setup code here, to run once:
  pinMode(11,OUTPUT);
  pinMode(13,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(11,digitalRead(13));
}
