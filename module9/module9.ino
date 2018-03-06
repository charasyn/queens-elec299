void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

int transform(int rec){
  if(!((rec >= 'a' && rec <= 'z') || (rec >= 'A' && rec <= 'Z')))
    return rec;
  int adj = rec & 0x1f;
  int adj2 = (adj + 13) % 26;
  if(adj2 == 0)
    adj2 = 26;
  return rec + adj2 - adj;
}

void loop() {
  // put your main code here, to run repeatedly:
  int rec = Serial.read();
  if(rec >= 0){
    Serial.print((char)transform(rec));
  }
}
