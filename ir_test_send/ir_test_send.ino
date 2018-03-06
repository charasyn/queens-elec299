#include <util/delay.h>

void sendBit(bool data){
  for(byte i=0; i < 133; i++){
    digitalWriteFast(12,data);
    _delay_us(1000000/40000/2 - 0);
    digitalWriteFast(12,HIGH);
    _delay_us(1000000/40000/2 - 0);
  }
}

void sendByte(byte data){
  sendBit(0);
  for(int i=0;i<8;i++){
    sendBit(data&1);
    data>>=1;
  }
  sendBit(1);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(12, OUTPUT);
  digitalWriteFast(12, HIGH);
  Serial.begin(1000000);
}

const char str[] PROGMEM = "Hello World!\n";
#define str_len ((sizeof(str)/sizeof(str[0])))

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0; i < str_len; i++){
    byte val = pgm_read_byte(str + i);
    sendByte(val);
    Serial.write(val);
    //delay(1);
  }
  delay(1000);
}
