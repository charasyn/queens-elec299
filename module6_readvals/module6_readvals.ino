#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("L: ");
  Serial.println(EEPROM.read(0));
  Serial.print("R: ");
  Serial.println(EEPROM.read(1));
}

void loop() {
  // put your main code here, to run repeatedly:

}
