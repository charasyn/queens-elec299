#define PIN_LED_A   (13)
#define PIN_LED_B   (11)
#define PIN_BUTTON  (12)
#define PIN_SPEAKER ( 3)
#define PIN_POT     ( 1)

#define MAXFREQ (880)
#define MINFREQ (110)

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED_A,  OUTPUT);
  pinMode(PIN_LED_B,  OUTPUT);
  pinMode(PIN_SPEAKER,OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
}

int state, counter, button_count;

void doSound() {
  int a = analogRead(PIN_POT);
  int freq = min(a + MINFREQ, MAXFREQ);
  if(counter == 0){
    tone(PIN_SPEAKER,freq,200);
  }
}

void loop() {
  if(digitalRead(PIN_BUTTON)){ // if button is up
    button_count = 5; // reset button count
  }
  else {
    button_count--;
    if(button_count == 0) {
      state = !state;
      counter = 0;
    }
  }
  if(state){
    digitalWrite(PIN_LED_A,0);
    doSound();
  }
  else {
    noTone(PIN_SPEAKER);
    digitalWrite(PIN_LED_A,counter < 60);
  }
  delay(10);
  counter++;
  if(counter >= 120) counter = 0;
}
