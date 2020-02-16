const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, HIGH);
}

void loop() {
  digitalWrite(ledPin, state);
}

void blink() {
  state = !state;
}
