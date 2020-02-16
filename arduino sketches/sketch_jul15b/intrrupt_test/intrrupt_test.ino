const byte interruptPin = 2;

void setup() {
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(0, blink, LOW);
}

void loop() {
  Serial.println("aaaa");
  delay(100);
}

void blink() {
  Serial.println("intrrupt at pin 2");
}
