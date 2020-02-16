int i;
void setup() {
  i=0;
Serial.begin(57600);
Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>.");
}

void loop() {
  Serial.println("aaaaa");
    Serial.println(i);
    ++i;
  delay(1000);

}
