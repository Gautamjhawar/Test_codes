void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(7,OUTPUT);
digitalWrite(7,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
if(Serial.available()){
  t = Serial.read();
  if(t=='h')
    digitalWrite(7,HIGH);
  else
    digitalWrite(7,LOW);
}
}
