int data=0;
void setup() {
 Serial.begin(9600);
 pinMode(6,OUTPUT);
}

void loop() {
  if(Serial.available()>0)
  {
    data=Serial.read();
   Serial.println(Serial.read()); 
  }
  if(data==49)
  {
    digitalWrite(6,LOW);
  }
  else if(data==48)
  {
    digitalWrite(6,HIGH);
  }
 
}
