char ch;

void setup() {
  Serial.begin(57600);
  digitalWrite(5,HIGH);
  pinMode(5,OUTPUT);
  delay(500);
  digitalWrite(9,LOW);
  pinMode(9,OUTPUT);
  delay(20);
  digitalWrite(9,HIGH);
}

void loop() {
  
  //++i;Serial.println(i);delay(500);
  if (Serial.available())
  {     ch = Serial.read();
      if(ch=='r')
       { 
          digitalWrite(5,LOW); 
       }
  }

}
