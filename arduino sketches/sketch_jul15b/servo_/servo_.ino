#include<Servo.h>
Servo s;
int deg = 180;
char ch;
void setup() {
  Serial.begin(57600);
  s.attach(5);
  s.write(deg);
 
  
}

void loop() {
  deg=0;
    if(Serial.available())
    {
      ch = Serial.read();
      switch(ch)
      {
      case 'a': deg-=180;
 
      case 'c': deg+=180 ;
      
      default:  break;
      }
      Serial.println(deg); 
      s.write(deg);      
      delay(5);
    }
  
}
