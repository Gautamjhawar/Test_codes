#include <TimerOne.h>
#include <Servo.h>
 float relativeAltitude=10;
Servo s;
int deg = 0;
void setup() 
{Serial.begin(9600);
  // Initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards
  s.attach(9);
  s.write(deg);
  pinMode(9,OUTPUT);

  Timer1.initialize(100000); 
  Timer1.attachInterrupt( BlinkYellow ); 
  
}
 
void loop()
{

Serial.println("aaaaaaaaa");
s.write(deg);delay(1000);
if(Serial.read() == 'd')
  {s.write(90);}
delay(1000);
}
 
void BlinkYellow()
{
if (Serial.available() && Serial.read()=='d'){
 analogWrite(9,250);
  digitalWrite(13,HIGH);
      Serial.println("----------------------------------");
  Serial.print("Droped at alt:");
  Serial.println(relativeAltitude*3.280084);
  Serial.println("----------------------------------");

 
  return;
}
}
