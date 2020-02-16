int rpin=6;
int i=0;
void(* resetFunc) (void) = 0; //declare reset function @ address 0
void setup() {
  Serial.begin(9600);
pinMode(rpin,OUTPUT);
digitalWrite(rpin,HIGH);
 }

void loop() {

if(Serial.available())
{
  if(Serial.read()=='r')
  {
    resetFunc();  //call reset 
  }
  
}
++i;
Serial.println(i);
delay(500);

}
