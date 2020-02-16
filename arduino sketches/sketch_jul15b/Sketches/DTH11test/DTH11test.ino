#include<DHT.h>
#define DHTPIN 6
#define DHTTYPE DHT11
#define r1 8
#define r2 9
DHT dht(DHTPIN,DHTTYPE);
void setup(){
 pinMode(r1,OUTPUT);
 pinMode(r2,OUTPUT);
Serial.begin(9600);
dht.begin();
}
void loop(){
delay(2000);
float h=dht.readHumidity();
float t=dht.readTemperature();
if(isnan(h)||isnan(t))
{
  Serial.println("Failed to read from sensor");
  return;
}
if(t>24)
{
digitalWrite(r1,LOW);    
}
else
{
digitalWrite(r1,HIGH);
}
if(h>40)
{
  digitalWrite(r2,LOW);
}
else
{
  digitalWrite(r2,HIGH);
}
Serial.print("Humidity:");
Serial.print(h);
Serial.print("%\t");
Serial.print("Temperature:");
Serial.print(t);
Serial.print("*C\n");
}

