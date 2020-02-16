float vout;
float pressure;
int pValue;
float airspeed;
const float vfss = 3.5;
void setup ()
{
 Serial.begin(9600);
}


void loop()
{

  int speedavg = 0;
 for(int i=0; i<10; i++)
 {
 pValue = analogRead(3);
/* vout=float(pValue*(5.0/1016));
 pressure = abs(((((vout+0.0625*vfss)/5.0)-0.5)/0.2)-0.21875) ;
 airspeed=sqrt((2*pressure*1000)/1.1664);
 airspeed-=6;
 Serial.println("-----------------------------");*/
 speedavg += pValue;
 /*Serial.println(vout);
 Serial.println(pressure,6);
 Serial.println("AIRSPEED:");
 Serial.println(airspeed,4);*/
 delay(20);
 }
 Serial.println(speedavg/10);
 delay(300);
}


