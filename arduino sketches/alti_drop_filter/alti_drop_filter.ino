#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <MS5611.h>
#include<Servo.h>
Servo s;
void(* resetFunc) (void) = 0; //declare reset function @ address 0
int Flag=0;
MS5611 baro;
double referencePressure;
float relativeAltitude;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

long x=millis();
char ch;
void setup() {
  
Serial.begin(57600);
pinMode(2,INPUT_PULLUP);
  // Initialize MS5611 sensor
  Serial.println("Initialize MS5611 Sensor");
   baro = MS5611();
  baro.begin();
  s.attach(5);
  s.write(90);
 
  // Get reference pressure for relative altitude
  referencePressure = baro.getPressure(); 


}

void loop(){
    
   if(Serial.available()>0){
    ch=Serial.read();
  if(ch=='r')
  {
    resetFunc();  //call reset 
  }
    if(ch=='d'){
      s.write(0);
  Serial.println("----------------------------------");
  Serial.print("Droped at alt:");
  Serial.println(relativeAltitude*3.280084);
  Serial.println("----------------------------------");
  
  }
  if(ch=='c')
  {
    s.write(90);
  }
  }
  if((millis()-x)>=500){
    x=millis();
  // Read true temperature & Pressure
  long realPressure = baro.getPressure();
 
  // Calculate altitude
   long esti_Pressure = pressureKalmanFilter.updateEstimate(realPressure);
   relativeAltitude = 44330.0f * (1.0f - pow((double)esti_Pressure / (double)referencePressure, 0.1902949f));
 
if(millis()>=3000) {  Serial.println("--");
  Serial.print("relativeAltitude = ");
  Serial.print(relativeAltitude*3.280084);
  Serial.println(" ft");
  }
  }
 
}

