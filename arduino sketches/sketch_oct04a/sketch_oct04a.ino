
#include <Wire.h>
#include<Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
sensors_event_t event;
Servo s;

float offset=0.0;
int deg;
float kp = 0.2;
float ki = 0;
float kd =0;
float IAZ = 2;
float errorT;
float lastError;
float proportional,integral,derivative;
float desiredval =45;

float setpoint = 90;
void setup(void) 
{
  Serial.begin(115200);
 
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  if(!mag.begin())
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
   mag.getEvent(&event);
 
 float heading = atan2(event.magnetic.y, event.magnetic.x);
  
 
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  float declinationAngle = 0.0203;
  heading += declinationAngle;

  if(heading < 0)
    heading += 2*PI;
 
  if(heading > 2*PI)
    heading -= 2*PI;
 
  float headingDegrees = heading * 180/M_PI; 
 offset = heading * 180/M_PI;
  offset=map(offset,0,359,359,0);
  offset=360-offset;
  Serial.println(offset);
  s.attach(9);
  s.write(90);
}

void loop(void) 
{ 
  mag.getEvent(&event);
 
 float heading = atan2(event.magnetic.y, event.magnetic.x);
  
 
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  float declinationAngle = 0.0203;
  heading += declinationAngle;

  if(heading < 0)
    heading += 2*PI;
 
  if(heading > 2*PI)
    heading -= 2*PI;
 
  float headingDegrees = heading * 180/M_PI; 
   headingDegrees=map(headingDegrees,0,359,359,0);
  headingDegrees+=offset;
  if(headingDegrees>359)
  {
    headingDegrees=headingDegrees-360;
    }
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  int output=headingDegrees+90;
  
 
  
  float error=desiredval-output;
  if(error<IAZ && error!=0)
  {
    errorT+=error;
  }
  else
  {
    errorT=0;
  }
  if(errorT>50/ki)
  {
    errorT=50/ki;
  }
  if(error==0)
  {
    derivative=0;
  }
  proportional = error * kp;
  integral = errorT * ki;
  derivative = (error - lastError) * kd;
  lastError = error;

  deg = proportional + integral + derivative;

  setpoint = setpoint + deg;
  setpoint=-setpoint;
  if(setpoint>180)
  {
    setpoint=180;
  }
  if(setpoint<0)
  {
    setpoint=0;
  }
  s.write(setpoint-90);
  Serial.print(deg);
  //delay(500);
}
