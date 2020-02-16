
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
sensors_event_t event;
float offset=0.0;
int x=millis();
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
  if((millis()-x)>1000)
  {
    Serial.println("****");
    x=millis();
    }
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
//delay(500);
}
