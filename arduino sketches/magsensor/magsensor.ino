/***************************************************************************
  This is a library example for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746
 
  *** You will also need to install the Adafruit_Sensor library! ***

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float offset=0.0;

void setup(void) 
{
  
  
  Serial.begin(115200);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
sensors_event_t event; 
  mag.getEvent(&event);
 
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians //ours is -1* 10' so in rad it equals to  
  float declinationAngle = 0.0203;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  offset = heading * 180/M_PI;
  offset=map(offset,0,359,359,0);
  offset=360-offset;
  //Serial.println(offset);
}

void loop(void) 
{
  
  sensors_event_t event; 
  mag.getEvent(&event);
 
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians //ours is -1* 10' so in rad it equals to  
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
    headingDegrees=360-headingDegrees;
    }
    headingDegrees=abs(headingDegrees);
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  Serial.print("Heading : "); Serial.println(heading* 180/M_PI);

  delay(500);
}
