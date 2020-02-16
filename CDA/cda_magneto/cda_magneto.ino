#include<Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float offset=0.0;


//float kp = 0.2, Kp = 0.2;
//float kd = 0.3, Kd = 0.3;
//float ki = 0.01, Ki = 0.01;
float kp = 1.6;
float kd = 0;
float ki = 0;
float setPoint = 250;
float lastError = 0; 
float errSum = 0;

Servo servoYaw;

void setup(){
  Serial.begin(115200);
  servoYaw.attach(6);
  servoYaw.write(90);
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
  delay(1000);
}

void loop(){
  sensors_event_t event;
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
  float feedBack;
  float err;
  float errProportional, errDerivative, errIntegral;
  float PID_OutYaw;

  feedBack=headingDegrees;
 /* YPR[1] = YPR[1]*100;
  feedBack = map(YPR[1],-130,130,0,180);
 */
  err = setPoint - feedBack;
  Serial.print("Error: "); Serial.print(err); Serial.print('\t');
  //Serial.print("\t\tErrorYaw: ");
  //Serial.print(err);

  errProportional = kp*err;

  errDerivative = kd*(err - lastError);
  lastError = err;

  errSum += err;
  errIntegral = ki*errSum;

  //Serial.print(" ");
  //Serial.println(errIntegral);
  
  PID_OutYaw = errProportional + errDerivative + errIntegral;
 
  //Serial.print(err); Serial.print(',');
  Serial.print("PID_OUTYaw: "); Serial.print(PID_OutYaw); Serial.print('\t');
  //Serial.println(feedBack);
    
  AngleYaw(PID_OutYaw);

  
  delay(50);
}

void AngleYaw(float x){
  Serial.print("Servo_Pos: ");Serial.print(servoYaw.read()); Serial.print('\t');
  Serial.print("Servo_In: ");Serial.println(servoYaw.read() + x);
  servoYaw.write(servoYaw.read() + x);
}

#include<Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float offset=0.0;


//float kp = 0.2, Kp = 0.2;
//float kd = 0.3, Kd = 0.3;
//float ki = 0.01, Ki = 0.01;
float kp = 1.6;
float kd = 0;
float ki = 0;
float setPoint = 250;
float lastError = 0; 
float errSum = 0;

Servo servoYaw;

void setup(){
  Serial.begin(115200);
  servoYaw.attach(6);
  servoYaw.write(90);
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
  delay(1000);
}

void loop(){
  sensors_event_t event;
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
  float feedBack;
  float err;
  float errProportional, errDerivative, errIntegral;
  float PID_OutYaw;

  feedBack=headingDegrees;
 /* YPR[1] = YPR[1]*100;
  feedBack = map(YPR[1],-130,130,0,180);
 */
  err = setPoint - feedBack;
  Serial.print("Error: "); Serial.print(err); Serial.print('\t');
  //Serial.print("\t\tErrorYaw: ");
  //Serial.print(err);

  errProportional = kp*err;

  errDerivative = kd*(err - lastError);
  lastError = err;

  errSum += err;
  errIntegral = ki*errSum;

  //Serial.print(" ");
  //Serial.println(errIntegral);
  
  PID_OutYaw = errProportional + errDerivative + errIntegral;
 
  //Serial.print(err); Serial.print(',');
  Serial.print("PID_OUTYaw: "); Serial.print(PID_OutYaw); Serial.print('\t');
  //Serial.println(feedBack);
    
  AngleYaw(PID_OutYaw);

  
  delay(50);
}

void AngleYaw(float x){
  Serial.print("Servo_Pos: ");Serial.print(servoYaw.read()); Serial.print('\t');
  if((servoYaw.read() + x)<160 && (servoYaw.read() + x)>20)
  Serial.print("Servo_In: ");Serial.println(servoYaw.read() + x);
  servoYaw.write(servoYaw.read() + x);
}


