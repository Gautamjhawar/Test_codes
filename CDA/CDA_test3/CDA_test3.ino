
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "mpu6050.h"
#include<Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float offset=0.0;
byte flag=0;
int state=0;
TinyGPSPlus gps;
SoftwareSerial ss(4,3);
static const double dest_LAT = 12.972455, dest_LON =79.155580 ;

//float kp = 0.2, Kp = 0.2;
//float kd = 0.3, Kd = 0.3;
//float ki = 0.01, Ki = 0.01;
float kp = 1.6, Kp = 1.5,kpy=1.5;
float kd = 0, Kd = 0,kdy = 0;
float ki = 0, Ki = 0,kiy = 0;
float setPoint = 90, setPointPitch = 90 ,setPointYaw;
float lastError = 0, lastErrorPitch = 0,lastErrorYaw=0;
float errSum = 0, errSumPitch = 0,errSumYaw=0;

Servo servoRoll1, servoRoll2, servoPitch,servoYaw;

void setup(){
  pinMode(8,INPUT_PULLUP);
  
  Serial.begin(115200);
  servoRoll1.attach(3);
  servoRoll2.attach(5);
  servoPitch.attach(6);
  servoYaw.attach(9);
  servoRoll1.write(90);
  servoRoll2.write(90);
  servoPitch.write(90);
  servoYaw.write(90);
  ss.begin(9600);
  setPointYaw = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(),dest_LAT,dest_LON);
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
  
  mpu6050.MPU6050Setup();
}

void loop(){
while(flag==0 && state==0){
float *YPR = mpu6050.MPU6050Loop();
    state=digitalRead(8);
    Serial.println("WAIT!");
}
flag=1;
  float *YPR = mpu6050.MPU6050Loop();
  float feedBack, feedBackPitch,feedBackYaw;
  float err, errPitch,errYaw;
  float errProportional, errDerivative, errIntegral;
  float errProportionalPitch, errDerivativePitch, errIntegralPitch;
  float errProportionalYaw, errDerivativeYaw, errIntegralYaw;
  float PID_Out, PID_OutPitch,PID_OutYaw;

    setPointYaw = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(),dest_LAT,dest_LON);
    Serial.print("SetpointYaw:");Serial.print(setPointYaw);
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
  feedBackYaw=headingDegrees;
 /* YPR[1] = YPR[1]*100;
  feedBack = map(YPR[1],-130,130,0,180);
 */
  errYaw = setPointYaw - feedBackYaw;
  Serial.print("ErrorYaw: "); Serial.print(errYaw); Serial.print('\t');

  errProportionalYaw = kpy*errYaw;

  errDerivativeYaw = kdy*(errYaw - lastErrorYaw);
  lastErrorYaw = errYaw;

  errSumYaw += errYaw;
  errIntegralYaw = kiy*errSumYaw;

  //Serial.print(" ");
  //Serial.println(errIntegral);
  
  PID_OutYaw = errProportionalYaw + errDerivativeYaw + errIntegralYaw;
 
  //Serial.print(err); Serial.print(',');
  Serial.print("PID_OUTYaw: "); Serial.print(PID_OutYaw); Serial.print('\t');
  //Serial.println(feedBack);
    
  AngleYaw(PID_OutYaw);

  YPR[1] = YPR[1]*100;
  feedBack = map(YPR[1],-130,130,0,180);
 
  //Serial.print(YPR[1]);
  //Serial.print("\t");
  //Serial.print(feedBack);

  err = setPoint - feedBack;
  
  //Serial.print("Error: "); Serial.print(err); Serial.print('\t');
  //Serial.print("\t\tErrorRoll: ");
  //Serial.print(err);

  errProportional = kp*err;

  errDerivative = kd*(err - lastError);
  lastError = err;

  errSum += err;
  errIntegral = ki*errSum;

  //Serial.print(" ");
  //Serial.println(errIntegral);
  
  PID_Out = errProportional + errDerivative + errIntegral;
 
  Serial.print(err); Serial.print(',');
  Serial.print("PID_OUT: "); Serial.print(PID_Out); Serial.print('\t');
  //Serial.println(feedBack);
    
  AngleRoll(PID_Out);
  
  
  //For Pitch***********************************
  YPR[2] = YPR[2]*100;
 feedBackPitch = map(YPR[2],-150,150,0,180);
  errPitch = setPointPitch  - feedBackPitch ;
  Serial.print("ErrorPitch: "); Serial.print(errPitch); Serial.print('\t');
  errProportionalPitch = Kp*errPitch;

  errDerivativePitch = Kd*(errPitch - lastErrorPitch);
  lastErrorPitch = errPitch;

  errSumPitch += errPitch;
  errIntegralPitch = Ki*errSumPitch;

  //Serial.print(" ");
  //Serial.println(errIntegralPitch);
  
  PID_OutPitch = errProportionalPitch + errDerivativePitch + errIntegralPitch;
 
  //Serial.print(err); Serial.print(',');
 Serial.print("PID_OUTPitch: "); Serial.println(PID_OutPitch);// Serial.println('\t');
  //Serial.println(feedBackPitch);

  AnglePitch(PID_OutPitch);
  delay(50);
}

void AngleRoll(float x){
  //Serial.print("Servo_Pos: ");Serial.print(servoRoll1.read()); Serial.print('\t');
  //Serial.print("Servo_In: ");Serial.println(servoRoll1.read() + x);
  if((servoRoll1.read() + x)<160 && (servoRoll1.read() + x)>20)
  {servoRoll1.write(servoRoll1.read() + x);
  servoRoll2.write(servoRoll2.read() + x);
  }
}
void AnglePitch(float x){
  //Serial.print("Servo_Pos: ");Serial.print(servoRoll1.read()); Serial.print('\t');
  //Serial.print("Servo_In: ");Serial.println(servoRoll1.read() + x);
  if((servoPitch.read() + x)<160 && (servoPitch.read() + x)>60)
  {
    servoPitch.write(servoPitch.read() + x);
  }
}
void AngleYaw(float x){
//  Serial.print("Servo_Pos: ");Serial.print(servoYaw.read()); Serial.print('\t');
 // Serial.print("Servo_In: ");Serial.println(servoYaw.read() + x);
  if((servoYaw.read() + x)<160 && (servoYaw.read() + x)>20)
{
  servoYaw.write(servoYaw.read() + x);
}
}
