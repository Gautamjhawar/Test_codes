/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
#include <SPI.h>
#include <SD.h>
#include "mpu6050.h"
#include<Servo.h>
byte flag=0;
int state=0;
float kp = 1.6;
float kd = 0;
float ki = 0;
float setPoint = 90;
float lastError = 0;
float errSum = 0;

Servo servoRoll1, servoRoll2;

void setup(){
  pinMode(8,INPUT_PULLUP);
  
  Serial.begin(115200);
  servoRoll1.attach(10);
  servoRoll2.attach(9);
  servoRoll1.write(90);
  servoRoll2.write(90);
  delay(1000);
  
  mpu6050.MPU6050Setup();
   Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(4)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop(){
  String dataString = "";
while(flag==0 && state==0){
float *YPR = mpu6050.MPU6050Loop();
    state=digitalRead(8);
    Serial.println("WAIT!");
}
flag=1;
  float *YPR = mpu6050.MPU6050Loop();
  float feedBack;
  float err;
  float errProportional, errDerivative, errIntegral;
  float PID_Out;

  YPR[1] = YPR[1]*100;
  feedBack = map(YPR[1],-130,130,0,180);


  err = setPoint - feedBack;

  errProportional = kp*err;

  errDerivative = kd*(err - lastError);
  lastError = err;

  errSum += err;
  errIntegral = ki*errSum;

  PID_Out = errProportional + errDerivative + errIntegral;
 
  Serial.print("err: ");Serial.print(err); Serial.print(',');
  dataString += String(err);dataString += ",";
  Serial.print("PID_OUT: "); Serial.print(PID_Out); Serial.print('\t');
  //Serial.println(feedBack);
    dataString += String(PID_Out);dataString += ";";
  AngleRoll(PID_Out);
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }
  else {
    Serial.println("error opening datalog.txt");
  }
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

