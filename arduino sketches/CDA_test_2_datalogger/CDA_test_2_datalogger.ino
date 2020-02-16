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
//float kp = 0.2, Kp = 0.2;
//float kd = 0.3, Kd = 0.3;
//float ki = 0.01, Ki = 0.01;
float kp = 1.6, Kp = 1.5;
float kd = 0, Kd = 0;
float ki = 0, Ki = 0;
float setPoint = 90, setPointPitch = 90;
float lastError = 0, lastErrorPitch = 0;
float errSum = 0, errSumPitch = 0;

Servo servoRoll1, servoRoll2, servoPitch;

void setup(){
  pinMode(8,INPUT_PULLUP);
  
  Serial.begin(115200);
  servoRoll1.attach(6);
  servoRoll2.attach(9);
  servoPitch.attach(5);
  servoRoll1.write(90);
  servoRoll2.write(90);
  servoPitch.write(90);
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
  float feedBack, feedBackPitch;
  float err, errPitch;
  float errProportional, errDerivative, errIntegral;
  float errProportionalPitch, errDerivativePitch, errIntegralPitch;
  float PID_Out, PID_OutPitch;

  if(Serial.available()){
    char buff = Serial.read();
    switch(buff){
      case 'P': kp+=0.1; Kp+=0.1; Serial.print("kp = "); Serial.print(kp); Serial.print("Kp = "); Serial.println(Kp); break;
      case 'p': kp-=0.1; Kp-=0.1; Serial.print("kp = "); Serial.print(kp); Serial.print("Kp = "); Serial.println(Kp); break;
      case 'D': kd+=0.1; Kd+=0.1; Serial.print("kd = "); Serial.print(kd); Serial.print("Kd = "); Serial.println(Kd); break;
      case 'd': kd-=0.1; Kd-=0.1; Serial.print("kd = "); Serial.print(kd); Serial.print("Kd = "); Serial.println(Kd); break;
      case 'I': ki+=0.01; Ki+=0.01; Serial.print("ki = "); Serial.print(ki); Serial.print("Ki = "); Serial.println(Ki); break;
      case 'i': ki-=0.01; Ki-=0.01; Serial.print("ki = "); Serial.print(ki); Serial.print("Ki = "); Serial.println(Ki); break;
      case 'T': setPointPitch++; break;
      case 't': setPointPitch++; break;
      case 'G': setPointPitch--; break;
      case 'g': setPointPitch--; break;
      case 'H': setPoint++; break;
      case 'h': setPoint++; break;
      case 'F': setPoint--; break;
      case 'f': setPoint--; break;
      default: setPoint = 90; setPointPitch = 90; break;
    }
    buff = 'n';
  }
 /*
  Serial.print("Roll: ");
  Serial.print(YPR[1]);
  Serial.print("\tPitch: ");
  Serial.println(YPR[2]);
*/
  YPR[1] = YPR[1]*100;
  feedBack = map(YPR[1],-130,130,0,180);
/*  Serial.print(YPR[1]);
  Serial.print('\t');
  Serial.println(feedBack);
*/
 
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
 
  Serial.print("err: ");Serial.print(err); Serial.print(',');
  dataString += String(err);dataString += ",";
  Serial.print("PID_OUT: "); Serial.print(PID_Out); Serial.print('\t');
  //Serial.println(feedBack);
    dataString += String(PID_Out);dataString += ",";
  AngleRoll(PID_Out);
  
  
  //For Pitch***********************************
  YPR[2] = YPR[2]*100;
 feedBackPitch = map(YPR[2],-150,150,0,180);
  errPitch = setPointPitch  - feedBackPitch ;
  Serial.print("ErrorPitch: "); Serial.print(errPitch); Serial.print('\t');
  dataString += String(errPitch);dataString += ",";
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
   dataString += String(PID_OutPitch);dataString += ";";
  AnglePitch(PID_OutPitch);
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
void AnglePitch(float x){
  //Serial.print("Servo_Pos: ");Serial.print(servoRoll1.read()); Serial.print('\t');
  //Serial.print("Servo_In: ");Serial.println(servoRoll1.read() + x);
  if((servoPitch.read() + x)<150 && (servoPitch.read() + x)>30)
  {
    servoPitch.write(servoPitch.read() + x);
  }
}
