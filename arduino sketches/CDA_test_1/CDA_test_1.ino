#include "mpu6050.h"
#include<Servo.h>

//float kp = 0.2, Kp = 0.2;
//float kd = 0.3, Kd = 0.3;
//float ki = 0.01, Ki = 0.01;
float kp = 1.6, Kp = 0.4;
float kd = 0, Kd = 0;
float ki = 0, Ki = 0;
float setPoint = 90, setPointPitch = 90;
float lastError = 0, lastErrorPitch = 0;
float errSum = 0, errSumPitch = 0;

Servo servoRoll1, servoRoll2;

void setup(){
  Serial.begin(115200);
  servoRoll1.attach(6);
  servoRoll2.attach(9);
  servoRoll1.write(90);
  servoRoll2.write(90);
  
  delay(1000);
  
  mpu6050.MPU6050Setup();
}

void loop(){

  float *YPR = mpu6050.MPU6050Loop();
  float feedBack, feedBackPitch;
  float err, errPitch;
  float errProportional, errDerivative, errIntegral;
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
  Serial.print("Error: "); Serial.print(err); Serial.print('\t');
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
 
  //Serial.print(err); Serial.print(',');
  Serial.print("PID_OUT: "); Serial.print(PID_Out); Serial.print('\t');
  //Serial.println(feedBack);
    
  AngleRoll(PID_Out);

  
  delay(50);
}

void AngleRoll(float x){
  Serial.print("Servo_Pos: ");Serial.print(servoRoll1.read()); Serial.print('\t');
  Serial.print("Servo_In: ");Serial.println(servoRoll1.read() + x);
  servoRoll1.write(servoRoll1.read() + x);
  servoRoll2.write(servoRoll2.read() + x);
}


                   /*     void increaseAnglePitch(float x){
                          servoPitch.write(servoPitch.read() + fabs(x));
                        }
                        
                        void decreaseAnglePitch(float x){
                          servoPitch.write(servoPitch.read() - fabs(x));
                        }
                        */
