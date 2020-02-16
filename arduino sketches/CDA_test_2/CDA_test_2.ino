#include "mpu6050.h"
#include<Servo.h>
byte flag=0;
int state=0;
//float kp = 0.2, Kp = 0.2;
//float kd = 0.3, Kd = 0.3;
//float ki = 0.01, Ki = 0.01;
float kp = 2, Kp = 1.5;
float kd = 0, Kd = 0;
float ki = 0, Ki = 0;
float setPoint = 90, setPointPitch = 90;
float lastError = 0, lastErrorPitch = 0;
float errSum = 0, errSumPitch = 0;


Servo servoRoll1, servoRoll2, servoPitch;

void setup(){
  pinMode(8,INPUT_PULLUP);
  
  Serial.begin(115200);
  servoRoll1.attach(5);
  servoRoll2.attach(6);
  servoPitch.attach(9);
  servoRoll1.write(90);
  servoRoll2.write(90);
  servoPitch.write(90);
  delay(1000);
  
  mpu6050.MPU6050Setup();
}

void loop(){
while(flag==0 && state==0){
float *YPR = mpu6050.MPU6050Loop();
    state=digitalRead(6);
    Serial.println("WAIT!");
}
flag=1;
  float *YPR = mpu6050.MPU6050Loop();
  float feedBack, feedBackPitch;
  float err, errPitch;
  float errProportional, errDerivative, errIntegral;
  float errProportionalPitch, errDerivativePitch, errIntegralPitch;
  float PID_Out, PID_OutPitch;

 
  YPR[1] = YPR[1]*100;
  feedBack = map(YPR[1],-130,130,0,180);
  err = setPoint - feedBack;
  
  errProportional = kp*err;

  errDerivative = kd*(err - lastError);
  lastError = err;

  errSum += err;
  errIntegral = ki*errSum;

  PID_Out = errProportional + errDerivative + errIntegral;
 
  Serial.print(err); Serial.print(',');
  Serial.print("PID_OUT: "); Serial.print(PID_Out); Serial.print('\t');
    
  AngleRoll(PID_Out);
  
  
  YPR[2] = YPR[2]*100;
 feedBackPitch = map(YPR[2],-150,150,0,180);
  errPitch = setPointPitch  - feedBackPitch ;
  Serial.print("ErrorPitch: "); Serial.print(errPitch); Serial.print('\t');
  errProportionalPitch = Kp*errPitch;

  errDerivativePitch = Kd*(errPitch - lastErrorPitch);
  lastErrorPitch = errPitch;

  errSumPitch += errPitch;
  errIntegralPitch = Ki*errSumPitch;
  
  PID_OutPitch = errProportionalPitch + errDerivativePitch + errIntegralPitch;
 
 Serial.print("PID_OUTPitch: "); Serial.println(PID_OutPitch);// Serial.println('\t');

  AnglePitch(PID_OutPitch);
  delay(50);
}

void AngleRoll(float x){
  if((servoRoll1.read() + x)<150 && (servoRoll1.read() + x)>30)
  {servoRoll1.write(servoRoll1.read() + x);
  servoRoll2.write(servoRoll2.read() + x);
  }
}
void AnglePitch(float x){
  //Serial.print("Servo_Pos: ");Serial.print(servoRoll1.read()); Serial.print('\t');
  //Serial.print("Servo_In: ");Serial.println(servoRoll1.read() + x);
  if((servoPitch.read() + x)<150 && (servoPitch.read() + x)>50)
  {
    servoPitch.write(servoPitch.read() + x);
  }
}
