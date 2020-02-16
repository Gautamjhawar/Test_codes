#include<Servo.h>
#include "mpu6050.h"

byte PWM_PIN1 = 3;
byte PWM_PIN2 = 4;
byte PWM_PIN3 = 5;
byte flag = 0;
int pwm_value2;
int aliv1;
int aliv2;

float kp = 1.2, Kp = 1.2;
float kd = 0, Kd = 0;
float ki = 0, Ki = 0;
float setPoint = 87, setPointPitch = 90;
float lastError = 0, lastErrorPitch = 0;
float errSum = 0, errSumPitch = 0;

Servo ail1,ail2;
 
void setup() {
  pinMode(PWM_PIN1, INPUT);
  pinMode(PWM_PIN2, INPUT);
  pinMode(PWM_PIN3, INPUT);
  ail1.attach(9);
  ail2.attach(10);  
  ail1.write(115);
  ail2.write(65);
  Serial.begin(115200);
    mpu6050.MPU6050Setup();
}
 
void loop() {
  pwm_value2 = pulseIn(PWM_PIN1, HIGH);
  if(pwm_value2>=1400)
  {
  aliv1 = pulseIn(PWM_PIN2, HIGH);
  aliv2 = pulseIn(PWM_PIN3, HIGH);
  Serial.println("---------------------------");
  Serial.println(pwm_value2);
  Serial.println(aliv1);
  Serial.println(aliv2);
  Serial.println("---------------------------");
  ail1.write((aliv1-1080)/4.5);
  ail2.write((aliv2-1080)/4.5);
  }
  else
  {
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
}
void AngleRoll(float x){
  Serial.print("Servo_Pos: ");Serial.print(ail1.read()); Serial.print('\t');
  Serial.print("Servo_In: ");Serial.println(ail1.read() + x);
  if((ail1.read() + x)<150 && (ail1.read() + x)>40)
  {ail1.write(ail1.read() + x);
  ail2.write(ail2.read() + x);
  }
}
