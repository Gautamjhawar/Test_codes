#include<Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

byte PWM_PIN1 = 8;
byte PWM_PIN2 = 4;
byte PWM_PIN3 = 5;
byte PWM_PIN4 = 6;
byte PWM_PIN5 = 7;
byte flag = 0;
int ruddr;
int pwm_value2;
int aliv;
int elev;
int thr;
Servo ele,ail1,ail2,rudd;
 int deg=0,d;

float kp = 0.25;
float ki = 0.001;
float kd =0;
float IAZ = 2;
float errorT;
float lastError;
float proportional,integral,derivative;
float desiredval =90;


MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;

double timeStep, time, timePrev;
double arx, ary, arz, grx, gry, grz, gsx, gsy, gsz, rx, ry, rz;

int i;
float setpoint = 90;
double gyroScale = 131;

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN1, INPUT);
  pinMode(PWM_PIN2, INPUT);
  pinMode(PWM_PIN3, INPUT);
  pinMode(PWM_PIN4, INPUT);
  pinMode(PWM_PIN5, INPUT);
  ail1.attach(3);
  ail2.attach(9);
  ele.attach(10);
  rudd.attach(11); 
  accelgyro.initialize();
   time = millis();
   i = 1;
  
}
 
void loop() {
  pwm_value2 = pulseIn(PWM_PIN2, HIGH);
  if(pwm_value2>=1500)
  {
  ruddr = pulseIn(PWM_PIN1, HIGH);
  aliv = pulseIn(PWM_PIN3, HIGH);
  elev = pulseIn(PWM_PIN4, HIGH);
  thr = pulseIn(PWM_PIN5, HIGH);
 Serial.println("---------------------------");
  Serial.println(ruddr);
  Serial.println(pwm_value2);
  Serial.println(aliv);
  Serial.println(elev);
  Serial.println(thr);
  Serial.println("---------------------------");
  ele.write((elev-1103)/4.5);
  ail1.write((aliv-1103)/4.5);
  ail2.write((aliv-1103)/4.5);
  rudd.write((ruddr-1103)/4.5);
  
  //delay(200);
  }
  else
  {
   int output=gyro()+90;
   float error=output-desiredval;
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
  
     if(setpoint>180)
      {
        setpoint=180;
      }
      else if(setpoint<0)
      {
        setpoint=0;
       }
      ail1.write(setpoint);
      ail2.write(setpoint);
      /*Serial.print(deg);
      Serial.print("--------////");
      Serial.print(output);Serial.print("..................////");
      Serial.println(setpoint);*/
      delay(10);
  
  
}
}
double gyro()
{
  timePrev = time;
  time = millis();
  timeStep = (time - timePrev) / 1000; // time-step in s

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  gsx = gx/gyroScale;   gsy = gy/gyroScale;   gsz = gz/gyroScale;
  
  arx = (180/3.141592) * atan(ax / sqrt(square(ay) + square(az))); 
  ary = (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
  arz = (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);

    grx = arx;
    gry = ary;
    grz = arz;
  
  rx = (0.1 * arx) + (0.9 * grx);
  ry = (0.1 * ary) + (0.9 * gry);
  rz = (0.1 * arz) + (0.9 * grz);
  i = i + 1;
  return ary;
  }
