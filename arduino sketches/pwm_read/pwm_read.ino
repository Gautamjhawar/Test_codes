#include<Servo.h>
byte PWM_PIN1 = 8;
/*byte PWM_PIN2 = 4;
byte PWM_PIN3 = 5;
byte PWM_PIN4 = 6;
byte PWM_PIN5 = 7;
byte flag = 0;
int ruddr;
int pwm_value2;
int aliv;
int elev;
int thr;
/*int degr(int val)
{
  int d;
  if(val>=1103 && val<=1132)
  d=0;
  else if(val>1132 && val<=1161)
  d=7;
  else if(val>1161 && val<=1190)
  d=14;
  else if(val>1190 && val<=1220)
  d=21;
  else if(val>1220 && val<=1250)
  d=28;
  else if(val>1250 && val<=1280)
  d=35;
  else if(val>1280 && val<=1310)
  d=42;
  else if(val>1310 && val<=1340)
  d=49;
  else if(val>1340 && val<=1370)
  d=56;
  else if(val>1370 && val<=1400)
  d=63;
  else if(val>1400 && val<=1460)
  d=70;
  else if(val>1460 && val<=1490)
  d=77;
  else if(val>1490 && val<=1510)
  d=84;
  else if(val>1510 && val<=1540)
  d=91;
  else if(val>1540 && val<=1570)
  d=98;
  else if(val>1740 && val<=1600)
  d=105;
  else if(val>1600 && val<=1630)
  d=112;
  else if(val>1630 && val<=1660)
  d=119;
  else if(val>1660 && val<=1690)
  d=126;
  else if(val>1690 && val<=1710)
  d=133;
  else if(val>1710 && val<=1740)
  d=140;
  else if(val>1740 && val<=1770)
  d=147;
  else if(val>1770 && val<=1800)
  d=154;
  else if(val>1800 && val<=1830)
  d=161;
  else if(val>1830 && val<=1860)
  d=168;
  else if(val>1860 && val<=1890)
  d=175;
  else if(val>1890 && val<=1920)
  d=180;
  return d;
}*/
// Servo ele,ail,rudd;

void setup() {
  pinMode(PWM_PIN1, INPUT);
  /*pinMode(PWM_PIN2, INPUT);
  pinMode(PWM_PIN3, INPUT);
  pinMode(PWM_PIN4, INPUT);
  pinMode(PWM_PIN5, INPUT);
  ail.attach(9);
  ele.attach(10);
  rudd.attach(11);  */
  Serial.begin(115200);
  //attachInterrupt(0, blink, LOW);
}
int pwm_value1; 
void loop() {
  pwm_value1 = pulseIn(PWM_PIN1, HIGH);
  Serial.println("---------------------------");
  Serial.println(pwm_value1);
  delay(500);
  /*if(pwm_value2>=1500)
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
 /* float e=degr(elev);
  float a=degr(aliv);
  float r=degr(ruddr);
  ele.write(e);
  ail.write(a);
  rudd.write(r);
  *//*
  ele.write((elev-1103)/4.5);
  ail.write((aliv-1103)/4.5);
  rudd.write((ruddr-1103)/4.5);
  
  //delay(200);
  }
  else
  {
    Serial.println("123456789");delay(200);
  }
*/}
