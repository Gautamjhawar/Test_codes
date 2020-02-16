#include<Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  //Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  //Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  //Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  //Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  //Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

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
   Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
   if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  displaySensorDetails(); 
  
}
 
void loop() {
   sensors_event_t event; 
  mag.getEvent(&event);
 
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
    float heading = atan2(event.magnetic.y, event.magnetic.x);
     float declinationAngle = 0.0203;
  heading += declinationAngle;
 if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI; 
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  }
}
