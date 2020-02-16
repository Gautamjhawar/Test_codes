

#include <Wire.h>  

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  Wire.begin();
  delay(500);
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");
}

void loop() {
  byte P_H,P_L,T_H,T_L;
unsigned int T_d;
double TR;
byte  address = 0x28;
  Wire.beginTransmission(address);
  Wire.endTransmission();
  delay(100);
Serial.println("---------------------------------------");
  Wire.requestFrom((int)address, (int) 4);//Request 4 bytes need 4 bytes are read
  P_H = Wire.read();
  P_L = Wire.read();
   T_H = Wire.read();
   T_L = Wire.read();
     T_L = (T_L >> 5);
  T_d = (((unsigned int)T_H) << 3) | T_L;
  TR = (double)((T_d*0.09770395701));
    TR = TR-50;

  Wire.endTransmission();
  Serial.println(P_H);
  Serial.println(P_L);
Serial.println(TR);
//Serial.println(T_L);
delay(500);
}
