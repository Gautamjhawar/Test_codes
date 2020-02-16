#include <Wire.h>   //I2C library 0x28H
float aspeed(void);
byte fetch_pressure(unsigned int *p_Pressure); //convert value to byte data type

float Alti,Calc_Range,Air_speed;

void setup() {
  Serial.begin(57600);

}

void loop() {
  Serial.println("aaaaaaa");
  Alti=100;
  Air_speed=aspeed();
  Serial.print("speed m/s :");
  Serial.println(Air_speed,5);
  Serial.print("speed Km/h :");
  Serial.println(Air_speed*3.6,5);
  Calc_Range=Air_speed*sqrt((2*Alti)/9.8);
  Serial.println("to be droped at range:");
  Serial.println();
  delay(1000);
      
  }  
  


float aspeed(void)
{
  byte _status;
  unsigned int P_dat;
  double PR;
  double V;
  double VV;
  while (1)
  {
    _status = fetch_pressure(&P_dat);
    PR = (double)((P_dat-819.15)/(14744.7)) ;
    PR = (PR - 0.49060678) ;
    PR = abs(PR);
     V = ((PR*13789.5144)/1.1644
     );
    VV = (sqrt((V)));
    VV-=6;
    return(VV);
  }
}
byte fetch_pressure(unsigned int *p_P_dat)
{


  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;


  address = 0x28;
  Wire.beginTransmission(address);
  Wire.endTransmission();
  delay(100);

  Wire.requestFrom((int)address, (int) 2);//Request 2 bytes need 2 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
  Wire.endTransmission();


  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;

 return (_status);

}

