
//#include <WireMW.h>
#include <Wire.h>   //I2C library 0x28H
byte fetch_pressure(unsigned int *p_Pressure); //convert value to byte data type
float Alti,Calc_Range,Air_speed;
#define TRUE 1
#define FALSE 0

void setup(void)
{
  Serial.begin(57600);
  Wire.begin();
  delay(500);
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");  // just to be sure things are working
}

void loop()
{
  byte _status;
  unsigned int P_dat;
  unsigned int T_dat;
  double PR;
  double TR;
  double V;
  double VV;
  while (1)
  {
    _status = fetch_pressure(&P_dat);

    switch (_status)
    {
      case 0: Serial.println("Ok ");
        break;
      case 1: Serial.println("Busy");
        break;
      case 2: Serial.println("Slate");
        break;
      default: Serial.println("Error");
        break;
    }


    PR = (double)((P_dat-819.15)/(14744.7)) ;
    PR = (PR - 0.49060678) ;
    PR = abs(PR);
     V = ((PR*13789.5144)/1.1644
     );
    VV = (sqrt((V)));
    VV-=6;
       
  Serial.println("aaaaaaa");
  Alti=100;
  Air_speed=VV;
  Serial.print("speed m/s :");
  Serial.println(Air_speed,5);
  Serial.print("speed Km/h :");
  Serial.println(Air_speed*3.6,5);
  Calc_Range=Air_speed*sqrt((2*Alti)/9.8);
  Serial.println("to be droped at range:");
  Serial.println(Calc_Range);
  delay(1000);
  
    delay(1000);
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

  Wire.requestFrom((int)address, (int) 4);//Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
  byte Temp_H = Wire.read();
  byte  Temp_L = Wire.read();
  Wire.endTransmission();


  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;
  return (_status);



}
