
//#include <WireMW.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h> 
SoftwareSerial mySerial(4, 3);
TinyGPS gps;
void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

byte fetch_pressure(unsigned int *p_Pressure); //convert value to byte data type

#define TRUE 1
#define FALSE 0

void setup(void)
{  mySerial.begin(9600);
    Serial.begin(57600);
  Wire.begin();
  delay(500);
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>");  // just to be sure things are working
}

void loop()
{
  byte _status;
  unsigned int P_dat;
  double PR;
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
    PR = abs(PR - 0.49060678) ;
    
     V = ((PR*13789.5144)/1.1644);
     Serial.println("pr:");
     Serial.println(PR);
    VV = (sqrt((V)));
    VV-=6;
   
 
   
 //  Serial.print("raw Pressure:");
   Serial.println(P_dat);
   //Serial.println(P_dat,DEC);
   //Serial.println(P_dat,BIN);
 //  Serial.print("pressure psi:");
  // Serial.println(PR,10);
 //  Serial.print(" ");
 //  Serial.print("raw Temp:");
  // Serial.println(T_dat);
 //  Serial.print("temp:");
 //  Serial.println(TR);
   Serial.print("speed m/s :");
   Serial.println(VV,5);
   Serial.print("speed Km/h :");
   Serial.println(VV*3.6,5);
bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < 1000) 
  {
    if (mySerial.available()) 
    {
      char c = mySerial.read();
     // Serial.print(c);  // uncomment to see NMEA format data
      if (gps.encode(c)) 
      {
        newdata = true;
        break;  
      }
    }
  }
  
  if (newdata) 
  {
    gpsdump(gps);
  }

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

  Wire.requestFrom((int)address, (int) 2);//Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
  Wire.endTransmission();


  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;
return (_status);

}
void gpsdump(TinyGPS &gps)
{
  float flat, flon;
 gps.f_get_position(&flat, &flon);
   printFloat(flat, 5); Serial.print(","); 
   printFloat(flon, 5);Serial.print(",");
   printFloat(gps.f_speed_kmph()/3.6);
   Serial.println();
   //prints the lat,long and velocity(in m/s) seperated by commas
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) 
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) 
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
