#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <MS5611.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 7, TXPin = 6;
static const uint32_t GPSBaud = 9600;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
int Flag1=0;
int Flag2=0;
MS5611 baro;
double referencePressure;
float relativeAltitude,kalAltitude;
long esti_Pressure;;

void setup()
{
  Serial.begin(57600);
  ss.begin(GPSBaud);
pinMode(2,INPUT_PULLUP);
  // Initialize MS5611 sensor
  Serial.println("Initialize MS5611 Sensor");
   baro = MS5611();
  baro.begin();
  
  // Get reference pressure for relative altitude
  referencePressure = baro.getPressure(); 
attachInterrupt(0, blink, LOW);
attachInterrupt(1, isr, LOW);
  Serial.println();
//Serial.println(F("Latitude   Longitude   course Speed Card  "));
  Serial.println(F(" (deg)      (deg)    --- from GPS ----  l"));
  Serial.println(F("----------------------------------------"));
}

void loop()
{
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
long realPressure= baro.getPressure();
   
  // Calculate altitude
   relativeAltitude = 44330.0f * (1.0f - pow((double)realPressure / (double)referencePressure, 0.1902949f));
  esti_Pressure = pressureKalmanFilter.updateEstimate(realPressure);
   kalAltitude = 44330.0f * (1.0f - pow((double)esti_Pressure / (double)referencePressure, 0.1902949f));

 // Serial.println("--");
  Serial.print("relativeAltitude = ");
  Serial.print(relativeAltitude*3.280084);     //prints just the value of altitude in ft continously with a delay without unit marker nor the stating string of relative altidude 
 Serial.print(",");
  Serial.println(kalAltitude*3.280084);
  Serial.println(" ft");   
  //printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  //printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  //printInt(gps.location.age(), gps.location.isValid(), 5);
  //printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  //printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  //printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

 /* unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);*/




  
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
void blink() {
  if(Flag1==0){
  //Serial.println("----------------------------------");
  Serial.print("Droped supplies at alt:");                         // can comment this out but how to make sure this is recorded ? need special marker ?
  Serial.println(relativeAltitude*3.280084);
 Serial.print(",");
  Serial.println(kalAltitude*3.280084);
 // Serial.println("----------------------------------");
  Flag1=1;

  
  }
 
}
void isr()
{
  if(Flag2==0){
 // Serial.println("----------------------------------");
  Serial.print("Droped CDA at alt:");                                 // can comment this out but how to make sure this is recorded ? need special marker? different from the other isr?
  Serial.println(relativeAltitude*3.280084);
   Serial.print(",");
  Serial.println(kalAltitude*3.280084);
 // Serial.println("----------------------------------");
  Flag2=1;
  }
 
}
