#include <TinyGPS++.h>
#include <SoftwareSerial.h>
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9800;
TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
Serial.println("*********************");

}

void loop() {
  static const double Des_LAT = 12.969246, Des_LON = 79.155753;

  //Serial.println(gps.course.deg());

  double courseToCenter =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      Des_LAT, 
      Des_LON);

  printFloat(courseToCenter, gps.location.isValid(), 7, 2);

  const char *cardinalToCenter = TinyGPSPlus::cardinal(courseToCenter);

  printStr(gps.location.isValid() ? cardinalToCenter : "*** ", 6); 
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

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
static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
