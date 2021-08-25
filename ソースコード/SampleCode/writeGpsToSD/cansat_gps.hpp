#ifndef __CANSAT_GPS_H__
#define __CANSAT_GPS_H__

#include "Arduino.h"
#include <stdio.h>
#include <TinyGPS++.h>

class Cansat_gps {
  public:
    Cansat_gps();

    static const uint32_t GPSBaud = 9600;

    bool lat(char raw_gps, double* ret); // 緯度
    bool lng(char raw_gps, double* ret); // 経度
    bool gpsCsv(char raw_gps, char** ret);

  private:
    TinyGPSPlus gps;
    bool dupFlg = false;
};

#endif // __CANSAT_GPS_H__
