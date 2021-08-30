#include "cansat_gps.hpp"
#include "sd_lib.hpp"

MySD mysd = MySD();

Cansat_gps cansat_gps = Cansat_gps();
HardwareSerial ss(2);

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  
  Serial.println("GPS start!");
}

void loop() {

  char* temp;

  while(ss.available() > 0){
    char c = ss.read();    // GPSセンサからの値を読み込み
    if(cansat_gps.gpsCsv(c, &temp) == true){
      mysd.appendLog(temp);
      Serial.println(temp);
    }
  }
  
  delay(1000);
}
