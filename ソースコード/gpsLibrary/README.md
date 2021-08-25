# GPS取得用ライブラリ

## 利用例 

```c
#include "cansat_gps.hpp"

Cansat_gps cansat_gps = Cansat_gps();

double lat_value;

while(ss.available() > 0){
     char c = ss.read();    // GPSセンサからの値を読み込み
     // 緯度の取得
     if(cansat_gps.lat(c, &lat_value) == true){
          Serial.println(lat_value);
     }
}

```