# GPS取得用ライブラリ (エラー修正中)

cansat_gps.hpp内でHardwareSerial ss(2);と宣言しているのですが、エラーが出力されます。
原因を調べています。アドバイスなどあればよろしくお願いします。

```c
cansat_gps.hpp:17:23: error: expected identifier before numeric constant

     HardwareSerial ss(2);

cansat_gps.hpp:17:23: error: expected ',' or '...' before numeric constant
```

## 利用例 
完成したら、このように動作させたい。

```c
#include "cansat_gps.hpp"

Cansat_gps cansat_gps = Cansat_gps();

double lat_value;
double lng_value;
// 緯度の取得
lat_value = cansat_gps.lat();
// 経度の取得
lng_value = cansat_gps.lng();

```