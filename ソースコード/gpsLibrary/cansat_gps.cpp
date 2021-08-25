#include "cansat_gps.hpp"

Cansat_gps::Cansat_gps() {
}

/**
 * 緯度のみを返す
 */
bool Cansat_gps::lat(char raw_gps, double* ret) {
    gps.encode(raw_gps); // NMEAフォーマットの文字列をパースする
    if(gps.location.isUpdated()){ // 位置情報が更新された
      if(dupFlg != true){
        *ret = gps.location.lat();
        dupFlg = true;
        return true;
      }
      else{
        return false; 
      }
    }
    else{
      dupFlg = false;
      return false;
    }
}

/**
 * 経度のみを返す
 */
bool Cansat_gps::lng(char raw_gps, double* ret) {
    gps.encode(raw_gps); // NMEAフォーマットの文字列をパースする
    if(gps.location.isUpdated()){ // 位置情報が更新された
      if(dupFlg != true){
        *ret = gps.location.lng();
        dupFlg = true;
        return true;
      }
      else{
        return false; 
      }
    }
    else{
      dupFlg = false;
      return false;
    }
}

/**
 * 緯度と経度をCSV形式で返す(char*)
 */
bool Cansat_gps::gpsCsv(char raw_gps, char** ret) {
    gps.encode(raw_gps); // NMEAフォーマットの文字列をパースする
    if(gps.location.isUpdated()){ // 位置情報が更新された
      if(dupFlg != true){
        double a = gps.location.lat();
        double b = gps.location.lng();

        String a_str = String(a, 6);
        String b_str = String(b, 6);
        a_str.concat(",");
        a_str.concat(b_str);
        a_str.concat("\n");

        //メモリ確保
        char* cstr = new char[a_str.length() + 1];
        strcpy(cstr, a_str.c_str());
        
        *ret = cstr;
        
        dupFlg = true;
        return true;
      }
      else{
        return false; 
      }
    }
    else{
      dupFlg = false;
      return false;
    }
}
