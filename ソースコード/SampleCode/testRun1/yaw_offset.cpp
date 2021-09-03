#include "yaw_offset.hpp"

YawOffset::YawOffset() {
  this->target_sum = 0;
  this->target_min = 0;
  this->yaw_sum = 0;
  this->yaw_offset = 0;
  this->data_count = 0;
}

void YawOffset::update(double target_val, double yaw) {
  target_sum += target_val;
  // 生のyaw値をオフセットに変換して加算
  if (yaw > 0) {
    yaw_sum += (yaw - 180);
  } else {
    yaw_sum += (yaw + 180);
  }
  data_count++;
  if (data_count >= 20) {
    double target_mean = target_sum / data_count;
    // 最小値とオフセットを保持
    if (target_min > target_mean) {
      target_min = target_mean;
      yaw_offset = yaw_sum / data_count;
    }
    // String msg = "";
    // msg += target_mean;
    // msg += ",";
    // msg += yaw_sum / data_count;
    // Serial.println(msg);

    target_sum = 0; yaw_sum = 0;
    data_count = 0;
  }
}

double YawOffset::getOffset() {
  return this->yaw_offset;
}