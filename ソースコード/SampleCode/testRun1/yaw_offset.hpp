#ifndef __YAW_OFFSET_H__
#define __YAW_OFFSET_H__

#include "Arduino.h"

class YawOffset {
  public:
    YawOffset();

    void update(double target_val, double yaw);
    double getOffset();

  private:
    double target_sum;
    double target_min;
    double yaw_sum;
    double yaw_offset;
    unsigned int data_count;
};

#endif //__YAW_OFFSET_H__