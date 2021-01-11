#ifndef PID_H
#define OID_H
#include <HardwareSerial.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

class PID {
  public:
    PID(float Kp, float Ki, float Kd, double set_point, double dead_zone, double max_out, bool flip_out = false, float error_division_factor = 1.0f) :
      Kp(Kp), Ki(Ki), Kd(Kd), set_point(set_point), dead_zone(dead_zone), max_out(max_out), flip_out(flip_out), error_division_factor(error_division_factor)
    {
      default_set_point = set_point;
    }
    double updatePID(double cur_val) {
      if (!PID_flag) {
        if (flip_out) return -out;
        else return out;
      }
      error = (set_point - cur_val) / error_division_factor;
      error_d = error - p_error;
      error_sum += error * Ki;
      p_error = error;
      if (error_sum > max_out / 1.0) error_sum = max_out / 1.0;
      else if (error_sum < -(max_out / 1.0)) error_sum = -max_out / 1.0;
      out = error * Kp + error_d * Kd + error_sum;
      if (out > max_out) out = max_out;
      else if (out < -max_out) out = -max_out;
//      else if (out > -dead_zone && out < dead_zone) out = 0;
      PID_flag = false;
      if (flip_out) return -out;
      else return out;
    }
    void resetErrorSum() {
      error_sum = 0;
    }
    double set_point;
    double default_set_point;
    volatile bool PID_flag = false;
  private:
    float Kp, Ki, Kd;
    double dead_zone, error, p_error, error_d, error_sum, out, max_out;
    int error_division_factor;
    bool flip_out;
};

#endif