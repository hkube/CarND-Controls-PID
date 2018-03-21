#include "PID.h"

#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(const double minCtrlVal, const double maxCtrlVal)
: minCtrlVal_(minCtrlVal)
, maxCtrlVal_(maxCtrlVal) {
  Reset();
}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
  Kp_ = Kp;
  Kd_ = Kd;
  Ki_ = Ki;
  Reset();
}

void PID::Reset() {
  err_prev_valid = false;
  err_prev = 0.0;
  err_int = 0.0;
}

double PID::CalculateControlValue(double err, double steering_angle, double speed) {
  if ( ! err_prev_valid) {
    // Init cte_prev
    err_prev = err;
    err_prev_valid = true;
  }

  const double err_diff = err - err_prev;
  err_int += err;
  double result = -(Kp_*err + Kd_*err_diff + Ki_*err_int);
  // Limit result
  if (maxCtrlVal_ < result) {
    result = maxCtrlVal_;
  } else if (minCtrlVal_ > result) {
    result = minCtrlVal_;
  }

  err_prev = err;
  return result;
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
  return 0.0; //p_error + d_error + i_error;
}

