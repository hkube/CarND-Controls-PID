#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Constructor
  */
  PID(const double minCtrlVal, const double maxCtrlVal);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Kd, double Ki);

  /*
   * Reset the internal variables
   */
  void Reset();

  /*
  * Calculate steering angle.
  */
  double CalculateControlValue(double cte, double steering_angle, double speed);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  /*
  * Initialize PID.
  */
  void ResetCTE();

  /*
  * Errors
  */
//  double p_error;
//  double i_error;
//  double d_error;

  /*
  * Coefficients
  */
  double Kp_;
  double Kd_;
  double Ki_;

  /*
   * Limits of the output value
   */
  const double minCtrlVal_;
  const double maxCtrlVal_;

  bool err_prev_valid;
  double err_prev;
  double err_int;
};

#endif /* PID_H */
