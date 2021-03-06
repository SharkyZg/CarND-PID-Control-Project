#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double b_error;


  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double counter;
  double dp[3];
  double K[3];
  bool K_increased; 
  int i;
  double throttle;
  
  double steering_smooth;
  
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double _Kp, double _Ki, double _Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  void Twiddle (double cte);
  
  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
