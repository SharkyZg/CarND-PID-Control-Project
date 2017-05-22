#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID1 class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {
  p_error = -1; 
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

void PID::Twiddle (double cte) {
  double K[] = {Kp, Ki, Kd};
  double dp[] = {1, 1, 1};
  
  for (int i=0; i<3; i++) {
    
    
    if (cte < b_error){
      b_error = cte;
    }
    else {
      K[i]-=2*dp[i];
    }
  K[i]+=dp[i];
  }

}

double PID::TotalError() {
  double total_error;

  total_error = -Kp * p_error - 1*Kd * d_error - Ki * i_error;
  return total_error;
}

