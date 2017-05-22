#include "PID.h"
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID1 class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double _Kp, double _Ki, double _Kd) {
  p_error = -6; 
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
  K[0] = Kp;
  K[1] = Ki;
  K[2] = Kd;

    
  dp[0] = .005;
  dp[1] = .0000001;
  dp[2] = .05;
  
  i = 0;
  counter = 0;
  K_increased = true;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

void PID::Twiddle (double cte) {
  
  if (counter > 19) {
    if (i==0){
      i = 1;
    } else if (i==1){
      i = 2;
    } else if (i==2){
      i = 0;
    }

  }
  double K_dbl = K[i];
  
  counter+=1;
    
  if (K_increased==true) {
    if (fabs(cte) < fabs(b_error)){
      b_error = cte;
      K[i]=K_dbl+dp[i];
      dp[i]*= 1.1;
      K_increased = true;
    } else {
      K[i]=K_dbl-dp[i];
      K_increased = false;
      dp[i]*= 0.9;
    } 
  } else {
    if (fabs(cte) < fabs(b_error)){
      K[i]=K_dbl-dp[i];
      b_error = cte;
      dp[i]*= 1.1;
      K_increased = false;
    } else {
      K[i]=K_dbl+dp[i];
      K_increased = true;
      dp[i] *= 0.9;
    }
  }
        

}

double PID::TotalError() {
  double total_error;

  total_error = -Kp * p_error - 1*Kd * d_error - Ki * i_error;
  return total_error;
}

