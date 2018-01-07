#include "PID.h"
#include <iostream>
#include <math.h>
using namespace std;

#define DEBUG 0

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

const double tolerance = 0.01;

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = 0.1;
  i_error = 0.001;
  d_error = 0.1;

  // Initialize counter
	update_count_ = 0;

	// Initialize errors
	total_error_ = 0.00;
	best_error_ = 9999.9;

	// Initialize counter
	count_threshold_ = 0;

	// Initialize state
	current_state_ = 0;

  // Initialize parameter index
	i = 0;
}

void PID::UpdateError(double cte) {
  double previous_cte = p_error;

  // Proportional error is the Cross Track Error (CTE)
  p_error = cte;

  // Integral error
  i_error = i_error + cte;

  // Differential error is the rate of change of the CTE
  d_error = cte - previous_cte;

	// debugging
	if (DEBUG)
		printf("\n p_error : %.02f i_error: %.02f d_error: %.02f", p_error, i_error, d_error);
}

void PID::Twiddle(double cte) {
  double p[] = {Kp, Ki, Kd};
  double dp[] = {p_error, i_error, d_error};

  if (TotalError() > tolerance) {
    switch (current_state_) {
      case 0: {
        p[i] += dp[i];
        current_state_ = 1;
        break;
      }
      case 1: {
        if (fabs(cte) < fabs(best_error_)) {
          best_error_ = cte;
          dp[i] *= 1.1;
          current_state_ = 3;
        } else {
          p[i] -= 2 * dp[i];
          current_state_ = 2;
        }
        break;
      }
      case 2: {
        if (fabs(cte) < fabs(best_error_)) {
          best_error_ = cte;
          dp[i] *= 1.1;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
        current_state_ = 3;
        break;
      }
      case 3: {
        i = (i + 1) % 3;
        current_state_ = 0;
        break;
      }
    }
    cout << "\n P values: {"<< p[0] << ", "<< p[1]  << ", "<< p[2] << "}"<< endl;
    p_error = dp[0];
    i_error = dp[1];
    d_error = dp[2];
    Kp = p[0];
    Ki = p[1];
    Kd = p[2];
  }
}

double PID::Calculate() {
	// Calculate output
	double output_val = -1.0 * (Kp*p_error + Kd*d_error + Ki*i_error);

	// Limit between zero and one
	if (output_val > 1.0)
		output_val = 1.0;
	else if (output_val < -1.0)
		output_val = -1.0;

  // debugging
	if(DEBUG)
		printf("\n Output is %.03f", output_val);

	// Return output
	return output_val;
}

double PID::TotalError() {

  double tot_error = fabs(p_error) + fabs(i_error) + fabs(d_error);

  // debugging
  if(DEBUG)
    printf("\n Total Error is %.03f", tot_error);

  return tot_error;
}
