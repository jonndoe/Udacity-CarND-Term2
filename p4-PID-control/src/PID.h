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
  double total_error_;

  // cross track errors
  double cte_;
  double cte_prev_;
  double cte_mem_;

  /*
  * Coefficients
  */
  double Kp_, Ki_, Kd_;
  double Kp0_, Ki0_, Kd0_;
  double alpha_p_, alpha_i_, alpha_d_;

  /*
  * Other parameters
  */
  double v_;
  double mem_frac_;

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
  void Init(double Kp0, double Ki0, double Kd0, double alpha_p, double alpha_i,double alpha_d, double v, double mem_frac);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
