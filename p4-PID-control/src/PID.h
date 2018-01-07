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
  double best_error_;

	/*
	 * Coefficients
	 */
	double Kp;
	double Ki;
	double Kd;
  int i;  // parameter index

	// Count of error updates received
	int update_count_;

	// current value of total error
	double total_error_;

	// Count threshold for error calculation vs twiddling
	int count_threshold_;

  // Current state for determining next step in Twiddle process
  int current_state_;

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
	void Init(double Kp, double Ki, double Kd);

	/*
	 * Update the PID error variables given cross track error.
	 */
	void UpdateError(double cte);

	/*
	 * Calculate the total PID error.
	 */
	double TotalError();

	/*
	 * Twiddle method to tune PID coefficients
	 */
	void Twiddle(double cte);

	/*
	 * Output Value for PID instantiation
	 */
	double Calculate();
};

#endif /* PID_H */
