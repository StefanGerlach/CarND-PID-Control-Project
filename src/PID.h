#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  std::vector<double> last_errors;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd, int length_error_memory = 4096);

  /*
   * Updates the PID parameters Tau_p, Tau_d and Tau_i
   * */
  void SetParams(const double& Kp, const double& Ki, const double& Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
   * Computes the current steering with respect to current steering and the cross track error.
   * */
  double GetUpdatedSteering();

  /*
   * Returns the last error
   * */
  double GetLastError() const;

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
