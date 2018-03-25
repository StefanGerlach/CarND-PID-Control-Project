/*
 * optimizer.h
 *
 *  Created on: Mar 25, 2018
 *      Author: skutch
 */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <vector>
#include "PID.h"

class Optimizer {

 private:

  // Parameter
  std::vector<double> params, params_deltas, base_delta_values, base_param_values;

  // Adjustment parameters
  double decrease_factor;
  double decrease_min;

  // Current parameter index to adjust
  int current_parameter_id;
  int current_parameter_tune_direction;

  // The last error value
  double last_error;

  /*
   * Does an optimization step on the current parameter.
   * */
  void OptimizeParameter(const double& current_error);

  /*
   * Switches to the next parameter that should be optimized.
   * */
  void NextParameter();

  void UpdateLastError(const double& current_error);

  void Print();

 public:
  Optimizer(const std::vector<double> &params, const std::vector<double> &params_deltas);

  /*
   * Returns the current parameter set.
   * */
  const std::vector<double>& GetParams() const;

  /*
   * Does an optimization step on the current parameter.
   * This function adjusts the pid-controller parameters.
   * */
  void OptimizeOnRun(PID& pid_controller, long driven_steps);

};

#endif /* OPTIMIZER_H_ */
