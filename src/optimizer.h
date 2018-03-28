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
  std::vector<double> params, best_param, params_deltas, base_delta_values, base_param_values;

  // Adjustment parameters
  double decrease_factor;
  double decrease_min;

  // Gridsearch paramters
  std::vector<std::vector<double> > grid_search_combinations;
  
  int gridsearch_id; 
  int gridsearch_max_id;
  bool gridsearch;

  // Current parameter index to adjust
  int current_parameter_id;
  int current_parameter_tune_direction;

  // The best error value
  double best_error;

  /*
   * Does an optimization step on the current parameter.
   * */
  void OptimizeParameter(const double& current_error);

  void OptimizeGridSearch(const double &current_score);

  /*
   * Switches to the next parameter that should be optimized.
   * */
  void NextParameter();

  void UpdateBestError(const double& current_error);

  void Print();

 public:

  Optimizer(const std::vector<double> &params, const std::vector<double> &params_deltas);


  void InitializeGridSearch(const std::vector<double> &min_params, const std::vector<double> &max_params, const std::vector<double> &step_params);

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
