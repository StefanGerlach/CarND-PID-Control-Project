/*
 * optimizer.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: skutch
 */

#include "optimizer.h"
#include <iostream>


Optimizer::Optimizer(const std::vector<double> &params, const std::vector<double> &params_deltas) :
  params(params),
  params_deltas(params_deltas),
  best_param(params),
  base_delta_values(params_deltas),
  base_param_values(params)
{
  decrease_factor = 0.75;
  decrease_min = 1e-4;

  current_parameter_id = 0;
  current_parameter_tune_direction = 1;

  best_error = 0.0;
}

const std::vector<double>& Optimizer::GetParams() const {
  return params;
}


void Optimizer::NextParameter() {
  // The base value of the current parameter is set to the parameter value!
  base_param_values[current_parameter_id] = params[current_parameter_id];

  // The delta for the current parameter is being reset
  params_deltas[current_parameter_id] = base_delta_values[current_parameter_id];

  // The direction is set to positive !
  current_parameter_tune_direction = 1;

  // Switch to next param
  current_parameter_id++;
  current_parameter_id = current_parameter_id % static_cast<int>(params.size());

  return;
}

void Optimizer::UpdateBestError(const double& current_error) {
  // Update best error
  best_error = current_error;

  // Update best parameter
  best_param = params;
}


void Optimizer::OptimizeParameter(const double& current_error) {
  // Check if we are initialized
  if(best_error < 0) {
    // Update last error and return
    best_error = current_error;
    return;
  }

  // Get the reference to the current parameter
  double & param = params[current_parameter_id];

  // Get the reference to the base value
  double & base_value = base_param_values[current_parameter_id];

  // Get the reference to the current delta value
  double & delta = params_deltas[current_parameter_id];


  // Check if we improved
  bool improved = (current_error > best_error);

  if(improved) {

    UpdateBestError(current_error);

    // The base value of the current parameter is the parameter itself now.
    base_value = param;

    // Get the directional factor
    double direction = current_parameter_tune_direction == 1 ? 1.0 : -1.0;

    // Update the parameter again
    param += direction * delta;

  } else {
    // OK, so we did not improve!
    // Reset parameter to base_value
    param = base_value;

    // Update the delta
    delta *= decrease_factor;

    // If the current delta is below a threshold, go into the other direction
    if(delta < decrease_min) {
      // Reset delta to default / base value
      delta = base_delta_values[current_parameter_id];

      // Go into the next direction!
      if(current_parameter_tune_direction == 1) {

        current_parameter_tune_direction = 0;
      } else {
        // Oh, we did try all directions. Now we stop and take the next Parameter
        NextParameter();
        return;
      }
    }
    // Get the directional factor
    double direction = current_parameter_tune_direction == 1 ? 1.0 : -1.0;

    // Update the parameter !
    param += direction * delta;

    // Finally check, if the parameter is negative
    if(param < 0.0) {
      param = 0.0;
      NextParameter();
    }
  }

  return;
}

void Optimizer::Print() {

  std::string direction = current_parameter_tune_direction == 1 ? "+1" : "-1";

  std::cout << "Optimizer working on parameter [" << current_parameter_id << "] in direction ["<< direction << "]" << std::endl;
  std::cout << "Best error was " << best_error << " with parameter: ";
  for(const auto& p : best_param) {
      std::cout << "[" << p << "] ";
    }
  std::cout << std::endl;
  std::cout << "Current parameter set: ";
  for(const auto& p : params) {
    std::cout << "[" << p << "] ";
  }
  std::cout << std::endl;
  std::cout << "Current delta set: ";
  for(const auto& p : params_deltas) {
    std::cout << "[" << p << "] ";
  }
  std::cout << std::endl;
}

void Optimizer::OptimizeOnRun(PID& pid_controller, long driven_steps) {

  // Compute current average CTE
  double current_error = pid_controller.TotalError() / static_cast<double>(driven_steps);

  // Optimize on the currently targetted parameter
  OptimizeParameter(driven_steps);

  // Print Debug
  Print();

  // Reset PID controller
  pid_controller.Init(params);
}



