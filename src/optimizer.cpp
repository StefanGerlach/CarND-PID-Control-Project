/*
 * optimizer.cpp
 *
 *  Created on: Mar 25, 2018
 *      Author: skutch
 */

#include "optimizer.h"
#include <iostream>
#include <cmath>


Optimizer::Optimizer(const std::vector<double> &params, const std::vector<double> &params_deltas) :
  params(params),
  params_deltas(params_deltas),
  best_param(params),
  base_delta_values(params_deltas),
  base_param_values(params)
{
  decrease_factor = 0.8;
  decrease_min = 1e-3;

  current_parameter_id = 0;
  current_parameter_tune_direction = 1;

  best_error = -1.0;
  gridsearch = false;
  gridsearch_max_id = 0;
  gridsearch_id = 0;
}


void Optimizer::InitializeGridSearch(const std::vector<double> &min_params, const std::vector<double> &max_params, const std::vector<double> &step_params) {

  gridsearch = true;
  gridsearch_id = 0;

  grid_search_combinations.clear();

  if((min_params.size() != max_params.size()) || (min_params.size() != step_params.size())) {
    std::cout << "Error initializing Gridsearch!" << std::endl;
    gridsearch = false;
    return;
  }

  // Create lists of values
  int num_params= static_cast<int>(min_params.size());

  std::vector<std::vector<double> > value_lists(min_params.size());

  for(int p = 0; p < static_cast<int>(min_params.size()); p++) {
    double c_value = min_params[p];

    if(max_params[p] < min_params[p]) {
      std::cout << "Error initializing Gridsearch! Max value is greater than min!" << std::endl;
      gridsearch = false;
      return;
    }

    while(c_value < max_params[p]) {
      value_lists[p].push_back(c_value);
      c_value += step_params[p];
    }    
  }  
  
  std::vector<double> permutation(min_params.size());
  std::vector<int> indices(num_params, 0);
  std::vector<int> indices_offsets(num_params, 1);

  // Set the max count of gridsearch combinations
  gridsearch_max_id = static_cast<int>(std::pow(value_lists.size(), value_lists[0].size()));

  int current_idx = 0;
  for(int i = 0; i < gridsearch_max_id; i++) {
    
    if(indices[current_idx] + 1 >= static_cast<int>(value_lists[i].size())) {
      indices[current_idx] = indices_offsets[current_idx];
      indices_offsets[current_idx]++;
      indices_offsets[current_idx] = indices_offsets[current_idx] % static_cast<int>(value_lists[current_idx].size());

      current_idx++;
      current_idx = current_idx % num_params;
    }
    
    for(int idx = 0; idx < static_cast<int>(indices.size()); idx++)
      permutation[idx] = value_lists[idx][indices[idx]];

    grid_search_combinations.push_back(permutation);
   
    indices[current_idx]++;
  }

  std::cout << "GridSearch initialized with " << static_cast<int>(grid_search_combinations.size()) << " combinations." << std::endl;
}


const std::vector<double>& Optimizer::GetParams() const {
  return params;
}


void Optimizer::NextParameter() {
  // Set the best one to the current
  params[current_parameter_id] = best_param[current_parameter_id];

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
  best_param[current_parameter_id] = params[current_parameter_id];
  base_param_values[current_parameter_id] = params[current_parameter_id];
}

void Optimizer::OptimizeGridSearch(const double &current_score) {

  params = grid_search_combinations[gridsearch_id];

  if(current_score > best_error) {
    best_param = grid_search_combinations[gridsearch_id];
    best_error = current_score;
  }

  if(gridsearch_id++ > static_cast<int>(grid_search_combinations.size())) {
    gridsearch = false;
  } 
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

    // Clamp to zero
    if(param < 0.0) {
      param = 0.0;
    }
  }

  return;
}

void Optimizer::Print() {

  std::string direction = current_parameter_tune_direction == 1 ? "+1" : "-1";

  if(gridsearch) {
    std::cout << "GridSearching parameter. Step " << gridsearch_id << " / " << gridsearch_max_id << std::endl;
  } else {
    std::cout << "Optimizer working on parameter [" << current_parameter_id << "] in direction ["<< direction << "]" << std::endl;
  }
  std::cout << "Best score was " << best_error << " with parameter: ";
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

  double error_score = 1.0 - (current_error / 2.5);
  double distance_score = static_cast<double>(driven_steps) / 7000.0;
  
  distance_score *= 10.0;

  double current_score = (error_score + distance_score) / 11.0;

  if(gridsearch) {
    OptimizeGridSearch(current_score);
  } else {
    // Optimize on the currently targetted parameter
    OptimizeParameter(current_score);
  }

  // Print Debug
  Print();

  // Reset PID controller
  pid_controller.Init(params);
}



