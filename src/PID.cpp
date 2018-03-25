#include "PID.h"
#include <iostream>
#include <algorithm>
#include <numeric>

using namespace std;

/*
* Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, int length_error_memory) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->last_errors = std::vector<double>(length_error_memory, 0.0);
}

void PID::Init(const std::vector<double>& params, int length_error_memory) {

  if(params.size() != 3){
     std::cout << "Illegal parameter vector given in SetParams! " << std::endl;
     return;
   }
  this->Kp = params[0];
  this->Ki = params[1];
  this->Kd = params[2];
  this->last_errors = std::vector<double>(length_error_memory, 0.0);
}

void PID::SetParams(const double& Kp, const double& Ki, const double& Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::SetParams(const std::vector<double>& params) {

  if(params.size() != 3){
    std::cout << "Illegal parameter vector given in SetParams! " << std::endl;
    return;
  }
  this->Kp = params[0];
  this->Ki = params[1];
  this->Kd = params[2];
}

void PID::UpdateError(double cte) {
  // Shift the errors in the error memory (FIR filter)
  std::rotate(last_errors.begin(), last_errors.begin() + 1, last_errors.end());
  *last_errors.rbegin() = cte;
}

double PID::GetLastError() const {
  if(last_errors.size() > 0)
    return *last_errors.rbegin();

  return -1.0;
}

double PID::GetUpdatedSteering() {

  if(last_errors.size() < 2) {
    std::cout << "Could not retrieve last error!" << std::endl;
    return -1.0;
  }
  double last_error = last_errors[last_errors.size() - 2];
  double curr_error = last_errors[last_errors.size() - 1];
  double diff_error = curr_error - last_error;

  double updated_steering = (-Kp * curr_error) - (Kd * diff_error) - (Ki * TotalError());
  return updated_steering;
}

double PID::TotalError() {
  double accumulated_abs_error = 0.0;
  for(const auto& e : last_errors)

    accumulated_abs_error += fabs(e);
  return accumulated_abs_error;
}

