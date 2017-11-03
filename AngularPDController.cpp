#include "AngularPDController.h"
#include <Arduino.h>
#include "utils.h"

AngularPDController::AngularPDController(float kp, float kd) {
  pd_constants.Kp = kp;
  pd_constants.Kd = kd;

  last_angular_error = 0.0;
  last_error_deriv = 0.0;
  last_command = 0.0;
  last_pterm = 0.0;
  last_dterm = 0.0;
}

/*
 * Need to change dt to microseconds to speed up calculations
 */
float AngularPDController::compute_command(const float& error, const unsigned long& dt_micros) {
  // Compute p term
  last_pterm = pd_constants.Kp*error;

  // Compute angular velocity of error
  if(first_loop) {
    last_error_deriv = 0.0;
    first_loop = false;
  } else {
    last_error_deriv = 1000000*utils_angle_difference(error, last_angular_error) /
        (dt_micros);
  }
  last_angular_error = error;

  // Compute d term
  last_dterm = pd_constants.Kd*last_error_deriv;

  // Compute final command
  last_command = last_pterm + last_dterm;

  // Constrain the output between -1.0 and 1.0
  last_command = last_command > 1.0 ? 1.0 : last_command;
  last_command = last_command < -1.0 ? -1.0 : last_command;

  return last_command;
}

float AngularPDController::get_error() {
  return last_angular_error;
}

float AngularPDController::get_error_deriv() {
  return last_error_deriv;
}

float AngularPDController::get_command() {
  return last_command;
}

void AngularPDController::get_command(float& command) {
  command = last_command;
}
void AngularPDController::get_error_terms(float& pterm, float& dterm) {
  pterm = last_pterm;
  dterm = last_dterm;
}

float AngularPDController::get_pterm() {
  return last_pterm;
}

float AngularPDController::get_dterm() {
  return last_dterm;
}

void AngularPDController::get_gains(float& Kp, float& Kd) {
  Kp = pd_constants.Kp;
  Kd = pd_constants.Kd;
}
