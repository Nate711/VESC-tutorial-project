#ifndef ANGULARPDCONTROLLER_H
#define ANGULARPDCONTROLLER_H

class AngularPDController {
private:
  // Stores whether this is fthe first time through, important for initializing
  // time derivatives
  bool first_loop = true;

  // PID constants
  float Kp,Ki,Kd;

  // Constrains the output of the PID controller between -threshold and +threshold
  float threshold;

  // stores last given error
  float last_error_deriv;

  // stores the last angular error float
  float last_angular_error;

  // stores the last outputted command
  float last_command;

  // stores the last computed pterm and dterm
  float last_pterm, last_dterm;

  // struct for holding PD constants
  struct PD_Constants {
  	float Kp;
  	float Ki;
  	float Kd;
  } pd_constants;

public:
  AngularPDController(float kp, float kd);
  float compute_command(const float& error, const unsigned long& dt_micros);

  float get_error();
  float get_error_deriv();

  void get_command(float& command);
  float get_command();
  void get_error_terms(float& pterm, float& dterm);
  float get_dterm();
  float get_pterm();
  void get_gains(float& Kp, float& Kd);
};

#endif /*ANGULARPDCONTROLLER_H*/
