#include "Controller.h"

int64_t PI_Vel(float SetPoint, float CurrentValue, float Ts) {
  float error = SetPoint - CurrentValue;
  float up = 0, ud = 0;
  static float ui, errorReset, pre_error;
  int64_t pwm;

  int64_t Hlim = 1000;
  int64_t Llim = -1000;

  float Kp = 10.8111;
  float Ki = 464.0847;
  float Kd = 0;
  float Kb = 42.9266;

  up = Kp * error;
  ui += Ki*error*Ts + Kb*errorReset*Ts;
  ud = Kd*(error - pre_error)/Ts;
  pre_error = error;

  int64_t uout = (int64_t)(up + ui + ud);

  if (uout > Hlim)  pwm = Hlim;
  else if (uout < Llim) pwm = Llim;
  else pwm = uout;

  errorReset = pwm - uout;

  return pwm;
}

int64_t PID_Pos(float SetPoint, float CurrentValue, float Ts) {
  float error = SetPoint - CurrentValue;
  float up = 0, ud = 0;
  static float ui, errorReset, pre_error;
  int64_t pwm;

  int64_t Hlim = 900;
  int64_t Llim = -900;

  float Kp = 1144.3917;
  float Ki = 9281.6948;
  float Kd = 21.6222;
  float Kb = 20.7187;

  up = Kp * error;
  ui += Ki*error*Ts + Kb*errorReset*Ts;
  ud = Kd*(error - pre_error)/Ts;
  pre_error = error;

  int64_t uout = (int64_t)(up + ui + ud);

  if (uout > Hlim)  pwm = Hlim;
  else if (uout < Llim) pwm = Llim;
  else pwm = uout;

  errorReset = pwm - uout;

  return pwm;
}
