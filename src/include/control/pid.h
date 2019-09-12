// Adapted from https://gist.github.com/bradley219/5373998

#ifndef _CONTROL_PID_H_
#define _CONTROL_PID_H_

namespace control {
class PIDImpl;
class PID {
public:
  PID();

  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  PID(double max, double min, double Kp, double Kd, double Ki);
  void SetConstants(double max, double min, double Kp, double Kd, double Ki);

  void Reset();

  // dt -  loop interval time
  // Returns the manipulated variable given a setpoint and current process value
  double calculate(const double &dt, const double &setpoint, const double &pv);
  ~PID();

private:
  PIDImpl *pimpl;
};
} // namespace control
#endif
