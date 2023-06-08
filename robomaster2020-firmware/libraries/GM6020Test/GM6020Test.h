// #pragma once
#ifndef __GM6020__
#define __GM6020__
#include <Arduino.h>

class GM6020 {
 private:
  double deltaT;  // Control cycle [sec]

  int16_t generalPosition;
  int16_t defaultPosition;
  int16_t count;
  const int16_t LV = 10;
  double lastangle[2];
  const int EncResolution_degree = 8191;

  double prev_error_position;
  double error_Integral_position;
  double Kp_position;
  double Ki_position;
  double Kd_position;

 public:
  GM6020(double _deltaT);
  ~GM6020();
  void setPIDgain_position(double _kp, double _ki, double _kd);

  void setGM6020Data(uint8_t data[8]);
  int16_t culcVoltageFromPosition(double targetPosition);
};

#endif