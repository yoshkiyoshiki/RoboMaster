#include "GM6020Test.h"

#include <Arduino.h>
GM6020::GM6020(double _deltaT) {
  this->deltaT = _deltaT;
  this->setPIDgain_position(3.0, 0.0, 0.0);

  this->generalPosition = -1;
  this->defaultPosition = 9000;
  this->count = 0;
  this->lastangle[0] = 0;
  this->lastangle[1] = LV;
}
GM6020::~GM6020() {}

void GM6020::setGM6020Data(uint8_t data[8]) {
  int16_t rawPosition = (data[0] << 8) + data[1];
  if (this->defaultPosition == 9000) {
    this->defaultPosition = rawPosition;
  }
  if (lastangle[1] == LV) {
    this->lastangle[1] = 0;
    this->count = 0;
  } else {
    int deff = rawPosition - this->lastangle[0];
    if (deff < -(this->EncResolution_degree / 2)) {
      this->count++;
    } else if ((this->EncResolution_degree / 2) < deff) {
      this->count--;
    }
  }
  this->lastangle[0] = rawPosition;
  this->generalPosition = ((this->count * this->EncResolution_degree) +
                           rawPosition - this->defaultPosition);
}

int16_t GM6020::culcVoltageFromPosition(double targetPosition) {
  double errorPosition = targetPosition - this->generalPosition;
  this->error_Integral_position +=
      (errorPosition + this->prev_error_position) / 2.0 * this->deltaT;
  double target =
      errorPosition * Kp_position +
      this->error_Integral_position * Ki_position +
      (errorPosition - this->prev_error_position) * this->Kd_position;

  this->prev_error_position = errorPosition;
  double voltage = constrain(target, -24, 24);
  double buf = map(voltage, -24, 24, -25000, 25000);
  return (int16_t)buf;
}

void GM6020::setPIDgain_position(double _kp, double _ki, double _kd) {
  Kp_position = _kp;
  Ki_position = _ki;
  Kd_position = _kd;
}
