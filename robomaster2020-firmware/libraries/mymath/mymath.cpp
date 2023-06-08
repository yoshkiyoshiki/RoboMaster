#include "mymath.h"
#include <Arduino.h>

double dmap(double target, double fromMin, double fromMax, double toMin, double toMax){
    return (target - fromMin)*(toMax - toMin)/(fromMax - fromMin) + toMin;
}

long lmap(long target, long fromMin, long fromMax, long toMin, long toMax){
    return (long)((target - fromMin)*((toMax - toMin)/(fromMax - fromMin))) + toMin;
}


double rad2eular(double rad){
    return (rad*RAD)/PI;
}
double eular2rad(double eular){
    return (eular*PI)/RAD;
}

double rad2encoder(double rad){
    return ((rad/PI)*ENCODER_ONE_AROUND/2.0);
}

double encoder2rad(double encoder){
    // return (encoder*2.0*PI)/((double)(ENCODER_ONE_AROUND));
    return encoder*(2*PI)/ENCODER_ONE_AROUND;
}

double eular2encoder(double eular){
    // return rad2encoder(rad2eular(eular));
    return eular*ENCODER_ONE_AROUND/360.0;
}

double encoder2eular(double encoder){
    // return rad2eular(encoder2rad(encoder));
    return encoder*360.0/ENCODER_ONE_AROUND;
}

double rpm2radpersec(double _rpm){
    return _rpm *2*PI/60.0 ;
}

double radpersec2rpm(double _radpersec){
    return _radpersec *60.0/(2*PI) ;
}


integrate::integrate(double _deltaT){
    deltaT = _deltaT;
    a_1 = 1.0;
    b_0 = (deltaT/2.0);
    b_1 = (deltaT/2.0);

    x_1 = 0.0;
    y_1 = 0.0;
}

integrate::integrate(double _a1, double _b0, double _b1){
    a_1 = _a1;
    b_0 = _b0;
    b_1 = _b1;

    x_1 = 0.0;
    y_1 = 0.0;
}

double integrate::calc_integral(double input){
    double output;
    output = (a_1*y_1) + (b_0*input) + (b_1*x_1);
    y_1 = output;
    x_1 = input;

    return output;
}

LowPassFilter::LowPassFilter(double deltaT, double riseTime){
    this->dt = deltaT;
    this->a = riseTime;

    this->prev_input = 0.0;
    this->prev_output = 0.0;
}

LowPassFilter::~LowPassFilter(){}

double LowPassFilter::filter(double input){
    double output = (this->dt/(this->dt+2*this->a))*input + (this->dt/(this->dt+2*this->a))*this->prev_input - ((this->dt-2*this->a)/(this->dt+2*this->a))*this->prev_output;

    this->prev_output = output;
    this->prev_input  = input;

    return output;
}

PIDController::PIDController( double _dt){
  this->dt = _dt;

  this->pgain = this->igain = this->dgain = 0.0;
  this->integrate = 0.0;
  this->prev_error = 0.0;
  this->prev_diff = 0.0;
}

PIDController::~PIDController(){}

void PIDController::setGain( double _Pgain, double _Igain, double _Dgain){
  this->pgain = _Pgain;
  this->igain = _Igain;
  this->dgain = _Dgain;
}

double PIDController::compute( double error){
  this->integrate += (error + this->prev_error)/2.0 * this->dt;

  double diff = error*2.0/(5.0*this->dt) - this->prev_error*2.0/(5.0*this->dt) + this->prev_diff*3.0/5.0;

  double ans = error*this->pgain + this->integrate*this->igain + diff*this->dgain;

  this->prev_diff = diff;
  this->prev_error = error;

  return ans;
}

double sigmoid( double gain, double x){
    return 1.0/(1.0 + pow(MATH_E, -gain*x));
}
