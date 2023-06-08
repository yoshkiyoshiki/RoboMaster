#include <Arduino.h>
#include <GM6020.h>

#include "mymath.h"


GM6020::GM6020( double _deltaT, int16_t _neutral){
    this->setNeutral(_neutral);
    this->deltaT = _deltaT;
    this->plantModel = 0.0;
    this->setPIDgain_mec( 3.0, 0.0, 0.0);
    this->setPIDgain_speed( 3.0, 0.0, 0.0);
    this->setPIDgain_position( 3.0, 0.0, 0.0);

    this->prev_error_mec = 0.0;

    this->generalPosition = -1;
    this->lastPosition = -1;
}
GM6020::~GM6020(){}

void GM6020::setPosition(int16_t position){
    Motor::setPosition( position);

    if (this->lastPosition != -1) {
        int diff;
        diff = position - this->lastPosition;
        if (diff < -(ENCODER_ONE_AROUND / 2)){
          diff += ENCODER_ONE_AROUND;
        }else if ((ENCODER_ONE_AROUND / 2) < diff){
          diff -= ENCODER_ONE_AROUND;
        }
        this->generalPosition += diff;

        // while(this->generalPosition <= -(ENCODER_ONE_AROUND*3 / 4) || (ENCODER_ONE_AROUND*3 / 4) < this->generalPosition){
        //   if(this->generalPosition <= -(ENCODER_ONE_AROUND*3 / 4))
        //     this->generalPosition += ENCODER_ONE_AROUND;
        //   else if((ENCODER_ONE_AROUND*3 / 4) < this->generalPosition)
        //     this->generalPosition -= ENCODER_ONE_AROUND;
        // }
      } else {      //初回のみ実行
        this->generalPosition = position - this->neutral;
        if (this->generalPosition > (ENCODER_ONE_AROUND / 2)) {
          this->generalPosition -= ENCODER_ONE_AROUND;
        }else if(this->generalPosition < -(ENCODER_ONE_AROUND / 2)){
          this->generalPosition += ENCODER_ONE_AROUND;
        }
      }
    this->lastPosition = position;
}

void GM6020::setNeutral(int16_t _neutral){
    neutral = _neutral;
}

int GM6020::getGeneralPosition(){
    return this->generalPosition;
}

double GM6020::Pm( double wantAmpare){

  this->plantModel = (wantAmpare*(gain*this->deltaT/(this->deltaT+Tc*2)) + this->prev_ampare_model*(gain*this->deltaT/(this->deltaT+Tc*2)) - this->prev_output_model*((this->deltaT-Tc*2)/(this->deltaT+Tc*2)))/60.0*100.0;

  this->prev_ampare_model = wantAmpare;
  this->prev_output_model = this->plantModel;

  return this->plantModel;
}

int GM6020::getGeneralPosition_limited(){
    // return sign(this->generalPosition)*(ABS(this->generalPosition)%8192);
    int16_t diff = sign(this->generalPosition)*(ABS(this->generalPosition)%8192);
    if(diff < -ENCODER_ONE_AROUND/2)
        diff += ENCODER_ONE_AROUND;
    else if(diff > ENCODER_ONE_AROUND/2)
        diff -= ENCODER_ONE_AROUND;
    return diff;
}

double GM6020::Compensator( double error){
    // double Kp = this->Kp_mec;
    // double Kd = this->Kd_mec;

    double output = this->Kp_mec*error + this->Kd_mec*(error - this->prev_error_mec) ;

    // prev_output = output;
    this->prev_error_mec = error;

    return output;
}

// 指令値をP制御
double GM6020::PIDControl( double error){
  this->error_Integral_speed = (error + this->prev_error_speed)/2 * this->deltaT;

  double output = error*Kp_speed + this->error_Integral_speed*Ki_speed + (error - this->prev_error_speed)*Kd_speed;

  this->prev_error_speed = error;

  return output;
}

int16_t GM6020::culcAmpareFromRPM( double targetRPM){

    // double filteredRPM = this->inputDelay(targetRPM);
    double filteredRPM = targetRPM;
    double needRPM = filteredRPM + PIDControl( filteredRPM - this->getRPM());

    // Convert rpm to Need Ampare(-16384 ~ 16384)
    double wantAmpare = needRPM * this->rpm2ampare;

    // Cululate RPM using Plant Model, if you input wantAmpare.
    double modelValue = this->Pm( wantAmpare);

    // Here is using MEC.
    double inputAmpare = wantAmpare + this->Compensator( modelValue - this->getRPM());

    // Output limit.
    // return (int16_t) constrain(inputAmpare, -16384, 16384);
    return (int16_t) constrain(inputAmpare, -30000, 30000);
}

int16_t GM6020::culcAmpareFromRPM_4Gimbal( double targetRPM, double gyro){

    // double filteredRPM = this->inputDelay(targetRPM);
    double filteredRPM = targetRPM;
    // double needRPM = filteredRPM + PIDControl( filteredRPM - this->getRPM());
    double needRPM = filteredRPM + PIDControl( filteredRPM + radpersec2rpm(gyro));

    // Convert rpm to Need Ampare(-16384 ~ 16384)
    double wantAmpare = needRPM * this->rpm2ampare;

    // Cululate RPM using Plant Model, if you input wantAmpare.
    double modelValue = this->Pm( wantAmpare);

    // Here is using MEC.
    double inputAmpare = wantAmpare + this->Compensator( modelValue - radpersec2rpm(gyro));
    // double inputAmpare = wantAmpare;

    // Serial.print( targetRPM);
    // Seial.print(" ");
    // Serial.print( needRPM);
    // Serial.print(" ");

    // Output limit.
    // return (int16_t) constrain(inputAmpare, -16384, 16384);
    return (int16_t) constrain(inputAmpare, -30000, 30000);
}


double GM6020::getPlantModel(){
  return this->plantModel;
}


int16_t GM6020::culcAmpareFromPosition( double targetPosition){
    double  errorPosition = (targetPosition - getGeneralPosition());

    this->error_Integral_position += (errorPosition + this->prev_error_position)/2.0 * this->deltaT;

    double targetRPM = errorPosition*Kp_position + this->error_Integral_position*Ki_position + (errorPosition - this->prev_error_position)*this->Kd_position;

    this->prev_error_position = errorPosition;

    return this->culcAmpareFromRPM(targetRPM);
}

int16_t GM6020::culcAmpareFromPosition_4Gimbal( double targetPosition, double position, double gyro){
    double  errorPosition = (targetPosition - position);

    this->error_Integral_position += (errorPosition + this->prev_error_position)/2.0 * this->deltaT;

    double targetRPM = errorPosition*Kp_position + this->error_Integral_position*Ki_position + (errorPosition - this->prev_error_position)*this->Kd_position;

    this->prev_error_position = errorPosition;

    // Serial.print( errorPosition);
    // Serial.print(" ");
    // Serial.print( targetRPM);
    // Serial.print(" ");

    return this->culcAmpareFromRPM_4Gimbal(targetRPM, gyro);
}

void GM6020::setPIDgain_mec( double _kp, double _ki, double _kd){
  Kp_mec = _kp;
  Ki_mec = _ki;
  Kd_mec = _kd;
}

void GM6020::setPIDgain_speed( double _kp, double _ki, double _kd){
  Kp_speed = _kp;
  Ki_speed = _ki;
  Kd_speed = _kd;
}

void GM6020::setPIDgain_position( double _kp, double _ki, double _kd){
  Kp_position = _kp;
  Ki_position = _ki;
  Kd_position = _kd;
}

void GM6020::setModelParams( double _gain, double _Tc){
  this->gain = _gain;
  this->Tc = _Tc;

  this->rpm2ampare = 1.0/this->gain;
}