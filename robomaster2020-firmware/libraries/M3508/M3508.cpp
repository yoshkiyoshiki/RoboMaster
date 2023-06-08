#include <Arduino.h>
#include <M3508.h>

#include "mymath.h"

M3508_SPEEDCONTROL::M3508_SPEEDCONTROL( double _deltaT){
    this->deltaT = _deltaT;
    this->plantModel = 0.0;
    this->setPIDgain_mec( 6.5, 0.0, 0.0);
    this->setPIDgain_speed( 1.0, 0.0, 0.0);

    this->prev_ampare = 0.0;
    this->prev_output = 0.0;

    this->prev_error_mec = 0.0;

    this->prev_input_UsinginputDelay = 0.0;
    this->prev_output_UsinginputDelay = 0.0;
}
M3508_SPEEDCONTROL::~M3508_SPEEDCONTROL(){}

double M3508_SPEEDCONTROL::Pm( double wantAmpare){
    // Here is Plant Model which I wish.
    this->plantModel = 0.085 * wantAmpare + 0.085 * this->prev_ampare + 0.95 * this->prev_output;

    this->prev_ampare = wantAmpare;
    this->prev_output = this->plantModel;

    return this->plantModel;
}

double M3508_SPEEDCONTROL::Compensator( double error){

    double output = Kp_mec*error + Kd_mec*(error - this->prev_error_mec);

    // prev_output = output;
    this->prev_error_mec = error;

    return output;
}

// 指令値を一次遅れにする
double M3508_SPEEDCONTROL::inputDelay(double input){
    double output = 0.04761904761904762*input + 0.04761904761904762*this->prev_input_UsinginputDelay + 0.9047619047619048*this->prev_output_UsinginputDelay;

    this->prev_input_UsinginputDelay = input;
    this->prev_output_UsinginputDelay = output;

    return output;
}

// 指令値をP制御
double M3508_SPEEDCONTROL::PIDControl( double error){

    this->error_Integral_speed = (error + this->prev_error_speed)/2 * this->deltaT;

    double output = error*Kp_speed + this->error_Integral_speed*Ki_speed + (error - this->prev_error_speed)*Kd_speed;
    this->prev_error_speed = error;

    return output;
}

int16_t M3508_SPEEDCONTROL::culcAmpareFromRPM( double targetRPM){

    double filteredRPM = this->inputDelay(targetRPM);
    double needRPM = filteredRPM + PIDControl( filteredRPM - this->getRPM());

    // Convert rpm to Need Ampare(-16384 ~ 16384)
    double wantAmpare = needRPM * this->rpm2ampare;

    // Cululate RPM using Plant Model, if you input wantAmpare.
    double modelValue = this->Pm( wantAmpare);

    // Here is using MEC.
    double inputAmpare = wantAmpare + this->Compensator( modelValue - this->getRPM());

    // Output limit.
    return (int16_t) constrain(inputAmpare, -16384, 16384);
}

double M3508_SPEEDCONTROL::getPlantModel(){
    return this->plantModel;
}

void M3508_SPEEDCONTROL::setPIDgain_mec( double _kp, double _ki, double _kd){
  Kp_mec = _kp;
  Ki_mec = _ki;
  Kd_mec = _kd;
}

void M3508_SPEEDCONTROL::setPIDgain_speed( double _kp, double _ki, double _kd){
  Kp_speed = _kp;
  Ki_speed = _ki;
  Kd_speed = _kd;
}


/******************/
/* ここから位置制御 */
/******************/

M3508_POSITIONCONTROL::M3508_POSITIONCONTROL( double _deltaT, int16_t _neutral):M3508_SPEEDCONTROL( _deltaT){

    this->deltaT = _deltaT;
    this->setNeutral(_neutral);
    this->setPIDgain_position(0.2, 0.0, 0.0);

    this->generalPosition = -1;
    this->lastPosition = -1;

}

M3508_POSITIONCONTROL::~M3508_POSITIONCONTROL(){

}

void M3508_POSITIONCONTROL::setPosition(int16_t position){
    M3508_SPEEDCONTROL::setPosition( position);

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
        }else if(this->generalPosition < (-ENCODER_ONE_AROUND / 2)){
          this->generalPosition += ENCODER_ONE_AROUND;
        }
      }
    this->lastPosition = position;
}


void M3508_POSITIONCONTROL::setNeutral(int16_t _neutral){
    this->lastPosition = -1;
    neutral = _neutral;
}

int M3508_POSITIONCONTROL::getGeneralPosition(){
    return this->generalPosition;
}

// 入力：目標角度[rad]
int16_t M3508_POSITIONCONTROL::culcAmpareFromPosition( double targetPosition){
    double  errorPosition = (targetPosition - getGeneralPosition());

    this->error_Integral_position += (errorPosition + this->prev_error_position)/2.0 * this->deltaT;
    double targetRPM = errorPosition*Kp_position + this->error_Integral_position*Ki_position + (errorPosition - this->prev_error_position)*this->Kd_position;
    this->prev_error_position = errorPosition;

    return M3508_SPEEDCONTROL::culcAmpareFromRPM(targetRPM);
}

void M3508_POSITIONCONTROL::setPIDgain_position( double _kp, double _ki, double _kd){
  Kp_position = _kp;
  Ki_position = _ki;
  Kd_position = _kd;
}
