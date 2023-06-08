// #pragma once
#ifndef __M2006__
#define __M2006__
#include <Arduino.h>
#include "Motor.h"

class M2006_SPEEDCONTROL: public Motor
{
// private:
protected:
    const double rpm2ampare = 0.29411764705882354;// convert Value of rpm to Ampare

    double plantModel; // RPM Value of Plant Model
    double deltaT; // Control cycle [sec]

    double prev_ampare;
    double prev_output;

    double prev_input_UsinginputDelay;
    double prev_output_UsinginputDelay;

    double prev_error_mec;
    double Kp_mec;
    double Ki_mec;
    double Kd_mec;

    double prev_error_speed;
    double error_Integral_speed;
    double Kp_speed;
    double Ki_speed;
    double Kd_speed;

    // Plant Model for MEC (Model Error Compensator)
    double Pm( double wantAmpare);
    // Compensator for MEC (Model Error Compensator)
    // you should adjust PD Controller
    double Compensator( double error);
    // 指令値を一次遅れにする
    double inputDelay(double finalRPM);
    // 指令値をPID制御
    double PIDControl( double error);

public:
    M2006_SPEEDCONTROL( double _deltaT);
    ~M2006_SPEEDCONTROL ();

    // culcuate ampare value(-16384 ~ 16384) from RPM
    int16_t culcAmpareFromRPM( double targetRPM);
    void setPIDgain_mec( double _kp, double _ki, double _kd);
    void setPIDgain_speed( double _kp, double _ki, double _kd);

    double getPlantModel();
};

class M2006_POSITIONCONTROL: public M2006_SPEEDCONTROL
{
private:
    int16_t neutral;

    int16_t lastPosition;
    int generalPosition;

    double prev_error_position;
    double error_Integral_position;
    double Kp_position;
    double Ki_position;
    double Kd_position;
public:
    M2006_POSITIONCONTROL( double _deltaT, int16_t _neutral);
    ~M2006_POSITIONCONTROL();

    void setPosition(int16_t position) override;
    void setNeutral(int16_t _neutral);

    void setPIDgain_position( double _kp, double _ki, double _kd);
    int getGeneralPosition();
    int16_t culcAmpareFromPosition( double targetPosition);
};




#endif