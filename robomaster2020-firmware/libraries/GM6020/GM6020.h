// #pragma once
#ifndef __GM6020__
#define __GM6020__
#include <Arduino.h>
#include "Motor.h"

class GM6020: public Motor
{
private:
    double rpm2ampare = 10000.0/140.0;// convert Value of rpm to Ampare

    // GM6020の数理モデル
    // 入力：電圧値(-30000 ~ 30000),
    // 出力: RPM
    //         0.014
    // P = ------------
    //      0.005s + 1
    double gain = 0.014;
    double Tc = 0.005;

    double plantModel; // RPM Value of Plant Model
    double deltaT; // Control cycle [sec]
    int16_t neutral;

    int16_t lastPosition;
    int generalPosition;


    double prev_ampare_model;
    double prev_output_model;

    double prev_error_mec;
    double Kp_mec;
    double Ki_mec;
    double Kd_mec;

    double prev_error_position;
    double error_Integral_position;
    double Kp_position;
    double Ki_position;
    double Kd_position;

    double prev_error_speed;
    double error_Integral_speed;
    double Kp_speed;
    double Ki_speed;
    double Kd_speed;
    // double prev_input_UsinginputDelay;
    // double prev_output_UsinginputDelay;

    // Plant Model for MEC (Model Error Compensator)
    double Pm( double wantAmpare);
    // Compensator for MEC (Model Error Compensator)
    // you should adjust PD Controller
    double Compensator( double error);
    // // 指令値を一次遅れにする
    // double inputDelay(double finalRPM);
    // 指令値をPID制御
    double PIDControl( double error);

public:
    GM6020( double _deltaT, int16_t _neutral);
    ~GM6020();
    void setPosition(int16_t position) override;
    void setNeutral(int16_t _neutral);
    void setPIDgain_mec( double _kp, double _ki, double _kd);
    void setPIDgain_speed( double _kp, double _ki, double _kd);
    void setPIDgain_position( double _kp, double _ki, double _kd);
    void setModelParams( double _gain, double _Tc);

    int getGeneralPosition();
    int getGeneralPosition_limited();

    // culcuate ampare value(-16384 ~ 16384) from RPM
    int16_t culcAmpareFromRPM( double targetRPM);
    int16_t culcAmpareFromRPM_4Gimbal( double targetRPM, double gyro);
    int16_t culcAmpareFromPosition( double targetPosition);
    int16_t culcAmpareFromPosition_4Gimbal( double targetPosition, double position, double gyro);

    double getPlantModel();
};

#endif