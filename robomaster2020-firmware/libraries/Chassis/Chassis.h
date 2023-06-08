// #pragma once
#ifndef Chassis_h
#define Chassis_h
#include <Arduino.h>
#include <mymath.h>

//クラスの定義
//クラス名・コンストラクタ名・関数名や使用する変数名を定義します。
class Chassis{
    public:
        Chassis();
        // void doChassisTask(void);
        void doChassisTask( double _veloX, double _veloY, double _omega, double _theta, double outputs[4]);
        void setRobotVector(double x,double y,double r);

        void calcTwistOffset(double generalGimbalYaw);
        // double getTargetBodyAngle(RecieverToggle toggle, double nowYaw);

        // void setSensRotation(int16_t* rotation);
        double pidChassisRotation(double error);
        // void setYawNeutral(double yawNeutral);
        // //シャーシモーターの電流値
        // double wheelAmpere[4];
        // //シャーシモーターの角度
        // int16_t sensRotation[4];
        // void setPidChassisGain(double pGain, double iGain, double dGain);
        // void setPidWheelGain(double pGain, double iGain, double dGain);
        // void setDodgeParam(double width, double cycle, double rotatespeed);
        // void limitTotalAmpere(double maximumAmpere);
        // void calcTwist(double rotation);
    private:
        void convRobotVector2MotorRotation(double x, double y, double r, double* rotation);
        // void pidWheelRotation(double* error, double* u);
        double veloX, veloY, angVelo;
        double velox_bodylink, veloy_bodylink;
        // double yawNeutral = 0;

        // int timerInterruptInterval = 1; //[ms]

        // double pidGainChassis[3] = {12, 0.0, 0.0};
        // double pidGainWheel[3] = {1.0, 0.0, 0.0};

        // double dodgeWidth = 400;
        // double dodgeCycle = 1.5f; //回避周期[s]
        // double infiniteRotateSpeed = 500;

};
#endif