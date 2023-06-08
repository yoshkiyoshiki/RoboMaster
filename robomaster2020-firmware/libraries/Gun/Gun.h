// #pragma once
#ifndef Gun_h
#define Gun_h
#include <Arduino.h>
#include <Servo.h>

#define LOAD_CW true
#define LOAD_CCW false


//射撃ローラー制御クラス
class Gun{
    public:
        Gun();
        void init(void);
        void init( int , int);
        void stopLoader(void);
        void startLoader(void);
        void setActive(bool state);
        void driveLoader(bool state);
        void driveFire(bool state);
        void doGunTask();
        void setBackspin(float diffSpeedInput);

        uint16_t marginHeat = 30;
        float gainLoadSpeed = 0.5;

        //キーボードでの射撃モーター操作
        //射撃モーター操作キー（Eキー)の状態定数
        const int STATE_DEFAULT = 0;        //初期状態
        const int STATE_START_KEYDOWN = 1;  //モーター回転開始　KeyDown
        const int STATE_START_KEYUP   = 2;  //モーター回転開始　KeyUp
        const int STATE_STOP_KEYDOWN  = 3;  //モーター回転停止　KeyDown
        int stateFireKeyPress = STATE_DEFAULT;
        uint16_t beforeHeat = 0;
        boolean flgKeepLoadBullet = false;

        void setMaxSpeed(float maxSpeedInput);
        void setFixedValue(int);

    protected:
        void setLoadingSpeed(boolean mode, int speed);
        void setFiringSpeed(float spdUpper, float spdLower);
        int convertBulletSpeedToPwmSignalUs(float bulletSpeed_mps);

        bool flgGunActive = false;
        bool flgFireMotor = false;     //射撃モーター回転中フラグ: trueなら回転中
        bool flgLoadMotor = false;         //装填モーター回転フラグ trueで回転する。

        const int pinGunLoader = 21;// D21/A7 PWM3
        const int gunEscUpper = 22; // D22/A8 PWM2 射撃部のESC(上)
        const int gunEscLower = 23;  // D23/A9 PWM1 射撃部のESC(下)

        const int pinADBulletSpeed = A2; // D16/A2 射出速度調整用ボリューム

        int rotSpeedLoadMotor = 800;        //装填モーター回転速度
        const int reverseRotSpeedLoadMotor = 800;
        const int signalStopDcMotor = 1925;
        //const int signalStopDcMotor = 1500;

        float diffSpeed = 0.0f;
        float maxSpeed = 30.0f;

        Servo escGunLoader;                  //装填用ESC
        Servo escGunUpper;                    //射撃用ESC(左)
        Servo escGunLower;                   //射撃用ESC(右)

        int fixedValue = -1;    //射出用モーターの回転速度を固定値で指定する
};
#endif