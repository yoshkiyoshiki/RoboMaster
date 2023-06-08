// #pragma once
#ifndef Receiver_h
#define Receiver_h
#include <Arduino.h>

//クラスの定義
//クラス名・コンストラクタ名・関数名や使用する変数名を定義します。
class Receiver{
    public:
        Receiver();
        void init(void);
        void getRecvData(void);
        void debugPrint(void);

        bool flgRecvSuccess = false;
        long propOffset[4];
        const long JOY_ABS_MAX = 660;

        typedef struct{
            long rightX = 0;
            long rightY = 0;
            long leftX = 0;
            long leftY = 0;
        } Joystick;
        Joystick joy;

        typedef struct{
            long left = 1;
            long right = 1;
        } ToggleSwitch;
        ToggleSwitch toggle;

        typedef struct{
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t pressLeft;
            uint8_t pressRight;
        } MouseUI;
        MouseUI mouse;
        long keyData;
        typedef struct{
            bool W;
            bool S;
            bool A;
            bool D;
            bool Shift;
            bool Ctrl;
            bool Q;
            bool E;
            bool R;
            bool F;
            bool G;
            bool Z;
            bool X;
            bool C;
            bool V;
            bool B;
        } KeyboardUI;
        KeyboardUI key;
        void setPinNoBuzzer(int number);

    private:
        void normalizePropData(void);
        void applyKeyboardData(void);
        void checkPropSafety(void);
        long joyRightX = 1024;
        long joyRightY = 1024;
        long joyLeftX = 1024;
        long joyLeftY = 1024;
        long toggleL = 2;   //手前：2、真ん中：3、奥：1
        long toggleR = 2;   //手前：2、真ん中：3、奥：1
        const int DJI_RECV_LOW  = 364;
        const int DJI_RECV_HIGH = 1684;
        const int DJI_RECV_CENTER = 1024;
        int pinBuzzer = 5;//23;    // D5 (PWM) 圧電ブザー
};
#endif