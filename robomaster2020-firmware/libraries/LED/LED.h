// #pragma once
#ifndef LED_h
#define LED_h
#include <Arduino.h>

//クラスの定義
//クラス名・コンストラクタ名・関数名や使用する変数名を定義します。
class LED{
    public:
        LED();
        void init(void);
        void blue(boolean state);
        void red(boolean state);
        void white(boolean state);
    private:   
        const int pinLED2 = 11;     // D11 LED2
        const int pinLED3 = 12;     // D12 LED3
        const int pinLED4 = 13;     // D13 LED4(with マイコンLED)
};
#endif