#include <Arduino.h>
#include <LED.h>

//初期化処理を行う関数
//コンストラクタは、　クラス名::クラス名と同じ名前で構成します
LED::LED(void){

}

void LED::init(){
  //Blue
  pinMode(pinLED2, OUTPUT);
  digitalWrite(pinLED2, LOW);
  //Red
  pinMode(pinLED3, OUTPUT);
  digitalWrite(pinLED3, LOW);
  //White
  pinMode(pinLED4, OUTPUT);
  digitalWrite(pinLED4, LOW);
}

void LED::blue(boolean state) {
  digitalWrite(pinLED2,state);
}

void LED::red(boolean state) {
  digitalWrite(pinLED3,state);
}

void LED::white(boolean state) {
  digitalWrite(pinLED4,state);
}