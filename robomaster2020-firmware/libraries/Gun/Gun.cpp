#include <Arduino.h>
#include <Servo.h>
#include <Gun.h>


//射撃ローラー制御クラス
Gun::Gun(void){

}

//********************************************************************************
//　初期化
//********************************************************************************
void Gun::init(void){
  escGunLoader.attach(pinGunLoader);
  escGunLoader.writeMicroseconds(signalStopDcMotor);
  escGunUpper.attach(gunEscUpper);
  escGunLower.attach(gunEscLower);
  setFiringSpeed(0.0f, 0.0f); //(float spdUpper, float spdLower)
}

void Gun::init( int _gunEscUpper,int _gunEscLower){
  // escGunLoader.attach(_pinGunLoader);
  // escGunLoader.writeMicroseconds(signalStopDcMotor);
  escGunUpper.attach(_gunEscUpper);
  escGunLower.attach(_gunEscLower);
  setFiringSpeed(0.0f, 0.0f); //(float spdUpper, float spdLower)
}

//********************************************************************************
//　射撃制御メイン
//********************************************************************************
void Gun::doGunTask() {
  //緊急停止(脱力) の場合、射撃モーター停止
  if (flgGunActive == false) {
    flgLoadMotor = false;
    flgFireMotor = false;
    // flgKeepLoadBullet = false;
    setLoadingSpeed(LOAD_CW, 0);
    stopLoader();
    setFiringSpeed(0.0f, 0.0f); //(float spdUpper, float spdLower)
    return;
  }

  //装填モーターを回転するかどうか判定
  if ( flgFireMotor == false ) { //射撃モーターが回っているとき
    // flgKeepLoadBullet = false;
    flgLoadMotor = false;
  } else {
    gainLoadSpeed = 0.7;
  }

  //射撃用ESCを回す。
  if ( flgFireMotor == false) {
    setFiringSpeed(0.0f, 0.0f); //(float spdUpper, float spdLower)
  } else {
    int val;    //可変抵抗器の値を取得して射撃用ESCの回転速度にする
    if(fixedValue == -1)
      val = analogRead(pinADBulletSpeed);
    else
      val = fixedValue;
    float velocity = map(val, 0, 1023, 0, maxSpeed);
    if (velocity < 10.0f) {
      velocity = 10.0f;
    }
    setFiringSpeed(velocity - diffSpeed,velocity); //(float spdUpper, float spdLower)
  }

  //装填用DCモーターを回す。
  if ( flgLoadMotor == false ) {
    stopLoader();
  } else {
    startLoader();
    if ( (millis() % 2000) < 100) {   //2秒間あたり100msのみ逆回転して弾詰まり防止する
      setLoadingSpeed(LOAD_CW, rotSpeedLoadMotor);
    } else {
      setLoadingSpeed(LOAD_CCW, rotSpeedLoadMotor);
    }
  }
}

//********************************************************************************
//　装填用ESCの動作停止
//********************************************************************************
void Gun::stopLoader(void){
  escGunLoader.writeMicroseconds(signalStopDcMotor + 0);
  escGunLoader.detach();    //射出装填一旦止める
}

//********************************************************************************
//　装填用ESCの動作開始
//********************************************************************************
void Gun::startLoader(void){
  escGunLoader.attach(pinGunLoader);
  escGunLoader.writeMicroseconds(signalStopDcMotor);
}

//********************************************************************************
//　射出機構動作状態(true:通常、false:緊急停止)
//********************************************************************************
void Gun::setActive(bool state){
  flgGunActive = state;
}

//********************************************************************************
//　装填(ロード)モーター制御(true:回転、false:停止)
//********************************************************************************
void Gun::driveLoader(bool state){
  flgLoadMotor = state;
}

//********************************************************************************
//　射撃(ブラシレス)モーター制御(true:回転、false:停止)
//********************************************************************************
void Gun::driveFire(bool state){
  flgFireMotor = state;
}

//********************************************************************************
//　射出スピードの設定
//********************************************************************************
void Gun::setFiringSpeed(float spdUpper, float spdLower){
  escGunUpper.writeMicroseconds(convertBulletSpeedToPwmSignalUs(spdUpper));
  escGunLower.writeMicroseconds(convertBulletSpeedToPwmSignalUs(spdLower));
}

//********************************************************************************
//　装填モーター制御
//　mode  : 回転方向 正回転:true 逆回転:false
//　speed   : 回転量(0～255)
//********************************************************************************
void Gun::setLoadingSpeed(bool mode, int speed) {
  if (speed > 800) {
    speed = 800;
  }
  if (speed < 0) {
    speed = 0;
  }

  //回転方向
  if ( mode == LOAD_CW ) {
    escGunLoader.writeMicroseconds(signalStopDcMotor + speed);  //正転
  } else {
    escGunLoader.writeMicroseconds(signalStopDcMotor - speed);  //逆転
  }
}
//********************************************************************************
//　射出速度[単位：m/s]からESCのパルス幅[us]を算出する関数
//********************************************************************************
int Gun::convertBulletSpeedToPwmSignalUs(float bulletSpeed_mps) {
  int pwmSignal_us = 1000;
  if (bulletSpeed_mps == 0) {
    pwmSignal_us = 1000;
  } else if (bulletSpeed_mps < 10.0f) {
    pwmSignal_us = 1000;
  } else if (bulletSpeed_mps <= maxSpeed) {
    pwmSignal_us = 14.423f * bulletSpeed_mps + 1104;
  } else {
    pwmSignal_us = 1000;
  }
  return pwmSignal_us;
}

//********************************************************************************
// 上下の射撃ローラー回転数差設定(m/s)
//********************************************************************************
void Gun::setBackspin(float diffSpeedInput){
  diffSpeed = diffSpeedInput;
}

//********************************************************************************
// 射撃ローラー射出速度設定(m/s)
//********************************************************************************
void Gun::setMaxSpeed(float maxSpeedInput){
  maxSpeed = maxSpeedInput;
}

//********************************************************************************
// 上下の射撃ローラー射出速度設定 (可変抵抗の代わり: 0 ~ 1023)
//********************************************************************************
void Gun::setFixedValue(int inputValue){
  fixedValue = inputValue;
}