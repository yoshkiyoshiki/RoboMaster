#include <Metro.h>
#include <MsTimer2.h>
#include "mymath.h"
#include "GM6020.h"
#include <CANManager.h>


// 制御周期を決める。単位はミリ秒[ms]
#define dt 4
CANManager* canmanager = CANManager::getInstance();//返されたアドレスを*を使用して実際にアクセスしてる。
Metro ControlCycle = Metro(dt); // 制御周期計算間隔(ms)
Metro debugTiming = Metro(10);  // デバッグの情報をprint

// 制御周期を入れる。単位は秒なので、0.001を掛ける。
GM6020 motor( dt*0.001, 1000);
#define MOTOR_ID 1

void setup() {
  canmanager->init(true,true,true);
  canmanager->restartCan();
  Serial.begin(115200);  //シリアルモニタ(115,200)

  motor.setPIDgain_mec( 20.0, 0.0, 0.0);
  motor.setPIDgain_speed( 4.0, 0.0, -1.0);

  delay(100);
}

void loop() {
  // CANのデータを受信
  canmanager->getAllCANdata();
  getGM6020Datas();

    // 目標RPMを指定する
    double targetRPM = 100;

  // 制御周期の時に処理を行う
  if( ControlCycle.check()){
    // 目標RPMになるように指令値を計算する。
    int16_t ampare = motor.culcAmpareFromRPM( targetRPM);

    // IDが1のESCに対して、計算した指令値を送信
    canmanager->setCAN1C610620Ampere( MOTOR_ID, ampare);

    canmanager->sendAllCANdata();
  }

  if( debugTiming.check()){
    // 指定したRPMと実際のRPMをシリアルプリントして確認する
    Serial.print( targetRPM);
    Serial.print( " ");
    Serial.print( motor.getRPM());
    Serial.print( " ");

    Serial.println("");
  }

}

void getGM6020Datas(){
  if(canmanager->isNoCan1())
    return;

  uint8_t data[8];
  // ESCのIDが1の時、CANのIDは0x204+1=0x205
  canmanager->getCan1Data(0x204 + MOTOR_ID,data);

  motor.setPosition( (data[0]<<8) + data[1]);
  motor.setRPM( (data[2]<<8) + data[3]);
  motor.setCurrent( (data[4]<<8) + data[5]);
  motor.setTemperature( data[6]);

}
