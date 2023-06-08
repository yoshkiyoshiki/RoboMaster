#include <Metro.h>
#include <MsTimer2.h>
#include "mymath.h"
#include "M3508.h"
#include "M2006.h"
#include <CANManager.h>


// 制御周期を決める。単位はミリ秒[ms]
#define dt 4
CANManager* canmanager = CANManager::getInstance();//返されたアドレスを*を使用して実際にアクセスしてる。
Metro ControlCycle = Metro(dt); // 制御周期計算間隔(ms)
Metro debugTiming = Metro(10);  // デバッグの情報をprint

// 制御周期を入れる。単位は秒なので、0.001を掛ける。
// M3508_POSITIONCONTROL motor( dt*0.001, 1000);
M2006_POSITIONCONTROL motor( dt*0.001, 1000);
#define MOTOR_ID 8

void setup() {
  canmanager->init(true,true,true);
  canmanager->restartCan();
  Serial.begin(115200);  //シリアルモニタ(115,200)

  motor.setPIDgain_mec( 6.0, 0.0, 0.0);
  motor.setPIDgain_speed( 1.5, 0.0, 0.0);
  // motor.setPIDgain_position( 4.0, 0.0, 0.0);

  delay(5000);
  ControlCycle.reset();

  while(true){
    canmanager->getAllCANdata();
    getM3508Datas();
    if(ControlCycle.check()){
      canmanager->setCAN2C610620Ampere( MOTOR_ID, motor.culcAmpareFromRPM(-MOVE_CONSTANT));
      canmanager->sendAllCANdata();
      Serial.print( motor.getRPM());
      Serial.print(" ");
      // Serial.print( motor.detectInitialPosition());
      // Serial.print(" ");

      if( motor.detectInitialPosition(false) == 0)
        break;

      Serial.println(" ");
    }
  }
  motor.setNeutral( motor.getPosition());

  delay(100);
}

void loop() {
  // CANのデータを受信
  canmanager->getAllCANdata();
  getM3508Datas();

    // 目標RPMを指定する
    double targetPosition = 0;

  // 制御周期の時に処理を行う
  if( ControlCycle.check()){
    // 目標RPMになるように指令値を計算する。
    int16_t ampare = motor.culcAmpareFromPosition( targetPosition);

    // IDが1のESCに対して、計算した指令値を送信
    canmanager->setCAN2C610620Ampere( MOTOR_ID, ampare);

    canmanager->sendAllCANdata();
  }

  if( debugTiming.check()){
    // 指定したRPMと実際のRPMをシリアルプリントして確認する
    Serial.print( targetPosition);
    Serial.print( " ");
    Serial.print( motor.getGeneralPosition());
    Serial.print( " ");

    Serial.println("");
  }

}

void getM3508Datas(){
  if(canmanager->isNoCan2())
    return;

  uint8_t data[8];
  // ESCのIDが1の時、CANのIDは0x200+1=0x201
  canmanager->getCan2Data(0x200 + MOTOR_ID,data);

  motor.setPosition( (data[0]<<8) + data[1]);
  motor.setRPM( (data[2]<<8) + data[3]);
  motor.setCurrent( (data[4]<<8) + data[5]);
  motor.setTemperature( data[6]);

}
