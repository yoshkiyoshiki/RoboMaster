# CANManager Readme
author : Shiori Tanaka \
last update: 2020/1/11

## CANManagerの概要
Teensy4.0で行われる全てのCAN通信の送受信を管理する\
全てのCAN通信はこのライブラリを通して行われる。\
そのためこのライブラリはシングルトンである。


## 現在の状況
 - 受信側の実装　（未テスト)
 - リセット処理　（未テスト)

## 次にやること
 - 送信側の実装
 - 実機での検証

## 各関数の説明(public関数のみ)
```c++
static CANManager *getInstance()
```
インスタンスのアドレスを返す関数。\
CANManagerにアクセスする時に唯一インスタンスにアクセスすることができる関数。基本的に他のクラスでこれを呼び出してアロー演算子を使用してこのクラスのメンバ関数にアクセスする。
***

```c++
void init(bool canbus1,bool canbus2,bool canbus3)
```
初期化関数 \
引数はそれぞれのcanbusを使用するかどうかをtrue/falseで指定する。
falseに指定したcanbusは使用されない。
***

```c++
bool isinit()
```
初期化が完了しているかどうかを返す関数。\
初期化が完了している場合trueを返す。
***

```c++
bool isNoCan1()
bool isNoCan2()
bool isNoCan3()
```
それぞれのCANが途絶しているかどうかを返す関数。\
CANが途絶していた場合trueを返す
***
```c++
void EmergencyStop()
```
緊急停止するための関数
can送信のデータを全て0の状態で送信するようになる。\
ロボットが脱力状態になる。
***

```c++
void restartCan()
```
緊急停止した後に復帰したい時やCANをリセットしたい時に使用する。
***

```c++
bool CAN1isOpen()
bool CAN2isOpen()
bool CAN3isOpen()
```
それぞれのCANbusがbeginされているかどうかを返す関数
***

```c++
void getCAN1mes()
void getCAN2mes()
void getCAN3mes()
```
それぞれのcanbusからデータを受信してCANIDをキー、CAN_message_tを値としたmapに格納する。
***

```c++
void getCan1Data(uint32_t canid,uint8_t data[8])
void getCan2Data(uint32_t canid,uint8_t data[8])
void getCan3Data(uint32_t canid,uint8_t data[8])
```
格納されているデータ(buf)をCANIDを指定してとってくる。第２引数にはデータの送り先を指定する。
***

```c++
getAllCANData()
```
使用しているCANbusのデータを全て受信する。
***

```c++
void setCAN1C610620Ampere(int cmotorid,uint16_t ampere)
void setCAN2C610620Ampere(int cmotorid,uint16_t ampere)
void setCAN3C610620Ampere(int cmotorid,uint16_t ampere)
```
C610とC620のESCを使ったモータの電流値を指定する関数。
第１引数にコントローラーIDを指定、第２引数に電流値を指定する。
***

```c++
void setCAN1GM6020Voltage(int cmotorid,uint16_t voltage)
void setCAN2GM6020Voltage(int cmotorid,uint16_t voltage)
void setCAN3GM6020Voltage(int cmotorid,uint16_t voltage)
```
GM6020の電圧を指定する関数。
第１引数にモータIDを指定、第２引数に電圧を指定する。
***

```c++
void setrowCAN1Message(uint32_t canid, uint8_t buf[8])
void setrowCAN2Message(uint32_t canid, uint8_t buf[8])
void setrowCAN3Message(uint32_t canid, uint8_t buf[8])
```
第１引数に識別子を指定し、第２引数で指定したデータを送信する。
***

```c++
void sendAllCANdata()
```
使用しているCANbusにデータを全て送信する。
***

## 使い方
### setupより前に書く

```c++
#include <CANManager.h>
CANManager* canmanager = CANManager::getInstance()
```
getInstance()がCANManagerのインスタンスのアドレスを返すので、
返されたアドレスをポインタ変数としてcanmanagerに格納する。
***

### setupに書く
```c++
  canmanager->init(true,true,true);
  canmanager->restartCan();
```
initにtrue/falseを渡して、使用するCANbusを指定する。
***

### CANが途絶してるか確認したいとき
```c++
if(canmanager->isNoCan1()){
    println("CAN1途絶");
    return;
}
```
isNoCan()は、CAN通信が上手くいっていたらtrue、途絶していたらfalseを返すので
途絶したら、**CAN1途絶**と表示される。

***

### CANbeginされたか確認したいとき
```c++
if(canmanager->CAN1isOpen(){
    println("CAN1begin")
    return;
}
```
CANbusがbeginされていたら、**CAN1begin**と表示される。
***

### データを受信したいとき
```c++
uint32_t canid = 0;
uint8_t data[8] = {};
canmanager->getCan1Data(canid,data);
canmanager->getAllCANdata();
```
CANIDを指定すると、データを取ってきてdataに格納される。
***

### データを送信したいとき
```c++
uint32_t canid = 0;
uint8_t buf[8] = {};
canmanager->setrowCAN1Message(canid,buf);
canmanager->sendAllCANdata();
```
idを決め、データを送信する。
***

### 緊急停止させてから、再始動させたいとき
```c++
canmanager->EmergencyStop()
canmanager->restartCan();
```
***


### M3508からデータを取得する
速度コントローラーID1のとき
```c++
static int16_t angle = 0;
static int16_t diff = 0;
static int16_t lastangle = 0;
static int16_t i = 0;
static int16_t position = 0;
static int16_t speed = 0;
static int16_t torque = 0;

if(canmanager->isinit()){
  uint8_t data[8];
  uint32_t canid = 0x201;
  canmanager->getCan1Data(canid,data);
  canmanager->getAllCANdata();
  position = data[0] * 256;
  position += data[1];

  diff = position - lastangle;
  if(diff < -(8191 / 2)){
    i++;
  }else if((8191 / 2) < diff){
    i--;
  }
  lastangle = position;

  angle = i * 8191 + position;
  speed = data[2] * 256;
  speed += data[3];
  torque = data[4];
  torque += data[5];

  //Serial.println(angle);
  //Serial.println(speed);
  //Serial.println(torque);
}
```
**angle** = 角度
**speed** = 回転速度
**torque** = 電流値
***

### M3508の電流値を指定する
ID1のモータに、電流値を指定する場合
ID2のモータに、電流値を指定する場合
ID3のモータに、電流値を指定する場合
```c++
canmanager->setCAN1C610620Ampere(1,ampere)
canmanager->setCAN1C610620Ampere(2,ampere)
canmanager->setCAN1C610620Ampere(3,ampere)
canmanager->sendAllCANdata();
```
***

### GM6020のデータを取得する
motorID1のとき
```c++
static int16_t angle = 0;
static int16_t diff = 0;
static int16_t lastangle = 0;
static int16_t i = 0;
static int16_t position = 0;
static int16_t speed = 0;
static int16_t torque = 0;

if(canmanager->isinit()){
  uint8_t data[8];
  uint32_t canid = 0x205;
  canmanager->getCan1Data(canid,data);
  canmanager->getAllCANdata();
  position = data[0] * 256;
  position += data[1];

  diff = position - lastangle;
  if(diff < -(8191 / 2)){
    i++;
  }else if((8191 / 2) < diff){
    i--;
  }
  lastangle = position;

  angle = i * 8191 + position;
  speed = data[2] * 256;
  speed += data[3];
  torque = data[4];
  torque += data[5];

  //Serial.println(angle);
  //Serial.println(speed);
  //Serial.println(torque);
```
**angle** = 角度
**speed** = 回転速度
**torque** = 電流値
***

### GM6020のデータを取得する
ID1のモータに、電流値を指定する場合
ID2のモータに、電流値を指定する場合
ID3のモータに、電流値を指定する場合
```c++
canmanager->setCAN1GM6020Voltage(1, ampere);
canmanager->setCAN2GM6020Voltage(2, ampere);
canmanager->setCAN3GM6020Voltage(3, ampere);
canmanager->sendAllCANdata();
```
***

## 注意事項
- GM6020 とC610,C620のESCは同一CANbus上に混在させないようにしてください！CANIDの競合によりうまく動かない可能性が高いです。
- 主にCAN1,CAN2はモータようCAN3をマイコンか通信に使いようにしてください。一応プログラム的にはどのcanbusでも通信はできるようになってはいます。もしCAN1,CAN2のbusが空きまくっている場合にはそちらで通信するようにしても構いません。ただしCANIDの競合には注意してください。
-
