#include <CANManager.h>
#include <FlexCAN_T4.h>


//CANインスタンス
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

//受信側maptable
std::map<uint32_t, CAN_message_t> CAN1_mestable;
std::map<uint32_t, CAN_message_t> CAN2_mestable;
std::map<uint32_t, CAN_message_t> CAN3_mestable;

//送信側vector
std::vector<CAN_message_t> CAN1_senddata;
std::vector<CAN_message_t> CAN2_senddata;
std::vector<CAN_message_t> CAN3_senddata;



//インスタンスを取得することができる唯一の関数
CANManager *CANManager::getInstance(){
  static CANManager canmanager;
  return &canmanager;//canmanagerのインスタンスのアドレスを返す。


}

 CANManager::CANManager(){}

//初期化関数
 void CANManager::init(bool canbus1,bool canbus2,bool canbus3){
   if(canbus1){
   Can1.begin();
   Can1.setBaudRate(1000000);
   can1busisopen = true;
 }
  if(canbus2){
    Can2.begin();
    Can2.setBaudRate(1000000);
    can2busisopen = true;
  }
  if(canbus3){
    Can3.begin();
    Can3.setBaudRate(1000000);
    can3busisopen = true;
  }
  delay(500);
   inited = true;
 }


 bool CANManager::isinit(){
   return inited;
 }
 //mapテーブルクリア関数
  void CANManager::clearAllTable(){
      CAN1_mestable.clear();
      CAN2_mestable.clear();
      CAN3_mestable.clear();
  }
  //vectorテーブルクリア関数
    void CANManager::clearAllVector(){
      CAN1_senddata.clear();
      CAN2_senddata.clear();
      CAN3_senddata.clear();
    }

//canbus存在チェック関数
bool CANManager::CAN1isOpen(){
  return can1busisopen;
}
bool CANManager::CAN2isOpen(){
  return can2busisopen;
}
bool CANManager::CAN3isOpen(){
  return can3busisopen;
}

//CAN受信データ途絶検知関数
bool CANManager::isNoCan1(){
  return flgNoCan1Data;
}
bool CANManager::isNoCan2(){
  return flgNoCan2Data;
}
bool CANManager::isNoCan3(){
  return flgNoCan3Data;
}
//緊急停止用関数
void CANManager::EmergencyStop(){
  flgEmergencyStop = true;
}
//再始動関数
void CANManager::restartCan(){
  flgEmergencyStop = false;
  Can1.reset();
  Can2.reset();
  Can3.reset();
  clearAllTable();
  clearAllVector();
  init(can1busisopen,can2busisopen,can3busisopen);
}

//canbusからデータを受信する関数
void CANManager::getCAN1mes(){
  CAN_message_t CAN1msg;
  static int can1Cnt;
  while ( Can1.read(CAN1msg) ) {
    CAN1_mestable[CAN1msg.id] = CAN1msg;
    can1Cnt = 0;
  }
  //CAN受信エラーカウンタで分岐
  if (can1Cnt == 0) {  //正常受信
    flgNoCan1Data = false;
    can1Cnt += 1;
  } else if (can1Cnt < 16) { //エラーだが許容範囲
    can1Cnt += 1;
  } else {             //CANデータ受信途絶のとき緊急停止。
    flgNoCan1Data = true;
  }
}


void CANManager::getCAN2mes(){
  CAN_message_t CAN2msg;
  static int can2Cnt;
  while ( Can2.read(CAN2msg) ) {
    CAN2_mestable[CAN2msg.id] = CAN2msg;
    can2Cnt = 0;
  }
  //CAN受信エラーカウンタで分岐
  if (can2Cnt == 0) {  //正常受信
    flgNoCan2Data = false;
    can2Cnt += 1;
  } else if (can2Cnt < 16) { //エラーだが許容範囲
    can2Cnt += 1;
  } else {             //CANデータ受信途絶のとき緊急停止。
    flgNoCan2Data = true;
  }
}


void CANManager::getCAN3mes(){
  CAN_message_t CAN3msg;
  static int can3Cnt;
  while ( Can3.read(CAN3msg) ) {
    CAN3_mestable[CAN3msg.id] = CAN3msg;
    can3Cnt = 0;
  }
  //CAN受信エラーカウンタで分岐
  if (can3Cnt == 0) {  //正常受信
    flgNoCan3Data = false;
    can3Cnt += 1;
  } else if (can3Cnt < 16) { //エラーだが許容範囲
    can3Cnt += 1;
  } else {             //CANデータ受信途絶のとき緊急停止。
    flgNoCan3Data = true;
  }
}
//canbusからそれぞれのCAN1~3のデータを受信する
void CANManager::getAllCANdata(){
  if(can1busisopen){getCAN1mes();}
  if(can2busisopen){getCAN2mes();}
  if(can3busisopen){getCAN3mes();}

}


void CANManager::getCan1Data(uint32_t canid,uint8_t data[8]){
  CAN_message_t msg = CAN1_mestable[canid];
  memcpy(data,msg.buf,8);
}
void CANManager::getCan2Data(uint32_t canid,uint8_t data[8]){
  CAN_message_t msg = CAN2_mestable[canid];
  memcpy(data,msg.buf,8);
}
void CANManager::getCan3Data(uint32_t canid,uint8_t data[8]){
  CAN_message_t msg = CAN3_mestable[canid];
  memcpy(data,msg.buf,8);
}



  //CAN送信側関数
  //CAN1用モーター出力値設定関数
  void CANManager::setCAN1C610620Ampere(int cmotorid,uint16_t ampere){
    switch(cmotorid){
      case 1:
      CAN1C610620data1.firstmotor = ampere;
      CAN1C610620data1.usethisdata = true;
      break;

      case 2:
      CAN1C610620data1.secondmotor = ampere;
      CAN1C610620data1.usethisdata = true;
      break;

      case 3:
      CAN1C610620data1.thirdmotor = ampere;
      CAN1C610620data1.usethisdata = true;
      break;

      case 4:
      CAN1C610620data1.fourthmotor = ampere;
      CAN1C610620data1.usethisdata = true;
      break;

      case 5:
      CAN1C610620data2.firstmotor = ampere;
      CAN1C610620data2.usethisdata = true;
      break;

      case 6:
      CAN1C610620data2.secondmotor = ampere;
      CAN1C610620data2.usethisdata = true;
      break;

      case 7:
      CAN1C610620data2.thirdmotor = ampere;
      CAN1C610620data2.usethisdata = true;
      break;

      case 8:
      CAN1C610620data2.fourthmotor = ampere;
      CAN1C610620data2.usethisdata = true;
      break;
    }


  }
  void CANManager::setCAN1GM6020Voltage(int cmotorid,uint16_t voltage){
    switch(cmotorid){
      case 1:
      CAN1GM6020data1.firstmotor = voltage;
      CAN1GM6020data1.usethisdata = true;
      break;

      case 2:
      CAN1GM6020data1.secondmotor = voltage;
      CAN1GM6020data1.usethisdata = true;
      break;

      case 3:
      CAN1GM6020data1.thirdmotor = voltage;
      CAN1GM6020data1.usethisdata = true;
      break;

      case 4:
      CAN1GM6020data1.fourthmotor = voltage;
      CAN1GM6020data1.usethisdata = true;
      break;

      case 5:
      CAN1GM6020data2.firstmotor = voltage;
      CAN1GM6020data2.usethisdata = true;
      break;

      case 6:
      CAN1GM6020data2.secondmotor = voltage;
      CAN1GM6020data2.usethisdata = true;
      break;

      case 7:
      CAN1GM6020data2.thirdmotor = voltage;
      CAN1GM6020data2.usethisdata = true;
      break;
    }
  }


  //CAN2用モーター出力値設定関数
  void CANManager::setCAN2C610620Ampere(int cmotorid,uint16_t ampere){
    switch(cmotorid){
      case 1:
      CAN2C610620data1.firstmotor = ampere;
      CAN2C610620data1.usethisdata = true;
      break;

      case 2:
      CAN2C610620data1.secondmotor = ampere;
      CAN2C610620data1.usethisdata = true;
      break;

      case 3:
      CAN2C610620data1.thirdmotor = ampere;
      CAN2C610620data1.usethisdata = true;
      break;

      case 4:
      CAN2C610620data1.fourthmotor = ampere;
      CAN2C610620data1.usethisdata = true;
      break;

      case 5:
      CAN2C610620data2.firstmotor = ampere;
      CAN2C610620data2.usethisdata = true;
      break;

      case 6:
      CAN2C610620data2.secondmotor = ampere;
      CAN2C610620data2.usethisdata = true;
      break;

      case 7:
      CAN2C610620data2.thirdmotor = ampere;
      CAN2C610620data2.usethisdata = true;
      break;

      case 8:
      CAN2C610620data2.fourthmotor = ampere;
      CAN2C610620data2.usethisdata = true;
      break;
    }


  }
  void CANManager::setCAN2GM6020Voltage(int cmotorid,uint16_t voltage){
    switch(cmotorid){
      case 1:
      CAN2GM6020data1.firstmotor = voltage;
      CAN2GM6020data1.usethisdata = true;
      break;

      case 2:
      CAN2GM6020data1.secondmotor = voltage;
      CAN2GM6020data1.usethisdata = true;
      break;

      case 3:
      CAN2GM6020data1.thirdmotor = voltage;
      CAN2GM6020data1.usethisdata = true;
      break;

      case 4:
      CAN2GM6020data1.fourthmotor = voltage;
      CAN2GM6020data1.usethisdata = true;
      break;

      case 5:
      CAN2GM6020data2.firstmotor = voltage;
      CAN2GM6020data2.usethisdata = true;
      break;

      case 6:
      CAN2GM6020data2.secondmotor = voltage;
      CAN2GM6020data2.usethisdata = true;
      break;

      case 7:
      CAN2GM6020data2.thirdmotor = voltage;
      CAN2GM6020data2.usethisdata = true;
      break;
    }
  }


  //CAN3用モーター出力値設定関数
  void CANManager::setCAN3C610620Ampere(int cmotorid,uint16_t ampere){
    switch(cmotorid){
      case 1:
      CAN3C610620data1.firstmotor = ampere;
      CAN3C610620data1.usethisdata = true;
      break;

      case 2:
      CAN3C610620data1.secondmotor = ampere;
      CAN3C610620data1.usethisdata = true;
      break;

      case 3:
      CAN3C610620data1.thirdmotor = ampere;
      CAN3C610620data1.usethisdata = true;
      break;

      case 4:
      CAN3C610620data1.fourthmotor = ampere;
      CAN3C610620data1.usethisdata = true;
      break;

      case 5:
      CAN3C610620data2.firstmotor = ampere;
      CAN3C610620data2.usethisdata = true;
      break;

      case 6:
      CAN3C610620data2.secondmotor = ampere;
      CAN3C610620data2.usethisdata = true;
      break;

      case 7:
      CAN3C610620data2.thirdmotor = ampere;
      CAN3C610620data2.usethisdata = true;
      break;

      case 8:
      CAN3C610620data2.fourthmotor = ampere;
      CAN3C610620data2.usethisdata = true;
      break;
    }


  }
  void CANManager::setCAN3GM6020Voltage(int cmotorid,uint16_t voltage){
    switch(cmotorid){
      case 1:
      CAN3GM6020data1.firstmotor = voltage;
      CAN3GM6020data1.usethisdata = true;
      break;

      case 2:
      CAN3GM6020data1.secondmotor = voltage;
      CAN3GM6020data1.usethisdata = true;
      break;

      case 3:
      CAN3GM6020data1.thirdmotor = voltage;
      CAN3GM6020data1.usethisdata = true;
      break;

      case 4:
      CAN3GM6020data1.fourthmotor = voltage;
      CAN3GM6020data1.usethisdata = true;
      break;

      case 5:
      CAN3GM6020data2.firstmotor = voltage;
      CAN3GM6020data2.usethisdata = true;
      break;

      case 6:
      CAN3GM6020data2.secondmotor = voltage;
      CAN3GM6020data2.usethisdata = true;
      break;

      case 7:
      CAN3GM6020data2.thirdmotor = voltage;
      CAN3GM6020data2.usethisdata = true;
      break;
    }
  }
//canmessageの生データをvectorに登録
void CANManager::setrowCAN1Message(uint32_t canid, uint8_t buf[8]){
  CAN_message_t msg;
  msg.id = canid;
  for(int i = 0;i<8;i++){
    msg.buf[i] = buf[i];
  }
  CAN1_senddata.push_back(msg);
}
void CANManager::setrowCAN2Message(uint32_t canid, uint8_t buf[8]){
  CAN_message_t msg;
  msg.id = canid;
  for(int i = 0;i<8;i++){
    msg.buf[i] = buf[i];
  }
  CAN2_senddata.push_back(msg);
}
void CANManager::setrowCAN3Message(uint32_t canid, uint8_t buf[8]){
  CAN_message_t msg;
  msg.id = canid;
  for(int i = 0;i<8;i++){
    msg.buf[i] = buf[i];
  }
  CAN3_senddata.push_back(msg);
}

//CANBusにデータを送信

void CANManager::sendAllCANdata(){
  CAN_message_t msg;
  ///////////////////////////////////////////////////////////////////
  //CAN1
  ///////////////////////////////////////////////////////////////////
  //モーター駆動系のCANPacket作成
  if(CAN1C610620data1.usethisdata && CAN1isOpen()){
    msg.id = 0x200;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan1Data){
    msg.buf[0] = CAN1C610620data1.firstmotor >> 8;
    msg.buf[1] = CAN1C610620data1.firstmotor & 0x00ff;
    msg.buf[2] = CAN1C610620data1.secondmotor >> 8;
    msg.buf[3] = CAN1C610620data1.secondmotor & 0x00ff;
    msg.buf[4] = CAN1C610620data1.thirdmotor >> 8;
    msg.buf[5] = CAN1C610620data1.thirdmotor & 0x00ff;
    msg.buf[6] = CAN1C610620data1.fourthmotor >> 8;
    msg.buf[7] = CAN1C610620data1.fourthmotor & 0x00ff;
  }
    CAN1_senddata.push_back(msg);
  }

  if(CAN1C610620data2.usethisdata && CAN1isOpen()){
    msg.id = 0x1FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan1Data){
    msg.buf[0] = CAN1C610620data2.firstmotor >> 8;
    msg.buf[1] = CAN1C610620data2.firstmotor & 0x00ff;
    msg.buf[2] = CAN1C610620data2.secondmotor >> 8;
    msg.buf[3] = CAN1C610620data2.secondmotor & 0x00ff;
    msg.buf[4] = CAN1C610620data2.thirdmotor >> 8;
    msg.buf[5] = CAN1C610620data2.thirdmotor & 0x00ff;
    msg.buf[6] = CAN1C610620data2.fourthmotor >> 8;
    msg.buf[7] = CAN1C610620data2.fourthmotor & 0x00ff;
  }
    CAN1_senddata.push_back(msg);
  }
  if(CAN1GM6020data1.usethisdata && CAN1isOpen()){
    msg.id = 0x1FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan1Data){
    msg.buf[0] = CAN1GM6020data1.firstmotor >> 8;
    msg.buf[1] = CAN1GM6020data1.firstmotor & 0x00ff;
    msg.buf[2] = CAN1GM6020data1.secondmotor >> 8;
    msg.buf[3] = CAN1GM6020data1.secondmotor & 0x00ff;
    msg.buf[4] = CAN1GM6020data1.thirdmotor >> 8;
    msg.buf[5] = CAN1GM6020data1.thirdmotor & 0x00ff;
    msg.buf[6] = CAN1GM6020data1.fourthmotor >> 8;
    msg.buf[7] = CAN1GM6020data1.fourthmotor & 0x00ff;
  }
    CAN1_senddata.push_back(msg);
  }
  if(CAN1GM6020data2.usethisdata && CAN1isOpen()){
    msg.id = 0x2FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan1Data){
    msg.buf[0] = CAN1GM6020data2.firstmotor >> 8;
    msg.buf[1] = CAN1GM6020data2.firstmotor & 0x00ff;
    msg.buf[2] = CAN1GM6020data2.secondmotor >> 8;
    msg.buf[3] = CAN1GM6020data2.secondmotor & 0x00ff;
    msg.buf[4] = CAN1GM6020data2.thirdmotor >> 8;
    msg.buf[5] = CAN1GM6020data2.thirdmotor & 0x00ff;
    msg.buf[6] = CAN1GM6020data2.fourthmotor >> 8;
    msg.buf[7] = CAN1GM6020data2.fourthmotor & 0x00ff;
  }
    CAN1_senddata.push_back(msg);
  }
  //CANbusに送信
  if(CAN1isOpen()){
    for(CAN_message_t outmes: CAN1_senddata){
      Can1.write(outmes);
    }
  }
  CAN1_senddata.clear();


  ///////////////////////////////////////////////////////////////////
  //CAN2
  ///////////////////////////////////////////////////////////////////
  //モーター駆動系のCANPacket作成
  if(CAN2C610620data1.usethisdata && CAN2isOpen()){
    msg.id = 0x200;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan2Data){
    msg.buf[0] = CAN2C610620data1.firstmotor >> 8;
    msg.buf[1] = CAN2C610620data1.firstmotor & 0x00ff;
    msg.buf[2] = CAN2C610620data1.secondmotor >> 8;
    msg.buf[3] = CAN2C610620data1.secondmotor & 0x00ff;
    msg.buf[4] = CAN2C610620data1.thirdmotor >> 8;
    msg.buf[5] = CAN2C610620data1.thirdmotor & 0x00ff;
    msg.buf[6] = CAN2C610620data1.fourthmotor >> 8;
    msg.buf[7] = CAN2C610620data1.fourthmotor & 0x00ff;
  }
    CAN2_senddata.push_back(msg);
  }

  if(CAN2C610620data2.usethisdata && CAN2isOpen()){
    msg.id = 0x1FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan2Data){
    msg.buf[0] = CAN2C610620data2.firstmotor >> 8;
    msg.buf[1] = CAN2C610620data2.firstmotor & 0x00ff;
    msg.buf[2] = CAN2C610620data2.secondmotor >> 8;
    msg.buf[3] = CAN2C610620data2.secondmotor & 0x00ff;
    msg.buf[4] = CAN2C610620data2.thirdmotor >> 8;
    msg.buf[5] = CAN2C610620data2.thirdmotor & 0x00ff;
    msg.buf[6] = CAN2C610620data2.fourthmotor >> 8;
    msg.buf[7] = CAN2C610620data2.fourthmotor & 0x00ff;
  }
    CAN2_senddata.push_back(msg);
  }
  if(CAN2GM6020data1.usethisdata && CAN2isOpen()){
    msg.id = 0x1FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan2Data){
    msg.buf[0] = CAN2GM6020data1.firstmotor >> 8;
    msg.buf[1] = CAN2GM6020data1.firstmotor & 0x00ff;
    msg.buf[2] = CAN2GM6020data1.secondmotor >> 8;
    msg.buf[3] = CAN2GM6020data1.secondmotor & 0x00ff;
    msg.buf[4] = CAN2GM6020data1.thirdmotor >> 8;
    msg.buf[5] = CAN2GM6020data1.thirdmotor & 0x00ff;
    msg.buf[6] = CAN2GM6020data1.fourthmotor >> 8;
    msg.buf[7] = CAN2GM6020data1.fourthmotor & 0x00ff;
  }
    CAN2_senddata.push_back(msg);
  }
  if(CAN2GM6020data2.usethisdata && CAN2isOpen()){
    msg.id = 0x2FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan2Data){
    msg.buf[0] = CAN2GM6020data2.firstmotor >> 8;
    msg.buf[1] = CAN2GM6020data2.firstmotor & 0x00ff;
    msg.buf[2] = CAN2GM6020data2.secondmotor >> 8;
    msg.buf[3] = CAN2GM6020data2.secondmotor & 0x00ff;
    msg.buf[4] = CAN2GM6020data2.thirdmotor >> 8;
    msg.buf[5] = CAN2GM6020data2.thirdmotor & 0x00ff;
    msg.buf[6] = CAN2GM6020data2.fourthmotor >> 8;
    msg.buf[7] = CAN2GM6020data2.fourthmotor & 0x00ff;
  }
    CAN2_senddata.push_back(msg);
  }
  //CANbusに送信
  if(CAN2isOpen()){
    for(CAN_message_t outmes: CAN2_senddata){
      Can2.write(outmes);
    }
  }
  CAN2_senddata.clear();

  ///////////////////////////////////////////////////////////////////
  //CAN3
  ///////////////////////////////////////////////////////////////////
  //モーター駆動系のCANPacket作成
  if(CAN3C610620data1.usethisdata && CAN3isOpen()){
    msg.id = 0x200;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan3Data){
    msg.buf[0] = CAN3C610620data1.firstmotor >> 8;
    msg.buf[1] = CAN3C610620data1.firstmotor & 0x00ff;
    msg.buf[2] = CAN3C610620data1.secondmotor >> 8;
    msg.buf[3] = CAN3C610620data1.secondmotor & 0x00ff;
    msg.buf[4] = CAN3C610620data1.thirdmotor >> 8;
    msg.buf[5] = CAN3C610620data1.thirdmotor & 0x00ff;
    msg.buf[6] = CAN3C610620data1.fourthmotor >> 8;
    msg.buf[7] = CAN3C610620data1.fourthmotor & 0x00ff;
  }
    CAN3_senddata.push_back(msg);
  }

  if(CAN3C610620data2.usethisdata && CAN3isOpen()){
    msg.id = 0x1FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan3Data){
    msg.buf[0] = CAN3C610620data2.firstmotor >> 8;
    msg.buf[1] = CAN3C610620data2.firstmotor & 0x00ff;
    msg.buf[2] = CAN3C610620data2.secondmotor >> 8;
    msg.buf[3] = CAN3C610620data2.secondmotor & 0x00ff;
    msg.buf[4] = CAN3C610620data2.thirdmotor >> 8;
    msg.buf[5] = CAN3C610620data2.thirdmotor & 0x00ff;
    msg.buf[6] = CAN3C610620data2.fourthmotor >> 8;
    msg.buf[7] = CAN3C610620data2.fourthmotor & 0x00ff;
  }
    CAN3_senddata.push_back(msg);
  }
  if(CAN3GM6020data1.usethisdata && CAN3isOpen()){
    msg.id = 0x1FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan3Data){
    msg.buf[0] = CAN3GM6020data1.firstmotor >> 8;
    msg.buf[1] = CAN3GM6020data1.firstmotor & 0x00ff;
    msg.buf[2] = CAN3GM6020data1.secondmotor >> 8;
    msg.buf[3] = CAN3GM6020data1.secondmotor & 0x00ff;
    msg.buf[4] = CAN3GM6020data1.thirdmotor >> 8;
    msg.buf[5] = CAN3GM6020data1.thirdmotor & 0x00ff;
    msg.buf[6] = CAN3GM6020data1.fourthmotor >> 8;
    msg.buf[7] = CAN3GM6020data1.fourthmotor & 0x00ff;
  }
    CAN3_senddata.push_back(msg);
  }
  if(CAN3GM6020data2.usethisdata && CAN3isOpen()){
    msg.id = 0x2FF;
    for(int i= 0;i<8;i++){
      msg.buf[i] = 0;
    }
    if(!flgEmergencyStop  && !flgNoCan3Data){
    msg.buf[0] = CAN3GM6020data2.firstmotor >> 8;
    msg.buf[1] = CAN3GM6020data2.firstmotor & 0x00ff;
    msg.buf[2] = CAN3GM6020data2.secondmotor >> 8;
    msg.buf[3] = CAN3GM6020data2.secondmotor & 0x00ff;
    msg.buf[4] = CAN3GM6020data2.thirdmotor >> 8;
    msg.buf[5] = CAN3GM6020data2.thirdmotor & 0x00ff;
    msg.buf[6] = CAN3GM6020data2.fourthmotor >> 8;
    msg.buf[7] = CAN3GM6020data2.fourthmotor & 0x00ff;
  }
    CAN3_senddata.push_back(msg);
  }
  //CANbusに送信
  if(CAN3isOpen()){
    for(CAN_message_t outmes: CAN3_senddata){
      Can3.write(outmes);
    }
  }
  CAN3_senddata.clear();


}
