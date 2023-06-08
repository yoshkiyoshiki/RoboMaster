#include "CustomData.h"
#include "CRC.h"

CustomData::CustomData() {}

void CustomData::makeCustomDataPacket(unsigned char packet[28]){
  //ヘッダー情報
  unsigned char header[7];
  setFrameHeader(header);
  int i = 0;
  for(i = 0;i<7;i++){
    packet[i] = header[i];
  }
  int OFFSET = 7;
  //データ格納
  uint16_t dataContentsID = 0xD180;
  uint162byte.before = dataContentsID;
  packet[OFFSET +0] = uint162byte.a;
  packet[OFFSET +1] = uint162byte.b;
  uint162byte.before = CstData.senderID;
  packet[OFFSET +2] = uint162byte.a;
  packet[OFFSET +3] = uint162byte.b;
  uint162byte.before = CstData.ClientID;
  packet[OFFSET +4] = uint162byte.a;
  packet[OFFSET +5] = uint162byte.b;
  float2byte.before = CstData.data1;
  packet[OFFSET +6] = float2byte.a;
  packet[OFFSET +7] = float2byte.b;
  packet[OFFSET +8] = float2byte.c;
  packet[OFFSET +9] = float2byte.d;
  float2byte.before = CstData.data2;
  packet[OFFSET +10] = float2byte.a;
  packet[OFFSET +11] = float2byte.b;
  packet[OFFSET +12] = float2byte.c;
  packet[OFFSET +13] = float2byte.d;
  float2byte.before = CstData.data2;
  packet[OFFSET +14] = float2byte.a;
  packet[OFFSET +15] = float2byte.b;
  packet[OFFSET +16] = float2byte.c;
  packet[OFFSET +17] = float2byte.d;
  packet[OFFSET +18] = CstData.masks;
  Append_CRC16_Check_Sum(packet,26);

}

void CustomData::setFrameHeader(uint8_t header[7]){
  frameHeader.sof = 0xA5;
  frameHeader.dataLength = 19;
  frameHeader.seq = 0;
  frameHeader.cmdId = 0x0301;
  //CRCCheck
  header[0] = 0xA5; //sof
  uint162byte.before = 19; //DataLength
  header[1] = uint162byte.a;
  header[2] = uint162byte.b;
  header[3] = 0;//Seq
  uint8_t HeaderCRC = 0x00;
  HeaderCRC = Get_CRC8_Check_Sum(header,4,0xFF);
  frameHeader.crc8Value = HeaderCRC;//CRC8Check
  header[4] = 0x03;
  header[5] = 0x01;
  header[6] = HeaderCRC;

}

//データ登録系関数
 void CustomData::setData1(float value){
  CstData.data1 = value;
}
 void CustomData::setData2(float value){
  CstData.data2 = value;
}
 void CustomData::setData3(float value){
  CstData.data2 = value;
}
 void CustomData::setMasks(uint8_t mask){
  CstData.masks = mask;
}
 void CustomData::setClientID(uint16_t ID){
  CstData.ClientID= ID;
}
 void CustomData::setSenderID(uint16_t ID){
  CstData.senderID = ID;
}

//初期化関数
 void CustomData::init(uint16_t sender,uint16_t clientID){
   setData1(0);
   setData2(0);
   setData3(0);
   setMasks(0xFF);
   setSenderID(sender);
   setClientID(clientID);
 }
