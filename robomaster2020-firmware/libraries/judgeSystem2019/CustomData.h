#ifndef CustomData_h
#define CustomData_h


#include "CRC.h"

class CustomData {
 public:
    CustomData();
    void makeCustomDataPacket(unsigned char packet[28]);
    void setFrameHeader(uint8_t header[7]);
    void setData1(float value);
    void setData2(float value);
    void setData3(float value);
    void setMasks(uint8_t mask);
    void setClientID(uint16_t ID);
    void setSenderID(uint16_t ID);
    void init(uint16_t sender,uint16_t clientID);


    union {
    uint16_t before;
    struct {
        uint8_t a;
        uint8_t b;
    };
    } uint162byte;

    union {
    float before;
    struct{
        uint8_t a;
        uint8_t b;
        uint8_t c;
        uint8_t d;
    };
    } float2byte;


    //FrameHeaderフォーマット・コマンドID
    typedef struct {
    uint8_t sof;
    uint16_t dataLength;
    uint8_t seq;
    uint8_t crc8Value;
    uint16_t cmdId;
    } extFrameHeader_t;

    typedef struct {
    uint16_t dataContentsID;
    uint16_t senderID;
    uint16_t ClientID;
    float data1;
    float data2;
    float data3;
    uint8_t masks;
    } extClientCustomData_t;

    extClientCustomData_t CstData;
    extFrameHeader_t frameHeader;

    uint16_t crc16Value;
};


#endif
