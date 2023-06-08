#ifndef CANManager_h
#define CANManager_h
#include <map>
#include <vector>

//TestCommit

//シングルトン設計
//シングルトン設計のためインスタンスを呼び出すときはポインタ変数として確保する。
//代入演算子を使用した場合コピーされるのでシングルトンにならないので代入演算子、コピーコンストラクタの使用を禁止にしている。
//使用方法はCANDEBUG.inoを参照のこと
class CANManager{
  protected:

    CANManager();

    //モーターデータをCANmesに変換するようのコンストラクタ(GM6020,C610,C620用)
    typedef struct{
      int16_t firstmotor;
      int16_t secondmotor;
      int16_t thirdmotor;
      int16_t fourthmotor;
      bool usethisdata;
    }motordata;

    motordata CAN1C610620data1;
    motordata CAN2C610620data1;
    motordata CAN3C610620data1;
    motordata CAN1C610620data2;
    motordata CAN2C610620data2;
    motordata CAN3C610620data2;
    motordata CAN1GM6020data1;
    motordata CAN2GM6020data1;
    motordata CAN3GM6020data1;
    motordata CAN1GM6020data2;
    motordata CAN2GM6020data2;
    motordata CAN3GM6020data2;



    bool inited = false;
    bool can1busisopen = false;
    bool can2busisopen = false;
    bool can3busisopen = false;
      //CAN受信データ途絶時にTrue
      bool flgNoCan1Data = false;
      bool flgNoCan2Data = false;
      bool flgNoCan3Data = false;
      //緊急停止用フラグ
      bool flgEmergencyStop = false;
    void operator=(const CANManager& obj) {}
    CANManager(const CANManager &obj){}

    //mapテーブルクリア関数
    void clearAllTable();
    //vectorテーブルクリア関数
    void clearAllVector();

  public:
  //インスタンス操作
  static CANManager *getInstance();

  //init関連
  void init(bool canbus1,bool canbus2,bool canbus3);
  bool isinit();

    //CAN受信データ途絶検知関数
    bool isNoCan1();
    bool isNoCan2();
    bool isNoCan3();

    //緊急停止、再始動用関数
    void EmergencyStop();
    void restartCan();


    //CAN1メッセージ操作
    bool CAN1isOpen();//canがopenしているかどうか
    void getCAN1mes();//canbusからデータを受け取ってhasmap上に格納する。
    void getCan1Data(uint32_t canid,uint8_t data[8]);//格納されているデータ(buf)をCANIDを指定してとってくる。第２引数にはデータの送り先を指定する。

    //CAN2メッセージ操作
    bool CAN2isOpen();
    void getCAN2mes();
    void getCan2Data(uint32_t canid,uint8_t data[8]);

    //CAN3メッセージ操作
    bool CAN3isOpen();
    void getCAN3mes();
    void getCan3Data(uint32_t canid,uint8_t data[8]);

    //使用しているCANbusのデータを受信する関数
    void getAllCANdata();


  //CAN送信側関数
  void setCAN1C610620Ampere(int cmotorid,uint16_t ampere);//(-16384 ~ 0 ~ 16384)アンペア換算で(-20A ~ 0 ~ 20A)
  void setCAN1GM6020Voltage(int cmotorid,uint16_t voltage);
  void setCAN2C610620Ampere(int cmotorid,uint16_t ampere);
  void setCAN2GM6020Voltage(int cmotorid,uint16_t voltage);
  void setCAN3C610620Ampere(int cmotorid,uint16_t ampere);
  void setCAN3GM6020Voltage(int cmotorid,uint16_t voltage);
  void setrowCAN1Message(uint32_t canid, uint8_t buf[8]);
  void setrowCAN2Message(uint32_t canid, uint8_t buf[8]);
  void setrowCAN3Message(uint32_t canid, uint8_t buf[8]);
  //CANbusへの送信
  void sendAllCANdata();

};
#endif
