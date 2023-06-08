#include <Arduino.h>
#include <Receiver.h>

// ここでSerialポートを決定
// Teensy3.2を使用するときは Serial1
// Teensy4.0を使用するときは Serial2
#define RCV_SERIAL Serial2

//初期化処理を行う関数
//コンストラクタは、　クラス名::クラス名と同じ名前で構成します
Receiver::Receiver(void){
}

void Receiver::init(){
  RCV_SERIAL.begin(100000, SERIAL_8E1);    //DJI受信機(100,000)
  // RCV_SERIAL.begin(100000, SERIAL_8E1_RXINV);    //DJI受信機(100,000)
  checkPropSafety();

}

void Receiver::getRecvData(void){

  static int data[18];
  static int dataNumber = 0;
  static unsigned long lastConnectTime = 0;

  if (RCV_SERIAL.available() > 0) {
    for (int dataNum = RCV_SERIAL.available(); dataNum > 0; dataNum--) {
      if (dataNumber < 0) {
        RCV_SERIAL.read();
        dataNumber++;
        continue;
      }
      data[dataNumber % 18] = RCV_SERIAL.read();
      dataNumber++;
      if (dataNumber > 18) {
        dataNumber = 0;
      } else if (dataNumber == 18) {
        static unsigned long testch[6];

        testch[0] = (((data[1] & 0x07) << 8) | data[0]);          //ch0(364～1024～1684)
        testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));   //ch1(364～1024～1684)
        testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
        testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));   //ch3(364～1024～1684)

        if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684
           && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {

          for (int i = 1; i < 18; i++) {
            testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);  //ch0(364～1024～1684)
            testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3)); //ch1(364～1024～1684)
            testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
            testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1)); //ch3(364～1024～1684)
            if (364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684
             && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684) {
              dataNumber = -i;
              break;
            }
          }
          if (dataNumber > 18) {
            dataNumber = -1;
          }
        } else {
          flgRecvSuccess = true;
          dataNumber = 0;
          joyRightX = testch[0];
          joyRightY = testch[1];
          joyLeftX = testch[2];
          joyLeftY = testch[3];
          toggleL = ((data[5] & 0xC0) >> 6);  //s1(1～3)
          toggleR = ((data[5] & 0x30) >> 4);  //s2(1～3)

          mouse.x = (data[6]) | (data[7] << 8);
          mouse.y = (data[8]) | (data[9] << 8);
          mouse.z = (data[10]) | (data[11] << 8);
          mouse.pressLeft = data[12];
          mouse.pressRight = data[13];
          keyData = (data[14]) | (data[15] << 8);
          applyKeyboardData();
        }
      }
    }
    lastConnectTime = millis();
  } else {
    if (lastConnectTime + 50 < millis()) {
      flgRecvSuccess = false;
      joyRightX = DJI_RECV_CENTER;
      joyRightY = DJI_RECV_CENTER;
      joyLeftX = DJI_RECV_CENTER;
      joyLeftY = DJI_RECV_CENTER;
      toggleL = toggleL;
      toggleR = toggleR;
      mouse.x = 0;
      mouse.y = 0;
      mouse.z = 0;
      mouse.pressLeft = 0;
      mouse.pressRight = 0;
      keyData = 0;
      applyKeyboardData();
    } else if (lastConnectTime + 3 < millis()) {
      dataNumber = 0;
    }
  }
  normalizePropData();
}


void Receiver::debugPrint(void) {
  if(flgRecvSuccess == true){
    //Propo
    // Serial.print("RX:");
    // Serial.print(joyRightX);
    // Serial.print("\t");

    // Serial.print("RY:");
    // Serial.print(joyRightY);
    // Serial.print("\t");

    // Serial.print("LX:");
    // Serial.print(joyLeftX);
    // Serial.print("\t");

    // Serial.print("LY:");
    // Serial.print(joyLeftY);
    // Serial.print("\t");

    // Serial.print("SWR:");
    // Serial.print(toggleR);
    // Serial.print("\t");

    // Serial.print("SWL:");
    // Serial.print(toggleL);

    // Serial.println("");

    //Mouse
    Serial.print("X:");
    Serial.print(mouse.x);
    Serial.print("\t");

    Serial.print("Y:");
    Serial.print(mouse.y);
    Serial.print("\t");

    Serial.print("Z:");
    Serial.print(mouse.z);
    Serial.print("\t");

    Serial.print("L:");
    Serial.print(mouse.pressLeft);
    Serial.print("\t");

    Serial.print("R:");
    Serial.print(mouse.pressRight);
    Serial.print("\t");

    //Keyboard
    Serial.print("KEY:");
    Serial.print(keyData);
    Serial.print("\t");

    if(key.W) Serial.print("W\t");
    if(key.S) Serial.print("S\t");
    if(key.A) Serial.print("A\t");
    if(key.D) Serial.print("D\t");
    if(key.Shift) Serial.print("Shift\t");
    if(key.Ctrl) Serial.print("Ctrl\t");
    if(key.Q) Serial.print("Q\t");
    if(key.E) Serial.print("E\t");
    if(key.R) Serial.print("R\t");
    if(key.F) Serial.print("F\t");
    if(key.G) Serial.print("G\t");
    if(key.Z) Serial.print("Z\t");
    if(key.X) Serial.print("X\t");
    if(key.C) Serial.print("C\t");
    if(key.V) Serial.print("V\t");
    if(key.B) Serial.print("B\t");

    Serial.println("");

  }else{
    Serial.println("Recv : non connection");
  }

}


void Receiver::normalizePropData(void){
  joy.rightX = constrain((joyRightX - 1024), -JOY_ABS_MAX, JOY_ABS_MAX);
  joy.rightY = constrain((joyRightY - 1024), -JOY_ABS_MAX, JOY_ABS_MAX);
  joy.leftX = constrain((joyLeftX - 1024), -JOY_ABS_MAX, JOY_ABS_MAX);
  joy.leftY = constrain((joyLeftY - 1024), -JOY_ABS_MAX, JOY_ABS_MAX);
  toggle.left = toggleL;
  toggle.right = toggleR;
}

void Receiver::applyKeyboardData(void){
  key.W = (keyData & 0x0001) ? true : false;
  key.S = (keyData & 0x0002) ? true : false;
  key.A = (keyData & 0x0004) ? true : false;
  key.D = (keyData & 0x0008) ? true : false;
  key.Shift = (keyData & 0x0010) ? true : false;
  key.Ctrl = (keyData & 0x0020) ? true : false;
  key.Q = (keyData & 0x0040) ? true : false;
  key.E = (keyData & 0x0080) ? true : false;
  key.R = (keyData & 0x0100) ? true : false;
  key.F = (keyData & 0x0200) ? true : false;
  key.G = (keyData & 0x0400) ? true : false;
  key.Z = (keyData & 0x0800) ? true : false;
  key.X = (keyData & 0x1000) ? true : false;
  key.C = (keyData & 0x2000) ? true : false;
  key.V = (keyData & 0x4000) ? true : false;
  key.B = (keyData & 0x8000) ? true : false;
}

void Receiver::checkPropSafety(void){
  bool flgPropSafety = false;
  flgRecvSuccess = false;
  while(!flgPropSafety){
    getRecvData();
    int joyInitWidth = 20;
    int joyInitLow = DJI_RECV_CENTER - joyInitWidth;
    int joyInitHigh = DJI_RECV_CENTER + joyInitWidth;

    if((joyInitLow < joyRightX) && (joyRightX < joyInitHigh)
    && (joyInitLow < joyRightY) && (joyRightY < joyInitHigh)
    && (joyInitLow < joyLeftX) && (joyLeftX < joyInitHigh)
    && (joyInitLow < joyLeftY) && (joyLeftY < joyInitHigh)
    && (toggleL == 2) && (toggleR == 2) && (flgRecvSuccess == true)){
      flgPropSafety = true;
      tone(pinBuzzer, 523);  //ド
      delay(100);
      tone(pinBuzzer, 659);  //ミ
      delay(100);
      tone(pinBuzzer, 783);  //ソ
      delay(100);
      noTone(pinBuzzer);
      delay(100);
    }else{
      flgPropSafety = false;
      // tone(pinBuzzer, 659);  //ミ
      // delay(100);
      // tone(pinBuzzer, 587);  //レ
      // delay(100);
      // tone(pinBuzzer, 523);  //ド
      // delay(100);
      // noTone(pinBuzzer);
      // delay(700);
    }
  }

}

void Receiver::setPinNoBuzzer(int number){
  pinBuzzer = number;
}