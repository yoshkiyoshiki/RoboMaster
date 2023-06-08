#include "judgeSystem2020.h"
#include "Arduino.h"

#define JUDGESYSTEM_SERIAL Serial3

// -------------------------------------------------------------
// 歩兵プログラム ジャッジシステム通信 2019年版
// ジャッジシステムとの通信はシリアル３を使用(RX3, TX3)
// -------------------------------------------------------------
judgeSystem2020::judgeSystem2020() {
  JUDGESYSTEM_SERIAL.begin(115200);
}

void judgeSystem2020::getJudgeSystemData() {
  while (JUDGESYSTEM_SERIAL.available() > 0) {
  // if (JUDGESYSTEM_SERIAL.available() > 0) {
    countRecMiss = 0;
    unsigned int recData = JUDGESYSTEM_SERIAL.read();
    // Serial.println(recData);
    if (recData == 0xA5) {
      dataNumber = 0;
      data[dataNumber % 90] = recData;
    } else {
      dataNumber++;
      data[dataNumber % 90] = recData;
    }

    if ((dataNumber == 0)) {
      //ヘッダ情報---------------------------------------------------------------
      int sof = data[0];                             // SOF 0xA5固定
      long dataLength = (data[2] << 8) | (data[1]);  //データフレーム内の長さ
      int seq = data[3];                             //シーケンス番号
      int crc8Value = data[4];                       //ヘッダ
      header[0] = sof;
      header[1] = data[1];
      header[2] = data[2];
      header[3] = data[3];
      // int CRC8_value = Get_CRC8_Check_Sum(header, 4, 0);
      // Serial.print("CRC8:");
      // Serial.println(CRC8_value,HEX);

      //コマンドID---------------------------------------------------------------
      long cmdId = (data[6] << 8) | (data[5]);  //コマンドID
      if (DEBUG_PRINT_JUDGE) {
        Serial.println("<FrameHeader>");
        Serial.print("SOF:0x");
        Serial.print(sof, HEX);
        Serial.print('(');
        Serial.print(sof, DEC);
        Serial.println(')');
        Serial.print("DataLength:");
        Serial.println(dataLength);
        Serial.print("Seq:");
        Serial.println(seq);
        Serial.print("CRC8:0x");
        Serial.println(crc8Value, HEX);

        //      Serial.print("checkSum:0x");
        //      Serial.println( GetCRC8(data, 4), HEX);

        Serial.println("<CmdId>");
        Serial.print("CmdID:0x");
        if(cmdId < 0x0010){
          Serial.print("000");
        }else if(cmdId < 0x0100){
          Serial.print("00");
        }else if(cmdId < 0x1000){
          Serial.print("0");
        }
        Serial.println(cmdId, HEX);
      }

      if (cmdId == 0x0001) {
        //データセグメントの長さ：3byte
        //試合ステータスデータ
        //※1Hz周波数の周期
        competitionStatus.gametype = (data[DATAOFFSET + 0] & 0x0F);
        competitionStatus.gameprogress = (data[DATAOFFSET + 0] & 0xF0) >> 4;
        competitionStatus.stageRemainTime = (data[DATAOFFSET + 2] << 8) | (data[DATAOFFSET + 1]);

      } else if (cmdId == 0x0002) {
        //データセグメントの長さ：1byte
        //試合結果データ
        //※試合終了時に発信
        gameResult.winner = data[DATAOFFSET + 0];

      } else if (cmdId == 0x0003) {
        //データセグメントの長さ：2byte
        //ロボット生存データ
        //※1Hz周波数の周期
        robotSurvivalStatus.robot_legion =
            (data[DATAOFFSET + 1] << 8) | (data[DATAOFFSET + 0]);

      } else if (cmdId == 0x0101) {
        //データセグメントの長さ：4byte
        //フィールドイベントデータ
        //※イベントは発生したときに発信
        fieldEvent.event_type =
            (data[DATAOFFSET + 3] << 24) | (data[DATAOFFSET + 2] << 16) |
            (data[DATAOFFSET + 1] << 8) | (data[DATAOFFSET + 0]);

      } else if (cmdId == 0x0102) {
        //データセグメントの長さ：4byte
        //補給施設アクションデータ
        //アクション発生後に発信
        supplierActionData.supplier_id = (data[DATAOFFSET + 0]);
        supplierActionData.supply_robot_id = (data[DATAOFFSET + 1]);
        supplierActionData.supply_step = (data[DATAOFFSET + 2]);
        supplierActionData.quantity_projectile = (data[DATAOFFSET + 3]);

      } else if (cmdId == 0x0103) {
        //データセグメントの長さ：3byte
        //リクエスト補給施設
        //※補給施設upper10Hz
        requestsupplierData.supplier_id = (data[DATAOFFSET + 0]);
        requestsupplierData.supply_robot_id = (data[DATAOFFSET + 1]);
        requestsupplierData.quantity = (data[DATAOFFSET + 2]);

      } else if (cmdId == 0x0201) {
        //データセグメントの長さ：15byte
        //ロボットの状態
        //※10Hz

        robotStatus.robot_id = (data[DATAOFFSET + 0]);
        robotStatus.robot_level = (data[DATAOFFSET + 1]);
        robotStatus.remainHP =
            ((data[DATAOFFSET + 3]) << 8) | (data[DATAOFFSET + 2]);
        robotStatus.maxHP =
            ((data[DATAOFFSET + 5]) << 8) | (data[DATAOFFSET + 4]);
        robotStatus.shooterHeat0_coolingrate =
            ((data[DATAOFFSET + 7]) << 8) | (data[DATAOFFSET + 6]);
        robotStatus.shooterHeat0_coolinglimit =
            ((data[DATAOFFSET + 9]) << 8) | (data[DATAOFFSET + 8]);
        robotStatus.shooterHeat1_coolingrate =
            ((data[DATAOFFSET + 11]) << 8) | (data[DATAOFFSET + 10]);
        robotStatus.shooterHeat1_coolinglimit =
            ((data[DATAOFFSET + 13]) << 8) | (data[DATAOFFSET + 12]);
        robotStatus.mains_power_gimbal_output =
            (data[DATAOFFSET + 14]) & 0b00000001;
        robotStatus.mains_power_chassis_output =
            ((data[DATAOFFSET + 14]) & 0b00000010) >> 1;
        robotStatus.mains_power_shooter_output =
            ((data[DATAOFFSET + 14]) & 0b00000100) >> 2;

      } else if (cmdId == 0x0202) {
        //データセグメントの長さ：14byte
        //リアルタイム出力、熱量データ
        //※50Hz周波数の周期で発信
        realtimePowerData.chassisVolt = ((data[DATAOFFSET + 1]) << 8) | (data[DATAOFFSET + 0]);
        realtimePowerData.chassisCurrent = ((data[DATAOFFSET + 3]) << 8) | (data[DATAOFFSET + 2]);
        wk.l = ((data[DATAOFFSET + 7] << 24) | (data[DATAOFFSET + 6] << 16) |
                (data[DATAOFFSET + 5] << 8) | (data[DATAOFFSET + 4]));
        realtimePowerData.chassisPower = wk.f;
        realtimePowerData.chassisPowerBuffer =
            ((data[DATAOFFSET + 9]) << 8) | (data[DATAOFFSET + 8]);
        realtimePowerData.shooterHeat0 =
            ((data[DATAOFFSET + 11]) << 8) | (data[DATAOFFSET + 10]);
        realtimePowerData.shooterHeat1 =
            ((data[DATAOFFSET + 13]) << 8) | (data[DATAOFFSET + 12]);

      } else if (cmdId == 0x0203) {
        //データセグメントの長さ：16byte
        //ロボットの場所と銃口向きの情報
        //※50Hz周波数の周期で発信
        wk.l = (data[DATAOFFSET + 3] << 24) | (data[DATAOFFSET + 2] << 16) |
               (data[DATAOFFSET + 1] << 8) | (data[DATAOFFSET + 0]);
        positionData.x = wk.f;
        wk.l = (data[DATAOFFSET + 7] << 24) | (data[DATAOFFSET + 6] << 16) |
               (data[DATAOFFSET + 5] << 8) | (data[DATAOFFSET + 4]);
        positionData.y = wk.f;
        wk.l = (data[DATAOFFSET + 11] << 24) | (data[DATAOFFSET + 10] << 16) |
               (data[DATAOFFSET + 9] << 8) | (data[DATAOFFSET + 8]);
        positionData.z = wk.f;
        wk.l = (data[DATAOFFSET + 15] << 24) | (data[DATAOFFSET + 14] << 16) |
               (data[DATAOFFSET + 13] << 8) | (data[DATAOFFSET + 12]);
        positionData.yaw = wk.f;
      } else if (cmdId == 0x0204) {
        //データセグメントの長さ：1byte
        //ロボットバフデータ
        //※情報更新時に発信
        buffMusk.power_rune_buff = (data[DATAOFFSET + 0]);

      } else if (cmdId == 0x0205) {
        //データセグメントの長さ　: 2byte
        //空中ロボットエネルギーステータス
        // 10Hzで発信
        aerialEnergyData.energy_point = (data[DATAOFFSET + 0]);
        aerialEnergyData.attack_time = (data[DATAOFFSET + 1]);

      } else if (cmdId == 0x0206) {
        //データセグメントの長さ　1byte
        //ロボットダメージデータ
        //ダメージ判定後に発信
        robotHurt.armor_id = data[DATAOFFSET + 0] & 0x0F;
        robotHurt.hurtType = (data[DATAOFFSET + 0] & 0xF0) >> 4;
      } else if (cmdId == 0x0207) {
        //データセグメントの長さ　6byte
        //リアルタイム射撃データ
        //射撃時に発信
        realtimeShootData.bulletType = data[DATAOFFSET + 0];
        realtimeShootData.bulletFreq = data[DATAOFFSET + 1];
        wk.l = (data[DATAOFFSET + 5] << 24) | (data[DATAOFFSET + 4] << 16) |
               (data[DATAOFFSET + 3] << 8) | (data[DATAOFFSET + 2]);
        realtimeShootData.bulletSpeed = wk.f;
      }

      // uint16_t limitShooterHeat0 = 90;
      // switch (robotStatus.robot_level) {
      //   case 1:
      //     limitShooterHeat0 = 90;
      //     break;
      //   case 2:
      //     limitShooterHeat0 = 180;
      //     break;
      //   case 3:
      //     limitShooterHeat0 = 360;
      //     break;
      // }

      // //カスタムデータ発信
      // unsigned char customdataPacket[28];
      // float data = (float)robotStatus.remainHP;
      // setData1(data);
      // makeCustomDataPacket(customdataPacket);
      // Serial.print("--->0x");
      // int i = 0;
      // for (i = 0; i < 28; i++) {
      //   Serial.print(customdataPacket[i], HEX);
      // }
      // Serial.println();
      // JUDGESYSTEM_SERIAL.write(customdataPacket, 28);

      //////////////////
      if (DEBUG_PRINT_JUDGE) {
        printAll();
      }
    }
  }

  // if (countRecMiss > 10000) {
  //   Serial.println("JudgeSystem Not Avalable ");
  //   Serial.print("\t miss = ");
  //   Serial.println(countRecMiss);
  //   dataNumber = 0;

  // } else {
  //   countRecMiss++;
  // }
}

void judgeSystem2020::printAll(){
  Serial.println("<0x0001 CompetitionStatus>");
  Serial.print("gametype:");
  Serial.print(competitionStatus.gametype);
  Serial.print("\t");
  Serial.print("gameProgress:");
  Serial.print(competitionStatus.gameprogress);
  Serial.print("\t");
  Serial.print("stageRemainTime:");
  Serial.print(competitionStatus.stageRemainTime);
  Serial.println("");

  Serial.println("<0x0002 gameResult>");
  Serial.print("winner:");
  Serial.print(gameResult.winner);
  Serial.println("");

  Serial.println("<0x0003 robotSurvivalStatus>");
  Serial.print("robotSurvivalStatus:");
  Serial.print(robotSurvivalStatus.robot_legion);
  Serial.println("");

  Serial.println("<0x0101 fieldEvent>");
  Serial.print("FieldEvent:");
  Serial.print(fieldEvent.event_type);
  Serial.println("");

  Serial.println("<0x0102 supplierActionData>");
  Serial.print("supplier_id:");
  Serial.print(supplierActionData.supplier_id);
  Serial.print("\t");
  Serial.print("supply_robot_id:");
  Serial.print(supplierActionData.supply_robot_id);
  Serial.print("\t");
  Serial.print("supply_step:");
  Serial.print(supplierActionData.supply_step);
  Serial.print("\t");
  Serial.print("quantity:");
  Serial.print(supplierActionData.quantity_projectile);
  Serial.println("");

  Serial.println("<0x0103 requestsupplierData>");
  Serial.print("supply_robot_id:");
  Serial.print(requestsupplierData.supplier_id);
  Serial.print("\t");
  Serial.print("supply_step:");
  Serial.print(requestsupplierData.supply_robot_id);
  Serial.print("\t");
  Serial.print("quantity:");
  Serial.print(requestsupplierData.quantity);
  Serial.println("");

  Serial.println("<0x0201 RobotStatusData>");
  Serial.print("robot_id:");
  Serial.print(robotStatus.robot_id);
  Serial.print("\t");
  Serial.print("robot_level:");
  Serial.print(robotStatus.robot_level);
  Serial.print("\t");
  Serial.print("remainHP:");
  Serial.print(robotStatus.remainHP);
  Serial.print("\t");
  Serial.print("maxHP:");
  Serial.print(robotStatus.maxHP);
  Serial.println("");
  Serial.print("17mm_CoolingRate:");
  Serial.print(robotStatus.shooterHeat0_coolingrate);
  Serial.print("\t");
  Serial.print("17mm_CoolingLimit:");
  Serial.print(robotStatus.shooterHeat0_coolinglimit);
  Serial.print("\t");
  Serial.print("42mm_CoolingRate:");
  Serial.print(robotStatus.shooterHeat1_coolingrate);
  Serial.print("\t");
  Serial.print("42mm_CoolingLimit:");
  Serial.print(robotStatus.shooterHeat1_coolinglimit);
  Serial.println("");
  Serial.print("PowerOutputEnable Gimbal:");
  Serial.print(robotStatus.mains_power_gimbal_output);
  Serial.print("\t");
  Serial.print("Chassis:");
  Serial.print(robotStatus.mains_power_chassis_output);
  Serial.print("\t");
  Serial.print("Shooter:");
  Serial.print(robotStatus.mains_power_shooter_output);
  Serial.println("");

  Serial.println("<0x0202 realtimePowerData>");
  Serial.print("chassisVolt:");
  Serial.print(realtimePowerData.chassisVolt * 0.001);
  Serial.print("[V]");
  Serial.print("\t");
  Serial.print("chassisCurrent:");
  Serial.print(realtimePowerData.chassisCurrent * 0.001);
  Serial.print("[A]");
  Serial.print("\t");
  Serial.print("chassisPower:");
  Serial.print(realtimePowerData.chassisPower);
  Serial.print("[W]");
  Serial.println("");
  Serial.print("chassisPowerBuffer:");
  Serial.print(realtimePowerData.chassisPowerBuffer);
  Serial.print("[J]");
  Serial.print("\t");
  Serial.print("shooterHeat0:");
  Serial.print(realtimePowerData.shooterHeat0);
  Serial.print("\t");
  Serial.print("shooterHeat1:");
  Serial.print(realtimePowerData.shooterHeat1);
  Serial.println("");

  Serial.println("<0x0203 positionData>");
  Serial.print("x:");
  Serial.print(positionData.x);
  Serial.print("[m]");
  Serial.print("\t");
  Serial.print("y:");
  Serial.print(positionData.y);
  Serial.print("[m]");
  Serial.print("\t");
  Serial.print("z:");
  Serial.print(positionData.z);
  Serial.print("[m]");
  Serial.print("\t");
  Serial.print("yaw:");
  Serial.print(positionData.yaw);
  Serial.print("[deg]");
  Serial.println("");

  Serial.println("<0x0204 buffMusk>");
  Serial.print("power_rune_buff:");
  Serial.print(buffMusk.power_rune_buff);
  Serial.println("");

  Serial.println("<0x0205 aerialEnergyData>");
  Serial.print("energy_point:");
  Serial.print(aerialEnergyData.energy_point);
  Serial.print("\t");
  Serial.print("attack_time:");
  Serial.print(aerialEnergyData.attack_time);
  Serial.println("");

  Serial.println("<0x0206 robotHurt>");
  Serial.print("armor_id:");
  Serial.print(robotHurt.armor_id);
  Serial.print("\t");
  Serial.print("hurtType:");
  Serial.print(robotHurt.hurtType);
  Serial.println("");

  Serial.println("<0x0207 realtimeShootData>");
  Serial.print("bulletType:");
  Serial.print(realtimeShootData.bulletType);
  Serial.print("\t");
  Serial.print("bulletFreq:");
  Serial.print(realtimeShootData.bulletFreq);
  Serial.print("\t");
  Serial.print("bulletSpeed:");
  Serial.print(realtimeShootData.bulletSpeed);
  Serial.println("");

  Serial.println(" ");
  Serial.println(" ");

  // CRC---------------------------------------------------------------
  //      Serial.println("<CRC16>");
  //      Serial.print("CRC16:");
  //      Serial.print(crc16Value, HEX);

  //      Serial.println("");
}
