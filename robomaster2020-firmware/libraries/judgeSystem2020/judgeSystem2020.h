#ifndef judgeSystem2020_h
#define judgeSystem2020_h

#include "Arduino.h"
#include "CRC.h"
#include "CustomData.h"

class judgeSystem2020 {
 public:
  judgeSystem2020();
  void getJudgeSystemData();
  void printAll();
  unsigned int dataNumber;
  unsigned int data[90];
  unsigned char header[4];
  union {
    float f;
    unsigned long l;
  } wk;

  typedef struct {
    uint8_t sof;
    uint16_t dataLength;
    uint8_t seq;
    uint8_t crc8Value;
    uint16_t cmdId;
  } extFrameHeader_t;
  extFrameHeader_t frameHeader;

  typedef struct {
    uint8_t gametype;
    uint8_t gameprogress;
    uint16_t stageRemainTime;
  } extCompetition_status_t;
  extCompetition_status_t competitionStatus;

  typedef struct {
    uint8_t winner;
  } extGameResult_t;
  extGameResult_t gameResult;

  typedef struct {
    uint16_t robot_legion;
  } ext_robot_survivors_t;
  ext_robot_survivors_t robotSurvivalStatus;

  typedef struct {
    uint32_t event_type;
  } ext_field_event_t;
  ext_field_event_t fieldEvent;

  typedef struct {
    uint8_t supplier_id;
    uint8_t supply_robot_id;
    uint8_t supply_step;
    uint8_t quantity_projectile;
  } ext_supply_projectile_t;
  ext_supply_projectile_t supplierActionData;

  typedef struct {
    uint8_t supplier_id;
    uint8_t supply_robot_id;
    uint8_t quantity;
  } ext_supply_projectile_booking_t;
  ext_supply_projectile_booking_t requestsupplierData;

  typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remainHP;
    uint16_t maxHP;
    uint16_t shooterHeat0_coolingrate;
    uint16_t shooterHeat0_coolinglimit;
    uint16_t shooterHeat1_coolingrate;
    uint16_t shooterHeat1_coolinglimit;
    uint8_t mains_power_gimbal_output;
    uint8_t mains_power_chassis_output;
    uint8_t mains_power_shooter_output;
  } ext_robot_status_data_t;
  ext_robot_status_data_t robotStatus;

  typedef struct {
    uint16_t chassisVolt;
    uint16_t chassisCurrent;
    float chassisPower;
    uint16_t chassisPowerBuffer;
    uint16_t shooterHeat0;
    uint16_t shooterHeat1;
  } ext_realtime_power_data_t;
  ext_realtime_power_data_t realtimePowerData;

  typedef struct {
    float x;
    float y;
    float z;
    float yaw;
  } ext_robot_position_data_t;
  ext_robot_position_data_t positionData;

  typedef struct {
    uint8_t power_rune_buff;
  } ext_buff_musk_t;
  ext_buff_musk_t buffMusk;

  typedef struct {
    uint8_t energy_point;
    uint8_t attack_time;
  } ext_aerial_robot_energy_t;
  ext_aerial_robot_energy_t aerialEnergyData;

  typedef struct {
    uint8_t armor_id;
    uint8_t hurtType;
  } extRobotHurt_t;
  extRobotHurt_t robotHurt;

  typedef struct {
    uint8_t bulletType;
    uint8_t bulletFreq;
    float bulletSpeed;
  } ext_shoot_data_t;
  ext_shoot_data_t realtimeShootData;

 private:
  const bool DEBUG_PRINT_JUDGE = false;
  const int DATAOFFSET = 7;
  int countRecMiss = 0;
};

#endif
