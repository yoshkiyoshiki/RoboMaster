#include "Chassis.h"
#include "Receiver.h"
#include <Metro.h>
#include "M3508.h"
#include "M2006.h"
#include "mymath.h"
#include <CANManager.h>
#include "Parameters.h"

//モーターはプラスで逆転

#define KEYBOARD    1
#define CONTROLLER  0
#define PI_NUM 3.14159265
#define G (9.80665 * 1000)
#define Z_GEAR_RADIUS 24.0
#define BOX_SIZE  210.0
#define REDUCTION_RATIO (3591.0 / 187.0)//M3508の減速比
#define BOX_FALL_HEIGHT 1200.0  //資源島のボックスがある高さ
#define BOX_FALL_DETECT_HEIGHT  1200.0 //資源島から落ちてくるボックスを検出し始める位置
#define MOTOR_MOVE_ASAP 200
#define MOTOR_STOP_ASAP 100
#define EXPANDED_HEIGHT 760.0
#define DOWN_HEIGHT     470.5
#define LIMIT(X,Y) ((X>0) ? ((X>Y)?Y:X) : ((X<-1*Y)?-1*Y:X))
#define JUDGEZERORPM(X, Y) ((abs(X) > abs(Y)) ? 0 : 1)
#define SPEED_LIMIT   2
#define CHANGE_SPEED  0.1
#define CHANGE_RAD    ((rcv.joy.rightX + (rcv.mouse.x*2)) * 3.0/660.0)
#define LITTLE_Z_MOVE 10000
#define MOTORRPM  1000.0  // (MOTOR_RPM / 1000 + 0.3)(Aアンペア)流れる（フリー時） ※200で0.5A 300で0.6Aかかる
#define CLAMP_CURRENT  3.0  //何アンペアで締め付けるか
#define CLAMP_MODE_CURRENT  ((MOTORRPM / 1000.0) + 0.3) + 0.2    //定電流で締め付け始めるときの電流検出（.0をつけないと整数の計算とみなされて切り下げられる！！！）
#define SAMPLING_NUM  10  //何個データを取って信頼値とするか
#define LITTLE_LOW_TIME 65
#define LITTLE_HIGH_TIME  100
#define LITTLE_BACK_TIME 0
#define KP  700 //比例ゲイン
#define KI  70 //積分ゲイン

enum
{
   y_motor = 0,
   z_motor_right = 1,
   z_motor_left = 2,
   arm_motor = 3,
   joint_right = 4,
   joint_left = 5,
   camera_motor = 6
};

enum
{
  y_out = 0,
  y_in = 1,
  z_low = 2,
  z_high = 3,
  z_little_high = 4,
  z_little_low = 5,
  gravity_fall = 6,
  catch_box = 7,
  release_box = 8,
  front_joint = 9,
  back_joint = 10,
  y_little_back = 11,
  stop_move = 50
};

enum
{
  mec_para = 0,
  speed_para = 1,
  position_para = 2  
};

enum
{
  kp = 0,
  ki = 1,
  kd = 2 
};
//getboxAuto
enum
{
  init_mode = -1,
  z_up_y_out = 0,
  joint_front = 1,
  z_little_down = 2,
  box_catch = 3,
  z_little_up = 4,
  joint_back = 5, 
  box_release = 6,
  z_down_y_in = 7,
  zup_jointfront_yout = 8,
  z_down = 9,
  boxrelease_jointback_zdown_yin = 10,
  fall_gravity = 11,
  box_catch_y_out = 12,
  y_out_joint_front = 13,
  joint_back_y_in_z_down = 14,
  little_joint_up = 15,
  emergency = 16
};

//camera
enum
{
  camera_front = 1,
  camera_back = 2
};

//subbord_motor
enum
{
  motor_control1 = 0,
  motor_control2 = 1,
  chassisSpeedX = 2,
  chassisSpeedY = 3,
  omega = 4,
  obstacle = 5
};

enum
{
  toggle_left1 = 0,
  toggle_left2 = 1,
  front = 2,
  back = 3,
  right = 4,
  left = 5,
  small_resource_island = 6,
  resource_island = 7,
  put_exchange_state = 8,
  sirahadori_state = 9,
  object_state = 10,
  RFID_state1 = 11,
  RFID_state2 = 12
};


CANManager* canmanager = CANManager::getInstance();//返されたアドレスを*を使用して実際にアクセスしてる。

Chassis chassis;
Receiver rcv;
Metro ControlCycle = Metro(10); // 制御周期計算間隔(ms)
Metro Debug = Metro(10);
elapsedMillis boxdetect;
elapsedMillis little_back;
elapsedMillis little_high;
elapsedMillis emergence_time;
elapsedMillis measure;
elapsedMillis accelerateX;
elapsedMillis accelerateY;
elapsedMillis accelerateRad;
elapsedMillis accelerateObstacle;
elapsedMillis notGetBox;

//                             y_motor     z_right    z_left    arm   joint_right  joint_left
M3508_POSITIONCONTROL motor[MOTOR_NUM] = {{0.01,0}, {0.01,0}, {0.01,0}, {0.01,0}, {0.01,0}, {0.01,0}};  //CAN1
M2006_POSITIONCONTROL CameraMotor(0.01, 0);  

int inkey = 0;
int sampling_count = 0;
int falling_box_reach_time = 0;
double obstacle_speed = 0;
double xSpeed = 0;
double ySpeed = 0;
double rad = 0;
double box_fall_rpm = 0;
double box_fall_time = 0;
double box_fall_distance = 0;
double box_fall_speed = 0;
int16_t before_joint_right = 0;
int16_t before_joint_left = 0;
int motor_rpm[MOTOR_NUM + 1] = {2500, 3500, 3500, 2500/*1000*/, 2500, 2500, 100};
int16_t ampare[MOTOR_NUM + 1] = {0, 0, 0, 0, 0, 0, 0};// 実際のデータを格納。
double current_diff[MOTOR_NUM + 1];
double kpGain[MOTOR_NUM + 1] = {700.0, 700.0, 700.0, 500.0, 700.0, 700.0, 700.0};
double kiGain[MOTOR_NUM + 1] = {70.0, 70.0, 70.0, 30.0, 70.0, 70.0, 70.0};
double clampModeCurrent[MOTOR_NUM + 1] = {9.0, 10.0, 10.0, 9.0, 10.0, 10.0, 2.0};
double clampCurrent[MOTOR_NUM + 1] = {12.0, 11.0, 11.0, 17.0, 15.0, 15.0, 3.0};
int z_low_time = 0;
int8_t subbord_state[8] = {0, 0, 0, 0, 0, 0, 0, 0};// subbord[0]:subbordのモータを動かすための指令を格納する 7: revive 6: revive_rev 5: obstacle 4: obstacle_rev 3: chassis_left 2: chassis_right 1: chassis_back 0: chassis_front
bool box_catch_clamp = false;
bool box_release_clamp = false;
bool joint_back_clamp = false;
bool joint_front_clamp = false;
bool controller_flg[13] = {false};
/*bool y_out = false;
bool y_in = false;*/

void setup() {
  
  rcv.init();
  delay(2000);
  canmanager->init(true,true,true);
  canmanager->restartCan();
  Serial.begin(115200);

  pinMode(32,OUTPUT);
  pinMode(6,INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  //pinMode(5, OUTPUT);

  delay(2000);
  ControlCycle.reset();
  
  setNeutralAll();

  setPIDParaAll();
}

bool first_init = false;

int state[MOTOR_NUM + 1] = {stop_move, stop_move, stop_move, stop_move, stop_move, stop_move, camera_front};
int smallGetboxAutoState = init_mode;
int getboxAutoState = init_mode;
int putExchangeState = init_mode;
int sirahadoriAutoState = init_mode;
int emergency_state = init_mode;
int clampFlg[MOTOR_NUM + 1] = {0, 0, 0, 0, 0, 0, 0};
bool initFlg = false;

int16_t add_current[MOTOR_NUM + 1] = {0, 0, 0, 0, 0, 0, 0};

bool controller = CONTROLLER;
bool w = false;
bool s = true;//trueにして電源投入時にモーターを初期の状態にする
bool e = false;
bool d = false;
bool r = false;
bool f = true;//trueにして電源投入時にモーターを初期の状態にする
bool t = false;
bool g = true;//trueにして電源投入時にモーターを初期の状態にする
bool y = false;
bool h = false;
bool p = false;

bool falling_detect = false;
bool sirahadori_start = false;
bool detectFlg = false;

bool soonPIDPreventFlg[MOTOR_NUM + 1] = {false, false, false, false, false, false, false};

bool clampMode[MOTOR_NUM + 1] = {false, false, false, false, false, false, false};
bool getboxAutoFlg = false;
bool putExchangeZone = false;
int smallGetboxAuto = false;
bool zeroRPMFlg[MOTOR_NUM + 1] = {false, false, false, false, false, false, false};

double current[MOTOR_NUM + 1];
double current_judge[MOTOR_NUM + 1];

bool keypushflag[255] = {true};    //キーボードのスイッチフラグ群
uint8_t keycountflag[255] = {0};  //キーボードの押し回数フラグ群

void loop() {

  canmanager->getAllCANdata();
  getM3508Datas();  //足回りのデータ取得
  getM2006Datas();
  //digitalWrite(5,HIGH);
  
  rcv.getRecvData();


  double output_MperSec[MOTOR_NUM] = {0,0,0,0,0,0};

  /*if(Serial.available())
  {
     inkey = Serial.read();
  }*/
  //controller = CONTROLLER;
  controller = KEYBOARD;
  controller_task(controller);

  if(!first_init)
  {
    if((clampMode[y_motor] == true) && (clampMode[arm_motor] == true) && (clampMode[joint_right] == true) && (clampMode[joint_left] == true))
    {
      s = true;
      f = false;
      g = false;
      first_init = true;
    }
  }
  else
  {
    if(controller_flg[toggle_left1] || controller)
    {
      if(subbord_state[chassisSpeedY] > 2)//前方向
      {
          subbord_state[motor_control1] &= 0xfd;
          subbord_state[motor_control1] |= 0x01;
          if(keypushflag[7])
          {
            state[camera_motor] = camera_front;
            soonPIDPreventFlg[camera_motor] = true;
            current_diff[camera_motor] = 0;
            keypushflag[7] = false;
          }
      }
      else if(subbord_state[chassisSpeedY] < -2)//後ろ方向
      {
          subbord_state[motor_control1] &= 0xfe;
          subbord_state[motor_control1] |= 0x02;
          if(keypushflag[7])
          {
            state[camera_motor] = camera_back;
            soonPIDPreventFlg[camera_motor] = true;
            current_diff[camera_motor] = 0;
          }
      }
      else
      {
          subbord_state[motor_control1] &= 0xfc;
          keypushflag[7] = true;
      }
      
      if(subbord_state[chassisSpeedX] > 2)//右方向
      {
          subbord_state[motor_control1] &= 0xf7;
          subbord_state[motor_control1] |= 0x04;
      }
      else if(subbord_state[chassisSpeedX] < -2)//左方向
      {
          subbord_state[motor_control1] |= 0x08;
          subbord_state[motor_control1] &= 0xfb;
      }
      else
      {
          subbord_state[motor_control1] &= 0xf3;
      }
    
      if(subbord_state[omega] < -2)//旋回
      {
          subbord_state[motor_control2] &= 0xfd;
          subbord_state[motor_control2] |= 0x01;
      }
      else if(subbord_state[omega] > 2)//旋回
      {
          subbord_state[motor_control2] &= 0xfe;
          subbord_state[motor_control2] |= 0x02;
      }
      else
      {
          subbord_state[motor_control2] &= 0xf0;
      }
    }
    
    if(controller_flg[toggle_left2] || controller)
    {
      if(controller_flg[RFID_state1])//receive
      {
        if(keypushflag[5])
        {
          if(subbord_state[motor_control1] & 0x80)
          {
            subbord_state[motor_control1] &= 0x7f;
          }
          else
          {
            subbord_state[motor_control1] &= 0xbf;
            subbord_state[motor_control1] |= 0x80;
          }
          keypushflag[5] = false;
        }
      }
      else
      {
         keypushflag[5] = true;
      }
      
      if(controller_flg[RFID_state2])//receive
      {
        if(keypushflag[6])
        {
          if(subbord_state[motor_control1] & 0x40)
          {
            subbord_state[motor_control1] &= 0xbf;
          }
          else
          {
            subbord_state[motor_control1] &= 0x7f;
            subbord_state[motor_control1] |= 0x40;
          }
          keypushflag[6] = false;
        }
      }
      else
      {
         keypushflag[6] = true;
      }
  
      
      if(subbord_state[obstacle] > 5)//obstacle
      {
         subbord_state[motor_control1] |= 0x20;
      }
      else if(subbord_state[obstacle] < -5)//obstacle
      {
         subbord_state[motor_control1] |= 0x10;
      }
      else
      {
        subbord_state[motor_control1] &= 0xcf;
      }
      
      if(controller_flg[sirahadori_state] && !(sirahadoriAutoState == emergency))
      {
          controller_flg[sirahadori_state] = false;
          if(p == true)
          {
            p = false;
            soonPIDPreventFlg[y_motor] = true;
            soonPIDPreventFlg[z_motor_right] = true;
            soonPIDPreventFlg[z_motor_left] = true;
            soonPIDPreventFlg[arm_motor] = true;
            soonPIDPreventFlg[joint_right] = true;
            soonPIDPreventFlg[joint_left] = true;
            sirahadoriAutoState = emergency;
            emergency_state = init_mode;
            sirahadori_start = false;
          }
          else
          {
            p = true;  
          }
          
          falling_detect = false;
          
          if(e == true) 
          {
            e = false;
            soonPIDPreventFlg[z_motor_right] = true;
            soonPIDPreventFlg[z_motor_left] = true;
            current_diff[z_motor_right] = 0;
            current_diff[z_motor_left] = 0;
          }
          else if(d == true) 
          {
            d = false;
            soonPIDPreventFlg[z_motor_right] = true;
            soonPIDPreventFlg[z_motor_left] = true;
            current_diff[z_motor_right] = 0;
            current_diff[z_motor_left] = 0;
          }
          else if(h == true)
          {
            h = false;
          }
      }
      
      if(controller_flg[small_resource_island] && !(smallGetboxAutoState == emergency))
      {
        controller_flg[small_resource_island] = false;
          if(smallGetboxAuto == true)
          {
            smallGetboxAutoState = emergency;
            emergency_state = init_mode;
            emergence_time = 0;
            soonPIDPreventFlg[y_motor] = true;
            soonPIDPreventFlg[z_motor_right] = true;
            soonPIDPreventFlg[z_motor_left] = true;
            soonPIDPreventFlg[arm_motor] = true;
            soonPIDPreventFlg[joint_right] = true;
            soonPIDPreventFlg[joint_left] = true;
            smallGetboxAuto = false;
          }
          else
          {
            smallGetboxAuto = true;
            smallGetboxAutoState = init_mode;
          }
      }
      
      if(controller_flg[resource_island] && !(getboxAutoState == emergency))
      {
          controller_flg[resource_island] = false;
          if(getboxAutoFlg == true)
          {
            getboxAutoState = emergency;
            emergency_state = init_mode;
            emergence_time = 0;
            soonPIDPreventFlg[y_motor] = true;
            soonPIDPreventFlg[z_motor_right] = true;
            soonPIDPreventFlg[z_motor_left] = true;
            soonPIDPreventFlg[arm_motor] = true;
            soonPIDPreventFlg[joint_right] = true;
            soonPIDPreventFlg[joint_left] = true;
            getboxAutoFlg = false;
          }
          else
          {
            getboxAutoFlg = true;
            getboxAutoState = init_mode;
          }
      }
      
      if(controller_flg[put_exchange_state] && !(putExchangeState == emergency))
      {
          controller_flg[put_exchange_state] = false;
          if(putExchangeZone == true)
          {
            putExchangeState = emergency;
            emergency_state = init_mode;
            emergence_time = 0;
            soonPIDPreventFlg[y_motor] = true;
            soonPIDPreventFlg[z_motor_right] = true;
            soonPIDPreventFlg[z_motor_left] = true;
            soonPIDPreventFlg[arm_motor] = true;
            soonPIDPreventFlg[joint_right] = true;
            soonPIDPreventFlg[joint_left] = true;
            putExchangeZone = false;
          }
          else
          {
            putExchangeZone = true;
            putExchangeState = init_mode;
          }
      }
    }
  }
  /*-----------------------------------------------------------mainbordキーボード制御-----------------------------------------------------------------*/
  if(inkey == 'w')
  {
    if(w == true) w = false;
    else          w = true; 

    if(s == true) 
    {
      s = false;
      soonPIDPreventFlg[y_motor] = true;
      current_diff[y_motor] = 0;
    }
  }
  if(inkey == 's')
  {
    if(s == true) s = false;
    else          s = true;  
    
    if(w == true)
    {
      w = false;
      soonPIDPreventFlg[y_motor] = true;
      current_diff[y_motor] = 0;
    }
  }
  if(inkey == 'e')
  {
    if(e == true) e = false;
    else          e = true;  
    
    if(d == true) 
    {
      d = false;
      soonPIDPreventFlg[z_motor_right] = true;
      soonPIDPreventFlg[z_motor_left] = true;
      current_diff[z_motor_right] = 0;
      current_diff[z_motor_left] = 0;
    }
    else if(p == true) 
    {
      p = false;
      soonPIDPreventFlg[z_motor_right] = true;
      soonPIDPreventFlg[z_motor_left] = true;
      current_diff[z_motor_right] = 0;
      current_diff[z_motor_left] = 0;
    }
    else if(h == true)
    {
      h = false;
    }
  }
  if(inkey == 'd')
  {
    if(d == true) d = false;
    else          d = true;  
    
    if(e == true) 
    {
      e = false;
      soonPIDPreventFlg[z_motor_right] = true;
      soonPIDPreventFlg[z_motor_left] = true;
      current_diff[z_motor_right] = 0;
      current_diff[z_motor_left] = 0;
    }
    else if(p == true) 
    {
      p = false;
      soonPIDPreventFlg[z_motor_right] = true;
      soonPIDPreventFlg[z_motor_left] = true;
      current_diff[z_motor_right] = 0;
      current_diff[z_motor_left] = 0;
    }
    else if(h == true)
    {
      h = false;
    }
  }
  if(inkey == 'p')
  {
    if(p == true)
    {
      p = false;
      soonPIDPreventFlg[y_motor] = true;
      soonPIDPreventFlg[z_motor_right] = true;
      soonPIDPreventFlg[z_motor_left] = true;
      soonPIDPreventFlg[arm_motor] = true;
      soonPIDPreventFlg[joint_right] = true;
      soonPIDPreventFlg[joint_left] = true;
      sirahadoriAutoState = emergency;
      sirahadori_start = false;
    }
    else
    {
      p = true;  
    }
    
    falling_detect = false;
    
    if(e == true) 
    {
      e = false;
      soonPIDPreventFlg[z_motor_right] = true;
      soonPIDPreventFlg[z_motor_left] = true;
      current_diff[z_motor_right] = 0;
      current_diff[z_motor_left] = 0;
    }
    else if(d == true) 
    {
      d = false;
      soonPIDPreventFlg[z_motor_right] = true;
      soonPIDPreventFlg[z_motor_left] = true;
      current_diff[z_motor_right] = 0;
      current_diff[z_motor_left] = 0;
    }
    else if(h == true)
    {
      h = false;
    }
  }

//  if(inkey == 'i')
//  {
//    if(putExchangeZone == true)
//    {
//      putExchangeState = emergency;
//    }
//    else
//    {
//      putExchangeZone = true;
//      putExchangeState = init_mode;
//    }
//  }
  
  if(inkey == 'r')
  {
    measure = 0;
    if(r == true) r = false;
    else          r = true;  
    
    if(f == true)
    {
      f = false;
      soonPIDPreventFlg[arm_motor] = true;
      current_diff[arm_motor] = 0;
    }
  }
  if(inkey == 'f')
  {
    if(f == true) f = false;
    else          f = true;  
    
    if(r == true)
    {
      r = false;
      soonPIDPreventFlg[arm_motor] = true;
      current_diff[arm_motor] = 0;
    }
  }
  if(inkey == 't')
  {
    if(t == true) t = false;
    else          t = true;  
    
    if(g == true)
    {
      g = false;
      soonPIDPreventFlg[joint_right] = true;
      soonPIDPreventFlg[joint_left] = true;
      current_diff[joint_right] = 0;
      current_diff[joint_left] = 0;
    }
  }
  if(inkey == 'g')
  {
    if(g == true) g = false;
    else          g = true;  
    
    if(t == true)
    {
      t = false;
      soonPIDPreventFlg[joint_right] = true;
      soonPIDPreventFlg[joint_left] = true;
      current_diff[joint_right] = 0;
      current_diff[joint_left] = 0;
    }
  }
  if(inkey == 'y')
  {
    motor[z_motor_right].setNeutral(motor[z_motor_right].getGeneralPosition());
    motor[z_motor_left].setNeutral(motor[z_motor_left].getGeneralPosition());
    if(y == true) y = false;
    else          y = true;

    if(h == true)
    {
      h = false;
    }
  }
  if(inkey == 'h')
  {
    motor[z_motor_right].setNeutral(motor[z_motor_right].getGeneralPosition());
    motor[z_motor_left].setNeutral(motor[z_motor_left].getGeneralPosition());
    if(h == true) h = false;
    else          h = true;

    if(y == true)
    {
      y = false;
    }
    if(e == true)
    {
      e = false;
    }
    if(d == true)
    {
      d = true;
    }
  }
//  if(inkey == 'o')
//  {
//    if(getboxAutoFlg == true)
//    {
//      getboxAutoState = emergency;
//      getboxAutoFlg = false;
//    }
//    else
//    {
//      sirahadoriAutoState = 0;
//      getboxAutoFlg = true;
//      getboxAutoState = init_mode;
//    }
//  }

  //sirahadoriAutoStateをプラスする
//  if(inkey == 'b')
//  {
//
//    if(p == true)
//    {
//      sirahadoriAutoState++;
//    }
//    else
//    {
//      sirahadoriAutoState = 0;
//    }
//  }
  
  if(inkey == 'c')
  {
    if(state[camera_motor] == camera_front)
    {
      state[camera_motor] = camera_back;
    }
    else if(state[camera_motor] == camera_back)
    {
      state[camera_motor] = camera_front;
    }
    soonPIDPreventFlg[camera_motor] = true;
    current_diff[camera_motor] = 0;
  }

  /*--------------------------------------------------------------------------------------------------------------------------------------*/
  
  /*-----------------------------------------------------------subbordキーボード制御-----------------------------------------------------------------*/
  //左方向
//  if(inkey == 'm')
//  {
//    if(subbord_state[motor_control1] & 0x08)
//    {
//      subbord_state[motor_control1] &= 0xf7;
//    }
//    else
//    {
//      subbord_state[motor_control1] |= 0x08;
//    }
//  }
//  //前方向
//  if(inkey == 'b')
//  {
//    if(subbord_state[motor_control1] & 0x01)
//    {
//      subbord_state[motor_control1] &= 0xfe;
//    }
//    else
//    {
//      subbord_state[motor_control1] |= 0x01;
//    }
//  }
//
//  //後ろ方向
//  if(inkey == 'n')
//  {
//    if(subbord_state[motor_control1] & 0x02)
//    {
//      subbord_state[motor_control1] &= 0xfd;
//    }
//    else
//    {
//      subbord_state[motor_control1] |= 0x02;
//    }
//  }
//  //右方向
//  if(inkey == 'v')
//  {
//    if(subbord_state[motor_control1] & 0x04)
//    {
//      subbord_state[motor_control1] &= 0xfb;
//    }
//    else
//    {
//      subbord_state[motor_control1] |= 0x04;
//    }
//  }

  //receive
  if(inkey == 'q')
  {
    if(subbord_state[motor_control1] & 0x80)
    {
      subbord_state[motor_control1] &= 0x7f;
    }
    else
    {
      subbord_state[motor_control1] &= 0xbf;
      subbord_state[motor_control1] |= 0x80;
    }
  }

  //receive
  if(inkey == 'a')
  {
    if(subbord_state[motor_control1] & 0x40)
    {
      subbord_state[motor_control1] &= 0xbf;
    }
    else
    {
      subbord_state[motor_control1] &= 0x7f;
      subbord_state[motor_control1] |= 0x40;
    }
  }
  //obstacle
  if(inkey == 'z')
  {
    if(subbord_state[motor_control1] & 0x20)
    {
      subbord_state[motor_control1] &= 0xdf;
    }
    else
    {
      subbord_state[motor_control1] &= 0xef;
      subbord_state[motor_control1] |= 0x20;
    }
  }
  //obstacle
  if(inkey == 'x')
  {
    if(subbord_state[motor_control1] & 0x10)
    {
      subbord_state[motor_control1] &= 0xef;
    }
    else
    {
      subbord_state[motor_control1] &= 0xdf;
      subbord_state[motor_control1] |= 0x10;
    }
  }
  /*--------------------------------------------------------------------------------------------------------------------------------------*/

  if( ControlCycle.check())
  {
      if(h == true)
      {
        z_low_time++;
      }
      else
      {
        z_low_time = 0;
      }

      //白羽取り
      if(sirahadoriAutoState == fall_gravity)
      {
        if(!digitalRead(2) && detectFlg == false) //光電センサが反応したら
        {
          detectFlg = true;
          boxdetect = 0;  //タイマーを初期化
          falling_detect = true;
        }
      }
      else
      {
          detectFlg = false;
          falling_detect = false;
          boxdetect = 0;
      }
      
      if(falling_detect == true)
      {
        box_fall_distance = (pow(((boxdetect + 300) * 0.001 + sqrt(((BOX_FALL_HEIGHT - BOX_FALL_DETECT_HEIGHT) * 2) / G)), 2) * 0.5 * G); //単位はmm
        box_fall_time = ((boxdetect + 300) * 0.001 + sqrt(((BOX_FALL_HEIGHT - BOX_FALL_DETECT_HEIGHT) * 2) / G));
        box_fall_speed = sqrt(2.0 * G * box_fall_distance); //単位はmm/s
        box_fall_rpm = ((box_fall_speed * 60.0) / (2.0 * PI_NUM * Z_GEAR_RADIUS));
        box_fall_rpm = (box_fall_rpm * REDUCTION_RATIO);//減速比REDUCTION_RATIO（19）なので
        if((box_fall_distance >= (BOX_FALL_HEIGHT - EXPANDED_HEIGHT - MOTOR_MOVE_ASAP)) && (box_fall_distance <= (BOX_FALL_HEIGHT - DOWN_HEIGHT - MOTOR_STOP_ASAP)))
        {
          sirahadori_start = true;
        }
        else if(box_fall_distance > (BOX_FALL_HEIGHT - DOWN_HEIGHT - MOTOR_STOP_ASAP)) //320msくらいで下に到達するはず
        {
          sirahadori_start = false;
          falling_detect = false;
          box_fall_rpm = 0;
        }
      }

      
      /*if(rcv.flgRecvSuccess == false){
        Serial.println("fail...");
        
      }else{*/
  
        //↓キーボードとコントローラーの関数同時使用禁止！！！
        //controller_task();
        //keyboard_task();
        
      //}


        for(int i = 0; i < MOTOR_NUM; i++)
        {
          if((motor[i].getRPM() == 0) && (state[i] != stop_move)) //各ステートが何らかの動作中である場合，かつ各ステートに対応するモーターが停止している場合
          {
            zeroRPMFlg[i] = true;
          }
        }
      /*------------------------------------------白羽取りのプログラム----------------------------------------------------*/
      if(p == true)
      {
        /*for(int i = 0; i < MOTOR_NUM; i++)
        {
          if((motor[i].getRPM() == 0) && (state[i] != stop_move)) //各ステートが何らかの動作中である場合，かつ各ステートに対応するモーターが停止している場合
          {
            
            zeroRPMFlg[i] = true;
          }
        }*/

        switch(sirahadoriAutoState)
        {
          case init_mode:
               soonPIDPreventFlg[y_motor] = true;
               sirahadoriAutoState++;
               break;
          case z_up_y_out:
               d = false;
               s = false;
               e = true;
               w = true;
               if((clampMode[y_motor] == true) && (clampMode[z_motor_right] == true) && (clampMode[z_motor_left] == true) && (zeroRPMFlg[y_motor] == true) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true) && (soonPIDPreventFlg[y_motor] == false)) //各モーターが終端に達して動かない状態
               {
                 box_release_clamp = true;
                 box_catch_clamp = false;
                 sirahadoriAutoState = joint_front;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
                 soonPIDPreventFlg[joint_right] = true;
                 soonPIDPreventFlg[joint_left] = true;
               }
               break;
           case joint_front:
               if((motor[joint_right].getGeneralPosition() < -33000) && (motor[joint_left].getGeneralPosition() > 33000))
               {
                 t = false;
                 joint_front_clamp = true;
                 if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false))
                 {
                   f = false;
                   soonPIDPreventFlg[arm_motor] = true;
                   sirahadoriAutoState = fall_gravity;
                 }
               }
               else if((motor[joint_right].getGeneralPosition() < -10000) && (motor[joint_left].getGeneralPosition() > 10000))
               {
                 soonPIDPreventFlg[arm_motor] = true;
                 f = true;
               }
               else
               {
                 g = false;
                 t = true;
               }
                break;
           case fall_gravity:
               e = false;
               if(falling_detect == true)
               {
                  setPIDForSirahadori();
                  motor_rpm[arm_motor] = 4500;
                  r = true;
               }  
               if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false))
               {
                 box_catch_clamp = true;
                 box_release_clamp = false;
                 sirahadoriAutoState = joint_back;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
                 soonPIDPreventFlg[joint_right] = true;
                 soonPIDPreventFlg[joint_left] = true;
               }
               break;
            case joint_back:
               h = true;
               if((motor[joint_right].getGeneralPosition() > 60000) || (motor[joint_left].getGeneralPosition() < -60000))
               {
                 joint_front_clamp = false;
                 sirahadoriAutoState++;
                 setPIDParaAll();
                 motor_rpm[arm_motor] = 2500;
                 box_catch_clamp = false;
                 joint_front_clamp = false;
                 joint_back_clamp = true;
               }
               else if((motor[joint_right].getGeneralPosition() > 40000) && (motor[joint_left].getGeneralPosition() < -40000))
               {
                 joint_front_clamp = false;
                 joint_back_clamp = true;
                 soonPIDPreventFlg[arm_motor] = true;
                 r = false;
                 f = true;
                 g = false;
               }
               else
               {
                 t = false;
                 g = true;
               }
               break;
          case box_release://ボックスを放す
               if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && soonPIDPreventFlg[arm_motor] == false)
               {
                 f = false;
                 box_release_clamp = true;
                 box_catch_clamp = false;
                 sirahadoriAutoState++;
                 soonPIDPreventFlg[y_motor] = true;
                 /*soonPIDPreventFlg[z_motor_right] = true;
                 soonPIDPreventFlg[z_motor_left] = true;*/
               }
               break;
          case z_down_y_in:
               e = false;
               w = false;
               g = false;
               h = false;
               d = true;
               s = true;
               box_release_clamp = false;
               if((clampMode[y_motor] == true)/* && (clampMode[z_motor_right] == false) && (clampMode[z_motor_left] == false)*/ && (zeroRPMFlg[y_motor] == true) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true)/* && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true)*/ && soonPIDPreventFlg[y_motor] == false/* && soonPIDPreventFlg[z_motor_right] == false && soonPIDPreventFlg[z_motor_left] == false*/) //各モーターが終端に達して動かない状態
               {
                 sirahadoriAutoState = init_mode;
                 box_fall_distance = 0;
                 h = false;
                 d = false;
                 s = false;
                 p = false;
                 f = false;
               }
               break;
               
               
           
        }
        /*for(int i = 0; i < MOTOR_NUM; i++)
        {
          zeroRPMFlg[i] = false;
        }*/
      }

      /*------------------------------------------ボックスをエクスチェンジゾーンに置く------------------------------------------------------*/
      if(putExchangeZone == true)
      {
        /*for(int i = 0; i < MOTOR_NUM; i++)
        {
          if((motor[i].getRPM() == 0) && (state[i] != stop_move)) //各ステートが何らかの動作中である場合，かつ各ステートに対応するモーターが停止している場合
          {
            zeroRPMFlg[i] = true;
          }
        }*/
        
        switch(putExchangeState)
        {
          case init_mode:
               putExchangeState = box_catch;
               motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
               motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
               break;
          case little_joint_up:
               if((motor[joint_right].getGeneralPosition() < -2000) && (motor[joint_left].getGeneralPosition() > 2000))
               {
                 t = false;
                 soonPIDPreventFlg[arm_motor] = true;
                 putExchangeState = box_catch;
               }
               else
               {
                 t = true;
               }
               break;
          case box_catch:
               f = false;
               r = true;
               if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false)) //各モーターが終端に達して動かない状態
               {
                 box_release_clamp = false;
                 box_catch_clamp = true;
                 putExchangeState = zup_jointfront_yout;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
                 soonPIDPreventFlg[y_motor] = true;
                 soonPIDPreventFlg[joint_right] = true;
                 soonPIDPreventFlg[joint_left] = true;
               }
               break;
          case zup_jointfront_yout:
               if((motor[joint_right].getGeneralPosition() < -70000) && (motor[joint_left].getGeneralPosition() > 70000) && (clampMode[y_motor] == true) && (clampMode[z_motor_right] == true) && (clampMode[z_motor_left] == true) && (zeroRPMFlg[y_motor] == true) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true) && (soonPIDPreventFlg[y_motor] == false))
               {
                 putExchangeState = box_release;
               }
               else if((motor[joint_right].getGeneralPosition() < -34000) && (motor[joint_left].getGeneralPosition() > 34000))
               {
                 joint_front_clamp = true;
                 joint_back_clamp = false;
                 t = false;
               }
               else if((clampMode[z_motor_right] == true) && (clampMode[z_motor_left] == true))
               {
                 t = true;
               }
               else
               {
                 d = false;
                 g = false;
                 s = false;
                 w = true;
                 e = true;
               }
               break;
          case z_down:
               e = false;
               h = false;
               d = true;
               if((motor[joint_right].getGeneralPosition() > 1000) && (motor[joint_left].getGeneralPosition() < -1000))
               {
                 d = false;
                 h = true;
                 putExchangeState = box_release;
                 soonPIDPreventFlg[arm_motor] = true;
               }
               break;
          case box_release:
               r = false;
               f = true;
               if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false))
               {
                 box_release_clamp = true;
                 box_catch_clamp = false;
                 little_back = 0;
                 soonPIDPreventFlg[y_motor] = true;
                 putExchangeState = y_little_back;
               }
               break;
          case y_little_back:
               if(little_back < 1200)
               {
                 w = false;
                 s = true;
               }
               else
               {
                 little_back = 0;
                 soonPIDPreventFlg[arm_motor] = true;
                 soonPIDPreventFlg[y_motor] = true;
                 putExchangeState = box_catch_y_out;
               }
               break;
          case box_catch_y_out:
               s = false;
               w = true;
               f = false;
               r = true;
               if(little_back > 100 && little_back < 200)//backって書いてるけど前に出るやつ
               {
                 subbord_state[chassisSpeedY] = 20;
                 subbord_state[0] &= 0xfd;
                 subbord_state[0] |= 0x01;
               }
               else
               {
                 subbord_state[0] &= 0xf0;
               }
               if((clampMode[arm_motor] == true) && (clampMode[y_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (zeroRPMFlg[y_motor] == true) && (soonPIDPreventFlg[arm_motor] == false) && (soonPIDPreventFlg[y_motor] == false))
               {
                 box_release_clamp = false;
                 box_catch_clamp = true;
                 little_back = 0;
                 soonPIDPreventFlg[y_motor] = true;
                 soonPIDPreventFlg[arm_motor] = true;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
                 putExchangeState = boxrelease_jointback_zdown_yin;
               }
               break;
          case boxrelease_jointback_zdown_yin:
               if(little_back < LITTLE_BACK_TIME)
               {
                 subbord_state[chassisSpeedY] = -20;
                 subbord_state[0] &= 0xfe;
                 subbord_state[0] |= 0x02;
               }
               else
               {
                 subbord_state[0] &= 0xf0;
               }
               
               if((motor[joint_right].getGeneralPosition() > 50000) && (motor[joint_left].getGeneralPosition() < -50000) && (clampMode[y_motor] == true) && (clampMode[arm_motor] == true) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true) && (zeroRPMFlg[y_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[y_motor] == false) && (soonPIDPreventFlg[arm_motor] == false))
               {
                 putExchangeState = init_mode;
                 putExchangeZone = false;
                 d = false;
                 s = false;
                 f = false;
                 box_release_clamp = false;
               }
               else if((motor[joint_right].getGeneralPosition() > 40000) && (motor[joint_left].getGeneralPosition() < -40000))
               {
                 g = false;
                 joint_front_clamp = false;
                 joint_back_clamp = true;
               }
               else
               {
                 t = false;
                 h = false;
                 r = false;
                 w = false;
                 e = false;
                 d = true;
                 g = true;
                 s = true;
                 f = true;
               }
               break;
        }
      
       /* for(int i = 0; i < MOTOR_NUM; i++)
        {
          zeroRPMFlg[i] = false;
        }*/
      }

      /*------------------------------------------ボックスの取得を自動で行う(資源島)----------------------------------------------------*/
      if(getboxAutoFlg == true)
      {
        /*for(int i = 0; i < MOTOR_NUM; i++)
        {
          if((motor[i].getRPM() == 0) && (state[i] != stop_move)) //各ステートが何らかの動作中である場合，かつ各ステートに対応するモーターが停止している場合
          {
            zeroRPMFlg[i] = true;
          }
        }*/
        
        switch(getboxAutoState)
        {
          case init_mode:
               soonPIDPreventFlg[y_motor] = true;
               soonPIDPreventFlg[arm_motor] = true;
               soonPIDPreventFlg[z_motor_right] = true;
               soonPIDPreventFlg[z_motor_left] = true;
               soonPIDPreventFlg[joint_right] = true;
               soonPIDPreventFlg[joint_left] = true;
               motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
               motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
               getboxAutoState++;
               break;
          case z_up_y_out://同時にzを上げ，yを前に出し，腕を前に振る
               if((motor[joint_right].getGeneralPosition() < -80000) && (motor[joint_left].getGeneralPosition() > 80000) && (clampMode[y_motor] == true) && (clampMode[z_motor_right] == true) && (clampMode[z_motor_left] == true) && (clampMode[arm_motor] == true) && (zeroRPMFlg[y_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[y_motor] == false)) //各モーターが終端に達して動かない状態
               {
                 getboxAutoState = z_little_down;
               }
               else if((motor[joint_right].getGeneralPosition() < -30000) && (motor[joint_left].getGeneralPosition() > 30000))
               {
                  f = true;
                  s = false;
                  w = true;
                  t = false;
                 joint_front_clamp = true;
                 joint_back_clamp = false;
               }
               else
               {
                 d = false;
                 s = false;
                 g = false;
                 t = true;
                 e = true;
               }
               break;
          case z_little_down://少し下がる
               e = false;
               h = true;
               if((clampMode[z_motor_right] == false) && (clampMode[z_motor_left] == false) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true))
               {
                 getboxAutoState++;
                 soonPIDPreventFlg[arm_motor] = true;
               }
               break;
          case box_catch://ボックスをつかむ
               r = true;
               if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false))
               {
                 box_release_clamp = false;
                 box_catch_clamp = true;
                 getboxAutoState++;
                 little_back = 0;
                 soonPIDPreventFlg[y_motor] = true;
                 soonPIDPreventFlg[joint_left] = true;
                 soonPIDPreventFlg[joint_right] = true;
                 soonPIDPreventFlg[joint_left] = true;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
               }
               break;
          case z_little_up://少し上げる（箱をちょっと出して引っかからないようにする）と同時にちょっと下がり，アームを後ろにする
               if(little_back > 1000 && little_back < 1200)
               {
                 g = true;
               }
               if((motor[joint_right].getGeneralPosition() > 70000) || (motor[joint_left].getGeneralPosition() < -70000)/* && (clampMode[y_motor] == true) && (zeroRPMFlg[y_motor] == true) && (soonPIDPreventFlg[y_motor] == false) *//*&& *//* && (clampMode[joint_left] == true) && (zeroRPMFlg[joint_right] == true)*/ /* (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true)*/)
               {
                 r = false;
                 f = true;
                 box_catch_clamp = false;
                 getboxAutoState = box_release;
               }
               else if((motor[joint_right].getGeneralPosition() > 40000) && (motor[joint_left].getGeneralPosition() < -40000))
               {
                 g = false;
                 joint_front_clamp = false;
                 joint_back_clamp = true;
               }
               else if((motor[joint_right].getGeneralPosition() > 14000) && (motor[joint_left].getGeneralPosition() < -14000))
               {
                 soonPIDPreventFlg[arm_motor] = true;
                 little_back = 0;
                 w = false;
                 s = true;
               }
               else
               {
                 h = false;
                 t = false;
                 e = true;
                 g = true;
                 if(little_back < LITTLE_BACK_TIME)
                 {
                   subbord_state[0] &= 0xfe;
                   subbord_state[0] |= 0x02;
                 }
                 else
                 {
                   e = false;
                   d = true;
                   w = false;
                   subbord_state[0] &= 0xf0;
                 }
               }
               break;
          case box_release://ボックスを放す
               if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true))
               {
                 box_release_clamp = false;
                 box_catch_clamp = false;
                 getboxAutoState = init_mode;
                 getboxAutoFlg = false;
                 d = false;
                 f = false;
                 s = false;
               }
               break;
        }
      
        /*for(int i = 0; i < MOTOR_NUM; i++)
        {
          zeroRPMFlg[i] = false;
        }*/
      }

      if(smallGetboxAuto == true)
      {
        switch(smallGetboxAutoState)
        {
          case init_mode:
               smallGetboxAutoState = zup_jointfront_yout;
               motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
               motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
               soonPIDPreventFlg[y_motor] = true;
               little_high = 0;
               break;
          case zup_jointfront_yout:
               if((motor[joint_right].getGeneralPosition() < -80000) && (motor[joint_left].getGeneralPosition() > 80000) && (clampMode[y_motor] == true) /*&& (clampMode[joint_right] == true) && (clampMode[joint_left] == true) */&& (clampMode[z_motor_right] == true) && (clampMode[z_motor_left] == true) && (zeroRPMFlg[y_motor] == true) &&/* (zeroRPMFlg[joint_right] == true) && (zeroRPMFlg[joint_left] == true) && */(zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true) && (soonPIDPreventFlg[y_motor] == false)/* && (soonPIDPreventFlg[joint_right] == false) && (soonPIDPreventFlg[joint_left] == false)*/)
               {
                 smallGetboxAutoState = z_little_down;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
               }
               else if((motor[joint_right].getGeneralPosition() < -34000) && (motor[joint_left].getGeneralPosition() > 34000))
               {
                  t = false;
                  f = true;
                 joint_front_clamp = true;
                 joint_back_clamp = false;
               }
               else
               {
                 d = false;
                 g = false;
                 s = false;
                 w = true;
                 e = true;
                 t = true;
               }
               break;
          case z_little_down://yを前に出し，腕を前に振る
               e = false;
               h = true;
               if((clampMode[z_motor_right] == false) && (clampMode[z_motor_left] == false) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true)) //各モーターが終端に達して動かない状態
               {
                 smallGetboxAutoState = box_catch;
                 soonPIDPreventFlg[arm_motor] = true;
               }
               break;
          case box_catch://ボックスをつかむ
               f = false;
               r = true;
               if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false))
               {
                 box_catch_clamp = true;
                 box_release_clamp = false;
                 smallGetboxAutoState = joint_back_y_in_z_down;
                 soonPIDPreventFlg[arm_motor] = true;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
               }
               break;
          case joint_back_y_in_z_down://zを下げてyを引っ込める
               if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false)/* && *//*(clampMode[y_motor] == true) && *//*(clampMode[joint_right] == true) && (clampMode[joint_left] == true) && *//**(zeroRPMFlg[y_motor] == true) && (zeroRPMFlg[joint_right] == true) && (zeroRPMFlg[joint_left] == true) && (soonPIDPreventFlg[y_motor] == false) && (soonPIDPreventFlg[joint_right] == false) && (soonPIDPreventFlg[joint_left] == false)*/) //各モーターが終端に達して動かない状態
               {
                 smallGetboxAutoState = box_release;
                 soonPIDPreventFlg[y_motor] = true;
               }
               else if((motor[joint_right].getGeneralPosition() > 50000) || (motor[joint_left].getGeneralPosition() < -50000))
               {
                 r = false;
                 f = true;
                 box_catch_clamp = false;
               }
               else if((motor[joint_right].getGeneralPosition() > 34000) && (motor[joint_left].getGeneralPosition() < -34000))
               {
                 soonPIDPreventFlg[arm_motor] = true;
                 g = false;
                 joint_front_clamp = false;
                 joint_back_clamp = true;
               }
               else
               {
                 g = true;
               }
               break;
          case box_release://ボックスを放す
               w = false;
               s = true;
               f = false;
               h = false;
               d = true;
               if((clampMode[y_motor] == true) && (zeroRPMFlg[y_motor] == true) && (soonPIDPreventFlg[y_motor] == false) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true))
               {
                 smallGetboxAutoState = init_mode;
                 smallGetboxAuto = false;
                 box_catch_clamp = false;
                 box_release_clamp = false;
                 f = false;
                 d = false;
                 s = false;
               }
               break;
        }
      }

      if(getboxAutoState == emergency || smallGetboxAutoState == emergency || putExchangeState == emergency || sirahadoriAutoState == emergency)
      {
        switch(emergency_state)
        {
          case init_mode:
               h = true;
               soonPIDPreventFlg[arm_motor] = true;
               falling_detect = false;
               sirahadori_start = false;
               emergency_state = box_catch;
               break;
          case box_catch:
               r = true;
               if(box_catch_clamp == true)
               {
                 before_joint_right = motor[joint_right].getGeneralPosition();
                 before_joint_left = motor[joint_left].getGeneralPosition();
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
                 emergency_state = joint_front;
               }
               else if((clampMode[arm_motor] == true) && (zeroRPMFlg[arm_motor] == true) && (soonPIDPreventFlg[arm_motor] == false))
               {
                 before_joint_right = motor[joint_right].getGeneralPosition();
                 before_joint_left = motor[joint_left].getGeneralPosition();
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
                 emergency_state = joint_front;
               }
               break;
          case joint_front:
               if(joint_front_clamp == true)
               {
                 soonPIDPreventFlg[y_motor] = true;
                 soonPIDPreventFlg[arm_motor] = true;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
                 emergency_state = boxrelease_jointback_zdown_yin;
               }
               else if((motor[joint_right].getGeneralPosition() < -(80000 - abs(before_joint_right))) && (motor[joint_left].getGeneralPosition() > (80000 - abs(before_joint_left))))
               {
                 soonPIDPreventFlg[y_motor] = true;
                 soonPIDPreventFlg[arm_motor] = true;
                 motor[joint_right].setNeutral(motor[joint_right].getGeneralPosition());
                 motor[joint_left].setNeutral(motor[joint_left].getGeneralPosition());
                 emergency_state = boxrelease_jointback_zdown_yin;
               }
               else if((motor[joint_right].getGeneralPosition() < -((80000 - abs(before_joint_right))/2)) && (motor[joint_left].getGeneralPosition() > ((80000 - abs(before_joint_left))/2)))
               {
                 t = false;
                 joint_front_clamp = true;
                 joint_back_clamp = false;
               }
               else
               {
                 t = true;
               }
               break; 
          case boxrelease_jointback_zdown_yin:
               if(joint_back_clamp && (clampMode[y_motor] == true) && (clampMode[arm_motor] == true) && (zeroRPMFlg[z_motor_right] == true) && (zeroRPMFlg[z_motor_left] == true) && (zeroRPMFlg[y_motor] == true) && (zeroRPMFlg[arm_motor] == true)/* && (soonPIDPreventFlg[y_motor] == false) && (soonPIDPreventFlg[arm_motor] == false)*/)
               {
                 s = false;
                 d = false;
                 f = false;
                 g = false;
                 joint_front_clamp = false;
                 joint_back_clamp = true;
                 getboxAutoState = init_mode;
                 smallGetboxAutoState = init_mode;
                 putExchangeState = init_mode;
                 sirahadoriAutoState = init_mode;
                 emergency_state = init_mode;
               }
               else if((motor[joint_right].getGeneralPosition() > 34000) && (motor[joint_left].getGeneralPosition() < -34000))
               {
                  joint_back_clamp = true;
                 joint_front_clamp = false;
               }
               else
               {
                 w = false;
                 e = false;
                 h = false;
                 r = false;
                 h = false;
                 f = true;
                 g = true;
                 s = true;
                 d = true;
               }
               break;
        }
      }
        
      for(int i = 0; i < MOTOR_NUM; i++)
      {
        zeroRPMFlg[i] = false;
      }
      /*------------------------------------------------------------------------------------------------------------------------------------------*/
      if(rcv.flgRecvSuccess == false)
      {
        Serial.println("fail...");
        
      }
      else
      {
  
        //↓キーボードとコントローラーの関数同時使用禁止！！！
        //controller_task();
        keyboard_task();
        
      }
      //keyboard_task();      //入力したキーボードからのデータによって，モーターのムーブを決める

      /*-----------電流値が安定しないので10個くらいとってその中の最大値を信頼する（こうしないと制御が安定しない）------------*/
      if(sampling_count < SAMPLING_NUM)
      {
        for(int i = 0; i < (MOTOR_NUM + 1); i++)
        {
          current[i] = max(current[i], abs(motor[i].getCurrent()));
        }
        sampling_count++;
      }
      else
      {  
        for(int i = 0; i < (MOTOR_NUM + 1); i++)
        {
          current_judge[i] = current[i];
          current[i] = 0;
        }
        sampling_count = 0;
      }
      /*--------------------------------------------------------------------------------------------------*/


      /*-------------モーターが締め付け位置に入ったら--------------*/
      for(int i = 0; i < (MOTOR_NUM + 1); i++)
      {
        /*if(state[z_motor_right] == z_low && state[z_motor_left] == z_low)
        {
          clampModeCurrent[z_motor_right] = (clampModeCurrent[z_motor_right] - 3);
          clampModeCurrent[z_motor_left] = (clampModeCurrent[z_motor_left] - 3);
          clampCurrent[z_motor_right] = (clampCurrent[z_motor_right] - 3);
          clampCurrent[z_motor_left] = (clampCurrent[z_motor_left] - 3);
        }*/
        if(current_judge[i] > clampModeCurrent[i])
        {
          if(clampFlg[i] > 5)
          {
            clampMode[i] = true; 
          } 
          else
          {
            clampFlg[i]++;
          }
        }
        else
        {
          /*if(!(clampModeCurrent[i] > clampCurrent[i]))
          {*/
            clampMode[i] = false;
          /*}*/
          clampFlg[i] = 0;
        }

        if(clampMode[i] && (soonPIDPreventFlg[i] == false))
        {
          current_diff[i] += (clampCurrent[i] - current_judge[i]);
          add_current[i] = (int16_t)(kpGain[i] * (clampCurrent[i] - current_judge[i]) + kiGain[i] * current_diff[i]);
        }
        else
        {
          current_diff[i] = 0;
          add_current[i] = 0;
        }

        if(!clampMode[i])
        {
          soonPIDPreventFlg[i] = false;
        }
      }
      /*-----------------------------------------------*/
      
      /*------------------------------------------------------mainbordそれぞれのモーターのampareを格納---------------------------------------------------*/
      //z方向(右モータ)
      if(state[z_motor_right] == z_high)
      {
        ampare[z_motor_right] = motor[z_motor_right].culcAmpareFromRPM(-motor_rpm[z_motor_right]) - add_current[z_motor_right];
      }
      else if(state[z_motor_right] == z_low)
      {
        ampare[z_motor_right] = motor[z_motor_right].culcAmpareFromRPM(/*-motor_rpm[z_motor_right]*/1000)/* - add_current[z_motor_right]*/;
        if(h == true)
        {
          if(getboxAutoState == emergency || smallGetboxAutoState == emergency || putExchangeState == emergency || sirahadoriAutoState == emergency)
          {
            ampare[z_motor_right] = motor[z_motor_right].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/-1500)/* + add_current[z_motor_left]*/;
          }
          else if(sirahadoriAutoState != init_mode)
          {
            ampare[z_motor_right] = motor[z_motor_right].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/-1500)/* + add_current[z_motor_left]*/;
          }
          else if((smallGetboxAutoState == -1))
          {
            if(z_low_time > 50)
            {
              ampare[z_motor_right] = motor[z_motor_right].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/-1500)/* + add_current[z_motor_left]*/;
            }
          }
          else
          {
            if(z_low_time > 120)
            {
              ampare[z_motor_right] = motor[z_motor_right].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/-1500)/* + add_current[z_motor_left]*/;
            }
          }
        }
        else if(p == true)
        {
          if(sirahadoriAutoState == fall_gravity)
          {
            if(sirahadori_start == true)
            {
              ampare[z_motor_right] = motor[z_motor_right].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/box_fall_rpm - 600)/* + add_current[z_motor_left]*/;
            }
            else
            {
              ampare[z_motor_right] = motor[z_motor_right].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/-1500)/* + add_current[z_motor_left]*/;
            }
          }
        }
      }
      else if(state[z_motor_right] == z_little_high)
      {
        //ampare[z_motor_right] = (int16_t)motor[z_motor_right].culcAmpareFromPosition(LITTLE_Z_MOVE);
      }
      else if(state[z_motor_right] == z_little_low)
      {
        //ampare[z_motor_right] = (int16_t)motor[z_motor_right].culcAmpareFromPosition(-LITTLE_Z_MOVE);
      }
      else
      {
        ampare[z_motor_right] = 0;
      }
      
      //z方向(左モータ)
      if(state[z_motor_left] == z_high)
      {
          ampare[z_motor_left] = motor[z_motor_left].culcAmpareFromRPM(+motor_rpm[z_motor_left]) + add_current[z_motor_left];
      }
      else if(state[z_motor_left] == z_low)
      {
        ampare[z_motor_left] = motor[z_motor_left].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/-1000)/* + add_current[z_motor_left]*/;
        if(h == true)
        {
          if(getboxAutoState == emergency || smallGetboxAutoState == emergency || putExchangeState == emergency || sirahadoriAutoState == emergency)
          {
            ampare[z_motor_left] = motor[z_motor_left].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/1500)/* + add_current[z_motor_left]*/;
          }
          else if(sirahadoriAutoState != init_mode)
          {
            ampare[z_motor_left] = motor[z_motor_left].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/1500)/* + add_current[z_motor_left]*/;
          }
          else if((smallGetboxAutoState == -1))
          {
            if(z_low_time > 50)
            {
              ampare[z_motor_left] = motor[z_motor_left].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/1500)/* + add_current[z_motor_left]*/;
            }
          }
          else
          {
            if(z_low_time > 120)
            {
              ampare[z_motor_left] = motor[z_motor_left].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/1500)/* + add_current[z_motor_left]*/;
            }
          }
        }
        else if(p == true)
        {
          if(sirahadoriAutoState == fall_gravity)
          {
            if(sirahadori_start == true)
            {
              ampare[z_motor_left] = motor[z_motor_left].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/-box_fall_rpm + 600)/* + add_current[z_motor_left]*/;
            }
            else
            {
              ampare[z_motor_left] = motor[z_motor_left].culcAmpareFromRPM(/*-motor_rpm[z_motor_left]*/1500)/* + add_current[z_motor_left]*/;
            }
          }
        }
      }
      else if(state[z_motor_left] == z_little_high)
      {
        ampare[z_motor_left] = (int16_t)motor[z_motor_left].culcAmpareFromPosition(-LITTLE_Z_MOVE);
      }
      else if(state[z_motor_left] == z_little_low)
      {
        ampare[z_motor_left] = (int16_t)motor[z_motor_left].culcAmpareFromPosition(LITTLE_Z_MOVE);
      }
      else
      {
        ampare[z_motor_left] = 0;
      }

      //y方向
      if(state[y_motor] == y_out)
      {
        ampare[y_motor] = motor[y_motor].culcAmpareFromRPM(-motor_rpm[y_motor]) - add_current[y_motor];
      }
      else if(state[y_motor] == y_in)
      {
        ampare[y_motor] = motor[y_motor].culcAmpareFromRPM(motor_rpm[y_motor]) + add_current[y_motor];
      }
      else
      {
        ampare[y_motor] = 0;
      }

      //arm方向
      if(state[arm_motor] == catch_box)
      {
        /*if(sirahadoriAutoState == fall_gravity && r == true)
        {
          ampare[arm_motor] = -12000;
        }
        else
        {*/
          ampare[arm_motor] = motor[arm_motor].culcAmpareFromRPM(-motor_rpm[arm_motor])/* - add_current[arm_motor]*/;
        //}
      }
      else if(state[arm_motor] == release_box)
      {
        ampare[arm_motor] = motor[arm_motor].culcAmpareFromRPM(motor_rpm[arm_motor])/* + add_current[arm_motor]*/;
      }
      else
      {
        ampare[arm_motor] = 0;
      }
      
      //joint方向
      //joint(右モータ)
      if(state[joint_right] == front_joint)
      {
        ampare[joint_right] = motor[joint_right].culcAmpareFromRPM(-motor_rpm[joint_right])/* - add_current[joint_right]*/;
      }
      else if(state[joint_right] == back_joint)
      {
        ampare[joint_right] = motor[joint_right].culcAmpareFromRPM(motor_rpm[joint_right])/* + add_current[joint_right]*/;
      }
      else
      {
        ampare[joint_right] = 0;
      }
      //joint(左モータ)
      if(state[joint_left] == front_joint)
      {
        ampare[joint_left] = motor[joint_left].culcAmpareFromRPM(motor_rpm[joint_left])/* + add_current[joint_left]*/;
      }
      else if(state[joint_left] == back_joint)
      {
        ampare[joint_left] = motor[joint_left].culcAmpareFromRPM(-motor_rpm[joint_left])/* - add_current[joint_left]*/;
      }
      else
      {
        if(getboxAutoState)
        {
          ampare[joint_left] = 0;
        }
      }

      //camera
      if(state[camera_motor] == camera_front)
      {
        ampare[camera_motor] = CameraMotor.culcAmpareFromRPM(motor_rpm[camera_motor]) + add_current[camera_motor];
      }
      else if(state[camera_motor] == camera_back)
      {
        ampare[camera_motor] = CameraMotor.culcAmpareFromRPM(-motor_rpm[camera_motor]) - add_current[camera_motor];
      }
      else
      {
        ampare[camera_motor] = 0;
      }

      /*----------------------------------------------------------------------------------------------------------------------------------*/

      //更新した電流値によってモーターを制御する．
      moveMotorAll();
      moveCamera();
      canmanager->setrowCAN2Message(0x210,subbord_state);
      canmanager->sendAllCANdata();

  }

  if(Debug.check())
  {
      /*for(int i = 0; i < MOTOR_NUM; i++)
      {
        //Serial.print(" ");
        Serial.print(current[i]);
        Serial.print("   ");
      }*/
      //Serial.print(digitalRead(2));
      //Serial.print(state[z_motor_right]);
//      Serial.print("  ");
//      Serial.print(box_fall_time);
//      Serial.print("  ");
      /*Serial.print(motor[arm_motor].getRPM());
      Serial.print("  ");
        Serial.print(zeroRPMFlg[arm_motor]);
        Serial.print("  ");
//      Serial.print(box_fall_rpm);
        Serial.print(motor[joint_right].getGeneralPosition());
        Serial.print("  ");
        Serial.print(motor[joint_left].getGeneralPosition());
        Serial.print("  ");*/
        /*Serial.print(digitalRead(2));
        Serial.print("  ");
        Serial.print(r);
        Serial.print("  ");*/
      /*Serial.print(current[arm_motor]);
      Serial.print("  ");
      Serial.print(measure);
      Serial.print("  ");
      Serial.print(motor[arm_motor].getRPM());
      Serial.print("  ");*/
      Serial.println();
  }
}

void controller_task(int use)
{
  if(use == CONTROLLER)
  {
    if(rcv.toggle.left == 1)
    {
      controller_flg[toggle_left1] = true;
      controller_flg[toggle_left2] = false;
      
      subbord_state[chassisSpeedX] = (int8_t)((rcv.joy.leftX / 660.0) * 20);
      subbord_state[chassisSpeedY] = (int8_t)((rcv.joy.leftY / 660.0) * 20);
      subbord_state[omega] = (int8_t)((rcv.joy.rightX / 660.0) * 20);
      
      subbord_state[chassisSpeedX] = LIMIT(subbord_state[chassisSpeedX], SPEED_LIMIT * 10);
      subbord_state[chassisSpeedY] = LIMIT(subbord_state[chassisSpeedY], SPEED_LIMIT * 10);
      subbord_state[omega] = LIMIT(subbord_state[omega], SPEED_LIMIT * 10);
    }
    else if(rcv.toggle.left == 2)
    {
      controller_flg[toggle_left1] = false;
      controller_flg[toggle_left2] = true;
      
      if(rcv.joy.rightY > 400)
      {
        if(keypushflag[0])
        {
          keypushflag[0] = false;
          controller_flg[sirahadori_state] = true;
        }
      }
      else if(rcv.joy.rightY < -400)
      {
        if(keypushflag[1])
        {
          keypushflag[1] = false;
          controller_flg[small_resource_island] = true;
        }
      }
      else if(rcv.joy.leftY > 400)
      {
        if(keypushflag[2])
        {
          keypushflag[2] = false;
          controller_flg[resource_island] = true;
        }
      }
      else if(rcv.joy.leftY < -400)
      {
        if(keypushflag[3])
        {
          keypushflag[3] = false;
          controller_flg[put_exchange_state] = true;
        }
      }
      else
      {
        keypushflag[0] = true;
        keypushflag[1] = true;
        keypushflag[2] = true;
        keypushflag[3] = true;
        controller_flg[sirahadori_state] = false;
        controller_flg[small_resource_island] = false;
        controller_flg[resource_island] = false;
        controller_flg[put_exchange_state] = false;
      }
      
      if(rcv.joy.leftX > 400)
      {
          controller_flg[RFID_state1] = true;
      }
      else if(rcv.joy.leftX < -400)
      {
          controller_flg[RFID_state2] = true;
      }
      else
      {
          controller_flg[RFID_state1] = false;
          controller_flg[RFID_state2] = false;
      }
      
      subbord_state[obstacle] = (int8_t)((rcv.joy.rightX / 660.0) * 100.0);
      subbord_state[obstacle] = LIMIT(subbord_state[obstacle], 100.0);
    }

  }
  else if(use == KEYBOARD)
  {
    if(rcv.key.Z)
    {
        if(keypushflag[0])
        {
          controller_flg[small_resource_island] = true;
          keypushflag[0] = false;
        }
    }
    else if(rcv.key.X)
    {
        if(keypushflag[1])
        {
          controller_flg[resource_island] = true;
          keypushflag[1] = false;
        }
    }
    else if(rcv.key.C)
    {
        if(keypushflag[2])
        {
          controller_flg[put_exchange_state] = true;
          keypushflag[2] = false;
        }
    }
    else if(rcv.key.V)
    {
        if(keypushflag[3])
        {
          controller_flg[sirahadori_state] = true;
          keypushflag[3] = false;
        }
    }
    else
    {
      keypushflag[0] = true;
      keypushflag[1] = true;
      keypushflag[2] = true;
      keypushflag[3] = true;
    }
    
    if(rcv.key.W)
    {
      ySpeed = ((accelerateY / 2000.0) * 660.0);
    }
    else if(rcv.key.S)
    {
      ySpeed = -((accelerateY / 2000.0) * 660.0);
    }
    else
    {
      ySpeed = 0;
      accelerateY = 0;
    }

    if(rcv.key.D)
    {
      xSpeed = ((accelerateX / 2000.0) * 660.0);
    }
    else if(rcv.key.A)
    {
      xSpeed = -((accelerateX / 2000.0) * 660.0);
    }
    else
    {
      xSpeed = 0;
      accelerateX = 0;
    }

    if(rcv.key.E)
    {
      rad = ((accelerateRad / 2000.0) * 660.0);
    }
    else if(rcv.key.Q)
    {
      rad = -((accelerateRad / 2000.0) * 660.0);
    }
    else
    {
      rad = 0;
      accelerateRad = 0;
    }

    if(rcv.key.R)
    {
      obstacle_speed = ((accelerateObstacle / 2000.0) * 660.0);
    }
    else if(rcv.key.F)
    {
      obstacle_speed = -((accelerateObstacle / 2000.0) * 660.0);
    }
    else
    {
      obstacle_speed = 0;
      accelerateObstacle = 0;
    }
    
    subbord_state[obstacle] = (int8_t)((obstacle_speed / 660.0) * 100.0);
    subbord_state[obstacle] = LIMIT(subbord_state[obstacle], 100.0);
    
    subbord_state[chassisSpeedX] = (int8_t)((xSpeed / 660.0) * 20);
    subbord_state[chassisSpeedY] = (int8_t)((ySpeed / 660.0) * 20);
    subbord_state[omega] = (int8_t)((rad / 660.0) * 20);
    
    subbord_state[chassisSpeedX] = LIMIT(subbord_state[chassisSpeedX], SPEED_LIMIT * 10);
    subbord_state[chassisSpeedY] = LIMIT(subbord_state[chassisSpeedY], SPEED_LIMIT * 10);
    subbord_state[omega] = LIMIT(subbord_state[omega], SPEED_LIMIT * 10);
  }
}


void keyboard_task(){
  //keyboard

  if(/* rcv.key.A *//*rcv.joy.leftY > 400*/w)state[y_motor] = y_out;
  else if(/* rcv.key.D *//*rcv.joy.leftY < -400*/s)state[y_motor] = y_in;
  else state[y_motor] = stop_move;
  
  if(/* rcv.key.W *//*rcv.joy.rightY > 400*/e)
  {
    state[z_motor_right] = z_high;
    state[z_motor_left] = z_high;
  }
  else if(/* rcv.key.S *//*rcv.joy.rightY < -400*/d || h || p)
  {
    state[z_motor_right] = z_low;
    state[z_motor_left] = z_low;
  }
  else if(y)
  {
    state[z_motor_right] = z_little_high;
    state[z_motor_left] = z_little_high;
  }
  else
  {
    state[z_motor_right] = stop_move;
    state[z_motor_left] = stop_move;
  }
  
  if(/* rcv.key.A *//*rcv.joy.rightX > 400*/r)state[arm_motor] = catch_box;
  else if(/* rcv.key.D *//*rcv.joy.rightX < -400*/f)state[arm_motor] = release_box;
  else state[arm_motor] = stop_move;
  
  if(/* rcv.key.A *//*rcv.joy.leftX > 400*/t)
  {
    state[joint_right] = front_joint;
    state[joint_left] = front_joint;
  }
  else if(/* rcv.key.D *//*rcv.joy.leftX < -400*/g)
  {
    state[joint_right] = back_joint;
    state[joint_left] = back_joint;
  }
  else
  {
    state[joint_right] = stop_move;
    state[joint_left] = stop_move;
  }
  
}
