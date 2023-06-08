#include <Arduino.h>
#include <Chassis.h>
#include "mymath.h"

#define myfabs(a) ((a > 0) ? a : -a)

//初期化処理を行う関数
//コンストラクタは、　クラス名::クラス名と同じ名前で構成します
Chassis::Chassis(void) {}

//********************************************************************************
// シャーシ用モーターの制御
// メインループ
//********************************************************************************
// void Chassis::doChassisTask(void) {
//   // M3508のフィードバック
//   double targetRotation[4];
//   convRobotVector2MotorRotation(veloX, veloY, angVelo, targetRotation);
//   double errorRotation[4];
//   for (int i = 0; i < 4; i++) {
//     errorRotation[i] = targetRotation[i] - sensRotation[i];
//   }
//   pidWheelRotation(errorRotation, wheelAmpere);
// }

//********************************************************************************
// ホイール回転速度の設定
// veloX,veloY:  並進速度                   (  m/sec)[double]
// omega:        回転速度                   (rad/sec)[double]
// theta:        シャーシとジンバルのずれ角度(  rad  )[double]
// *rotation:    各モーターの角速度         (  rpm  )[double]
//********************************************************************************
void Chassis::doChassisTask( double _veloX, double _veloY, double _omega, double _theta, double* outputs){
  setRobotVector( _veloX, _veloY, _omega);
  // 1.機体の回転に関する制御?

  // 2.カメラ視点での向きに直す
  calcTwistOffset( _theta);
  // 3.無限回転時の補正?

  this->convRobotVector2MotorRotation( veloX, veloY, angVelo, outputs);
}


//********************************************************************************
// ホイール回転速度の設定
// x,y:       並進速度          (  m/sec)[double]
// r:         回転速度          (rad/sec)[double]
// *rotation: 各モーターの角速度(rpm)[double]
//********************************************************************************
void Chassis::convRobotVector2MotorRotation(double x, double y, double r, double *rotation) {

  //回転数制限用の各定数
  const int WHEEL_COUNT = 4;
  // const long SPEED_LIMIT = rpm2radpersec(469);//指令値の限界。ESCの性能面の限界
  const long SPEED_LIMIT = 482*12;//指令値の限界。ESCの性能面の限界(rpm)
  const double ROOT2 = 1.0/1.41421356;//sqrt(2)
  const double Convert_mps2rps = 1.0/0.075;// convert m/sec to rad/sec

  // 行列演算用
  const double gainx[WHEEL_COUNT] = {-1.00, -1.00, 1.00, 1.00};
  const double gainy[WHEEL_COUNT] = {-1.00, 1.00, 1.00, -1.00};
  // 機体の中心からの距離。SQRT(0.4**2 + 0.4**2) = 0.5656854249492381
  const double gainr[WHEEL_COUNT] = {0.5656854249492381, 0.5656854249492381, 0.5656854249492381, 0.5656854249492381};

  // 回転数制限用の各変数
  double spd[WHEEL_COUNT] = {0};
  double max_spd = 0;

  int i;
  for (i = 0; i < WHEEL_COUNT; i++) {
    double fast;
    spd[i] = M3508_GEAR_RATIO*radpersec2rpm(Convert_mps2rps*(ROOT2 * (x * gainx[i] + y * gainy[i]) + r * gainr[i]));
    fast = ABS(spd[i]);
    if (fast > max_spd)
      max_spd = fast;
  }

  if (max_spd > SPEED_LIMIT) {
    for (i = 0; i < WHEEL_COUNT; i++) {
      spd[i] *= SPEED_LIMIT / max_spd;
    }
  }

  for (i = 0; i < WHEEL_COUNT; i++) {
    rotation[i] = spd[i];
  }
}

//********************************************************************************
// ロボットの速度（X,Y）・角速度の設定
// x,y: 並進速度(  m/sec)[double]
// r:   回転速度(rad/sec)[double]
//********************************************************************************
void Chassis::setRobotVector(double x, double y, double r) {
  velox_bodylink = x;
  veloy_bodylink = y;
  angVelo = r;
}

//********************************************************************************
// ロボットの移動速度ベクトルの回転
// カメラ座標系からシャーシ座標系への座標変換
// generalGimbalYaw: (rad)[double]
//********************************************************************************
void Chassis::calcTwistOffset(double generalGimbalYaw) {
  double gimbalRelativeAngle = -generalGimbalYaw;

  veloX = velox_bodylink * cos(gimbalRelativeAngle) - veloy_bodylink * sin(gimbalRelativeAngle);
  veloY = veloy_bodylink * cos(gimbalRelativeAngle) + velox_bodylink * sin(gimbalRelativeAngle);

}

// //********************************************************************************
// //シャーシのジンバルに対する目標角の取得(左トグルスイッチの値)
// // toggle:      [RecieverToggle]
// // nowYaw: (rad)[double]
// //********************************************************************************
// double Chassis::getTargetBodyAngle(RecieverToggle toggle, double nowYaw){
//   double targetBodyAngle = 0;
//   if (toggle == Near) {
//     //トグル手前
//     targetBodyAngle = 0;
//   } else if (toggle == Center) {
//     //トグル真ん中
//     targetBodyAngle = (nowYaw + infiniteRotateSpeed);
//   }else{
//     //トグル奥
//     targetBodyAngle = 4096;
//     // targetBodyAngle = 0;
//   }
//   return targetBodyAngle;
// }

// //********************************************************************************
// // 回転速度のPID制御
// // error: 各モーターの角速度の誤差(rad/sec)[double]
// // u:     各モーターの入力電流  (Ampare)[double]
// //********************************************************************************
// void Chassis::pidWheelRotation(double *error, double *u) {
//   const int movingAverageCount = 35;
//   const double integralEffectLimit = 5000;

//   static double integralError[4] = {0.0, 0.0, 0.0, 0.0};
//   int differentialError[4] = {0, 0, 0, 0};
//   static int movingAveragePointer[4] = {0, 0, 0, 0};
//   static int lastError[4] = {0, 0, 0, 0};
//   static double differentialErrorFragment[4][movingAverageCount] = {0.0f};

//   for (int i = 0; i < 4; i++) {
//     integralError[i] += error[i] * (timerInterruptInterval / 1000.0);

//     if (integralEffectLimit < pidGainWheel[1] * integralError[i]) {
//       integralError[i] = integralEffectLimit / pidGainWheel[1];
//     }
//     differentialErrorFragment[i][movingAveragePointer[i]] =
//         error[i] - lastError[i];
//     movingAveragePointer[i]++;
//     if (movingAverageCount <= movingAveragePointer[i]) {
//       movingAveragePointer[i] = 0;
//     }
//     for (int ii = 0; ii < movingAverageCount; ii++) {
//       differentialError[i] +=
//           differentialErrorFragment[i][ii] * (1000.0 / timerInterruptInterval);
//     }
//     differentialError[i] /= movingAverageCount;
//     u[i] = pidGainWheel[0] * error[i] + pidGainWheel[1] * integralError[i] +
//            pidGainWheel[2] * differentialError[i];
//     lastError[i] = error[i];
//   }
// }

//********************************************************************************
// 機体の回転角速度成分のP制御
// error: (rad)[double]
//********************************************************************************
double Chassis::pidChassisRotation(double error) {
  double output = 0.0;
  const double kp = 3.5;

  output = (error * kp);
  return constrain(output, -3.0, 3.0);
}

// //********************************************************************************
// // シャーシ用モータの角速度セット(角速度の配列)
// // rotation: (rad/sec)[int16_t]
// //********************************************************************************
// void Chassis::setSensRotation(int16_t *rotation) {
//   for (int i = 0; i < 4; i++) {
//     sensRotation[i] = rotation[i];
//   }
// }

// //********************************************************************************
// // ジンバルYaw軸ニュートラル角度セット
// // yawNeutralInput: (rad)[double]
// //********************************************************************************
// void Chassis::setYawNeutral(double yawNeutralInput) {
//   yawNeutral = yawNeutralInput;
// }

// //********************************************************************************
// // シャーシフィードバック制御PIDゲインセット
// //********************************************************************************
// void Chassis::setPidChassisGain(double pGain, double iGain, double dGain) {
//   pidGainChassis[0] = pGain;
//   pidGainChassis[1] = iGain;
//   pidGainChassis[2] = dGain;
// }

// //********************************************************************************
// // ホイールフィードバック制御PIDゲインセット
// //********************************************************************************
// void Chassis::setPidWheelGain(double pGain, double iGain, double dGain) {
//   pidGainWheel[0] = pGain;
//   pidGainWheel[1] = iGain;
//   pidGainWheel[2] = dGain;
// }

// //********************************************************************************
// // 腰振りパラメータ設定
// // width:         (    rad)[double]
// // cycle:         (    sec)[double]
// // rotationspeed: (rad/sec)[double]
// //********************************************************************************
// void Chassis::setDodgeParam(double width, double cycle, double rotatespeed){
//   dodgeWidth = width; // 腰降りしてたときの振幅
//   dodgeCycle = cycle; // 腰降りのsin周期
//   infiniteRotateSpeed = rotatespeed; // 無限回転時の回転速度
// }

// //********************************************************************************
// // 出力するトータル電流を制限する
// // maximumAmpere: (Ampare)[double]
// //********************************************************************************
// void Chassis::limitTotalAmpere(double maximumAmpere){
//   double totalAmpere = 0;
//   for(int i=0; i<4; i++){
//     totalAmpere += (double)myfabs(wheelAmpere[i]);
//   }
//   // totalAmpere *= 20.;
//   // totalAmpere /= 16384.;

//   if( totalAmpere > maximumAmpere){
//     for(int i=0; i<4; i++){
//       wheelAmpere[i] *= (maximumAmpere/totalAmpere);
//     }
//   }
// }

// //********************************************************************************
// //ロボットの修正用の回転行列
// // rotation: (rad)[double]
// //********************************************************************************
// void Chassis::calcTwist(double rotation) {
//   double rotateAngle = rotation;

//   int buf_vx = veloX;
//   int buf_vy = veloY;

//   veloX = buf_vx * cos(rotateAngle) - buf_vy * sin(rotateAngle);
//   veloY = buf_vy * cos(rotateAngle) + buf_vx * sin(rotateAngle);

// }