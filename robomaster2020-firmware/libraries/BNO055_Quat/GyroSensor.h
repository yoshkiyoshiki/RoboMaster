#ifndef _GYROSENSOR_H_
#define _GYROSENSOR_H_

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

class GyroSensor
{
  public:
        //コンストラクタ
        GyroSensor(Adafruit_BNO055*);

	void start();
        //クォータニオンからオイラー角に変換
        void quaternionToEuler();
        float roll();
        float pitch();
        float yaw();
	float angleVelo();

  private:

	void getAxisData(sensors_event_t* event);
	
	Adafruit_BNO055 *bno;
        sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
        imu::Quaternion quat;

        uint8_t system, gyro, accel, mag = 0;
        float roll_data;
        float pitch_data;
        float yaw_data;
	float accel_data[3] = {0};
	float gyro_data = 0;
	float mag_data[3] = {0};
        double w_data = 0, x_data = 0, y_data = 0, z_data = 0;//各クォータニオンの値
        double ysqr, xsqr, zsqr;//各クォータニオンベクトル部の２乗
        double t0, t1, t2, t3, t4;//計算に使用する変数
};

#endif 
