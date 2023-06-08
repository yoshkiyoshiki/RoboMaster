#ifndef __IMU_MANAGER__
#define __IMU_MANAGER__

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "IMUmanager.h"

class IMUmanager: public Adafruit_BNO055{
private:
    double dt;

    String yawDirection;
    String rollDirection;
    String pitchDirection;

    imu::Vector<3> eular;
    imu::Vector<3> rad_per_s;
    bool isSetting;

    double yaw_dot;
    double roll_dot;
    double pitch_dot;

    double yaw;
    double roll;
    double pitch;

    double prev_yaw_dot;

    double directionValueAngle(String s);
    double directionValueOmega(String s);
public:
    IMUmanager(double _dt);
    ~IMUmanager();

    void setDirection( String yawD, String rollD, String pitchD);
    bool start();
    void update();

    double getYaw();
    double getRoll();
    double getPitch();

    double getYawVelo();
    double getRollVelo();
    double getPitchVelo();
};





#endif