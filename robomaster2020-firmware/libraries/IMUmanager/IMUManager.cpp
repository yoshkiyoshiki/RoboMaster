#include <Arduino.h>
#include <IMUmanager.h>

#include "mymath.h"
#include <utility/imumaths.h>



IMUmanager::IMUmanager(double _dt):Adafruit_BNO055(-1, 0x28){
    this->dt = _dt;

    this->isSetting = false;

    this->yaw = this->roll = this->pitch = 0.0;
    this->yaw_dot = this->roll_dot = this->pitch_dot = 0.0;
    this->prev_yaw_dot = 0.0;

}

IMUmanager::~IMUmanager(){}

void IMUmanager::setDirection( String yawD, String rollD, String pitchD){
    yawDirection   = yawD;
    rollDirection  = rollD;
    pitchDirection = pitchD;

    isSetting = true;
}

bool IMUmanager::start(){
    if(!this->begin()){
        return false;
    }
    delay(500);
    this->setExtCrystalUse(true);
    this->setMode( this->OPERATION_MODE_IMUPLUS);

    return true;
}

void IMUmanager::update(){
    // 角度を取得 unit[degree]
    this->eular = this->getVector(Adafruit_BNO055::VECTOR_EULER);
    // 角速度を取得 unit[deg/sec]
    this->rad_per_s = this->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    this->roll_dot = -rad_per_s.y();
    this->roll  = eular.y()/360 * 2.0*PI; //uit[rad]

    this->pitch_dot = -rad_per_s.x();
    this->pitch = eular.z()/360 * 2.0*PI; // unit[rad]

    // this->yaw_dot = (sin(roll)/cos(pitch)*rad_per_s.y() + cos(roll)/cos(pitch)*rad_per_s.z() );
    // this->yaw_dot = (sin(this->roll)/cos(this->pitch)* this->roll_dot + cos(this->roll)/cos(this->pitch)*rad_per_s.z() );
    this->yaw_dot = rad_per_s.z();
    this->yaw += this->dt * (this->yaw_dot + this->prev_yaw_dot)/2;


    this->prev_yaw_dot = yaw_dot;
}

double IMUmanager::directionValueAngle(String s){
    if(s == "Yaw" || s == "yaw")
        return eular2rad(this->yaw);
    else if(s == "-Yaw" || s == "-yaw")
        return -eular2rad(this->yaw);
    else if(s == "Roll" || s == "roll")
        return this->roll;
    else if(s == "-Roll" || s == "-roll")
        return -this->roll;
    else if(s == "Pitch" || s == "pitch")
        return this->pitch;
    else if(s == "-Pitch" || s == "-pitch")
        return -this->pitch;
}
double IMUmanager::directionValueOmega(String s){
    if(s == "Yaw" || s == "yaw")
        return eular2rad(this->yaw_dot);
    else if(s == "-Yaw" || s == "-yaw")
        return -eular2rad(this->yaw_dot);
    else if(s == "Roll" || s == "roll")
        return this->roll_dot;
    else if(s == "-Roll" || s == "-roll")
        return -this->roll_dot;
    else if(s == "Pitch" || s == "pitch")
        return this->pitch_dot;
    else if(s == "-Pitch" || s == "-pitch")
        return -this->pitch_dot;
}


double IMUmanager::getYaw(){
    if( isSetting )
        return directionValueAngle( yawDirection);
    else
        return eular2rad(this->yaw);
}
double IMUmanager::getRoll(){
    double _roll = 0.0;
    if( isSetting )
        _roll = directionValueAngle( rollDirection);
    else
        _roll = this->roll;

    if( _roll<-PI/2)
        _roll = -(PI+_roll);
    else if(PI/2<_roll)
        _roll = (PI-_roll);
    return _roll;
}
double IMUmanager::getPitch(){
    double _pitch = 0.0;
    if( isSetting )
        _pitch = directionValueAngle( pitchDirection);
    else
        _pitch = this->pitch;

    if(_pitch<-PI/2)
        _pitch = -(PI+_pitch);
    else if(PI/2<_pitch)
        _pitch = (PI-_pitch);
    return _pitch;
}

double IMUmanager::getYawVelo(){
    if( isSetting )
        return directionValueOmega( yawDirection);
    else
        return this->yaw_dot;
}
double IMUmanager::getRollVelo(){
    if( isSetting )
        return directionValueOmega( rollDirection);
    else
        return this->roll_dot;
}
double IMUmanager::getPitchVelo(){
    if( isSetting )
        return directionValueOmega( pitchDirection);
    else
        return this->pitch_dot;
}