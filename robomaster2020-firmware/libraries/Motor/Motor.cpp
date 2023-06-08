#include <Motor.h>
#include <Arduino.h>
#include "mymath.h"


Motor::Motor(){
    this->stopCount = 0;
}

Motor::~Motor(){}

int16_t Motor::detectInitialPosition(bool direction){
    if(ABS(this->getRPM())<STOP_RECOGNITION)
        this->stopCount++;
    else
        this->stopCount = 0;

    return (this->stopCount>300)? 0: ((direction)?MOVE_CONSTANT:-MOVE_CONSTANT);
}

void Motor::setRPM(int16_t _rpm){
    this->rpm = (double)_rpm;
}
void Motor::setPosition(int16_t position){
    this->position = (double)position;
}
void Motor::setCurrent(int16_t current){
    this->current = dmap((double)current, -16384.0, 16384.0, -20.0, 20.0);
}
void Motor::setTemperature(int16_t temperature){
    this->temperature = (double)temperature;
}

double Motor::getRPM()      { return this->rpm;}
double Motor::getPosition() { return this->position;}
double Motor::getCurrent()  { return this->current;}
double Motor::getTemperature(){ return this->temperature;}