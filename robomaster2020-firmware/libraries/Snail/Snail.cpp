#include <Arduino.h>
#include <Snail.h>
#include <Servo.h>

#include "mymath.h"



Snail::Snail( int Pin, int minPulse, int maxPulse,  bool useInterrupt=false){
    this->isUsingInterrupt = useInterrupt;

    this->pin = Pin;
    this->min = minPulse;
    this->max = maxPulse;

    this->lastTime = millis();

    this->rpm = 0;
    this->acc = 2;

}
Snail::Snail( int Pin, int minPulse, int maxPulse){
    this->isUsingInterrupt = false;

    this->pin = Pin;
    this->min = minPulse;
    this->max = maxPulse;

    this->lastTime = millis();

    this->rpm = 0;
    this->acc = 2;

}

Snail::~Snail(){}

void Snail::init(){
    this->sv.attach( pin, min, max);
    this->sv.writeMicroseconds( max);
    delay(1500);
    this->sv.writeMicroseconds( min);
    delay(1500);
}


void Snail::writeCommand(){
    int cmd = map( this->rpm, 0, 28800, min, max );

    if( ( cmd - this->sv.readMicroseconds()) >= this->acc){
        this->sv.writeMicroseconds( this->sv.readMicroseconds() + this->acc);
    }else if( this->acc >= (cmd - this->sv.readMicroseconds()) && (cmd - this->sv.readMicroseconds()) >= -this->acc ){
        this->sv.writeMicroseconds( cmd);
    }else if( -this->acc >= ( cmd - this->sv.readMicroseconds()) ){
        this->sv.writeMicroseconds( this->sv.readMicroseconds() - this->acc);
    }

}

void Snail::setRPM( unsigned int _rpm){
    this->rpm = _rpm;
    if( ( !this->isUsingInterrupt && ((millis()-this->lastTime) > this->controlCycle_ms)) ){
        this->writeCommand();
        this->lastTime = millis();
    }
}

void Snail::run(){
    if( this->isUsingInterrupt ){
        this->writeCommand();
    }
}