// #pragma once
#ifndef __MOTOR__
#define __MOTOR__
#include <Arduino.h>

#define STOP_RECOGNITION 50
#define MOVE_CONSTANT 300

class Motor
{
private:
    double rpm;         // now motor's rpm [rpm]
    double position;    // now motor's position [8192]
    double current;     // now motor's curent [A]
    double temperature;  // now motor's temperature

    int stopCount;
public:
    Motor();
    ~Motor();

    int16_t detectInitialPosition(bool direction = true);

    void setRPM(int16_t rpm);
    virtual void setPosition(int16_t position);
    void setCurrent(int16_t current);
    void setTemperature(int16_t temperature);

    double getRPM();
    double getPosition();
    double getCurrent();
    double getTemperature();

};

#endif