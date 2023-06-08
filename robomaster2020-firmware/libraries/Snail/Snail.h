#ifndef __SNAIL__
#define __SNAIL__

#include <Arduino.h>
#include <Servo.h>


class Snail
{
private:
    bool isUsingInterrupt;

    int pin;
    int min;
    int max;

    Servo sv;
    unsigned long lastTime;
    const unsigned long controlCycle_ms = 5;

    int rpm;
    int acc;
    void writeCommand();

public:
    Snail( int Pin, int minPulse, int maxPulse, bool useInterrupt);
    Snail( int Pin, int minPulse, int maxPulse);
    ~Snail();

    void init();
    void setRPM( unsigned int _rpm);
    void run();

};

#endif