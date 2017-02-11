#ifndef XRotor_H
#define XRotor_H

#include <Arduino.h>
#include <math.h>
//#include <Servo.h>

class XRotor {
    unsigned int pwmWriteFrequency;
    byte pwmResolution;
    unsigned int escMin;
    unsigned int escMax;
    long remap(long long value, long long low1, long long high1, long long low2, long long high2);
public:
    void config(long inputMin, long inputMax, unsigned int frequency, byte resolution);
    void init(byte pin);
    void update(byte pin, long signal);
    unsigned int getMinThrottle(){ return escMin;};
    unsigned int getMaxThrottle(){ return escMax;};
    long signalMin;
    long signalMax;
    void demo(byte pin);
};
#endif // XRotor_H