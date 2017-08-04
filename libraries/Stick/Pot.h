
#ifndef POT_H
#define POT_H

#include <RollingAverage.h>

class Pot {
    byte pin;
    RollingAverage avg;
    float range = 512.0;
    bool flip = false;
    
    public:
        float v;

        void config(byte _pin, float _outputRange, bool _flip){
            pin = _pin;
            range = _outputRange;
            flip = _flip;
            avg.config(4);
        };
        void setOutputRange(float _outputRange){
            range = _outputRange;
        };
        
        void update(){
            //unsigned long time = millis();
            v = float(analogRead(pin));
            //Serial.printf("raw %f\n", v, time);
            v = avg.update(v);
            //Serial.printf("damp %f %i\n", v, time);
            v = flip ? 1024.0 - v : v;
            v = v * (range/1024.0);
            //Serial.printf("map %f %i\n", v, time);
        };
};

#endif // POT_H