/*
Joystick library 
http://www.third-helix.com/2013/04/12/doing-thumbstick-dead-zones-right.html

*/

#ifndef STICK_H
#define STICK_H

#include <RollingAverage.h>

class Stick {
    byte pin;
    RollingAverage avg;
    float center = 512.0;
    float outerEdge = 0.0;
    float deadZone = 0.0;
    float range = 512.0;
    
    float mapf(float x, float in_min, float in_max, float out_min, float out_max){
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };

    public:
        float v;

        void config(byte _pin, float _outerEdge, float _deadZone, float _range){
            pin = _pin;
            outerEdge = _outerEdge;
            deadZone = _deadZone;
            range = _range;
            avg.config(4);
        };
        void setRange(float _range){
            range = _range;
        };
        void calibrate(){
            center = 0.0;
            for(int i=0; i<200; i++){
                center += float(analogRead(pin));
                delay(2);
            }
            center /= 200.0;
        };
        
        void update(){
            //unsigned long time = millis();
            v = float(analogRead(pin));
            //Serial.printf("raw %f\n", v, time);
            v = avg.update(v);
            //Serial.printf("damp %f %i\n", v, time);

            if(v < center-deadZone){
                v = mapf(v, 0.0+outerEdge, center-deadZone, -range, 0.0);
            }else if(v > center+deadZone){
                v = mapf(v, center+deadZone, 1024.0-outerEdge, 0.0, range);
            }else{
                v = 0.0;
            }
            
            //Serial.printf("map %f %i\n", v, time);
        };
};

#endif // STICK_H
