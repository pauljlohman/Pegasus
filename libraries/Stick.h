/*
Joystick library 
http://www.third-helix.com/2013/04/12/doing-thumbstick-dead-zones-right.html

*/

#ifndef STICK_H
#define STICK_H

//#include <Dampen.h>
#include <RollingAverage.h>

class Stick {
    byte pinX;
    byte pinY;
    //Dampen dampenX;
    //Dampen dampenY;
    RollingAverage avgX;
    RollingAverage avgY;

    float mapf(float x, float in_min, float in_max, float out_min, float out_max){
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };

    float vecLen(float x, float y){
        return sqrt(x*x+y*y);
    };

    public:
        float centerX = 512.0;
        float centerY = 512.0;
        float outerEdge = 0.0;
        float deadZone = 0.0;
        float x, y;

        void config(byte _pinX, byte _pinY, float _outerEdge, float _deadZone, float _dampen){
            pinX = _pinX;
            pinY = _pinY;
            outerEdge = _outerEdge;
            deadZone = _deadZone;
            //dampenX.config(_dampen);
            //dampenY.config(_dampen);
            avgX.config(4);
            avgY.config(4);
        };
        
        void calibrate(){
            centerX = 0.0;
            centerY = 0.0;
            for(int i=0; i<200; i++){
                centerX += float(analogRead(pinX));
                centerY += float(analogRead(pinY));
                delay(2);
            }
            centerX /= 200.0;
            centerY /= 200.0;
        };
        
        void update(){
            //unsigned long time = millis();
            
            x = float(analogRead(pinX));
            y = float(analogRead(pinY));
            //Serial.printf("raw %f %f %i\n", x, y, time);

            //x = dampenX.update(x);
            //y = dampenY.update(y);
            x = avgX.update(x);
            y = avgY.update(y);
            //Serial.printf("damp %f %f %i\n", x, y, time);

            //float len = vecLen(x-centerX, y-centerY);

            if(x < centerX-deadZone){
                x = mapf(x, 0.0+outerEdge, centerX-deadZone, -511.0, 0.0);
            }else if(x > centerX+deadZone){
                x = mapf(x, centerX+deadZone, 1024.0-outerEdge, 0.0, 511.0);
            }else{
                x = 0.0;
            }
            
            if(y < centerY-deadZone){
                y = mapf(y, 0.0+outerEdge, centerY-deadZone, -511.0, 0.0);
            }else if(y > centerY+deadZone){
                y = mapf(y, centerY+deadZone, 1024.0-outerEdge, 0.0, 511.0);
            }else{
                y = 0.0;
            }
            
            //Serial.printf("map %f %f %i\n", x, y, time);
        };
};

#endif // STICK_H