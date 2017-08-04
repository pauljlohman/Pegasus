/*
Joystick library 
http://www.third-helix.com/2013/04/12/doing-thumbstick-dead-zones-right.html

*/

#ifndef CAPACITIVESWITCH_H
#define CAPACITIVESWITCH_H

class CapacitiveSwitch {
    byte pin;
    long threshold = 1700;
    short offset = 50;
    short countRange = 200;
    short count = 0;
    
    public:
        void config(byte _pin, long _threshold, short _offset, short _countRange){
            pin = _pin;
            threshold = _threshold;
            offset = _offset;
            countRange = _countRange;
        };
        void setThreshold(long _threshold){
            threshold = _threshold;
        };
        void calibrate(){
            float avg = 0.0;
            for(int i=0; i<200; i++){
                avg += float(touchRead(pin));
                delay(2);
            }
            avg /= 200.0;
            threshold = long(avg);
        };
        
        bool check(bool currentState){
            count += touchRead(pin) > (threshold + offset) ? 1 : -1;
            count = max(count, 0);
            count = min(count, countRange);
            if( count > countRange-1){
                return true;
            }
            if( count < 1){
                return false;
            }
            return currentState;
        };
};

#endif // CAPACITIVESWITCH_H