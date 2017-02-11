/*

*/
#ifndef DAMPEN_H
#define DAMPEN_H

class Dampen{
    bool isFirstSample = true;
    float dry = 0.01;
    float wet = 1.0-dry;
    float outValue;
    public:
        void config(float f){
            f = constrain(f,0.0,1.0);
            dry = 1.0 - f;
            wet = f;
        };
        float update(float value){
            if(isFirstSample){
                isFirstSample = false;
                outValue = value;
            }
            outValue = (value*dry) + (outValue*wet);
            return outValue;
        };
};

#endif
