/*
RollingAverage ra1;
ra1.bankSize = 32;
float a1 = ra1.update( analogRead(9)-512);
*/
#ifndef ROLLINGAVERAGE_H
#define ROLLINGAVERAGE_H

class RollingAverage{
    float sample[32] = {0.0};
    byte bankSize = 32;
    byte sampCount = 0;
    byte index = 0;
    float average;
    public:
        
        void config(byte _bankSize){
            bankSize = constrain(_bankSize, 1, 32);
        };
        
        float update(float value){
            sampCount = min(bankSize, sampCount+1);
            average -= sample[index];
            //float temp = value / float(sampCount);
            float temp = value / float(bankSize);
            sample[index] = temp;
            average += temp;
            index++;
            if(index >= bankSize){
                index = 0;
            }
            if(sampCount >= bankSize){
                return average;
            }else{
                return value;
            }

        };
};

#endif