/*
RollingAverage ra1;
ra1.bankSize = 32;
float a1 = ra1.update( analogRead(9)-512);
*/
#ifndef ROLLINGAVERAGE_H
#define ROLLINGAVERAGE_H

#define MAXBANKSIZE 8
class RollingAverage{
    float sample[MAXBANKSIZE] = {0.0};
    byte bankSize = MAXBANKSIZE;
    byte sampCount = 0;
    byte index = 0;
    float average;
    public:
        
        void config(byte _bankSize){
            bankSize = constrain(_bankSize, 1, MAXBANKSIZE);
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
