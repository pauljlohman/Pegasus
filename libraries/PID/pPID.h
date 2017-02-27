/*
https://en.wikipedia.org/wiki/PID_controller
PID controller
    (P) Accounts for present value of the error.
        i.e. if the error is large, the control output will also be large.
    (I) Accounts for past values of the error.
        i.e. if the current output is not sufficiently strong, error will accumulate over time, 
        and the controller will respond by applying a stronger action.
    (D) Accounts for possible future values of the error, based on its current rate of change.

Tuning:
From http://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
    1.Set all gains to zero.
    2.Increase P gain until the response to a disturbance is steady oscillation.
    3.Increase D gain until the the oscillations go away (i.e. it's critically damped).
    4.Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
    5.Set P and D to the last stable values.
    6.Increase I gain until it brings you to the set point with the number of oscillations desired.
    (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot)


Reference
http://controlguru.com/table-of-contents/
https://en.wikipedia.org/wiki/PID_controller
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
https://github.com/nzjrs/pid

*/

#ifndef PPID_H
#define PPID_H

class pPID{
    float error;
    float previous_measured_point;
    float integral;
    float derivative;
    float Kp, Ki, Kd;
    float outMin, outMax;
    float output;
    
    float clamp(float v){
        if(v > outMax){
            v = outMax;
        }else if(v < outMin){
            v = outMin;
        }
        return v;
    };
    
public:
    float compute(float set_point, float measured_point){
        error = set_point - measured_point;
        integral += Ki * error;
        integral = clamp(integral);
        derivative = measured_point - previous_measured_point;
        previous_measured_point = measured_point;

        output = Kp * error;
        output += integral;
        output -= Kd * derivative;
        //output = max( min(output, outMax), outMin);
        output = clamp(output);

        return output;
    };
    
    void config(unsigned short sampleSi, float p, float i, float d, float outputMin, float outputMax){
        float samplesPerSec = sampleSi / 1000000.0; // microseconds
        Kp = p;
        Ki = i * samplesPerSec;
        Kd = d / samplesPerSec;
        outMin = outputMin;
        outMax = outputMax;
        output = clamp(output);
        integral = clamp(integral);
    };
    
    float getError(){return error;};
    float getIntegral(){return integral;};
    float getDerivative(){return derivative;};     
};

#endif // pPID_H
