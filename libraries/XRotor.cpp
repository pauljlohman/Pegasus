/* THis library is written to control HobbyWing XRotor from a Teensy 3.2 or LC
see https://www.pjrc.com/teensy/td_pulse.html

suggest using pins on the same timer:
pins 5, 6, 9, 10, 20, 21, 22, 23 for Teensy 3.2
pins 6, 9, 10, 20, 22, 23 for Teensy LC

Resolution  PWM Value   Ideal Frequency
8           0 - 255     187500 Hz
16          0 - 65535   732.4218 Hz
Teensy default resolution is 8, frequency default is 488.28 Hz

XRotor Max ESC signal 500 Hz, resolution ?
*/
#include "XRotor.h"


void XRotor::config(long inputMin, long inputMax, unsigned int frequency, byte resolution){
    signalMin = inputMin;
    signalMax = inputMax;

    // value is hz, for PWM Analog Write frequency, 1hz = 1000ms period
    pwmWriteFrequency = frequency; // tested good up to 480

    // bit precision of the PWM Analog Write, range 2 to 16
    pwmResolution = resolution; // tested 16

    // 0 to pwmMax is the number range at pwmResolution, 8bit = 255, 16bit = 65535
    unsigned int pwmMax = pow(2, pwmResolution)-1;

    // pwm value that results in a 1ms pulse width, which is a typical esc's min throttle
    // ex. 3276.75 when 50hz 16bit, 26214 when 400hz 16bit
    escMin = long(0.5 + (1.0/(1000.0/float(pwmWriteFrequency))) * pwmMax);

    // pwm value that results in a 2ms pulse width, which is a typical esc's max throttle
    // ex. 6553.5 when 50hz 16bit,  52428 when 400hz 16bit
    escMax = long(0.0 + (2.0/(1000.0/float(pwmWriteFrequency))) * pwmMax);
}

void XRotor::init(byte pin){
  pinMode(pin, OUTPUT);
  analogWriteFrequency(pin, pwmWriteFrequency);
  analogWriteResolution(pwmResolution);
}

void XRotor::update(byte pin, long signal){
    signal = constrain(signal, signalMin, signalMax);
    signal = remap(signal, signalMin, signalMax, escMin, escMax);
    //Serial.printf("%i: %i\t",pin, signal);
    analogWrite(pin, signal);
}

// the arduino map function was getting out of range
long XRotor::remap(long long value, long long low1, long long high1, long long low2, long long high2){
  return low2 + (value-low1) * (high2-low2) / (high1-low1);
}

void XRotor::demo(byte pin){
    // wind up
    for (unsigned int i=escMin; i <= escMax; i++){
      analogWrite(pin, i);
      delay(1);
    }

    // drop to half throttle and hold
    analogWrite(pin, escMin + ((escMax-escMin) / 2));
    delay(2000);

    // wind down from max throttle
    for (unsigned int i=escMax; i >= escMin; i--){
      analogWrite(pin, i);
      delay(1);
    }
}
