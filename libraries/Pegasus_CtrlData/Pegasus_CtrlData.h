/*
RX data format, tested with 16 byte packet, 128 bits
minimum data requirements
    yaw, pitch, roll, throttle, a few extra bits

11 bit resolution(2048) for yaw, pitch, roll, and throttle,
and 4 bits for control is probably enough

If bit packed thats 6 bytes
BIT PACKED FORMAT
    7   6   5   4   3   2   1   0
0   e   e   e   e   y   y   y   y
1   y   y   y   y   y   y   y   p
2   p   p   p   p   p   p   p   p
3   p   p   r   r   r   r   r   r
4   r   r   r   r   r   t   t   t
5   t   t   t   t   t   t   t   t
UNPACKING BIT
"e" control_code byte
7   6   5   4   3   2   1   0
x   x   x   x   07  06  05  04
"y" yaw word
15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
x   x   x   x   x   03  02  01  00  17  16  15  14  13  12  11
"p" pitch word
15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
x   x   x   x   x   10  27  26  25  24  23  22  21  20  37  36
"r" roll word
15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
x   x   x   x   x   35  34  33  32  31  30  47  46  45  44  43
"t" throttle word
15  14  13  12  11  10  9   8   7   6   5   4   3   2   1   0
x   x   x   x   x   42  41  40  57  56  55  54  53  52  51  50

If byte packed it's 9 bytes, more byte aligned, better resolution
ALIGNED PACKET FORMAT
control_code = byte 0
yaw = byte 1(upper), byte 2(lower),
pitch = byte 3(upper), byte 4(lower),
roll = byte 5(upper), byte 6(lower),
throttle = byte 7(upper), byte 8(lower)

*/

#ifndef PEGASUS_CTRLDATA_H
#define PEGASUS_CTRLDATA_H

class Pegasus_CtrlData {
    
public:
    byte control_code = 0;
    static const byte KILL_CODE = 182;          // 1011,0110
    static const byte NAV_CODE = 15;            // 0000,1111
    static const byte PID_YAW_CODE = 218;       // 1101,1010
    static const byte PID_PITCH_CODE = 165;     // 1010,0101
    static const byte PID_ROLL_CODE = 92;       // 0101,1100

    // Nav Mode; 0=absolute(stable), 1=relative(acro), etc
    static const byte FLY_ABS = 0;
    static const byte FLY_REL = 1;
    byte navModeYaw = FLY_REL;
    byte navModePitch = FLY_ABS;
    byte navModeRoll = FLY_ABS;
    byte navModeThrottle = FLY_ABS;
    
    // Nav Range
    byte navRangeYaw = 0;
    byte navRangePitch = 0;
    byte navRangeRoll = 0;
    unsigned short navRangeThrottle = 0;
    
    // PID settings
    // Yaw
    short Kp_yaw = 0;
    short Ki_yaw = 0;
    short Kd_yaw = 0;
    // Pitch
    short Kp_pitch = 0;
    short Ki_pitch = 0;
    short Kd_pitch = 0;
    // Roll
    short Kp_roll = 0;
    short Ki_roll = 0;
    short Kd_roll = 0;
    
    bool live = true;
    short buf[4];
    float yawf, pitchf, rollf, throttlef;

    void pack(byte *b){
        b[0] = control_code;
        b[1] = byte(buf[0] >> 8);
        b[2] = byte(buf[0]);
        b[3] = byte(buf[1] >> 8);
        b[4] = byte(buf[1]);
        b[5] = byte(buf[2] >> 8);
        b[6] = byte(buf[2]);
        b[7] = byte(buf[3] >> 8);
        b[8] = byte(buf[3]);
    };

    void unpack(byte *b){
        control_code = b[0];
        switch (control_code){
            case KILL_CODE:
                live = false;
                break;
            case NAV_CODE: // control packet
                live = true;
                yawf      = getShort(b[1], b[2]) / 100.0;
                pitchf    = getShort(b[3], b[4]) / 100.0;
                rollf     = getShort(b[5], b[6]) / 100.0;
                throttlef = (unsigned short)(getShort(b[7], b[8])) / 1.0;
                break;
            case PID_YAW_CODE: // PID settings for yaw mode
                Kp_yaw = getShort(b[1], b[2]);
                Ki_yaw = getShort(b[3], b[4]);
                Kd_yaw = getShort(b[5], b[6]);
                break;
            case PID_PITCH_CODE: // PID settings for pitch mode
                Kp_pitch = getShort(b[1], b[2]);
                Ki_pitch = getShort(b[3], b[4]);
                Kd_pitch = getShort(b[5], b[6]);
                break;
            case PID_ROLL_CODE: // PID settings for roll mode
                Kp_roll = getShort(b[1], b[2]);
                Ki_roll = getShort(b[3], b[4]);
                Kd_roll = getShort(b[5], b[6]);
                break;
        }
    };

    short getShort(short upper, short lower){
        return (upper << 8) | lower;
    };

};

#endif // PEGASUS_CTRLDATA_H
