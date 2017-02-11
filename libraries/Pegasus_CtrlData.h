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
    static const byte NAV_ROT_MODE_CODE = 237;  // 1110,1101
    static const byte NAV_ROT_RANGE_CODE = 222; // 1101,1110
    static const byte PID_YAW_CODE = 218;       // 1101,1010
    static const byte PID_PITCH_CODE = 165;     // 1010,0101
    static const byte PID_ROLL_CODE = 92;       // 0101,1100

    // Nav Rotation Mode; 0=absolute, 1=acrobatic, etc
    static const byte FLY_STAB = 0;
    static const byte FLY_ACRO = 1;
    byte navRotModeYaw = FLY_ACRO;
    byte navRotModePitch = FLY_STAB;
    byte navRotModeRoll = FLY_STAB;

    // Nav Rotation Range - joystick
    byte navRotRangeYaw = 0;
    byte navRotRangePitch = 0;
    byte navRotRangeRoll = 0;

    // PID settings
    float pidScale = 4.0;
    // Yaw
    float Kp_yaw = 0.0;
    float Ki_yaw = 0.0;
    float Kd_yaw = 0.0;
    // Pitch
    float Kp_pitch = 0.0;
    float Ki_pitch = 0.0;
    float Kd_pitch = 0.0;
    // Roll
    float Kp_roll = 0.0;
    float Ki_roll = 0.0;
    float Kd_roll = 0.0;
    
    bool live = true;
    short yaw;
    short pitch;
    short roll;
    unsigned short throttle = 0;
    float yawf;
    float pitchf;
    float rollf;

    void pack(byte *b){
        b[0] = control_code;
        b[1] = byte(yaw >> 8);
        b[2] = byte(yaw);
        b[3] = byte(pitch >> 8);
        b[4] = byte(pitch);
        b[5] = byte(roll >> 8);
        b[6] = byte(roll);
        b[7] = byte(throttle >> 8);
        b[8] = byte(throttle);
    };

    void unpack(byte *b){
        control_code = b[0];
        switch (control_code){
            case KILL_CODE:
                live = false;
                break;
            case NAV_CODE: // control packet
                live = true;
                // -32767 / (65535/1=65535) = -0.5
                // -32767 / (65535/50=1310.7) = -25
                // -32767 / (65535/180=364.083) = -90
                yawf   = getShort(b[1], b[2]) / (65535.0 / navRotRangeYaw);
                pitchf = getShort(b[3], b[4]) / (65535.0 / navRotRangePitch);
                rollf  = getShort(b[5], b[6]) / (65535.0 / navRotRangeRoll);
                throttle = getShort(b[7], b[8]);
                break;
            case NAV_ROT_MODE_CODE: // navigation rotation mode; stable or rate
                navRotModeYaw   = b[2];
                navRotModePitch = b[4];
                navRotModeRoll  = b[6];
                break;
            case NAV_ROT_RANGE_CODE: // navigation rotation range
                navRotRangeYaw   = b[2];
                navRotRangePitch = b[4];
                navRotRangeRoll  = b[6];
                break;
            case PID_YAW_CODE: // PID settings for yaw mode
                Kp_yaw = getShort(b[1], b[2]) / pidScale;
                Ki_yaw = getShort(b[3], b[4]) / pidScale;
                Kd_yaw = getShort(b[5], b[6]) / pidScale;
                break;
            case PID_PITCH_CODE: // PID settings for pitch mode
                Kp_pitch = getShort(b[1], b[2]) / pidScale;
                Ki_pitch = getShort(b[3], b[4]) / pidScale;
                Kd_pitch = getShort(b[5], b[6]) / pidScale;
                break;
            case PID_ROLL_CODE: // PID settings for roll mode
                Kp_roll = getShort(b[1], b[2]) / pidScale;
                Ki_roll = getShort(b[3], b[4]) / pidScale;
                Kd_roll = getShort(b[5], b[6]) / pidScale;
                break;
        }
    };

    short getShort(short upper, short lower){
        return (upper << 8) | lower;
    };

};

#endif // PEGASUS_CTRLDATA_H