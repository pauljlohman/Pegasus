/*
Atmel i2c Serial EEPROM
256-Kbit (32,768 bytes)
AT24C256C 8SOIC
64-byte page
*/
#include "EEPROM_I2C.h"

#ifndef PEGASUS_LOG_H
#define PEGASUS_LOG_H

// log label
#define LOG_IMU     0x20  // yaw, pitch, roll
#define LOG_PID     0x40 // yaw, pitch, roll
#define LOG_RX      0x60 // yaw, pitch, roll, throttle
#define LOG_MP      0x80 // FL, FR, BR, BL

/*
6 byte format
IMU and PID format
label   lll.....            3 bits
time    .ttttttt tttttttt   15 bits
roll /x ......xx xxxxxxxx   10 bits
pitch/y ......yy yyyyyyyy   10 bits
yaw  /z ......zz zzzzzzzz   10 bits
lllttttt tttttttt ttxxxxxx xxxxyyyy yyyyyyzz zzzzzzzz
lllttttt tttttttt ttrrrrrr rrrrpppp ppppppyy yyyyyyyy

RX and MP format
label        lll.....           3 bits
time         ...ttttt tttttttt  13 bits
roll    /FL  xxxxxxxx           8 bits
pitch   /FR  yyyyyyyy           8 bits
yaw     /BR  zzzzzzzz           8 bits
throttle/BL  wwwwwwww           8 bits
lllttttt tttttttt xxxxxxxx yyyyyyyy zzzzzzzz wwwwwwww
*/
#define BUFSIZE 30
class Pegasus_Log{
    EEPROM_I2C mem;
    byte buf[BUFSIZE];
    byte logSize = 0; //size in bytes
    unsigned short memAddr = 0;
    unsigned short byteCapacity = 32768;
    // constants
    float imu_f = 511.0 / 180.0;
    short pid_f = 128;
    short mp_f = 65535 / 255;
    float rx_f = 127.0 / 45.0;
public:

    void init(){
        mem.devAddr = B1010000;
    };
    void clear(){
        mem.clear(byteCapacity);
    };
    void append_IMU(unsigned long t, float x, float y, float z){
        if(logSize+6 > BUFSIZE){
            return;//writeLog();
        }
        uint16_t us;
        int16_t s;
        // 65535ms ~ 1min, greatest 15 bits
        // uint16_t(t >> 1) & 0x7FFF = 0ttttttt tttttttt
        us = (uint16_t)(t >> 1) & 0x7FFF;
        // byte(0ttttttt tttttttt >> 10) = 000ttttt
        // lll00000 | 000ttttt = lllttttt
        buf[logSize] = LOG_IMU | (byte)(us >> 10); logSize++;
        // byte(0ttttttt tttttttt >> 2) = tttttttt
        buf[logSize] = (byte)(us >> 2); logSize++;
        // byte(0ttttttt tttttttt << 6) = tt000000
        buf[logSize] = (byte)(us << 6);
        // scale 180 to 511
        // int16_t(x) & 0x3FF = 000000xx xxxxxxxx
        s = (int16_t)(x * imu_f) & 0x3FF;
        // byte(000000xx xxxxxxxx >> 4) = 00xxxxxx
        // tt000000 | 00xxxxxx = ttxxxxxx
        buf[logSize] = buf[logSize] | (byte)(s >> 4); logSize++;
        //byte(000000xx xxxxxxxx << 4) = xxxx0000
        buf[logSize] = (byte)(s << 4);
        // scale 180 to 511
        // int16_t(y) & 0x3FF = 000000yy yyyyyyyy
        s = (int16_t)(y * imu_f) & 0x3FF;
        // byte(000000yy yyyyyyyy >> 6) = 0000yyyy
        // xxxx0000 | 0000yyyy = xxxxyyyy
        buf[logSize] = buf[logSize] | (byte)(s >> 6); logSize++;
        // byte(000000yy yyyyyyyy << 2) = yyyyyy00
        buf[logSize] = (byte)(s << 2);
        // scale 180 to 511
        // int16_t(z) & 0x3FF = 000000zz zzzzzzzz
        s = (int16_t)(z * imu_f) & 0x3FF;
        // byte(000000zz zzzzzzzz >> 8) = 000000zz
        // yyyyyy00 | 000000zz = yyyyyyzz
        buf[logSize] = buf[logSize] | (byte)(s >> 8); logSize++;
        //byte(000000zz zzzzzzzz) = zzzzzzzz
        buf[logSize] = (byte)(s); logSize++;
    };
    void append_PID(unsigned long t, long roll, long pitch, long yaw){
        if(logSize+6 > BUFSIZE){
            return;//writeLog();
        }
        uint16_t us;
        int16_t s;
        // 65535ms ~ 1min, greatest 15 bits
        // uint16_t(t >> 1) & 0x7FFF = 0ttttttt tttttttt
        us = (uint16_t)(t >> 1) & 0x7FFF;
        // byte(0ttttttt tttttttt >> 10) = 000ttttt
        // lll00000 | 000ttttt = lllttttt
        buf[logSize] = LOG_PID | (byte)(us >> 10); logSize++;
        // byte(0ttttttt tttttttt >> 2) = tttttttt
        buf[logSize] = (byte)(us >> 2); logSize++;
        // byte(0ttttttt tttttttt << 6) = tt000000
        buf[logSize] = (byte)(us << 6);
        // f = 511 = 65535 * 0.0077973601892119;
        // int16_t(roll / f) & 0x3FF = 000000xx xxxxxxxx
        s = (int16_t)(roll / pid_f) & 0x3FF;
        // byte(000000xx xxxxxxxx >> 4) = 00xxxxxx
        // tt000000 | 00xxxxxx = ttxxxxxx
        buf[logSize] = buf[logSize] | (byte)(s >> 4); logSize++;
        //byte(000000xx xxxxxxxx << 4) = xxxx0000
        buf[logSize] = (byte)(s << 4);
        // int16_t(pitch / f) & 0x3FF = 000000yy yyyyyyyy
        s = (int16_t)(pitch / pid_f) & 0x3FF;
        // byte(000000yy yyyyyyyy >> 6) = 0000yyyy
        // xxxx0000 | 0000yyyy = xxxxyyyy
        buf[logSize] = buf[logSize] | (byte)(s >> 6); logSize++;
        // byte(000000yy yyyyyyyy << 2) = yyyyyy00
        buf[logSize] = (byte)(s << 2);
        // f = 511 = 65535 * 0.0077973601892119;
        // int16_t(yaw / f) & 0x3FF = 000000zz zzzzzzzz
        s = (int16_t)(yaw / pid_f) & 0x3FF;
        // byte(000000zz zzzzzzzz >> 8) = 000000zz
        // yyyyyy00 | 000000zz = yyyyyyzz
        buf[logSize] = buf[logSize] | (byte)(s >> 8); logSize++;
        //byte(000000zz zzzzzzzz) = zzzzzzzz
        buf[logSize] = (byte)s; logSize++;
    };
    void append_MP(unsigned long t, long fl, long fr, long br, long bl){
        if(logSize+6 > BUFSIZE){ 
            return;//writeLog(); 
        }
        byte x, y, z, w;
        x = (byte)(fl / mp_f);
        y = (byte)(fr / mp_f);
        z = (byte)(br / mp_f);
        w = (byte)(bl / mp_f);
        append_v4(LOG_MP, t, x, y, z, w);
    };
    void append_RX(unsigned long t, float roll, float pitch, float yaw, unsigned short throttle){
        if(logSize+6 > BUFSIZE){ 
            return;//writeLog(); 
        }
        int8_t x, y, z, w;
        x = (int8_t)(roll * rx_f);
        y = (int8_t)(pitch * rx_f);
        z = (int8_t)(yaw * rx_f);
        w = (byte)(throttle / mp_f);
        append_v4(LOG_RX, t, x, y, z, w);
    };
    void append_v4(byte label, unsigned long t, byte x, byte y, byte z, byte w){
        uint16_t us;
        // 65535ms ~ 1min, greatest 13 bits
        // uint16_t(t >> 3) & 0x1FFF = 000ttttt tttttttt
        us = (uint16_t)(t >> 3) & 0x1FFF;
        // byte(000ttttt tttttttt >> 8) = 000ttttt
        // lll00000 | 000ttttt = lllttttt
        buf[logSize] = label | (byte)(us >> 8); logSize++;
        // byte(000ttttt tttttttt) = tttttttt
        buf[logSize] = (byte)us; logSize++;
        buf[logSize] = x; logSize++;
        buf[logSize] = y; logSize++;
        buf[logSize] = z; logSize++;
        buf[logSize] = w; logSize++;
    };
    byte getLogSize(){
        return logSize;
    };
    void writeOnFull(){
        if (logSize >= BUFSIZE){
            writeLog();
        }
    };
    void writeLog(){        
        // Serial.printf("Saving %i bytes at %i \n",logSize, memAddr);
        // for(byte i=0; i<logSize; i++){
            // Serial.printf("%X ", buf[i]);
        // } Serial.println();        
        mem.write(memAddr, logSize, buf);
        memAddr = getSafeAddr( memAddr + logSize);
        logSize = 0;
        // Serial.printf("Done\n");
    };
    unsigned short getSafeAddr(unsigned short addr){
        if( (addr + 4) % 64 == 0 ){
            return (addr + 4) % byteCapacity;
        }else{
            return addr % byteCapacity;
        }
    };
    void readLog(){
        byte rBuf[6];
        byte label;
        unsigned short i = 0;
        unsigned short previous_i = 0;
        Serial.print("Reading Saved Log\n");
        while(i<byteCapacity){
            i = getSafeAddr(i);
            if(i<previous_i){break;}//prevent wrapping back
            previous_i = i;
            mem.read(i, 6, rBuf);
            label = rBuf[0] & 0xE0;
            switch (label){
                case LOG_IMU:
                    print_IMU(rBuf);
                    break;
                case LOG_PID:
                    print_PID(rBuf);
                    break;
                case LOG_RX:
                    print_RX(rBuf);
                    break;
                case LOG_MP:
                    print_MP(rBuf);
                    break;
                default: // unknown label
                    Serial.printf("ERROR at %i, ", i);
                    for(byte j=0; j<6; j++){ 
                      Serial.printf("%X ",rBuf[j]); 
                    }
                    Serial.println();                    
                    return;
            }
            i += 6;
        }
        Serial.print("End Reading Log\n");
    };
    void print_IMU(byte *d){
        Serial.print("IMU ");
        /*
        for(byte j=0; j<6; j++){ 
            Serial.printf("%X ",d[j]); 
        } Serial.println();
        */
        uint16_t us;
        int16_t s;
        // ushort(lllttttt) << 11 = ttttt000 00000000
        // ttttt000 00000000 | ushort(tttttttt) << 3 =  tttttttt ttttt000
        // tttttttt ttttt000 | ((ttxxxxxx >> 5) & 0x6) = tttttttt ttttttt0
        us = (uint16_t)d[0] << 11;
        us = us | ((uint16_t)d[1] << 3);
        us = us | ((d[2] >> 5) & 0x6);
        Serial.printf("%u ",us); // time
        // (ttxxxxxx << 8) | xxxxyyyy = ttxxxxxxx xxxxyyyy
        // ttxxxxxx xxxxyyyy & 0x3FF0 = 00xxxxxx xxxx0000
        // 00xxxxxx xxxx0000 << 2 = xxxxxxxx xx000000
        // xxxxxxxx xx000000 >> 6 = ......xx xxxxxxxx
        s = ((int16_t)d[2] << 8) | (int16_t)d[3];
        s = s & 0x3FF0;
        s = s << 2;
        s = s >> 6;
        Serial.printf("%.2f ",s / imu_f); // x angle
        // (xxxxyyyy << 8) | yyyyyyzz = xxxxyyyy yyyyyyzz
        // xxxxyyyy yyyyyyzz & 0xFC =  0000yyyy yyyyyy00
        // 0000yyyy yyyyyy00 << 4 = yyyyyyyy yy000000
        // yyyyyyyy yy000000 >> 6 = ......yy yyyyyyyy
        s = ((int16_t)d[3] << 8) | (int16_t)d[4];
        s = s & 0xFFC;
        s = s << 4;
        s = s >> 6;
        Serial.printf("%.2f ",s / imu_f); // y angle
        // (yyyyyyzz << 8) | zzzzzzzz = yyyyyyzz zzzzzzzz
        // yyyyyyzz zzzzzzzz & 0x3FF = 000000zz zzzzzzzz
        // 000000zz zzzzzzzz << 6 = zzzzzzzz zz000000
        // zzzzzzzz zz000000 >> 6 = ......zz zzzzzzzz
        s = ((int16_t)d[4] << 8) | (int16_t)d[5];
        s = s & 0x3FF;
        s = s << 6;
        s = s >> 6;
        Serial.printf("%.2f\n",s / imu_f); // z angle
    };
    void print_PID(byte *d){
        Serial.print("PID ");
        /*
        for(byte j=0; j<6; j++){ 
            Serial.printf("%X ",d[j]); 
        } Serial.println();
        */
        uint16_t us;
        int16_t s;
        // ushort(lllttttt) << 11 = ttttt000 00000000
        // ttttt000 00000000 | ushort(tttttttt) << 3 =  tttttttt ttttt000
        // tttttttt ttttt000 | ((ttrrrrrr >> 5) & 0x6) = tttttttt ttttttt0
        us = (uint16_t)d[0] << 11;
        us = us | ((uint16_t)d[1] << 3);
        us = us | ((d[2] >> 5) & 0x6);
        Serial.printf("%u ",us); // time
        // (ttrrrrrr << 8) | rrrrpppp = ttrrrrrr rrrrpppp
        // ttrrrrrr rrrrpppp & 0x3FF0 = 00rrrrrr rrrr0000
        // 00rrrrrr rrrr0000 << 2 = rrrrrrrr rr000000
        // rrrrrrrr rr000000 >> 6 = ......rr rrrrrrrr
        s = ((int16_t)d[2] << 8) | (int16_t)d[3];
        s = s & 0x3FF0;
        s = s << 2;
        s = s >> 6;
        Serial.printf("%i ",s * pid_f); // roll
        // (rrrrpppp << 8) | ppppppyy = rrrrpppp ppppppyy
        // rrrrpppp ppppppyy & 0xFC =  0000pppp pppppp00
        // 0000pppp pppppp00 << 4 = pppppppp pp000000
        // pppppppp pp000000 >> 6 = ......pp pppppppp
        s = ((int16_t)d[3] << 8) | (int16_t)d[4];
        s = s & 0xFFC;
        s = s << 4;
        s = s >> 6;
        Serial.printf("%i ",s * pid_f); // pitch
        // (ppppppyy << 8) | yyyyyyyy = ppppppyy yyyyyyyy
        // ppppppyy yyyyyyyy & 0x3FF = 000000yy yyyyyyyy
        // 000000yy yyyyyyyy << 6 = yyyyyyyy yy000000
        // yyyyyyyy yy000000 >> 6 = ......yy yyyyyyyy
        s = ((int16_t)d[4] << 8) | (int16_t)d[5];
        s = s & 0x3FF;
        s = s << 6;
        s = s >> 6;
        Serial.printf("%i\n",s * pid_f); // yaw
    };
    void print_RX(byte *d){
        Serial.print("Nav ");
        /*
        for(byte j=0; j<6; j++){ 
            Serial.printf("%X ",d[j]); 
        } Serial.println();
        */
        uint16_t us;
        int8_t s;
        // lllttttt & 0x1F = 000ttttt
        // ushort(000ttttt) << 11 = ttttt000 00000000
        // ttttt000 00000000 | ushort(tttttttt) << 3 = tttttttt ttttt000
        us = (uint16_t)(d[0] & 0x1F) << 11;
        us = us | (uint16_t)d[1] << 3;
        Serial.printf("%u ",us); // time
        s = (int16_t)d[2];
        Serial.printf("%.2f ",s / rx_f); // roll
        s = (int16_t)d[3];
        Serial.printf("%.2f ",s / rx_f); // pitch
        s = (int16_t)d[4];
        Serial.printf("%.2f ",s / rx_f); // yaw
        us = (uint16_t)d[5];
        Serial.printf("%u\n",us * mp_f); // throttle
    };
    void print_MP(byte *d){
        Serial.print("MP ");
        /*
        for(byte j=0; j<6; j++){ 
            Serial.printf("%X ",d[j]); 
        } Serial.println();
        */
        uint16_t us;
        // lllttttt & 0x1F = 000ttttt
        // ushort(000ttttt) << 11 = ttttt000 00000000
        // ttttt000 00000000 | ushort(tttttttt) << 3 = tttttttt ttttt000
        us = (uint16_t)(d[0] & 0x1F) << 11;
        us = us | (uint16_t)d[1] << 3;
        Serial.printf("%u ",us); // time
        us = (uint16_t)d[2];
        Serial.printf("%u ",us * mp_f); // FL
        us = (uint16_t)d[3];
        Serial.printf("%u ",us * mp_f); // FR:
        us = (uint16_t)d[4];
        Serial.printf("%u ",us * mp_f); // BR
        us = (uint16_t)d[5];
        Serial.printf("%u\n",us * mp_f);// BL
    };
};


#endif // PEGASUS_LOG_H
