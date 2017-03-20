#include "EEPROM_I2C.h"

// ADDRESS
// NAV MODE 4 bytes
//+0 yaw, +1 pitch, +2 roll, +3 throttle
#define ADDR_NAV_MODE 0

// NAV RANGE 5 bytes
//+0 yaw, +1 pitch, +2 roll, +3 throttle
#define ADDR_NAV_RANGE_REL 4
#define ADDR_NAV_RANGE_ABS 9

// PID 6 bytes, 3 uint16
//+0 upper P, +1 lower P, +2 upper I, +3 lower I, +4 upper D, +5 lower D
#define ADDR_PID_YAW 14
#define ADDR_PID_PITCH 20
#define ADDR_PID_ROLL 26

#ifndef PEGASUS_CONFIGMEM_H
#define PEGASUS_CONFIGMEM_H

class Pegasus_ConfigMem{
    EEPROM_I2C mem;
    byte buf[2];
public:
    void init(){
        mem.devAddr = B1010000;
    };
    
    // write Nav Mode
    void write_NavMode_Yaw(byte mode){
        mem.writeByte(ADDR_NAV_MODE+0, mode);
    };
    void write_NavMode_Pitch(byte mode){
        mem.writeByte(ADDR_NAV_MODE+1, mode);
    };
    void write_NavMode_Roll(byte mode){
        mem.writeByte(ADDR_NAV_MODE+2, mode);
    };
    void write_NavMode_Throttle(byte mode){
        mem.writeByte(ADDR_NAV_MODE+3, mode);
    };
    
    // write Nav Range Relative
    void write_NavRange_Yaw_Rel(byte range){
        mem.writeByte(ADDR_NAV_RANGE_REL+0, range);
    };
    void write_NavRange_Pitch_Rel(byte range){
        mem.writeByte(ADDR_NAV_RANGE_REL+1, range);
    };
    void write_NavRange_Roll_Rel(byte range){
        mem.writeByte(ADDR_NAV_RANGE_REL+2, range);
    };
    void write_NavRange_Throttle_Rel(uint16_t range){
        buf[0] = byte(range >> 8);
        buf[1] = byte(range);
        mem.write(ADDR_NAV_RANGE_REL+3, 2, buf);
    };
    
    // write Nav Range Absolute
    void write_NavRange_Yaw_Abs(byte range){
        mem.writeByte(ADDR_NAV_RANGE_ABS+0, range);
    };
    void write_NavRange_Pitch_Abs(byte range){
        mem.writeByte(ADDR_NAV_RANGE_ABS+1, range);
    };
    void write_NavRange_Roll_Abs(byte range){
        mem.writeByte(ADDR_NAV_RANGE_ABS+2, range);
    };
    void write_NavRange_Throttle_Abs(uint16_t range){
        buf[0] = byte(range >> 8);
        buf[1] = byte(range);
        mem.write(ADDR_NAV_RANGE_ABS+3, 2, buf);
    };
    
    // write PID Yaw
    void write_PID_Yaw_P(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW+0, 2, buf);
    };
    void write_PID_Yaw_I(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW+2, 2, buf);
    };
    void write_PID_Yaw_D(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW+4, 2, buf);
    };
    
    // write PID Pitch
    void write_PID_Pitch_P(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH+0, 2, buf);
    };
    void write_PID_Pitch_I(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH+2, 2, buf);
    };
    void write_PID_Pitch_D(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH+4, 2, buf);
    };
    
    // write PID Roll
    void write_PID_Roll_P(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL+0, 2, buf);
    };
    void write_PID_Roll_I(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL+2, 2, buf);
    };
    void write_PID_Roll_D(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL+4, 2, buf);
    };

    // read Nav Mode
    byte read_NavMode_Yaw(){
        return mem.readByte(ADDR_NAV_MODE+0);
    };
    byte read_NavMode_Pitch(){
        return mem.readByte(ADDR_NAV_MODE+1);
    };
    byte read_NavMode_Roll(){
        return mem.readByte(ADDR_NAV_MODE+2);
    };
    byte read_NavMode_Throttle(){
        return mem.readByte(ADDR_NAV_MODE+3);
    };
    
    // read Nav Range Relative
    byte read_NavRange_Yaw_Rel(){
        return mem.readByte(ADDR_NAV_RANGE_REL+0);
    };
    byte read_NavRange_Pitch_Rel(){
        return mem.readByte(ADDR_NAV_RANGE_REL+1);
    };
    byte read_NavRange_Roll_Rel(){
        return mem.readByte(ADDR_NAV_RANGE_REL+2);
    };
    uint16_t read_NavRange_Throttle_Rel(){
        mem.read(ADDR_NAV_RANGE_REL+3, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };

    // read Nav Range Absolute
    byte read_NavRange_Yaw_Abs(){
        return mem.readByte(ADDR_NAV_RANGE_ABS+0);
    };
    byte read_NavRange_Pitch_Abs(){
        return mem.readByte(ADDR_NAV_RANGE_ABS+1);
    };
    byte read_NavRange_Roll_Abs(){
        return mem.readByte(ADDR_NAV_RANGE_ABS+2);
    };
    uint16_t read_NavRange_Throttle_Abs(){
        mem.read(ADDR_NAV_RANGE_ABS+3, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    
    // read PID Yaw Relative
    uint16_t read_PID_Yaw_P(){
        mem.read(ADDR_PID_YAW+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Yaw_I(){
        mem.read(ADDR_PID_YAW+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Yaw_D(){
        mem.read(ADDR_PID_YAW+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    // read PID Pitch Relative
    uint16_t read_PID_Pitch_P(){
        mem.read(ADDR_PID_PITCH+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Pitch_I(){
        mem.read(ADDR_PID_PITCH+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Pitch_D(){
        mem.read(ADDR_PID_PITCH+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    
    // read PID Roll Relative
    uint16_t read_PID_Roll_P(){
        mem.read(ADDR_PID_ROLL+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Roll_I(){
        mem.read(ADDR_PID_ROLL+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Roll_D(){
        mem.read(ADDR_PID_ROLL+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };

    void format(){
        // write Nav Mode
        write_NavMode_Yaw(1);
        write_NavMode_Pitch(0);
        write_NavMode_Roll(0);
        write_NavMode_Throttle(0);

        // write Nav Range Relative
        write_NavRange_Yaw_Rel(2);
        write_NavRange_Pitch_Rel(4);
        write_NavRange_Roll_Rel(4);
        write_NavRange_Throttle_Rel(8);

        // write Nav Range Absolute
        write_NavRange_Yaw_Abs(15);
        write_NavRange_Pitch_Abs(30);
        write_NavRange_Roll_Abs(30);
        write_NavRange_Throttle_Abs(32767);
        
        // write PID
        write_PID_Yaw_P(100);
        write_PID_Yaw_I(50);
        write_PID_Yaw_D(30);

        write_PID_Pitch_P(120);
        write_PID_Pitch_I(70);
        write_PID_Pitch_D(50);

        write_PID_Roll_P(120);
        write_PID_Roll_I(70);
        write_PID_Roll_D(50);
    };
};
#endif // Pegasus_ConfigMem
