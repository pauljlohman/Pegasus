#include "EEPROM_I2C.h"

// ADDRESS
// NAV ROTATION MODE 3 bytes
//+0 yaw, +1 pitch, +2 roll
#define ADDR_ROT_MODE 0

// NAV ROTATION RANGE 3 bytes
//+0 yaw, +1 pitch, +2 roll
#define ADDR_ROT_RANGE_ACRO 3
#define ADDR_ROT_RANGE_STAB 6

// PID 6 bytes, 3 uint16
//+0 upper P, +1 lower P, +2 upper I, +3 lower I, +4 upper D, +5 lower D
#define ADDR_PID_YAW_ACRO 12
#define ADDR_PID_YAW_STAB 18
#define ADDR_PID_PITCH_ACRO 24
#define ADDR_PID_PITCH_STAB 30
#define ADDR_PID_ROLL_ACRO 36
#define ADDR_PID_ROLL_STAB 42

#ifndef PEGASUS_CONFIGMEM_H
#define PEGASUS_CONFIGMEM_H

class Pegasus_ConfigMem{
    EEPROM_I2C mem;
    byte buf[2];
public:
    void init(){
        mem.devAddr = B1010000;
    };
    // write Rotation Mode
    void write_RotMode_Yaw(byte mode){
        mem.writeByte(ADDR_ROT_MODE+0, mode);
    };
    void write_RotMode_Pitch(byte mode){
        mem.writeByte(ADDR_ROT_MODE+1, mode);
    };
    void write_RotMode_Roll(byte mode){
        mem.writeByte(ADDR_ROT_MODE+2, mode);
    };
    // write Rotation Range Acro
    void write_RotRange_Yaw_Acro(byte range){
        mem.writeByte(ADDR_ROT_RANGE_ACRO+0, range);
    };
    void write_RotRange_Pitch_Acro(byte range){
        mem.writeByte(ADDR_ROT_RANGE_ACRO+1, range);
    };
    void write_RotRange_Roll_Acro(byte range){
        mem.writeByte(ADDR_ROT_RANGE_ACRO+2, range);
    };

    // write Rotation Range Stab
    void write_RotRange_Yaw_Stab(byte range){
        mem.writeByte(ADDR_ROT_RANGE_STAB+0, range);
    };
    void write_RotRange_Pitch_Stab(byte range){
        mem.writeByte(ADDR_ROT_RANGE_STAB+1, range);
    };
    void write_RotRange_Roll_Stab(byte range){
        mem.writeByte(ADDR_ROT_RANGE_STAB+2, range);
    };

    // write PID Yaw Acro
    void write_PID_Yaw_P_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW_ACRO+0, 2, buf);
    };
    void write_PID_Yaw_I_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW_ACRO+2, 2, buf);
    };
    void write_PID_Yaw_D_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW_ACRO+4, 2, buf);
    };
    // write PID Pitch Acro
    void write_PID_Pitch_P_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH_ACRO+0, 2, buf);
    };
    void write_PID_Pitch_I_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH_ACRO+2, 2, buf);
    };
    void write_PID_Pitch_D_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH_ACRO+4, 2, buf);
    };
    // write PID Roll Acro
    void write_PID_Roll_P_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL_ACRO+0, 2, buf);
    };
    void write_PID_Roll_I_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL_ACRO+2, 2, buf);
    };
    void write_PID_Roll_D_Acro(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL_ACRO+4, 2, buf);
    };

    // write PID Yaw Stab
    void write_PID_Yaw_P_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW_STAB+0, 2, buf);
    };
    void write_PID_Yaw_I_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW_STAB+2, 2, buf);
    };
    void write_PID_Yaw_D_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_YAW_STAB+4, 2, buf);
    };
    // write PID Pitch Stab
    void write_PID_Pitch_P_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH_STAB+0, 2, buf);
    };
    void write_PID_Pitch_I_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH_STAB+2, 2, buf);
    };
    void write_PID_Pitch_D_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_PITCH_STAB+4, 2, buf);
    };
    // write PID Roll Stab
    void write_PID_Roll_P_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL_STAB+0, 2, buf);
    };
    void write_PID_Roll_I_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL_STAB+2, 2, buf);
    };
    void write_PID_Roll_D_Stab(uint16_t v){
        buf[0] = byte(v >> 8);
        buf[1] = byte(v);
        mem.write(ADDR_PID_ROLL_STAB+4, 2, buf);
    };


    // read Rotation Mode
    byte read_RotMode_Yaw(){
        return mem.readByte(ADDR_ROT_MODE+0);
    };
    byte read_RotMode_Pitch(){
        return mem.readByte(ADDR_ROT_MODE+1);
    };
    byte read_RotMode_Roll(){
        return mem.readByte(ADDR_ROT_MODE+2);
    };
    
    // read Rotation Range Acro
    byte read_RotRange_Yaw_Acro(){
        return mem.readByte(ADDR_ROT_RANGE_ACRO+0);
    };
    byte read_RotRange_Pitch_Acro(){
        return mem.readByte(ADDR_ROT_RANGE_ACRO+1);
    };
    byte read_RotRange_Roll_Acro(){
        return mem.readByte(ADDR_ROT_RANGE_ACRO+2);
    };

    // read Rotation Range Stab
    byte read_RotRange_Yaw_Stab(){
        return mem.readByte(ADDR_ROT_RANGE_STAB+0);
    };
    byte read_RotRange_Pitch_Stab(){
        return mem.readByte(ADDR_ROT_RANGE_STAB+1);
    };
    byte read_RotRange_Roll_Stab(){
        return mem.readByte(ADDR_ROT_RANGE_STAB+2);
    };

    // read PID Yaw Acro
    uint16_t read_PID_Yaw_P_Acro(){
        mem.read(ADDR_PID_YAW_ACRO+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Yaw_I_Acro(){
        mem.read(ADDR_PID_YAW_ACRO+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Yaw_D_Acro(){
        mem.read(ADDR_PID_YAW_ACRO+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    // read PID Pitch Acro
    uint16_t read_PID_Pitch_P_Acro(){
        mem.read(ADDR_PID_PITCH_ACRO+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Pitch_I_Acro(){
        mem.read(ADDR_PID_PITCH_ACRO+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Pitch_D_Acro(){
        mem.read(ADDR_PID_PITCH_ACRO+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    // read PID Roll Acro
    uint16_t read_PID_Roll_P_Acro(){
        mem.read(ADDR_PID_ROLL_ACRO+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Roll_I_Acro(){
        mem.read(ADDR_PID_ROLL_ACRO+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Roll_D_Acro(){
        mem.read(ADDR_PID_ROLL_ACRO+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };

    // read PID Yaw Stab
    uint16_t read_PID_Yaw_P_Stab(){
        mem.read(ADDR_PID_YAW_STAB+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Yaw_I_Stab(){
        mem.read(ADDR_PID_YAW_STAB+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Yaw_D_Stab(){
        mem.read(ADDR_PID_YAW_STAB+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    // read PID Pitch Stab
    uint16_t read_PID_Pitch_P_Stab(){
        mem.read(ADDR_PID_PITCH_STAB+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Pitch_I_Stab(){
        mem.read(ADDR_PID_PITCH_STAB+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Pitch_D_Stab(){
        mem.read(ADDR_PID_PITCH_STAB+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    // read PID Roll Stab
    uint16_t read_PID_Roll_P_Stab(){
        mem.read(ADDR_PID_ROLL_STAB+0, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Roll_I_Stab(){
        mem.read(ADDR_PID_ROLL_STAB+2, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };
    uint16_t read_PID_Roll_D_Stab(){
        mem.read(ADDR_PID_ROLL_STAB+4, 2, buf);
        return ((uint16_t)buf[0]<<8) | buf[1];
    };

    void format(){
        // write Rotation Mode
        write_RotMode_Yaw(1);
        write_RotMode_Pitch(0);
        write_RotMode_Roll(0);

        // write Rotation Range Acro
        write_RotRange_Yaw_Acro(2);
        write_RotRange_Pitch_Acro(4);
        write_RotRange_Roll_Acro(4);

        // write Rotation Range Stab
        write_RotRange_Yaw_Stab(15);
        write_RotRange_Pitch_Stab(30);
        write_RotRange_Roll_Stab(30);

        // write PID Acro
        write_PID_Yaw_P_Acro(0);
        write_PID_Yaw_I_Acro(0);
        write_PID_Yaw_D_Acro(0);

        write_PID_Pitch_P_Acro(0);
        write_PID_Pitch_I_Acro(0);
        write_PID_Pitch_D_Acro(0);

        write_PID_Roll_P_Acro(0);
        write_PID_Roll_I_Acro(0);
        write_PID_Roll_D_Acro(0);

        // write PID Stab
        write_PID_Yaw_P_Stab(0);
        write_PID_Yaw_I_Stab(0);
        write_PID_Yaw_D_Stab(0);

        write_PID_Pitch_P_Stab(0);
        write_PID_Pitch_I_Stab(0);
        write_PID_Pitch_D_Stab(0);

        write_PID_Roll_P_Stab(0);
        write_PID_Roll_I_Stab(0);
        write_PID_Roll_D_Stab(0);
    };
};
#endif // Pegasus_ConfigMem