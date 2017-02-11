/*
written for EEPROM Microchip 24LC256
typical EEPROM pin layout
    ___
 A0| U |Vcc
 A1|   |WP
 A2|   |SCL
Vss|___|SDA

Pin Function
A0  : User Configurable Chip Select
A1  : User Configurable Chip Select
A2  : User Configurable Chip Select
VSS : Ground
SDA : Serial Data, 10kOhm pull-up for 100kHz, 2kOhm for 400kHz and 1 MHz
SCL : Serial Clock
WP  : Write-Protect Input, low - write enabled, high - write disabled
VCC : +2.5V to 5.5V

Addressing
starts 1010 
the next three bits are the chip select bits (A2, A1, A0)
the last bit is 1 for read, 0 for write
Device address format: start bit, 1, 0, 1, 0, A2, A1, A0, R/W, Acknowledge bit
the next 2 bytes define the date address
Data address format: x,14,13,12,11,10,9,8, 7,6,5,4,3,2,1,0

write operation are limited to a single page
page address are a int multiple of the page size

There is 5 ms write cycle time regardless of length of data written

24LC256
256 kbit = 32,000 bytes
64 byte page size
100 kHz and 400 kHz

AT24C256C
256 Kbit = 32,768 bytes
64 byte page size
400kHz and 1MHz

see
Wire.setClock (400000)
*/

//#include <Wire.h> //better to include in upper level
#ifndef EEPROM_I2C_H
#define EEPROM_I2C_H

class EEPROM_I2C{
    unsigned short timeout = 1000;
    unsigned long lastWrite = 0;
    byte buf[2];
    
public:
    byte devAddr = B1010000; // 0xA0 
    unsigned short pageSize = 64; // I've seen 16 suggested regardless of actual page size
    
    byte readByte(unsigned short address){
        read(address, 1, buf);
        return buf[0];
    };
    short readShort(unsigned short address_high){
        read(address_high, 2, buf);
        return word(buf[0], buf[1]);
    };
    void read(unsigned short address, byte length, byte *data){
        waitTillReady();
        byte c = 0;
        unsigned long t = millis();
        //Serial.printf("start Read at %i\n", address);
        Wire.beginTransmission(devAddr);
        Wire.send( byte(address >> 8) );   // MSB
        Wire.send( byte(address & 0xFF) ); // LSB
        Wire.endTransmission();
        Wire.beginTransmission(devAddr);
        Wire.requestFrom(devAddr, length);
        for (; Wire.available() && (millis() - t < timeout); c++) {
            data[c] = Wire.receive();
        }
        Wire.endTransmission();
    };
    
    void writeByte(unsigned short address, byte value){
        buf[0] = value;
        write(address, 1, buf);
    };
    void writeShort(unsigned short address_high, short value){
        buf[0] = byte(value >> 8);
        buf[1] = byte(value);
        write(address_high, 2, buf);
    };
    void write(unsigned short address, byte length, byte *data){
        waitTillReady();
        // start write session
        //Serial.printf("start write at %i\n", address);
        Wire.beginTransmission(devAddr);
        Wire.send(byte(address >> 8));   // MSB
        Wire.send(byte(address & 0xFF)); // LSB

        for (byte i=0; i<length; i++) {
            bool pageBreak = ((address % pageSize) == 0) && (i != 0); // address
            if (pageBreak){
                Wire.endTransmission();
                lastWrite = millis();
                //Serial.println("page break");
                //Serial.printf("start write at %i\n", address);
                // start new page write session
                waitTillReady();
                Wire.beginTransmission(devAddr);
                Wire.send(byte(address >> 8));   // MSB
                Wire.send(byte(address & 0xFF)); // LSB
            }
            Wire.send(data[i]);
            //Serial.printf("%X ",data[i]);
            address++;
        }
        Wire.endTransmission();
        lastWrite = millis();
        //Serial.println();
    };
    
    void clear(unsigned short byteCapacity){
        for(unsigned short i=0; i<(byteCapacity/pageSize); i++){
            unsigned short address = i * pageSize;
            waitTillReady();
            Wire.beginTransmission(devAddr);
            Wire.send(byte(address >> 8));   // MSB
            Wire.send(byte(address & 0xFF)); // LSB
            for (byte j=0; j<pageSize; j++) {
                Wire.send(0);
            }
            Wire.endTransmission();
            lastWrite = millis();
        }
    };
    
    void waitTillReady(){
        //Serial.print("Waiting till Ready...");
        // wait till previous write has completed
        unsigned long elapsed = millis() - lastWrite;
        if(elapsed<5){
            delay(5-elapsed);
        }
        //Serial.println("Ready!");
    };

};

#endif // EEPROM_I2C_H
