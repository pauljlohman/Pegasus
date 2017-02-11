#include "Wire.h"
#include "EEPROM_I2C.h"
EEPROM_I2C mem;

#define BUFSIZE 16

byte writeBuf[BUFSIZE];
byte readBuf[BUFSIZE];

void setup() {
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);
  while (!Serial);
  mem.devAddr = B1010000;  
  /*
  for(byte i=0; i<BUFSIZE; i++){
    writeBuf[i] = 0;
  } 
  
  Serial.print("Start\n");
  
  for(unsigned short i=0; i<(32768/BUFSIZE); i++){
    Serial.println(i*BUFSIZE);
    mem.write(i*BUFSIZE, BUFSIZE, writeBuf);    
  }
  */
  mem.clear();
  Serial.print("Done clearing\n");
  
  delay(1000);
  Serial.print("Reading Back\n");
  for(unsigned short i=0; i<(32768/BUFSIZE); i++){
    mem.read(i*BUFSIZE, BUFSIZE, readBuf);
    for(byte j=0; j<BUFSIZE; j++){
        Serial.print(readBuf[j], HEX);
    } 
    Serial.println();
  }
  Serial.print("Done\n");
  
  
}

void loop() {
}