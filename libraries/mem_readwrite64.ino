#include "Wire.h"
#include "EEPROM_I2C.h"
EEPROM_I2C mem;

byte buf[64];
byte startAddr = 50;
byte len = 64;
void setup() {
  Wire.begin();
  //Wire.setClock(400000L);
  Serial.begin(115200);
  while (!Serial);
  mem.devAddr = B1010000;  
  Serial.print("Buffer ");
  for(byte i=0; i<len; i++){
    byte v = random(255);
    buf[i] = v;
    Serial.printf("%i, ",v);
  }
  Serial.print("\nWrite\n");
  mem.write(startAddr, len, buf);
  
  buf[64] = {0};
  Serial.println("Read");
  mem.read(startAddr, len, buf);  
  for(byte i=0; i<len; i++){
    Serial.println(buf[i]);
  }
}

void loop() {
}