/* For Silicon Labs’ Si4455 Transceiver rev B1A
   Developed on RFSolutions ZETA-915, Teensy LC, Arduino IDE
   
   fixed packet length, can be configured using WDS
   consolidated used functions into single class
   re-facter to support blocking or non blocking flow
*/

#ifndef ZETA1_H
#define ZETA1_H

#include <Arduino.h>
#include <SPI.h>

#include "radio_config_peggy.h"


class ZETA1 {
    byte CSPin;   // chip select pin, "nSEL" input pin on ZETA
    byte nIRQPin;   // interrupt pin on ZETA
    byte SDNPin;    // set high to shutdown radio, low to enable
    byte CTSPin;    // connected to GPIO1 on ZETA, by default it goes high when clear to send commands
    
    byte channelNumber = RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER; 
    bool ctsWentHigh = false;
    union si4455_cmd_reply_union Si4455Cmd;
    byte radioCmd[16];
    
public:
    ZETA1(byte CS, byte INT, byte SDN);
    bool checkCTS();
    void waitTillReplyReady();
    void waitTillClearToSend();
    void sendBytes(byte byteCount, byte *data);
    void sendByte(byte data);
    void getReply(byte byteCount, byte *data);
    
    byte initialize();
    void getInterruptStatus(byte PH_CLR_PEND, byte MODEM_CLR_PEND, byte CHIP_CLR_PEND);
    void debugInterruptStatus();
    byte sendCmdGetResp(byte cmdByteCount, byte* pCmdData, byte respByteCount, byte* pRespData);
    void writeEZConfigArray(byte byteCount, const byte* pEzConfigArray);
    void reset();
    byte nirqLevel();
    void fifoInfo(bool resetRX, bool resetTX);
    void startRX();
    void startTX(byte *data);
    byte checkReceived(byte *data);
    bool checkTransmitted(void);

    const byte packetLength = RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH;
    enum
    {
      SI4455_SUCCESS,
      SI4455_NO_PATCH,
      SI4455_CTS_TIMEOUT,
      SI4455_PATCH_FAIL,
      SI4455_COMMAND_ERROR
    };
};

#endif // ZETA1_H
