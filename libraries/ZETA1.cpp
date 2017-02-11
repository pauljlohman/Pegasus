
#include "ZETA1.h"

#define DEBUG false
#define VERBOSE false
#if DEBUG
    #define debug(x) Serial.print(x);
    #define debughex(x) Serial.print(x,HEX);
    #if VERBOSE        
        #define vdebug(x) Serial.print(x);
        #define vdebughex(x) Serial.print(x,HEX);
    #else
        #define vdebug(x)
        #define vdebughex(x)
    #endif    
#else
    #define debug(x)
    #define debughex(x)
    #define vdebug(x)
    #define vdebughex(x)
#endif


ZETA1::ZETA1(byte CS, byte INT, byte SDN){
    CSPin = CS;    // chip select pin, "nSEL" input pin on ZETA
    nIRQPin = INT;   // interrupt pin on ZETA
    SDNPin = SDN;    // set high to shutdown radio, low to enable
    
    pinMode(CSPin, OUTPUT);
    pinMode(nIRQPin, INPUT);
    pinMode(SDNPin, OUTPUT);
    
    digitalWrite(CSPin, HIGH); // default unselected
}

bool ZETA1::checkCTS() {
    digitalWrite(CSPin, HIGH);
    digitalWrite(CSPin, LOW);
    SPI.transfer(0x44); // READ_CMD_BUFF
    return SPI.transfer(0) == 0xFF ? true : false;
}

void ZETA1::waitTillReplyReady() {
    while (!checkCTS()) {
        vdebug(".");
    }
}

void ZETA1::waitTillClearToSend() {
    vdebug("waitTillClearToSend ");
    waitTillReplyReady();
    vdebug(" all clear\n");
    digitalWrite(CSPin, HIGH);
}

void ZETA1::sendBytes(byte byteCount, byte *data) {
    vdebug("sendBytes ");
    digitalWrite(CSPin, LOW);
    for (byte i = 0; i < byteCount; i++) {
        SPI.transfer(data[i]);
        vdebughex(data[i]); vdebug(" ");
    }
    vdebug("\n");
    digitalWrite(CSPin, HIGH);
}

void ZETA1::sendByte(byte data) {
    vdebug("sendByte "); vdebughex(data); vdebug("\n");
    digitalWrite(CSPin, LOW);
    SPI.transfer(data);
    digitalWrite(CSPin, HIGH);
}

void ZETA1::getReply(byte byteCount, byte *data) {
    vdebug("getReply ");
    for (int i = 0; i < byteCount; i++) {
        data[i] = SPI.transfer(0xFF);
        vdebughex(data[i]); vdebug(" ");
    }
    vdebug("\n");
    digitalWrite(CSPin, HIGH);
}


byte ZETA1::initialize() {
  const byte Radio_Configuration_Data_Array[] = RADIO_CONFIGURATION_DATA_ARRAY;
  const byte *configurationArray = Radio_Configuration_Data_Array;
  debug("initialize\n");
  byte col;
  byte response;
  byte numOfBytes;

  // While cycle as far as the pointer points to a command
  while (*configurationArray != 0x00) {
    vdebug("parse next command\n");
    // Commands structure in the array:
    //  --------------------------------
    //  LEN | <LEN length of data>
    numOfBytes = *configurationArray++;
    if (numOfBytes > 16) {
      // Initial configuration of Si4x55
      // SI4455_CMD_ID_WRITE_TX_FIFO defined in si4455_defs.h, included in radio_config.h
      if (SI4455_CMD_ID_WRITE_TX_FIFO == *configurationArray) {
        if (numOfBytes > 128) {
          // Number of command bytes exceeds maximal allowable length
          debug("Number of command bytes exceeds maximal allowable length: "); debug(numOfBytes); debug(" > 128\n");
          for (col = 0; col < numOfBytes; col++) {
            debughex(*configurationArray); debug(" ");
            configurationArray++;
          } debug("\n");
          return SI4455_COMMAND_ERROR;
        }
        // Load array to the device
        vdebug("Load array to the device\n");
        configurationArray++;
        waitTillClearToSend();
        writeEZConfigArray(numOfBytes - 1, configurationArray);
        delay(1);
        // Point to the next command
        configurationArray += numOfBytes - 1;

        // Continue command interpreter
        continue;
      } 
      else {
        // Number of command bytes exceeds maximal allowable length
        debug("Number of command bytes exceeds maximal allowable length: "); debug(numOfBytes); debug(" > 16\n");
        for (col = 0; col < numOfBytes; col++) {
          debughex(*configurationArray); debug(" ");
          configurationArray++;
        } debug("\n");
        return SI4455_COMMAND_ERROR;
      }
    }

    for (col = 0; col < numOfBytes; col++) {
      radioCmd[col] = *configurationArray;
      configurationArray++;
    }
    
    byte resp = sendCmdGetResp(numOfBytes, radioCmd, 1, &response);
    if (resp != 0xFF) {
      // Timeout occurred
      debug("Timeout occurred\n");
      return SI4455_CTS_TIMEOUT;
    }

    // Check response byte of EZCONFIG_CHECK command
    if (SI4455_CMD_ID_EZCONFIG_CHECK == radioCmd[0]) {
      debug("EZCONFIG_CHECK ");
      if (response) {
        debug("ERROR\n");
        return SI4455_COMMAND_ERROR;
      } else {
        debug("SUCCESS\n");
      }
    }

    if (nirqLevel() == 0) {
      // Get and clear all interrupts.
      vdebug("Get and clear all interrupts: ");
      getInterruptStatus(0, 0, 0);
      if (Si4455Cmd.GET_INT_STATUS.CHIP_PEND & SI4455_CMD_GET_CHIP_STATUS_REP_CMD_ERROR_PEND_MASK) {
        debug(" CMD_ERROR!\n");
        return SI4455_COMMAND_ERROR;
      }else{
        vdebug(" OK\n");
      }
    }
  }

  return SI4455_SUCCESS;
}

byte ZETA1::sendCmdGetResp(byte cmdByteCount, byte* pCmdData, byte respByteCount, byte* pRespData) {
  vdebug("sendCmdGetResp\n");
  waitTillClearToSend();
  sendBytes(cmdByteCount, pCmdData);
  waitTillReplyReady();
  getReply(respByteCount, pRespData);  
  return 0xFF;
}

void ZETA1::getInterruptStatus(byte PH_CLR_PEND, byte MODEM_CLR_PEND, byte CHIP_CLR_PEND) {
  vdebug("getInterruptStatus\n");
  // see docs GET_INT_STATUS and Si4455Cmd
  radioCmd[0] = SI4455_CMD_ID_GET_INT_STATUS;
  radioCmd[1] = PH_CLR_PEND;
  radioCmd[2] = MODEM_CLR_PEND;
  radioCmd[3] = CHIP_CLR_PEND;
  
  sendCmdGetResp(4, radioCmd, SI4455_CMD_REPLY_COUNT_GET_INT_STATUS, radioCmd);

  Si4455Cmd.GET_INT_STATUS.INT_PEND       = radioCmd[0];
  Si4455Cmd.GET_INT_STATUS.INT_STATUS     = radioCmd[1];
  Si4455Cmd.GET_INT_STATUS.PH_PEND        = radioCmd[2];
  Si4455Cmd.GET_INT_STATUS.PH_STATUS      = radioCmd[3];
  Si4455Cmd.GET_INT_STATUS.MODEM_PEND     = radioCmd[4];
  Si4455Cmd.GET_INT_STATUS.MODEM_STATUS   = radioCmd[5];
  Si4455Cmd.GET_INT_STATUS.CHIP_PEND      = radioCmd[6];
  Si4455Cmd.GET_INT_STATUS.CHIP_STATUS    = radioCmd[7];
  debugInterruptStatus();
}

void ZETA1::debugInterruptStatus(){
#if VERBOSE
    Serial.printf("INT_PEND: %X\t", Si4455Cmd.GET_INT_STATUS.INT_PEND);
    if( Si4455Cmd.GET_INT_STATUS.INT_PEND & B00000100){ Serial.print(" CHIP_INT_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.INT_PEND & B00000010){ Serial.print(" MODEM_INT_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.INT_PEND & B00000001){ Serial.print(" PH_INT_PEND");}
    Serial.println();
    Serial.printf("INT_STATUS: %X\t", Si4455Cmd.GET_INT_STATUS.INT_STATUS);
    if( Si4455Cmd.GET_INT_STATUS.INT_STATUS & B00000100){ Serial.print(" CHIP_INT_STATUS,");}
    if( Si4455Cmd.GET_INT_STATUS.INT_STATUS & B00000010){ Serial.print(" MODEM_INT_STATUS,");}
    if( Si4455Cmd.GET_INT_STATUS.INT_STATUS & B00000001){ Serial.print(" PH_INT_STATUS");}
    Serial.println();
    Serial.printf("PH_PEND: %X\t", Si4455Cmd.GET_INT_STATUS.PH_PEND);
    if( Si4455Cmd.GET_INT_STATUS.PH_PEND & B00100000){ Serial.print(" PACKET_SENT_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.PH_PEND & B00010000){ Serial.print(" PACKET_RX_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.PH_PEND & B00001000){ Serial.print(" CRC_ERROR_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.PH_PEND & B00000010){ Serial.print(" TX_FIFO_ALMOST_EMPTY_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.PH_PEND & B00000001){ Serial.print(" RX_FIFO_ALMOST_FULL_PEND");}
    Serial.println();
    Serial.printf("PH_STATUS: %X\t", Si4455Cmd.GET_INT_STATUS.PH_STATUS);
    if( Si4455Cmd.GET_INT_STATUS.PH_STATUS & B00100000){ Serial.print(" PACKET_SENT,");}
    if( Si4455Cmd.GET_INT_STATUS.PH_STATUS & B00010000){ Serial.print(" PACKET_RX,");}
    if( Si4455Cmd.GET_INT_STATUS.PH_STATUS & B00001000){ Serial.print(" CRC_ERROR,");}
    if( Si4455Cmd.GET_INT_STATUS.PH_STATUS & B00000010){ Serial.print(" TX_FIFO_ALMOST_EMPTY,");}
    if( Si4455Cmd.GET_INT_STATUS.PH_STATUS & B00000001){ Serial.print(" RX_FIFO_ALMOST_FULL");}
    Serial.println();
    Serial.printf("MODEM_PEND: %X\t", Si4455Cmd.GET_INT_STATUS.MODEM_PEND);
    if( Si4455Cmd.GET_INT_STATUS.MODEM_PEND & B00100000){ Serial.print(" INVALID_SYNC_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.MODEM_PEND & B00001000){ Serial.print(" RSSI_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.MODEM_PEND & B00000100){ Serial.print(" INVALID_PREAMBLE_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.MODEM_PEND & B00000010){ Serial.print(" PREAMBLE_DETECT_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.MODEM_PEND & B00000001){ Serial.print(" SYNC_DETECT_PEND ");}
    Serial.println();
    Serial.printf("MODEM_STATUS: %X\t", Si4455Cmd.GET_INT_STATUS.MODEM_STATUS);
    if( Si4455Cmd.GET_INT_STATUS.MODEM_STATUS & B00100000){ Serial.print(" INVALID_SYNC,");}
    if( Si4455Cmd.GET_INT_STATUS.MODEM_STATUS & B00001000){ Serial.print(" RSSI,");}
    if( Si4455Cmd.GET_INT_STATUS.MODEM_STATUS & B00000100){ Serial.print(" INVALID_PREAMBLE,");}
    if( Si4455Cmd.GET_INT_STATUS.MODEM_STATUS & B00000010){ Serial.print(" PREAMBLE_DETECT,");}
    if( Si4455Cmd.GET_INT_STATUS.MODEM_STATUS & B00000001){ Serial.print(" SYNC_DETECT");}
    Serial.println();
    Serial.printf("CHIP_PEND: %X\t", Si4455Cmd.GET_INT_STATUS.CHIP_PEND);
    if( Si4455Cmd.GET_INT_STATUS.CHIP_PEND & B00100000){ Serial.print(" FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.CHIP_PEND & B00010000){ Serial.print(" STATE_CHANGE_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.CHIP_PEND & B00001000){ Serial.print(" CMD_ERROR_PEND,");}
    if( Si4455Cmd.GET_INT_STATUS.CHIP_PEND & B00000100){ Serial.print(" CHIP_READY_PEND");}
    Serial.println();
    Serial.printf("CHIP_STATUS: %X\t", Si4455Cmd.GET_INT_STATUS.CHIP_STATUS);
    if( Si4455Cmd.GET_INT_STATUS.CHIP_STATUS & B00100000){ Serial.print(" FIFO_UNDERFLOW_OVERFLOW_ERROR,");}
    if( Si4455Cmd.GET_INT_STATUS.CHIP_STATUS & B00010000){ Serial.print(" STATE_CHANGE,");}
    if( Si4455Cmd.GET_INT_STATUS.CHIP_STATUS & B00001000){ Serial.print(" CMD_ERROR,");}
    if( Si4455Cmd.GET_INT_STATUS.CHIP_STATUS & B00000100){ Serial.print(" CHIP_READY");}
    Serial.println();
#endif
}

void ZETA1::writeEZConfigArray(byte byteCount, const byte* data) {
    vdebug("writeEZConfigArray ");
    // SI4455_CMD_ID_WRITE_TX_FIFO defined in si4455_defs.h 
    digitalWrite(CSPin, LOW);
    SPI.transfer(SI4455_CMD_ID_WRITE_TX_FIFO);
    for (byte i = 0; i < byteCount; i++) {
        SPI.transfer(data[i]);
        vdebughex(data[i]); vdebug(" ");
    }
    digitalWrite(CSPin, HIGH);
    vdebug("\n");
}

void ZETA1::reset() {
  debug("reset Radio\n");
  digitalWrite(SDNPin, HIGH); // shutdown radio
  delay(2);
  digitalWrite(SDNPin, LOW);// take radio out of shutdown
  // wait for power on reset(POR) to complete
  //while (!digitalRead(CTSPin)) {}
  delay(7000);
  debug("reset Complete\n");
}

byte ZETA1::nirqLevel() {
  byte v = digitalRead(nIRQPin);
  vdebug("read interrupt pin "); vdebug(v); vdebug("\n");
  return v;
}

void ZETA1::fifoInfo(bool resetRX, bool resetTX) {
  vdebug("reset FIFO\n");
  /*  0 FIFO_INFO 0x15
      1 FIFO 2 bit
          bit 0, reset TX or not
          bit 1, reset RX or not
  */
  radioCmd[0] = SI4455_CMD_ID_FIFO_INFO;
  radioCmd[1] = (resetRX << 1) | resetTX;
  // responds with the the number of bytes currently in the RX then TX buffer
  sendCmdGetResp(2, radioCmd, SI4455_CMD_REPLY_COUNT_FIFO_INFO, radioCmd);
  Si4455Cmd.FIFO_INFO.RX_FIFO_COUNT   = radioCmd[0];
  Si4455Cmd.FIFO_INFO.TX_FIFO_SPACE   = radioCmd[1];
}

void ZETA1::startRX() {
  debug("StartRX\n");
  /*  0 START_RX cmd 0x32
      1 CHANNEL 0 as center frequency
      2 RESERVED0 0
      3 RX_LEN 12 bit, upper 4 bits
      4 RX_LEN 12 bit, lower 8 bits
      5 NEXT_STATE1 see docs, 8=if preamble times out, exit and re-enter RX state
      6 NEXT_STATE2 see docs, 8=after reception of packet: exit and re-enter RX state
      7 NEXT_STATE3 see docs, 8=after bad packet: exit and re-enter RX state
  */
  // Read ITs, clear pending ones
  getInterruptStatus(0, 0, 0);
  /* Start Receiving packet, channel 0, START immediately, Packet n bytes long */
  radioCmd[0] = SI4455_CMD_ID_START_RX; // 0x32
  radioCmd[1] = channelNumber;
  radioCmd[2] = 0;
  radioCmd[3] = (byte)(packetLength >> 8);
  radioCmd[4] = (byte)(packetLength);
  radioCmd[5] = SI4455_CMD_START_RX_ARG_RXTIMEOUT_STATE_ENUM_RX;  //8
  radioCmd[6] = SI4455_CMD_START_RX_ARG_RXVALID_STATE_ENUM_RX;    //8
  radioCmd[7] = SI4455_CMD_START_RX_ARG_RXINVALID_STATE_ENUM_RX;  //8
  
  waitTillClearToSend();
  sendBytes(8, radioCmd);  
}

void ZETA1::startTX(byte *data) {
    debug("StartTX\n");
    // Read interrupts, clear pending ones
    getInterruptStatus(0, 0, 0);    
    // 0 WRITE_TX_FIFO 0x66
    // 1+ bytes to send
    // Fill the TX fifo buffer with data
    //waitTillClearToSend();
    digitalWrite(CSPin, LOW);
    SPI.transfer(SI4455_CMD_ID_WRITE_TX_FIFO);
    for (byte i = 0; i < packetLength; i++) {
        SPI.transfer(data[i]);
        vdebughex(data[i]); vdebug(" ");
    } vdebug("\n");
    digitalWrite(CSPin, HIGH);
    /*  0 SI4455_CMD_ID_START_TX 0x31
    1 CHANNEL 0 as center frequency
    2 CONDITION 0x30
      TXCOMPLETE STATE upper 4 bits see docs, 3=Ready state
      RETRANSMIT 3rd bit, see docs, 0=send data in tx fifo
    3 TX_LEN 12 bit, upper 4 bits
    4 TX_LEN 12 bit, lower 8 bits
    5 TX_DELAY, if retransmitting last packet?
    */
    // Start sending packet, channel 0, START immediately, Packet n bytes long, go READY when done
    radioCmd[0] = SI4455_CMD_ID_START_TX;
    radioCmd[1] = channelNumber;
    radioCmd[2] = 0x30;
    radioCmd[3] = (byte)(packetLength >> 8);
    radioCmd[4] = (byte)(packetLength);
    waitTillClearToSend();
    sendBytes(5, radioCmd);
}

byte ZETA1::checkReceived(byte *data) {
    if (nirqLevel() == 0) {
        vdebug("CheckReceived, an interrupt is pending\n");
        // Read interrupts, clear pending ones
        getInterruptStatus(0, 0, 0);
        // check for packet pending interrupt
        if (Si4455Cmd.GET_INT_STATUS.PH_PEND & SI4455_CMD_GET_INT_STATUS_REP_PACKET_RX_PEND_BIT) {
            // get byte count in FIFO
            fifoInfo(false, false);
            byte packetCount = Si4455Cmd.FIFO_INFO.RX_FIFO_COUNT / packetLength;
            debug("Received "); debug(packetCount); debug(" Packets\n");
            
            digitalWrite(CSPin, LOW);
            SPI.transfer(SI4455_CMD_ID_READ_RX_FIFO);
            for (int i = 0; i < packetLength * packetCount; i++) {
                data[i] = SPI.transfer(0xFF);
                vdebughex(data[i]); vdebug(" ");
            } vdebug("\n");
            digitalWrite(CSPin, HIGH);
            
            // check for errors
            if(Si4455Cmd.GET_INT_STATUS.PH_PEND & SI4455_CMD_GET_INT_STATUS_REP_CRC_ERROR_PEND_BIT){
                debug("ERROR: CRC_ERROR_PEND\n"); 
                //fifoInfo(false, true); getInterruptStatus(0, 0, 0);
                return 0;
            }
            if(Si4455Cmd.GET_INT_STATUS.PH_STATUS & SI4455_CMD_GET_INT_STATUS_REP_CRC_ERROR_BIT){
                debug("ERROR: CRC_ERROR\n"); 
                //fifoInfo(false, true); getInterruptStatus(0, 0, 0);
                return 0;
            }
            
            if( Si4455Cmd.GET_INT_STATUS.CHIP_PEND & SI4455_CMD_GET_INT_STATUS_REP_FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND_BIT){ 
                debug("ERROR: FIFO_UNDERFLOW_OVERFLOW_ERROR_PEND\n");
                //fifoInfo(false, true); getInterruptStatus(0, 0, 0);
                return 0;
            }
            if( Si4455Cmd.GET_INT_STATUS.CHIP_STATUS & SI4455_CMD_GET_INT_STATUS_REP_FIFO_UNDERFLOW_OVERFLOW_ERROR_BIT){ 
                debug("ERROR: FIFO_UNDERFLOW_OVERFLOW_ERROR\n");
                //fifoInfo(false, true); getInterruptStatus(0, 0, 0);
                return 0;
            }
            
            if( Si4455Cmd.GET_INT_STATUS.CHIP_PEND & SI4455_CMD_GET_INT_STATUS_REP_CMD_ERROR_PEND_BIT){ 
                debug("ERROR: CMD_ERROR_PEND\n");
                //fifoInfo(false, true); getInterruptStatus(0, 0, 0);
                return 0;
            }
            if( Si4455Cmd.GET_INT_STATUS.CHIP_STATUS & SI4455_CMD_GET_INT_STATUS_REP_CMD_ERROR_BIT){ 
                debug("ERROR: CMD_ERROR\n");
                //fifoInfo(false, true); getInterruptStatus(0, 0, 0);
                return 0;
            }

            // no errors
            return packetCount;            
        }
    }
    return 0;
}

bool ZETA1::checkTransmitted(void) {
  vdebug("checkTransmitted\n");
  if (nirqLevel() == 0) {
    // Read ITs, clear pending ones
    getInterruptStatus(0, 0, 0);
    // check the reason for the IT
    if (Si4455Cmd.GET_INT_STATUS.PH_PEND & SI4455_CMD_GET_INT_STATUS_REP_PACKET_SENT_PEND_BIT)
    {
      return true;
    }
  }
  return false;
}



