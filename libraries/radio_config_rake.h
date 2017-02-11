/*! @file radio_config.h
 * @brief This file contains the automatically generated
 * configurations.
 *
 * @n WDS GUI Version: 3.2.10.0
 * @n Device: Si4455 Rev.: B1                                 
 *
    API Rev B1A
    Freq. 915 Mhz, 250 kHz spacing
    FSK 20kbs
    preamble 8 bytes, std pattern
    sync word 2 bytes
    payload 16 bytes
    CRC transmit and checked at end
    Interrupts
        PH Packet_RX
        PH Packet_Sent
    GPIO
        NIRQ active low interrupt signal, pull-up
    FRR
        A Chip  inter pending
        B Modem inter pending
        C Packet Handler  inter pending
        D Latched RSSI value as defined in MODEM_RSSI_CONTROL
 */

#ifndef RADIO_CONFIG_H_
#define RADIO_CONFIG_H_
#include "si4455_defs.h"

// USER DEFINED PARAMETERS
// Define your own parameters here

// INPUT DATA
/*
// Crys_freq(Hz): 30000000    Crys_tol(ppm): 30    IF_mode: 2    High_perf_Ch_Fil: 1    OSRtune: 0    Ch_Fil_Bw_AFC: 0    ANT_DIV: 0    PM_pattern: 0    
// MOD_type: 2    Rsymb(sps): 20000    Fdev(Hz): 30000    RXBW(Hz): 185000    Manchester: 0    AFC_en: 1    Rsymb_error: 0.0    Chip-Version: 2    
// RF Freq.(MHz): 915    API_TC: 28    fhst: 250000    inputBW: 0    BERT: 0    RAW_dout: 0    D_source: 0    Hi_pfm_div: 0    
// 
// # WB filter 3 (BW = 185.22 kHz);  NB-filter 3 (BW = 185.22 kHz) 
// 
// Modulation index: 3
*/


// CONFIGURATION PARAMETERS
#define RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ                     30000000L
#define RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER                    0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH               0x10
#define RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP        0x03
#define RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET       0xF000


// CONFIGURATION COMMANDS

/*
// Command:                  RF_POWER_UP
// Description:              Command to power-up the device and select the operational mode and functionality.
*/
#define RF_POWER_UP 0x02, 0x01, 0x00, 0x01, 0xC9, 0xC3, 0x80

/*
// Set properties:           RF_INT_CTL_ENABLE_2
// Number of properties:     2
// Group ID:                 0x01
// Start ID:                 0x00
// Default values:           0x04, 0x00, 
// Descriptions:
//   INT_CTL_ENABLE - This property provides for global enabling of the three interrupt groups (Chip, Modem and Packet Handler) in order to generate HW interrupts at the NIRQ pin.
//   INT_CTL_PH_ENABLE - Enable individual interrupt sources within the Packet Handler Interrupt Group to generate a HW interrupt on the NIRQ output pin.
*/
#define RF_INT_CTL_ENABLE_2 0x11, 0x01, 0x02, 0x00, 0x01, 0x30

/*
// Set properties:           RF_FRR_CTL_A_MODE_4
// Number of properties:     4
// Group ID:                 0x02
// Start ID:                 0x00
// Default values:           0x01, 0x02, 0x09, 0x00, 
// Descriptions:
//   FRR_CTL_A_MODE - Fast Response Register A Configuration.
//   FRR_CTL_B_MODE - Fast Response Register B Configuration.
//   FRR_CTL_C_MODE - Fast Response Register C Configuration.
//   FRR_CTL_D_MODE - Fast Response Register D Configuration.
*/
#define RF_FRR_CTL_A_MODE_4 0x11, 0x02, 0x04, 0x00, 0x08, 0x06, 0x04, 0x0A

/*
// Set properties:           RF_EZCONFIG_XO_TUNE_1
// Number of properties:     1
// Group ID:                 0x24
// Start ID:                 0x03
// Default values:           0x40, 
// Descriptions:
//   EZCONFIG_XO_TUNE - Configure the internal capacitor frequency tuning bank for the crystal oscillator.
*/
#define RF_EZCONFIG_XO_TUNE_1 0x11, 0x24, 0x01, 0x03, 0x52

/*
// Command:                  RF_WRITE_TX_FIFO
// Description:              Writes data byte(s) to the TX FIFO.
*/
#define RF_WRITE_TX_FIFO 0x66, 0x92, 0x6B, 0xF5, 0xD2, 0x58, 0x06, 0xC7, 0x83, 0x63, 0x44, 0xF7, 0xB2, 0x44, 0x81, 0x49, 0x18, 0x1D, 0x1B, 0x3C, \
0x62, 0x79, 0x71, 0x52, 0xA6, 0xF9, 0x4A, 0x89, 0x2A, 0x3D, 0x58, 0x74, 0xC2, 0x4D, 0x23, 0x9F, 0x28, 0x1C, 0x95, 0x74, \
0x2A, 0x34, 0x3E, 0x10, 0x2D, 0xFC, 0x20, 0x58, 0xBB, 0x98, 0xC2, 0x76, 0x2B, 0xA2, 0x8A, 0xCA, 0x18, 0xBB, 0xBA, 0xA4, \
0x06, 0x32, 0xF6, 0xA4, 0x1E, 0x59, 0x7F, 0x12, 0x0D, 0xB5, 0x12, 0xD5, 0x3B, 0xC4, 0x26, 0x6F, 0x90, 0x04, 0xAB, 0xC1, \
0x83, 0x82, 0x0D, 0x00, 0xA6, 0x66, 0x82, 0xF6, 0x9D, 0x82, 0x6D, 0x8F, 0xB4, 0xFA, 0xCF, 0x12, 0x79, 0xF4, 0x0C, 0x41, \
0xBE, 0x91, 0xEC, 0x40, 0x2A, 0xA2, 0xDA, 0x4F, 0xDC, 0x54, 0x90, 0x2B, 0x59, 0x59

/*
// Command:                  RF_NOP
// Description:              No Operation command.
*/
#define RF_NOP 0x00

/*
// Command:                  RF_WRITE_TX_FIFO_1
// Description:              Writes data byte(s) to the TX FIFO.
*/
#define RF_WRITE_TX_FIFO_1 0x66, 0x15, 0xFF, 0xFA, 0x9F, 0xE5, 0x53, 0x1E, 0x36, 0x96, 0xC8, 0x86, 0x28, 0xE2, 0x80, 0xA0, 0xA8, 0x8F, 0xF5, 0xDE, \
0x08, 0x19, 0x45, 0x8E, 0x6B, 0x65, 0x68, 0xA0, 0x6B, 0xA5, 0x9B, 0x6A, 0xFA, 0x75, 0x1B, 0x84, 0x23, 0x97, 0x93, 0xAB, \
0x9D, 0x79, 0x7B, 0x23, 0x28, 0x31, 0x7D, 0x0D, 0xE2, 0x55, 0x3F, 0x60, 0x30, 0x2F, 0x66, 0xDF, 0x64, 0x91, 0x16, 0xB8, \
0x10, 0xF2, 0x67, 0xDD, 0x61, 0x83, 0x6E, 0xC4, 0x0E, 0xBD, 0x84, 0x29, 0x13, 0x7D, 0x4F, 0x33, 0xF5, 0x93, 0x73, 0xE0, \
0x92, 0x2D, 0xA6, 0x95, 0x5E, 0xB8, 0x79, 0x26, 0x3A, 0x45, 0x23, 0xEF, 0x38, 0xAF, 0x13, 0x23, 0x3D, 0x49, 0x50, 0xF0, \
0x3D, 0xAF, 0x48, 0xFD, 0x4C, 0xAD, 0x6E, 0x20, 0x56, 0x39, 0x45, 0xA8

/*
// Command:                  RF_EZCONFIG_CHECK
// Description:              Validates the EZConfig array was written correctly.
*/
#define RF_EZCONFIG_CHECK 0x19, 0xAB, 0x29

/*
// Command:                  RF_GPIO_PIN_CFG
// Description:              Configures the GPIO pins.
*/
#define RF_GPIO_PIN_CFG 0x13, 0x01, 0x01, 0x01, 0x01, 0x67, 0x00, 0x00


// AUTOMATICALLY GENERATED CODE! 
// DO NOT EDIT/MODIFY BELOW THIS LINE!
// --------------------------------------------

#ifndef FIRMWARE_LOAD_COMPILE
#define RADIO_CONFIGURATION_DATA_ARRAY { \
        0x07, RF_POWER_UP, \
        0x06, RF_INT_CTL_ENABLE_2, \
        0x08, RF_FRR_CTL_A_MODE_4, \
        0x05, RF_EZCONFIG_XO_TUNE_1, \
        0x72, RF_WRITE_TX_FIFO, \
        0x01, RF_NOP, \
        0x70, RF_WRITE_TX_FIFO_1, \
        0x03, RF_EZCONFIG_CHECK, \
        0x08, RF_GPIO_PIN_CFG, \
        0x00 \
 }
#else
#define RADIO_CONFIGURATION_DATA_ARRAY { 0 }
#endif

// DEFAULT VALUES FOR CONFIGURATION PARAMETERS
#define RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ_DEFAULT                     30000000L
#define RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER_DEFAULT                    0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH_DEFAULT               0x10
#define RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP_DEFAULT        0x01
#define RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET_DEFAULT       0x1000

#define RADIO_CONFIGURATION_DATA_RADIO_PATCH_INCLUDED                      0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PATCH_SIZE                          0x00
#define RADIO_CONFIGURATION_DATA_RADIO_PATCH                               {  }

#ifndef RADIO_CONFIGURATION_DATA_ARRAY
#error "This property must be defined!"
#endif

#ifndef RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ
#define RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ          RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ_DEFAULT 
#endif

#ifndef RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER
#define RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER         RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER_DEFAULT 
#endif

#ifndef RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH
#define RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH    RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH_DEFAULT 
#endif

#ifndef RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP
#define RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP   RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP_DEFAULT 
#endif

#ifndef RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET
#define RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET  RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET_DEFAULT 
#endif

#define RADIO_CONFIGURATION_DATA { \
                            Radio_Configuration_Data_Array,                            \
                            RADIO_CONFIGURATION_DATA_CHANNEL_NUMBER,                   \
                            RADIO_CONFIGURATION_DATA_RADIO_PACKET_LENGTH,              \
                            RADIO_CONFIGURATION_DATA_RADIO_STATE_AFTER_POWER_UP,       \
                            RADIO_CONFIGURATION_DATA_RADIO_DELAY_CNT_AFTER_RESET       \
                            }

#endif /* RADIO_CONFIG_H_ */
