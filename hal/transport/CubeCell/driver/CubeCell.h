/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2019 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * Partly based on the MySensors RFTM95 driver, Copyright (C) 2017-2018 Olivier Mauti <olivier@mysensors.org>
 *
 * CubeCell driver written for MySensors, Copyright (C) 2020 Eduard Iten <eduard@iten.pro>
 *
 */
/**
* @file CubeCell.h
*
* @defgroup CubeCellgrp CubeCell
* @ingroup internals
* @{
*
* CubeCell driver-related log messages, format: [!]SYSTEM:[SUB SYSTEM:]MESSAGE
* - [!] Exclamation mark is prepended in case of error
*
* |E| SYS   | SUB  | Message                                | Comment
* |-|-------|------|----------------------------------------|-----------------------------------------------------------------------------------
* | | CCELL | INIT |                                        | Initialise LoRa App
* |!| CCELL | INIT | SANCHK FAIL                            | Sanity check failed, check module
* |!| CCELL | IRH  | CRC FAIL                               | Incoming packet has CRC error, skip
* | | CCELL | RCV  | SEND ACK                               | ACK request received, sending ACK back
* | | CCELL | PTC  | LEVEL=%%d                              | Set TX power level
* | | CCELL | SAC  | SEND ACK,TO=%%d,RSSI=%%d,SNR=%%d       | Send ACK to node (TO), RSSI of received message (RSSI), SNR of message (SNR)
* | | CCELL | ATC  | ADJ TXL,cR=%%d,tR=%%d..%%d,TXL=%%d     | Adjust TX level, current RSSI (cR), target RSSI range (tR), TX level (TXL)
* | | CCELL | SWR  | SEND,TO=%%d,RETRY=%%d,AT=%%dms         | Send message to (TO), NACK retry counter (RETRY), Airtime (AT)ms
* | | CCELL | SWR  | ACK FROM=%%d,SEQ=%%d,RSSI=%%d,SNR=%%d  | ACK received from node (FROM), seq ID (SEQ), (RSSI), (SNR)
* |!| CCELL | SWR  | NACK                                   | No ACK received
* | | CCELL | SPP  | PCT=%%d,TX LEVEL=%%d                   | Set TX level percent (PCT), TX level (LEVEL)
* | | CCELL | PWD  |                                        | Power down radio
* | | CCELL | PWU  |                                        | Power up radio
*
* Cubecell modem configuration
*
* BW = Bandwidth in kHz
* CR = Error correction code
* SF = Spreading factor, chips / symbol
*
* | CONFIG           | BW    | CR  | SF   | Comment               | air-time (15 bytes)
* |------------------|-------|-----|------|-----------------------|------------------------
* | BW125CR45SF128   | 125   | 4/5 | 128  | Default, medium range | 50ms
* | BW500CR45SF128   | 500   | 4/5 | 128  | Fast, short range     | 15ms
* | BW125CR48SF4096  | 125   | 4/8 | 4096 | Slow, long range      | 1500ms
*
* See here for air-time calculation: https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc
*
* @brief API declaration for CubeCell
*
*/

#include <Arduino.h>
#include <LoRaWan_APP.h>
#include <CubeCell_NeoPixel.h>

#ifndef _CubeCell_h
#define _CubeCell_h

// CubeCell standard frequencies
#define CUBECELL_169MHZ (169000000ul) //!< 169 Mhz
#define CUBECELL_315MHZ (315000000ul) //!< 315 Mhz
#define CUBECELL_434MHZ (433920000ul) //!< 433.92 Mhz
#define CUBECELL_868MHZ (868100000ul) //!< 868.1 Mhz
#define CUBECELL_915MHZ (915000000ul) //!< 915 Mhz

#define CUBECELL_MAX_PACKET_LEN (0x40u) //!< This is the maximum number of bytes that can be carried by the LORA
#define CUBECELL_PREAMBLE_LENGTH (8u)   //!< Preamble length, default=8

#define CUBECELL_PACKET_HEADER_VERSION (1u)     //!< CubeCell packet header version
#define CUBECELL_MIN_PACKET_HEADER_VERSION (1u) //!< Minimal Cubecell packet header version
#define CUBECELL_BIT_ACK_REQUESTED (7u)         //!< CubeCell header, controlFlag, bit 7
#define CUBECELL_BIT_ACK_RECEIVED (6u)          //!< CubeCell header, controlFlag, bit 6
#define CUBECELL_BIT_ACK_RSSI_REPORT (5u)       //!< CubeCell header, controlFlag, bit 5

#define CUBECELL_BROADCAST_ADDRESS (255u)  //!< Broadcasting address
#define CUBECELL_RSSI_OFFSET (137u)        //!< RSSI offset
#define CUBECELL_TARGET_RSSI (-70l)        //!< ATC target rssi
#define CUBECELL_ATC_TARGET_RANGE_DBM (2u) //!< ATC target range +/- dBm
#define CUBECELL_MIN_POWER_LEVEL_DBM (0u)  //!< min. power level
#if defined(MY_CUBECELL_MAX_POWER_LEVEL_DBM)
#define CUBECELL_MAX_POWER_LEVEL_DBM MY_CUBECELL_MAX_POWER_LEVEL_DBM //!< MY_CUBECELL_MAX_POWER_LEVEL_DBM
#else
#define CUBECELL_MAX_POWER_LEVEL_DBM (20u) //!< max. power level
#endif

// Modem settings and timeout
#define CUBECELL_SF7 (7u)             //!< Spreading factor 7 (128 Chips/bit)
#define CUBECELL_SF12 (12u)           //!< Spreading factor 12 (4096 Chips/bit)
#define CUBECELL_BW125 (0u)           //!< Bandwidth 125kHz
#define CUBECELL_BW500 (2u)           //!< Bandwidth 500kHz
#define CUBECELL_CR45 (1u)            //!< Coding rate 4/5
#define CUBECELL_CR48 (4u)            //!< Coding rate 4/8
#define CUBECELL_PREAMBLE_LENGTH (8u) //!< Preamble length

// Colors for the LED
#define CUBECELL_COLOR_TX 0x000F00      //!< Color during transmitting, green
#define CUBECELL_COLOR_TXDONE 0x0F0F00  //!< Color when TX done, yellow
#define CUBECELL_COLOR_ACKWAIT 0x00000F //!< Color waiting for an ack, blue
#define CUBECELL_COLOR_RXDONE 0x0F000F  //!< Color when RX done, purple
#define CUBECELL_COLOR_TIMEOUT 0x0F0000 //!< Color on Timeout, red
#define CUBECELL_COLOR_FLASHTIME (200u) //!< Flash time after RX, RX and Timeout

// For the timeouts, see https://www.loratools.nl/#/airtime.
#if MY_CUBECELL_MODEM_CONFIGRUATION == CUBECELL_BW125CR45SF128
#define CUBECELL_SF CUBECELL_SF7          //!< Spreading factor 7
#define CUBECELL_CR CUBECELL_CR45         //!< Coding rate 4/5
#define CUBECELL_BW CUBECELL_BW125        //!< Bandwidth 125kHz
#define CUBECELL_RETRY_TIMEOUT_MS (500ul) //!< 160ms Acknowledge timeout
#define CUBECELL_TX_TIMEOUT_MS (1500ul)   //!< 400ms TX timeout
#elif MY_CUBECELL_MODEM_CONFIGRUATION == CUBECELL_BW125CR48SF4096
#define CUBECELL_SF CUBECELL_SF12          //!< Spreading factor 12u
#define CUBECELL_CR CUBECELL_CR48          //!< Coding rate 4/5
#define CUBECELL_BW CUBECELL_BW125         //!< Bandwidth 125kHz
#define CUBECELL_RETRY_TIMEOUT_MS (6000ul) //!< 4.3us Acknowledge timeout
#define CUBECELL_TX_TIMEOUT_MS (20000u)    //!< 10.5s TX timeout
#elif MY_CUBECELL_MODEM_CONFIGRUATION == CUBECELL_BW500CR45SF128
#define CUBECELL_SF CUBECELL_SF7          //!< Spreading factor 7
#define CUBECELL_CR CUBECELL_CR45         //!< Coding rate 4/5
#define CUBECELL_BW CUBECELL_BW500        //!< Bandwidth 125kHz
#define CUBECELL_RETRY_TIMEOUT_MS (250ul) //!< 50ms Acknowledge timeout
#define CUBECELL_TX_TIMEOUT_MS (500ul)    //!< 110ms TX timeout
#endif

// helper macros
#define CUBECELL_getACKRequested(__value) ((bool)bitRead(__value, CUBECELL_BIT_ACK_REQUESTED))             //!< getACKRequested
#define CUBECELL_setACKRequested(__value, __flag) bitWrite(__value, CUBECELL_BIT_ACK_REQUESTED, __flag)    //!< setACKRequested
#define CUBECELL_getACKReceived(__value) ((bool)bitRead(__value, CUBECELL_BIT_ACK_RECEIVED))               //!< getACKReceived
#define CUBECELL_setACKReceived(__value, __flag) bitWrite(__value, CUBECELL_BIT_ACK_RECEIVED, __flag)      //!< setACKReceived
#define CUBECELL_setACKRSSIReport(__value, __flag) bitWrite(__value, CUBECELL_BIT_ACK_RSSI_REPORT, __flag) //!< setACKRSSIReport
#define CUBECELL_getACKRSSIReport(__value) ((bool)bitRead(__value, CUBECELL_BIT_ACK_RSSI_REPORT))          //!< getACKRSSIReport

/**
 * @brief CubeCell Modem Configurations
 **/
enum
{
    CUBECELL_BW125CR45SF128,
    CUBECELL_BW500CR45SF128,
    CUBECELL_BW125CR48SF4096
};

/**
* @brief CubeCell LoRa header
*/
typedef struct
{
    uint8_t version;         //!< Header version
    uint8_t recipient;       //!< Payload recipient
    uint8_t sender;          //!< Payload sender
    uint8_t controlFlags;    //!< Control flags, used for ACK
    uint16_t sequenceNumber; //!< Packet sequence number, used for ACK
} __attribute__((packed)) cubecell_header_t;

/**
* @brief CubeCell LoRa ACK packet structure
*/
typedef struct
{
    uint16_t sequenceNumber; //!< sequence number
    uint8_t RSSI;            //!< RSSI
    int8_t SNR;              //!< SNR
} __attribute__((packed)) cubecell_ack_t;

#define CUBECELL_HEADER_LEN sizeof(cubecell_header_t)                            //!< Size header inside LoRa payload
#define CUBECELL_MAX_PAYLOAD_LEN (CUBECELL_MAX_PACKET_LEN - CUBECELL_HEADER_LEN) //!< Max payload length

/**
* @brief LoRa packet structure
*/
typedef struct
{
    union
    {
        struct
        {
            cubecell_header_t header; //!< LoRa header
            union
            {
                uint8_t payload[CUBECELL_MAX_PAYLOAD_LEN]; //!< Payload, i.e. MySensors message
                cubecell_ack_t ACK;                        //!< Union: ACK
            };
        };
        uint8_t data[CUBECELL_MAX_PACKET_LEN]; //!< RAW
    };
    uint8_t payloadLen; //!< Length of payload (excluding header)
    uint8_t RSSI;       //!< RSSI of current packet, RSSI = value - 137
    int8_t SNR;         //!< SNR of current packet
} __attribute__((packed)) cubecell_packet_t;

/**
* @brief CubeCell internal variables
*/
typedef struct
{
    uint8_t address;                 //!< Node address
    cubecell_packet_t currentPacket; //!< Buffer for current packet
    uint16_t txSequenceNumber;       //!< CubeCell_txSequenceNumber
    int8_t powerLevel;               //!< TX power level dBm
    uint8_t ATCtargetRSSI;           //!< ATC: target RSSI
    // 8 bit
    RadioState_t radioMode; //!< current transceiver state
    bool channelActive;     //!< CubeCell_cad
    bool ATCenabled;        //!< ATC enabled
    bool ackReceived;       //!< ACK received
    bool dataReceived;      //!< Data received
    bool txDone;            //!< TX is completed
    bool rxTimeout;         //!< RX timed out
    bool txTimeout;         //!< TX timed out
} cubecell_internal_t;

/**
* @brief Initialise the driver transport hardware and software
* @return True if initialisation succeeded
*/
static bool CubeCell_init();

/**
* @brief Check if we can talk to the Radio
* @return True if sanity check passed, else check frequency or module
*/
static bool CubeCell_sanityCheck();

/**
* @brief cubecell_send
* @param recipient
* @param buffer
* @param bufferSize
* @param noACK
* @return True if packet successfully sent
*/
static bool CubeCell_sendRetry(const uint8_t recipient, const void *buffer,
                               const uint8_t bufferSize, const bool noACK);

#endif

/**
* @brief Set the node address
* @param addr
*/
static void CubeCell_setAddress(const uint8_t addr);

/**
* @brief Get the node address
* @return Node address
*/
static uint8_t CubeCell_getAddress(void);

/**
* @brief Tests whether new data is available
* @return True if a new, complete, error-free uncollected message is available to be retreived by @ref CubeCell_getData()
*/
static bool CubeCell_dataAvailable(void);

/**
* @brief If a valid message is received, copy it to buf and return length. 0 byte messages are permitted.
* @param buf Location to copy the received message
* @param maxBufSize Max buffer size
* @return Number of bytes
*/
const uint8_t CubeCell_getData(uint8_t *buf, const uint8_t maxBufSize);

/**
 * @brief Handles radio events
 **/
const void CubeCell_handler(void);

/**
* @brief CubeCell_ATCmode
* @param targetRSSI Target RSSI for transmitter (default -70)
* @param OnOff True to enable ATC
*/
static void CubeCell_ATCmode(const bool OnOff, const int16_t targetRSSI);

/** @}*/