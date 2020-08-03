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

#include "CubeCell.h"

// debug
#if defined(MY_DEBUG_VERBOSE_CUBECELL)
#define CUBECELL_DEBUG(x, ...) DEBUG_OUTPUT(x, ##__VA_ARGS__) //!< Debug print
#else
#define CUBECELL_DEBUG(x, ...) //!< DEBUG null
#endif

cubecell_internal_t CUBECELL; //!< internal variables

const inline int8_t CubeCell_RSSItoInternal(const int16_t externalRSSI)
{
    return static_cast<int8_t>(externalRSSI + CUBECELL_RSSI_OFFSET);
}

const inline int16_t CubeCell_internalToRSSI(const int8_t internalRSSI)
{
    return static_cast<int16_t>(internalRSSI - CUBECELL_RSSI_OFFSET);
}

static void onTxDone()
{
    Radio.Rx(0);
    CUBECELL.txDone = true;
}

static void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    CUBECELL.dataReceived = false;
    CUBECELL.ackReceived = false;
    CUBECELL.rxTimeout = false;
    if (size > CUBECELL_HEADER_LEN)
    {
        CUBECELL.currentPacket.RSSI = CubeCell_RSSItoInternal(rssi);
        CUBECELL.currentPacket.SNR = snr;
        CUBECELL.currentPacket.payloadLen = size - CUBECELL_HEADER_LEN;
        memcpy(CUBECELL.currentPacket.data, payload, size);

        //Message for us?
        if ((CUBECELL.currentPacket.header.version >= CUBECELL_MIN_PACKET_HEADER_VERSION) &&
            (CUBECELL.currentPacket.header.recipient == CUBECELL.address ||
             CUBECELL.currentPacket.header.recipient == CUBECELL_BROADCAST_ADDRESS))
        {
            CUBECELL.ackReceived = CUBECELL_getACKReceived(CUBECELL.currentPacket.header.controlFlags) &&
                                   !CUBECELL_getACKRequested(CUBECELL.currentPacket.header.controlFlags);
            CUBECELL.dataReceived = !CUBECELL.ackReceived;
        }
    }
    else
    {
        CUBECELL_DEBUG(PSTR("!CCELL:IRH:CRC ERROR\n"));
    }
}

static void onTxTimeout()
{
    Radio.Rx(0);
    CUBECELL.txTimeout = true;
}

static void onRxTimeout()
{
    CUBECELL_DEBUG(PSTR("!CCELL:IRH:RX TIMEOUT\n"));
    CUBECELL.rxTimeout = true;
}

static bool CubeCell_init()
{
    CUBECELL_DEBUG(PSTR("CCELL:INIT\n"));
    CUBECELL.address = CUBECELL_BROADCAST_ADDRESS;
    CUBECELL.ackReceived = false;
    CUBECELL.dataReceived = false;
    CUBECELL.txSequenceNumber = 0;
    CUBECELL.powerLevel = MY_CUBECELL_TX_POWER_DBM;
    CUBECELL.ATCenabled = true;
    CUBECELL.ATCtargetRSSI = CubeCell_RSSItoInternal(CUBECELL_TARGET_RSSI);
    CUBECELL.txDone = false;
    CUBECELL.txTimeout = false;
    CUBECELL.rxTimeout = false;

    boardInitMcu();

    // Radio callbacks
    static RadioEvents_t RadioEvents;
    RadioEvents.TxDone = onTxDone;
    RadioEvents.RxDone = onRxDone;
    RadioEvents.TxTimeout = onTxTimeout;
    RadioEvents.RxTimeout = onRxTimeout;

    // Init radio
    Radio.Init(&RadioEvents);

    // Setting frequency
    Radio.SetChannel(MY_CUBECELL_FREQUENCY);

    // Setting modem parameters
    Radio.SetTxConfig(MODEM_LORA, CUBECELL.powerLevel, /*fdev*/ 0,
                      CUBECELL_BW, CUBECELL_SF, CUBECELL_CR, CUBECELL_PREAMBLE_LENGTH,
                      /*fixed length*/ false, /*CRC*/ true, /*freqHopOn*/ false, /*hopPeriod*/ 0,
                      /*irqInverted*/ false, CUBECELL_TX_TIMEOUT_MS);
    Radio.SetRxConfig(MODEM_LORA, CUBECELL_BW, CUBECELL_SF, CUBECELL_CR, /*bwAfc*/ 0,
                      CUBECELL_PREAMBLE_LENGTH, /*symbTimeout*/ 0, /*fixLen*/ false, /*payloadLen*/ 0,
                      /*CRC*/ true, /*freqHopOn*/ false, /*hopPeriod*/ 0, /*irqInverted*/ false,
                      /*rcContinous*/ true);

    Radio.SetSyncWord(0x12);

    // Do a sanity check
    if (!CubeCell_sanityCheck())
    {
        CUBECELL_DEBUG(PSTR("!CCUBE:INIT:SANCHK FAIL\n"));
        return false;
    }

    return true;
}

void CubeCell_setPowerLevel(uint8_t newPowerLevel)
{
    if (newPowerLevel != CUBECELL.powerLevel &&
        newPowerLevel < CUBECELL_MAX_POWER_LEVEL_DBM &&
        newPowerLevel > CUBECELL_MIN_POWER_LEVEL_DBM)
    {
        CUBECELL.powerLevel = newPowerLevel;
        Radio.SetTxConfig(MODEM_LORA, CUBECELL.powerLevel, /*fdev*/ 0,
                          CUBECELL_BW, CUBECELL_SF, CUBECELL_CR, CUBECELL_PREAMBLE_LENGTH,
                          /*fixed length*/ false, /*CRC*/ true, /*freqHopOn*/ false, /*hopPeriod*/ 0,
                          /*irqInverted*/ false, CUBECELL_TX_TIMEOUT_MS);
    }
}
static void CubeCell_executeATC(const uint8_t currentRSSI, const uint8_t targetRSSI)
{
    int8_t newPowerLevel = CUBECELL.powerLevel;
    const int16_t ownRSSI = CubeCell_internalToRSSI(currentRSSI);
    const int16_t uRange = CubeCell_internalToRSSI(targetRSSI) + CUBECELL_ATC_TARGET_RANGE_DBM;
    const int16_t lRange = CubeCell_internalToRSSI(targetRSSI) - CUBECELL_ATC_TARGET_RANGE_DBM;
    if (ownRSSI < lRange && CUBECELL.powerLevel < CUBECELL_MAX_POWER_LEVEL_DBM)
    {
        // increase transmitter power
        newPowerLevel++;
    }
    else if (ownRSSI > uRange && CUBECELL.powerLevel > CUBECELL_MIN_POWER_LEVEL_DBM)
    {
        // decrease transmitter power
        newPowerLevel--;
    }
    if (newPowerLevel != CUBECELL.powerLevel)
    {
        CubeCell_setPowerLevel(newPowerLevel);
        CUBECELL_DEBUG(PSTR("CCELL:ATC:ADJ TXL,cR=%d,tR=%d..%d,TXL=%d\n"),
                       ownRSSI, lRange, uRange, CUBECELL.powerLevel);
    }
}

static bool CubeCell_sendFrame(cubecell_packet_t *packet, const bool increaseSequenceCounter)
{
    if (increaseSequenceCounter)
    {
        // increase sequence counter, overflow is ok
        CUBECELL.txSequenceNumber++;
    }
    packet->header.sequenceNumber = CUBECELL.txSequenceNumber;
    // write packet
    const uint8_t finalLen = packet->payloadLen + CUBECELL_HEADER_LEN;
    CUBECELL.txTimeout = false;
    CUBECELL.txDone = false;
    Radio.Send(packet->data, finalLen);
    while (!(CUBECELL.txDone || CUBECELL.txTimeout))
    {
        Radio.IrqProcess();
    }
    return !CUBECELL.txTimeout;
}

static bool CubeCell_send(const uint8_t recipient, uint8_t *data, const uint8_t len,
                          const uint8_t flags, const bool increaseSequenceCounter)
{
    cubecell_packet_t packet;
    packet.header.version = CUBECELL_PACKET_HEADER_VERSION;
    packet.header.sender = CUBECELL.address;
    packet.header.recipient = recipient;
    packet.payloadLen = min(len, (uint8_t)CUBECELL_MAX_PAYLOAD_LEN);
    packet.header.controlFlags = flags;
    (void)memcpy((void *)&packet.payload, (void *)data, packet.payloadLen);
    return CubeCell_sendFrame(&packet, increaseSequenceCounter);
}

static void CubeCell_sendACK(const uint8_t recipient, const uint16_t sequenceNumber,
                             const uint8_t RSSI, const int8_t SNR)
{
    CUBECELL_DEBUG(PSTR("CCELL:SAC:SEND ACK,TO=%u,SEQ=%u,RSSI=%i,SNR=%i\n"),
                   recipient, sequenceNumber,
                   CubeCell_internalToRSSI(RSSI), SNR);
    cubecell_ack_t ACK;
    ACK.sequenceNumber = sequenceNumber;
    ACK.RSSI = RSSI;
    ACK.SNR = SNR;
    uint8_t flags = 0u;
    CUBECELL_setACKReceived(flags, true);
    CUBECELL_setACKRSSIReport(flags, true);
    (void)CubeCell_send(recipient, (uint8_t *)&ACK, sizeof(cubecell_ack_t), flags, true);
}

static bool CubeCell_sendRetry(const uint8_t recipient, const void *buffer,
                               const uint8_t bufferSize, const bool noACK)
{
    for (uint8_t retry; retry < 5; retry++)
    {
        CUBECELL_DEBUG(PSTR("CCELL:SWR:SEND,TO=%u,SEQ=%u,RETRY=%u,AT=%ums\n"),
                       recipient,
                       retry == 0 ? CUBECELL.txSequenceNumber + 1 : CUBECELL.txSequenceNumber,
                       retry,
                       Radio.TimeOnAir(MODEM_LORA, bufferSize + CUBECELL_HEADER_LEN));
        uint8_t flags = 0;
        CUBECELL_setACKRequested(flags, !noACK);
        if (!CubeCell_send(recipient, (uint8_t *)buffer, bufferSize, flags, retry == 0 ? true : false))
        {
            return false;
        }
        if (noACK)
        {
            return true;
        }
        // receive for some ms
        uint32_t start = hwMillis();
        while (hwMillis() - start < 1000 && !CUBECELL.dataReceived)
        {
            Radio.IrqProcess();
            if (CUBECELL.ackReceived)
            {
                CUBECELL.ackReceived = false;
                const uint8_t sender = CUBECELL.currentPacket.header.sender;
                const uint16_t sequenceNumber = CUBECELL.currentPacket.ACK.sequenceNumber;
                const uint8_t flags = CUBECELL.currentPacket.header.controlFlags;
                const uint8_t RSSI = CUBECELL.currentPacket.ACK.RSSI;
                if (sender == recipient &&
                    sequenceNumber == CUBECELL.txSequenceNumber)
                {
                    CUBECELL_DEBUG(PSTR("CCELL:SWR:ACK FROM=%u,SEQ=%u,RSSI=%d\n"),
                                   sender,
                                   sequenceNumber,
                                   CubeCell_internalToRSSI(RSSI));
                    if (CUBECELL.ATCenabled && CUBECELL_getACKRSSIReport(flags))
                    {
                        CubeCell_executeATC(RSSI, CUBECELL.ATCtargetRSSI);
                    }
                    return true;
                }
            }
        }
    }
    int32_t start = hwMillis();
    CUBECELL_DEBUG(PSTR("!CCELL:SWR:NACK\n"));
    while (hwMillis() - start < random() % 99)
    {
        Radio.IrqProcess();
    }
    if (CUBECELL.ATCenabled)
    {
        // Raise power level in case we are out of reach
        CubeCell_setPowerLevel(CUBECELL.powerLevel + 2);
    }
    return false;
}

static bool CubeCell_sanityCheck()
{
    if (Radio.Rssi(MODEM_FSK) != 0)
    {
        return true;
    }
    return false;
}

static void CubeCell_setAddress(const uint8_t addr)
{
    CUBECELL.address = addr;
}

static uint8_t CubeCell_getAddress(void)
{
    return CUBECELL.address;
}

static bool CubeCell_dataAvailable()
{
    if (!(Radio.GetStatus() == RF_RX_RUNNING || Radio.GetStatus() == RF_TX_RUNNING))
    {
        Radio.Rx(0);
    }
    return CUBECELL.dataReceived;
}

const uint8_t CubeCell_getData(uint8_t *buf, const uint8_t maxBufSize)
{
    uint8_t flags = CUBECELL.currentPacket.header.controlFlags;
    if (CUBECELL_getACKRequested(flags) && !CUBECELL_getACKReceived(flags))
    {
        CubeCell_sendACK(
            CUBECELL.currentPacket.header.sender,
            CUBECELL.currentPacket.header.sequenceNumber,
            CUBECELL.currentPacket.RSSI,
            CUBECELL.currentPacket.SNR);
    }
    const uint8_t payloadLen = min(CUBECELL.currentPacket.payloadLen, maxBufSize);
    if (buf != NULL)
    {
        memcpy(buf, CUBECELL.currentPacket.payload, payloadLen);
    }
    CUBECELL.dataReceived = false;
    return payloadLen;
}

const void CubeCell_handler()
{
    Radio.IrqProcess();
}

static void CubeCell_ATCmode(const bool OnOff, const int16_t targetRSSI)
{
    CUBECELL.ATCenabled = OnOff;
    CUBECELL.ATCtargetRSSI = CubeCell_RSSItoInternal(targetRSSI);
}
