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
 */

#include "driver/CubeCell.h"

bool transportInit(void)
{
	const bool result = CubeCell_init();
	return result;
}

void transportSetAddress(const uint8_t address)
{
		CubeCell_setAddress(address);
}

uint8_t transportGetAddress(void)
{
	return CubeCell_getAddress();
}

bool transportSend(const uint8_t to, const void *data, const uint8_t len, const bool noACK)
{
	return CubeCell_sendRetry(to, data, len, noACK);
}

bool transportDataAvailable(void)
{
	CubeCell_handler();
	return CubeCell_dataAvailable();
}

bool transportSanityCheck(void)
{
	return CubeCell_sanityCheck;
}

uint8_t transportReceive(void *data)
{
	return CubeCell_getData((uint8_t*)data, MAX_MESSAGE_SIZE);
}

void transportSleep(void)
{
	Radio.Sleep();
}

void transportStandBy(void)
{
	Radio.Standby();
}

void transportPowerDown(void)
{
	Radio.Sleep();
}

void transportPowerUp(void)
{
	Radio.Rx(0);
}

void transportToggleATCmode(const bool OnOff, const int16_t targetRSSI)
{
	CubeCell_ATCmode(OnOff, targetRSSI);
}

int16_t transportGetSendingRSSI(void)
{
	return 0;
}

int16_t transportGetReceivingRSSI(void)
{
	return 0;
}

int16_t transportGetSendingSNR(void)
{
	return 0;
}

int16_t transportGetReceivingSNR(void)
{
	return 0;
}

int16_t transportGetTxPowerPercent(void)
{
	return 0;
}

int16_t transportGetTxPowerLevel(void)
{
	return 0;
}

bool transportSetTxPowerPercent(const uint8_t powerPercent)
{
	return 0;
}

