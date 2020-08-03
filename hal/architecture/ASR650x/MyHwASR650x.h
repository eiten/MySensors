/**
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2020 Sensnology AB
* Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* Radio wiring Teensy3.x: RF24, RFM69, RFM95:
* MISO	12
* MOSI	11
* SCK	13
* CSN	10
* CE	9 (RF24)
* IRQ	8 (opt. RF24, RFM69, RFM95)
*/

#ifndef MyHwASR650x_h
#define MyHwASR650x_h

#include <SPI.h>
#include "atomic"
#include "board-config.h"

#ifdef __cplusplus
#include <Arduino.h>
#endif

#define CRYPTO_BIG_ENDIAN

#ifndef MY_SERIALDEVICE
#define MY_SERIALDEVICE Serial
#endif

#ifndef MY_DEBUGDEVICE
#define MY_DEBUGDEVICE MY_SERIALDEVICE
#endif

// Define these as macros to save valuable space
#define hwDigitalWrite(__pin, __value) digitalWrite(__pin, __value)
#define hwDigitalRead(__pin) digitalReadFast(__pin)
#define hwPinMode(__pin, __value) pinMode(__pin, __value)
#define hwMillis() millis()
#define hwGetSleepRemaining() (0ul)

void hwRandomNumberInit(void);
bool hwInit(void);
void hwWatchdogReset(void);
void hwReboot(void);

uint8_t hwReadConfig(const int pos);
void hwWriteConfig(const int pos, uint8_t value);
void hwReadConfigBlock(void *buffer, const void *pos, size_t length);
void hwWriteConfigBlock(void *buffer, const void *pos, size_t length);


// SOFTSPI
#ifdef MY_SOFTSPI
#error Soft SPI is not available on this architecture!
#endif
#define hwSPI SPI //!< hwSPI

#define MY_HW_HAS_GETENTROPY

#ifndef digitalPinToInterrupt
#define digitalPinToInterrupt(__pin) (__pin)
#endif

#define yield()

#endif
