/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#include "../../basicmac/lmic/oslmic.h"

#include "../../driver/board/hal/hal.h"

//////////////////////////////////////////////////////////////////////
// EEPROM
//////////////////////////////////////////////////////////////////////

#define EEPROM_BASE             HAL_EEPROM_BASE
#define EEPROM_SIZE             512//CONF_SECTOR_SIZE   // one sector used


#define STACKDATA_BASE          (EEPROM_BASE + 0x0040)
#define PERSODATA_BASE          (EEPROM_BASE + 0x0060)
#define APPDATA_BASE            (EEPROM_BASE + 0x0100)

#define STACKDATA_SZ            (PERSODATA_BASE - STACKDATA_BASE)
#define PERSODATA_SZ            (APPDATA_BASE - PERSODATA_BASE)
#define APPDATA_SZ              (EEPROM_END - APPDATA_BASE)

// write 32-bit word to EEPROM
void eeprom_write (u4_t* addr, u4_t val);

// copy bytes to EEPROM (aligned, multiple of 4)
void eeprom_copy (void* dst, const void* src, u2_t len);

//////////////////////////////////////////////////////////////////////
// I2C
//////////////////////////////////////////////////////////////////////

void hal_i2c_ioInit(s2_t addr);
void hal_i2c_writeByte(u1_t reg_addr, u1_t data);
void hal_i2c_readBlock(u1_t reg_addr, u1_t *read_buff, u1_t length);

//////////////////////////////////////////////////////////////////////
// Sensor
//////////////////////////////////////////////////////////////////////

void sensor_power (u1_t state);

//////////////////////////////////////////////////////////////////////
// Led
//////////////////////////////////////////////////////////////////////

void hal_pin_led (u1_t val);

//////////////////////////////////////////////////////////////////////
// Radio
//////////////////////////////////////////////////////////////////////

void radio_check_rx_timeout (u1_t cancelJob);


