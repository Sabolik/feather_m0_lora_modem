/**
 * \file
 *
 *
 (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

#ifndef _BM280_H
#define _BM280_H

/**
  Section: Included Files
 */

#include "../../basicmac/lmic/oslmic.h"

/**
  Section: Macro Declarations
 */

// Device Information
#define BME280_ADDR 0x76
#define BME280_CHIP_ID 0x60

// Sensor Modes
#define BME280_SLEEP_MODE 0x00
#define BME280_FORCED_MODE 0x01
#define BME280_NORMAL_MODE 0x03
#define BME280_SOFT_RESET 0xB6

// Oversampling Options
#define BME280_OVERSAMP_SKIPPED 0x00
#define BME280_OVERSAMP_X1 0x01
#define BME280_OVERSAMP_X2 0x02
#define BME280_OVERSAMP_X4 0x03
#define BME280_OVERSAMP_X8 0x04
#define BME280_OVERSAMP_X16 0x05

// Standby Time
#define BME280_STANDBY_HALFMS 0x00
#define BME280_STANDBY_63MS 0x01
#define BME280_STANDBY_125MS 0x02
#define BME280_STANDBY_250MS 0x03
#define BME280_STANDBY_500MS 0x04
#define BME280_STANDBY_1000MS 0x05
#define BME280_STANDBY_10MS 0x06
#define BME280_STANDBY_20MS 0x07

// Filter Coefficients
#define BME280_FILTER_COEFF_OFF 0x00
#define BME280_FILTER_COEFF_2 0x01
#define BME280_FILTER_COEFF_4 0x02
#define BME280_FILTER_COEFF_8 0x03
#define BME280_FILTER_COEFF_16 0x04

/**
  Section: BME280 Sensor Driver APIs
 */

u1_t    BME280_getID(void);
void    BME280_reset(void);
void    BME280_sleep(void);
void    BME280_readFactoryCalibrationParams(void);
void    BME280_setStandbyTime(u1_t Value);
void    BME280_setFilterCoefficient(u1_t Value);
void    BME280_setOversamplingTemperature(u1_t Value);
void    BME280_setOversamplingPressure(u1_t Value);
void    BME280_setOversamplingHumidity(u1_t Value);
void    BME280_setSensorMode(u1_t Value);
void    BME280_initializeSensor(void);
void    BME280_startForcedSensing(void);
u1_t    BME280_isMeasuring(void);
void    BME280_readMeasurements(void);
float   BME280_getTemperature(void);
float   BME280_getHumidity(void);
float   BME280_getPressure(void);

#endif // _BM280_H