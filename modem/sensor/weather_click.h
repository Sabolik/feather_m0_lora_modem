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

#ifndef WEATHER_CLICK_H
#define WEATHER_CLICK_H

/**
  Section: Included Files
 */

#include "bme280.h"

/**
  Section: Weather Click Driver APIs
 */

/*
 * Called to read sensor data
 */
void Weather_readSensors(void);

/*
 * Weather_readSensors divided into two parts not to busy wait for finished measuring
 */
void Weather_readSensorsStart(void);
u1_t Weather_readSensorsFinished(void);

/*
 * Return compensated values in deg. Celsius,
 * kPascals, & %Relative Humidity
 */
float Weather_getTemperatureDegC(void);
float Weather_getPressureKPa(void);
float Weather_getHumidityRH(void);

void Weather_gotoSleep(void);

void Weather_reset(void);

#endif // _WEATHER_CLICK_H
