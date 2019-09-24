/*
 * Copyright (c) 2019 martin.sabol@seznam.cz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE
 */

#include <stdlib.h>
#include "../modem.h"
#include "../hal/hw.h"
#include "../../xprintf/xprintf.h"
#include "sensor.h"
#include "weather_click.h"

#define LPP_TEMPERATURE_SENSOR      103
#define LPP_HUMIDITY_SENSOR         104
#define LPP_BAROMETER               115

static void sensor_readmeasurements (osjob_t* job);
static void sensor_startmeasurements (osjob_t* job);
static void sensor_poweron (osjob_t* job);

static u1_t terminal_print = 0;
static u2_t radio_tx_period_s = 0;
static u1_t radio_tx_port = 1;
static u1_t radio_tx_confirm = 0;

static void sensor_sendchartomodem (u1_t c) {
    (void)frame_rx(c);
}   

static void sensor_readmeasurements (osjob_t* job) {
    // Measurement finshed, get data and sent it
    if ( Weather_readSensorsFinished() )
    {
        s2_t temperature_res_deg_0point1 = (s2_t)(Weather_getTemperatureDegC() * 10);
        u2_t humidity_res_rel_0point1 = (u2_t)(Weather_getHumidityRH() * 10);
        u1_t humidity_res_rel_0point5 = humidity_res_rel_0point1 / 5;
        u2_t barometer_res_hPa_0point1 = (u2_t)(Weather_getPressureKPa() * 100);
        
        if ( terminal_print )
        {
            terminal_print = 0;
            
            u1_t signchar = (temperature_res_deg_0point1 < 0 ? '-': '0');
            temperature_res_deg_0point1 = abs(temperature_res_deg_0point1);
            
            u1_t const bufLen = (6*3)+2+2;
            u1_t *buf = buffer_alloc(bufLen);
            xsprintf(buf, "%c%03d.%1u,%04u.%1u,%04u.%1u\r\n",
                signchar, temperature_res_deg_0point1 / 10, temperature_res_deg_0point1 % 10,
                humidity_res_rel_0point1 / 10, humidity_res_rel_0point1 % 10,
                barometer_res_hPa_0point1 / 10, barometer_res_hPa_0point1 % 10);
            modem_transmitdata(buf, bufLen);
        }
        else
        {
            xfprintf(sensor_sendchartomodem, "ATT%1d,%02x,01%02x%04x01%02x%02x01%02x%04x\r\n",
                radio_tx_confirm, radio_tx_port,
                LPP_TEMPERATURE_SENSOR, temperature_res_deg_0point1,
                LPP_HUMIDITY_SENSOR, humidity_res_rel_0point5,
                LPP_BAROMETER, barometer_res_hPa_0point1);

            // Schedule next sensor reading
            if ( radio_tx_period_s )
            {
                os_setTimedCallback(job, os_getTime()+sec2osticks(radio_tx_period_s), sensor_startmeasurements);
            }
        }
    }
    else
    {
        // Not measured yet, try again later
        os_setTimedCallback(job, os_getTime()+ms2osticks(100), sensor_readmeasurements);
    }
}

static void sensor_startmeasurements (osjob_t* job) {
    // Request measuring
    Weather_readSensorsStart();

    // Schedule data reading when measured
    os_setTimedCallback(job, os_getTime()+ms2osticks(100), sensor_readmeasurements);
}

static void sensor_init (osjob_t* job) {
    Weather_reset();

    // Schedule data measurement
    os_setTimedCallback(job, os_getTime()+ms2osticks(50), sensor_startmeasurements);
}

static void sensor_poweron (osjob_t* job) {
    sensor_power(1);

    // Schedule data measurement
    os_setTimedCallback(job, os_getTime()+ms2osticks(100), sensor_init);
}

static void sensor_tx(void)
{
    static osjob_t sensorjob;
    static bit_t poweron = 1;

    os_clearCallback(&sensorjob);
    os_setCallback(&sensorjob, poweron ? sensor_poweron : sensor_startmeasurements );
    
    poweron = 0;
}

void sensor_txradio(u2_t period, u1_t port, u1_t confirm)
{
    radio_tx_period_s = period;
    radio_tx_port = port;
    radio_tx_confirm = confirm;
    sensor_tx();
}

void sensor_txterminal(void)
{
    terminal_print = 1;
    sensor_tx();
}