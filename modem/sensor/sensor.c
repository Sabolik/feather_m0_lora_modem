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

#include "../modem.h"
#include "../hal/hw.h"
#include "sensor.h"
#include "weather_click.h"

#define LPP_TEMPERATURE_SENSOR      103
#define LPP_HUMIDITY_SENSOR         104
#define LPP_BAROMETER               115

#define SENT_TERMINAL               (1<<0)
#define SENT_RADIO                  (1<<1)

static void sensor_readmeasurements (osjob_t* job);
static void sensor_startmeasurements (osjob_t* job);

static u1_t sensor_data_processing = ~(SENT_TERMINAL | SENT_RADIO);  // invalid values by default
static u1_t radio_tx_port = 1;
static u1_t radio_tx_confirm = 0;
static u1_t lpp_data_channel = 0;  

static void sensor_readmeasurements (osjob_t* job) {
    // Measurement finshed, get data and sent it
    if ( Weather_readSensorsFinished() ) {
        s2_t temperature_res_deg_0point1 = (s2_t)(Weather_getTemperatureDegC() * 10);
        u2_t humidity_res_rel_0point1 = (u2_t)(Weather_getHumidityRH() * 10);
        u1_t humidity_res_rel_0point5 = humidity_res_rel_0point1 / 5;
        u2_t barometer_res_hPa_0point1 = (u2_t)(Weather_getPressureKPa() * 100);
        
        if (sensor_data_processing & SENT_TERMINAL) {
            
            u1_t const len = (6*3)+2+2; // 3x6 digits + 2 commas + 2 escape
            u1_t *buf = buffer_alloc(len);
            u1_t *pbuf = buf;
            u1_t signchar;
            
            if (temperature_res_deg_0point1 < 0) {
                signchar = '-';
                temperature_res_deg_0point1 *= -1;
            }
            else {
                signchar = '0';
            }
            
            *(pbuf++) = signchar;
            pbuf += putshort (pbuf, (u2_t*)&temperature_res_deg_0point1, 5, 1);
            *(pbuf++) = ',';
            pbuf += putshort (pbuf, &humidity_res_rel_0point1, 6, 1);
            *(pbuf++) = ',';
            pbuf += putshort (pbuf, &barometer_res_hPa_0point1, 6, 1);
            *(pbuf++) = '\r';
            *(pbuf++) = '\n';
            modem_transmitdata(buf, len);
        }
        if (sensor_data_processing & SENT_RADIO) {
            u1_t payload[11];

            // Cayenne LPP format
            payload[0] = lpp_data_channel;                  // Data Ch.
            payload[1] = LPP_TEMPERATURE_SENSOR;            // Temperature Sensor
            payload[2] = temperature_res_deg_0point1 >> 8;  // MSB Data
            payload[3] = temperature_res_deg_0point1;       // LSB Data
            
            payload[4] = lpp_data_channel;                  // Data Ch.
            payload[5] = LPP_HUMIDITY_SENSOR;               // Humidity Sensor
            payload[6] = humidity_res_rel_0point5;          // Data
            
            payload[7] = lpp_data_channel;                  // Data Ch.
            payload[8] = LPP_BAROMETER;                     // Barometer
            payload[9] = barometer_res_hPa_0point1 >> 8;    // MSB Data
            payload[10] = barometer_res_hPa_0point1;        // LSB Data
            
            LMIC_setTxData2(radio_tx_port, payload, sizeof(payload), radio_tx_confirm);
        }
    }
    else {
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

static u1_t sensor_check(void)
{
    static bit_t poweron = 1;
    static u1_t connectedLast = 0;
    u1_t connected = 0;
    
    if ( poweron ) {
        sensor_power(1);
        hal_waitUntil(os_getTime()+ms2osticks(5));
        poweron = 0;
    }
    
    if (BME280_getID() == BME280_CHIP_ID) {
        connected = 1;
        if ( !connectedLast ) {
            Weather_reset();
            hal_waitUntil(os_getTime()+ms2osticks(5));
        }
    }
    
    connectedLast = connected;
    
    return connected;
}

static u1_t sensor_tx(void)
{
    static osjob_t sensorjob;
    
    if (!sensor_check()) {
        return 0;
    }
    
    os_clearCallback(&sensorjob);
    os_setCallback(&sensorjob, sensor_startmeasurements );
    
    return 1;
}

u1_t sensor_txradio(u1_t port, u1_t confirm, u1_t channel)
{
    // do not print on terminal
    sensor_data_processing = SENT_RADIO;
    radio_tx_port = port;
    radio_tx_confirm = confirm;
    lpp_data_channel = channel;
    return sensor_tx();
}

u1_t sensor_txterminal(void)
{
    sensor_data_processing = SENT_TERMINAL;
    return sensor_tx();
}