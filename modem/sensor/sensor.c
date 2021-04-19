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
#include "sensor_bme280.h"
#include "sensor_bmp280.h"

#define LPP_TEMPERATURE_SENSOR      103
#define LPP_HUMIDITY_SENSOR         104
#define LPP_BAROMETER               115

#define SENT_TERMINAL               (1<<0)
#define SENT_RADIO                  (1<<1)

typedef bit_t (*sensor_init_fptr_t)(void);
typedef bit_t (*sensor_force_trigger_fptr_t)(u4_t *meas_delay);
typedef bit_t (*sensor_get_data_fptr_t)(struct sensor_data *comp_sensor_data);
typedef bit_t (*sensor_check_fptr_t)(void);

typedef struct
{
    sensor_init_fptr_t sensor_init;
    sensor_force_trigger_fptr_t sensor_force_trigger;
    sensor_get_data_fptr_t sensor_get_data;
    sensor_check_fptr_t sensor_check;
    uint8_t sensor_value_types;
} sensor_fce_t;

typedef enum 
{
  SENSOR_BME_280 = 0, 
  SENSOR_BMP_280,
} sensor_type_t;

typedef enum 
{
  SENSOR_TEMPERATURE = (1<<0), 
  SENSOR_HUMUDITY = (1<<1),
  SENSOR_PRESSURE = (1<<2),
} sensor_value_t;

static void sensor_readmeasurements (osjob_t* job);
static void sensor_startmeasurements (osjob_t* job);
static void sensor_send_message(char *txt, u1_t len);

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);

static u1_t sensor_data_processing = ~(SENT_TERMINAL | SENT_RADIO);  // invalid values by default
static u1_t radio_tx_port = 1;
static u1_t radio_tx_confirm = 0;
static u1_t lpp_data_channel = 0;  
static sensor_fce_t sensor_fce[] = {
    {
        sensor_init_bme280,
        sensor_force_trigger_bme280,
        sensor_get_data_bme280,
        sensor_check_bme280,
        (SENSOR_TEMPERATURE | SENSOR_HUMUDITY | SENSOR_PRESSURE),
    },
    {
        sensor_init_bmp280,
        sensor_force_trigger_bmp280,
        sensor_get_data_bmp280,
        sensor_check_bmp280,
        (SENSOR_TEMPERATURE | SENSOR_PRESSURE),
    },
};
static u1_t sensor_type;
static bit_t get_data_again = false;

/*!
 * @brief This function reading the sensor's registers through I2C bus.
 */
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    hal_i2c_readBlock(reg_addr, data, len);
    return 0;
}

/*!
 * @brief This function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    hal_i2c_writeBlock(reg_addr, data, len);
    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs
 */
void user_delay_us(uint32_t period, void *intf_ptr)
{
    hal_waitUntil(us2osticks(period));
}

static void sensor_readmeasurements (osjob_t* job) {
    /* Structure to get the pressure, temperature and humidity values */
    struct sensor_data comp_data;

    // Measurement finshed, get data and sent it
    if (sensor_fce[sensor_type].sensor_get_data(&comp_data)) {
        s2_t temperature_res_deg_0point1 = (s2_t)((comp_data.temperature + 5) / 10);
        u2_t humidity_res_rel_0point1 = (u2_t)(comp_data.humidity * 5);
        u1_t humidity_res_rel_0point5 = (u1_t) comp_data.humidity;
        u2_t barometer_res_hPa_0point1 = (u2_t)((comp_data.pressure + 5) / 10);
        
        if (sensor_data_processing & SENT_TERMINAL) {
            
            u1_t const len = (6*3)+2+2; // 3x6 digits + 2 commas + 2 escape
            u1_t *buf = buffer_alloc(len);
            memset(buf, 0x00, len);
            u1_t *pbuf = buf;
            u1_t signchar;
            
            if (temperature_res_deg_0point1 < 0) {
                signchar = '-';
                temperature_res_deg_0point1 *= -1;
            }
            else {
                signchar = '0';
            }
            
            if (sensor_fce[sensor_type].sensor_value_types & SENSOR_TEMPERATURE)
            {
                *(pbuf++) = signchar;
                pbuf += putshort (pbuf, (u2_t*)&temperature_res_deg_0point1, 5, 1);
                *(pbuf++) = ',';
            }
            if (sensor_fce[sensor_type].sensor_value_types & SENSOR_HUMUDITY)
            {
                pbuf += putshort (pbuf, &humidity_res_rel_0point1, 6, 1);
                *(pbuf++) = ',';
            }
            if (sensor_fce[sensor_type].sensor_value_types & SENSOR_PRESSURE)
            {
                pbuf += putshort (pbuf, &barometer_res_hPa_0point1, 6, 1);
            }
            *(pbuf++) = '\r';
            *(pbuf++) = '\n';
            modem_transmitdata(buf, len);
        }
        if (sensor_data_processing & SENT_RADIO) {
            u1_t payload[11];
            u1_t payload_len = 0;

            // Cayenne LPP format
            /*
            Cayenne LPP 2.0
            
            3.1. Data Channel
            The Data Channel uniquely identifies each sensor or actuator within a device.
            Acceptable range is from 0 to 64. The device developer is responsible to assign a unique channel
            for each of the devices sensor and actuator and conform to it across the device life cycle.
            */
            if (sensor_fce[sensor_type].sensor_value_types & SENSOR_TEMPERATURE)
            {
                payload[payload_len++] = lpp_data_channel++;                // Data Ch.
                payload[payload_len++] = LPP_TEMPERATURE_SENSOR;            // Temperature Sensor
                payload[payload_len++] = temperature_res_deg_0point1 >> 8;  // MSB Data
                payload[payload_len++] = temperature_res_deg_0point1;       // LSB Data
            }
            if (sensor_fce[sensor_type].sensor_value_types & SENSOR_HUMUDITY)
            {
                payload[payload_len++] = lpp_data_channel++;                // Data Ch.
                payload[payload_len++] = LPP_HUMIDITY_SENSOR;               // Humidity Sensor
                payload[payload_len++] = humidity_res_rel_0point5;          // Data
            }
            if (sensor_fce[sensor_type].sensor_value_types & SENSOR_PRESSURE)
            {
                payload[payload_len++] = lpp_data_channel;                  // Data Ch.
                payload[payload_len++] = LPP_BAROMETER;                     // Barometer
                payload[payload_len++] = barometer_res_hPa_0point1 >> 8;    // MSB Data
                payload[payload_len++] = barometer_res_hPa_0point1;        // LSB Data
            }
            LMIC_setTxData2(radio_tx_port, payload, payload_len, radio_tx_confirm);
        }
    }
    else if (get_data_again) {
        // Not measured yet, try again later
        os_setTimedCallback(job, os_getTime()+ms2osticks(100), sensor_readmeasurements);
    }

    get_data_again = false;
}

static void sensor_startmeasurements (osjob_t* job) {
    // Request measuring
    uint32_t req_delay;

    if (sensor_fce[sensor_type].sensor_force_trigger(&req_delay))
    {
        os_setTimedCallback(job, os_getTime()+ms2osticks(req_delay), sensor_readmeasurements);
        get_data_again = true;
    }
}

static u1_t sensor_start(void)
{
    static bit_t powerison = false;
    
    if ( ! powerison ) {
        powerison = true;
        sensor_power(powerison);
        hal_waitUntil(os_getTime()+ms2osticks(5));
        hal_i2c_ioInit(SENSOR_I2C_ADDR_PRIM);
    }
    bit_t sensor_found = false;

    for (int i = 0; i < (sizeof(sensor_fce) / sizeof(sensor_fce_t)); i++)
    {
        if (sensor_fce[i].sensor_check())
        {
            sensor_found = true;
            sensor_type = i;
            break;
        }
    }

    if (!sensor_found)
    {
        powerison = false;
        sensor_power(powerison);
        
    }

    return sensor_found && sensor_fce[sensor_type].sensor_init();
}

static u1_t sensor_tx(void)
{
    static osjob_t sensorjob;
    
    if (!sensor_start()) {
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