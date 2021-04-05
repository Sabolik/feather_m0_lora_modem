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

#include "../../basicmac/lmic/oslmic.h"
#include "sensor.h"

#include "BME280_driver/bme280.h"

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);

static struct bme280_dev dev = {
    .intf_ptr = NULL,
    .intf = BME280_I2C_INTF,
    .read = user_i2c_read,
    .write = user_i2c_write,
    .delay_us = user_delay_us,
};
static bit_t initialized = false;

bit_t sensor_check_bme280(void) {
    uint8_t chip_id = 0;
    /* Read the chip-id of bme280 sensor */
    bit_t found = (bme280_get_regs(BME280_CHIP_ID_ADDR, &chip_id, 1, &dev) == BME280_OK) && (chip_id == BME280_CHIP_ID);
    if (! found)
    {
        initialized = false;
    }
    return found;
}

bit_t sensor_init_bme280(void) {
    /* Initialize the bme280 */
    if (! initialized)
    {
        if(bme280_soft_reset(&dev) != BME280_OK ||
           bme280_init(&dev) != BME280_OK)
        {
            return false;
        }

        /* Variable to define the selecting sensors */
        uint8_t settings_sel = 0;

        /* Recommended mode of operation: Indoor navigation */
        dev.settings.osr_h = BME280_OVERSAMPLING_1X;
        dev.settings.osr_p = BME280_OVERSAMPLING_1X;
        dev.settings.osr_t = BME280_OVERSAMPLING_1X;
        dev.settings.filter = BME280_FILTER_COEFF_OFF;
        dev.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

        settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

        /* Set the sensor settings */
        if (bme280_set_sensor_settings(settings_sel, &dev) != BME280_OK)
        {
            return false;
        }
    }

    /* Check for chip id validity */
    initialized = true;

    return initialized;
}

bit_t sensor_force_trigger_bme280(u4_t *meas_delay) {
    /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
    *  and the oversampling configuration. */
    *meas_delay = bme280_cal_meas_delay(&dev.settings);

    /* Set the sensor to forced mode */
    return bme280_set_sensor_mode(BME280_FORCED_MODE, &dev) == BME280_OK;
}

bit_t sensor_get_data_bme280(struct sensor_data *comp_sensor_data) {
    int8_t rslt;
    struct bme280_data comp_data;

    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

    comp_sensor_data->humidity = comp_data.humidity/512;
    comp_sensor_data->temperature = comp_data.temperature;
    comp_sensor_data->pressure = comp_data.pressure;
    return rslt == BME280_OK;
}