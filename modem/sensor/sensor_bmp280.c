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

#include "BMP280_driver/bmp280.h"

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void user_delay_us(uint32_t period, void *intf_ptr);

static void delay_ms(uint32_t period_ms) {
    hal_waitUntil(ms2osticks(period_ms));
    //user_delay_us(period_ms * 1000, NULL);
}

static int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    return user_i2c_write(reg_addr, reg_data, length, NULL);
}

static int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    return user_i2c_read(reg_addr, reg_data, length, NULL);
}

static struct bmp280_dev dev = {  
    /* Map the delay function pointer with the function responsible for implementing the delay */
    .delay_ms = delay_ms,
    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    .dev_id = BMP280_I2C_ADDR_PRIM,
    /* Select the interface mode as I2C */
    .intf = BMP280_I2C_INTF,
    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    .read = i2c_reg_read,
    .write = i2c_reg_write,
};
static bit_t initialized = false;

bit_t sensor_check_bmp280(void) {
    uint8_t chip_id = 0;
    /* Read the chip-id of bme280 sensor */
    bit_t found = (bmp280_get_regs(BMP280_CHIP_ID_ADDR, &chip_id, 1, &dev) == BMP280_OK) && (chip_id == BMP280_CHIP_ID1 || chip_id == BMP280_CHIP_ID2 || chip_id == BMP280_CHIP_ID3);
    if (! found)
    {
        initialized = false;
    }
    return found;
}

bit_t sensor_init_bmp280(void) {
    struct bmp280_config conf;

    /* Initialize the bme280 */
    if (! initialized)
    {
        if (bmp280_soft_reset(&dev) != BMP280_OK ||
            bmp280_init(&dev) != BMP280_OK ||
            /* Always read the current settings before writing, especially when
            * all the configuration is not modified
            */
            bmp280_get_config(&conf, &dev) != BMP280_OK)
        {
            return false;
        }

        /* configuring the temperature oversampling, filter coefficient and output data rate */
        /* Overwrite the desired settings */
        conf.filter = BMP280_FILTER_OFF;

        /* Pressure oversampling set at 1x */
        conf.os_pres = BMP280_OS_1X;

        /* Temperature oversampling set at 1x */
        conf.os_temp = BMP280_OS_1X;

        /* Setting the output data rate */
        conf.odr = BMP280_ODR_0_5_MS;

        if (bmp280_set_config(&conf, &dev) != BMP280_OK)
        {
            return false;
        }
    }

    /* Check for chip id validity */
    initialized = true;//sensor_check_bmp280();

    return initialized;
}

bit_t sensor_force_trigger_bmp280(u4_t *meas_delay) {
    int8_t rslt;
    
    /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
    *  and the oversampling configuration. */
    *meas_delay = bmp280_compute_meas_time(&dev);

    /* Set the sensor to forced mode */
    rslt = bmp280_set_power_mode(BMP280_FORCED_MODE, &dev);
    return rslt == BMP280_OK;
}

bit_t sensor_get_data_bmp280(struct sensor_data *comp_sensor_data) {
    struct bmp280_uncomp_data ucomp_data;
    struct bmp280_status status;

    /* Reading the raw data from sensor */
    if (bmp280_get_status(&status, &dev) != BMP280_OK ||
        status.measuring == BMP280_MEAS_ONGOING ||
        bmp280_get_uncomp_data(&ucomp_data, &dev) != BMP280_OK ||
        bmp280_get_comp_temp_32bit(&comp_sensor_data->temperature, ucomp_data.uncomp_temp, &dev) != BMP280_OK ||
        bmp280_get_comp_pres_32bit(&comp_sensor_data->pressure, ucomp_data.uncomp_press, &dev) != BMP280_OK)
    {
        return false;
    }

    return true;
}