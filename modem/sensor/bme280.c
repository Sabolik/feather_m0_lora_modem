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
/**
  Section: Included Files
 */

#include "bme280.h"
#include "../hal/hw.h"

/**
  Section: Macro Declarations
 */

// Register Addresses
#define BME280_ID_REG 0xD0
#define BME280_RESET_REG 0xE0
#define BME280_CTRL_HUM_REG 0xF2
#define BME280_STATUS_REG 0xF3
#define BME280_CTRL_MEAS_REG 0xF4
#define BME280_CONFIG_REG 0xF5
#define BME280_PRESS_MSB_REG 0xF7
#define BME280_PRESS_LSB_REG 0xF8
#define BME280_PRESS_XLSB_REG 0xF9
#define BME280_TEMP_MSB_REG 0xFA
#define BME280_TEMP_LSB_REG 0xFB
#define BME280_TEMP_XLSB_REG 0xFC
#define BME280_HUM_MSB_REG 0xFD
#define BME280_HUM_LSB_REG 0xFE

// Sensor Data Read Bytes
#define BME280_PRESS_MSB 0
#define BME280_PRESS_LSB 1
#define BME280_PRESS_XLSB 2
#define BME280_TEMP_MSB 3
#define BME280_TEMP_LSB 4
#define BME280_TEMP_XLSB 5
#define BME280_HUM_MSB 6
#define BME280_HUM_LSB 7
#define BME280_DATA_FRAME_SIZE 8

// Factory Calibration Parameters
#define BME280_CALIB_DT1_LSB_REG 0x88
#define BME280_CALIB_DT1_MSB_REG 0x89
#define BME280_CALIB_DT2_LSB_REG 0x8A
#define BME280_CALIB_DT2_MSB_REG 0x8B
#define BME280_CALIB_DT3_LSB_REG 0x8C
#define BME280_CALIB_DT3_MSB_REG 0x8D
#define BME280_CALIB_DP1_LSB_REG 0x8E
#define BME280_CALIB_DP1_MSB_REG 0x8F
#define BME280_CALIB_DP2_LSB_REG 0x90
#define BME280_CALIB_DP2_MSB_REG 0x91
#define BME280_CALIB_DP3_LSB_REG 0x92
#define BME280_CALIB_DP3_MSB_REG 0x93
#define BME280_CALIB_DP4_LSB_REG 0x94
#define BME280_CALIB_DP4_MSB_REG 0x95
#define BME280_CALIB_DP5_LSB_REG 0x96
#define BME280_CALIB_DP5_MSB_REG 0x97
#define BME280_CALIB_DP6_LSB_REG 0x98
#define BME280_CALIB_DP6_MSB_REG 0x99
#define BME280_CALIB_DP7_LSB_REG 0x9A
#define BME280_CALIB_DP7_MSB_REG 0x9B
#define BME280_CALIB_DP8_LSB_REG 0x9C
#define BME280_CALIB_DP8_MSB_REG 0x9D
#define BME280_CALIB_DP9_LSB_REG 0x9E
#define BME280_CALIB_DP9_MSB_REG 0x9F
#define BME280_CALIB_DH1_REG 0xA1
#define BME280_CALIB_DH2_LSB_REG 0xE1
#define BME280_CALIB_DH2_MSB_REG 0xE2
#define BME280_CALIB_DH3_REG 0xE3
#define BME280_CALIB_DH4_MSB_REG 0xE4
#define BME280_CALIB_DH4_LSB_REG 0xE5
#define BME280_CALIB_DH5_MSB_REG 0xE6
#define BME280_CALIB_DH6_REG 0xE7

/**
  Section: Variable Definitions
 */

typedef union {

	struct {
		u1_t im_update : 1;
		u1_t : 2;
		u1_t measuring : 1;
		u1_t : 4;
	};
	u1_t statusReg;
} bme280_status_t;

typedef union {

	struct {
		u1_t mode : 2;
		u1_t osrs_P : 3;
		u1_t osrs_T : 3;
	};
	u1_t ctrlMeasReg;
} bme280_ctrl_meas_t;

typedef union {

	struct {
		u1_t spi3w_en : 1;
		u1_t filter : 3;
		u1_t t_sb : 3;
	};
	u1_t configReg;
} bme280_config_t;

typedef struct {
	u2_t        dig_T1;
	int         dig_T2;
	int         dig_T3;
	u2_t        dig_P1;
	int         dig_P2;
	int         dig_P3;
	int         dig_P4;
	int         dig_P5;
	int         dig_P6;
	int         dig_P7;
	int         dig_P8;
	int         dig_P9;
	u1_t        dig_H1;
	int         dig_H2;
	u1_t        dig_H3;
	int         dig_H4;
	int         dig_H5;
	signed char dig_H6;
} bme280_calibration_param_t;

bme280_config_t            bme280_config;
u1_t                       bme280_ctrl_hum;
bme280_ctrl_meas_t         bme280_ctrl_meas;
bme280_status_t            bme280_status;
bme280_calibration_param_t calibParam;

long adc_T, adc_H, adc_P, t_fine;

/**
  Section: Private function prototypes
 */

u1_t            BME280_getStatus(void);
static long     BME280_compensateTemperature(void);
static u4_t     BME280_compensatePressure(void);
static u4_t     BME280_compensateHumidity(void);

void            BME280_writeByte(u1_t reg_addr, u1_t data);
u1_t            BME280_readByte(u1_t reg_addr);
void            BME280_readBlock(u1_t reg_addr, u1_t *read_buff, u1_t length);

/**
  Section: Driver APIs
 */

u1_t BME280_getID(void)
{
	return BME280_readByte(BME280_ID_REG);
}

void BME280_reset(void)
{
	BME280_writeByte(BME280_RESET_REG, BME280_SOFT_RESET);
}

void BME280_sleep(void)
{
	bme280_ctrl_meas.mode = BME280_SLEEP_MODE;
	BME280_writeByte(BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
}

void BME280_readFactoryCalibrationParams(void)
{
	u1_t paramBuff[24];

	BME280_readBlock(BME280_CALIB_DT1_LSB_REG, paramBuff, 24);
	calibParam.dig_T1 = (((u2_t)paramBuff[1]) << 8) + paramBuff[0];
	calibParam.dig_T2 = (((int)paramBuff[3]) << 8) + paramBuff[2];
	calibParam.dig_T3 = (((int)paramBuff[5]) << 8) + paramBuff[4];
	calibParam.dig_P1 = (((u2_t)paramBuff[7]) << 8) + paramBuff[6];
	calibParam.dig_P2 = (((int)paramBuff[9]) << 8) + paramBuff[8];
	calibParam.dig_P3 = (((int)paramBuff[11]) << 8) + paramBuff[10];
	calibParam.dig_P4 = (((int)paramBuff[13]) << 8) + paramBuff[12];
	calibParam.dig_P5 = (((int)paramBuff[15]) << 8) + paramBuff[14];
	calibParam.dig_P6 = (((int)paramBuff[17]) << 8) + paramBuff[16];
	calibParam.dig_P7 = (((int)paramBuff[19]) << 8) + paramBuff[18];
	calibParam.dig_P8 = (((int)paramBuff[21]) << 8) + paramBuff[20];
	calibParam.dig_P9 = (((int)paramBuff[23]) << 8) + paramBuff[22];

	calibParam.dig_H1 = (u1_t)BME280_readByte(BME280_CALIB_DH1_REG);

	BME280_readBlock(BME280_CALIB_DH2_LSB_REG, paramBuff, 7);
	calibParam.dig_H2 = (((int)paramBuff[1]) << 8) + paramBuff[0];
	calibParam.dig_H3 = (u1_t)paramBuff[2];
	calibParam.dig_H4 = (((int)paramBuff[3]) << 4) | (paramBuff[4] & 0xF);
	calibParam.dig_H5 = (((int)paramBuff[5]) << 4) | (paramBuff[4] >> 4);
	calibParam.dig_H6 = (short)paramBuff[6];
}

void BME280_setStandbyTime(u1_t sbtime)
{
	bme280_config.t_sb = sbtime;
}

void BME280_setFilterCoefficient(u1_t coeff)
{
	bme280_config.filter = coeff;
}

void BME280_setOversamplingTemperature(u1_t osrs_t)
{
	bme280_ctrl_meas.osrs_T = osrs_t;
}

void BME280_setOversamplingPressure(u1_t osrs_p)
{
	bme280_ctrl_meas.osrs_P = osrs_p;
}

void BME280_setOversamplingHumidity(u1_t osrs_h)
{
	bme280_ctrl_hum = osrs_h;
}

void BME280_setSensorMode(u1_t mode)
{
	bme280_ctrl_meas.mode = mode;
}

void BME280_initializeSensor(void)
{
	BME280_writeByte(BME280_CONFIG_REG, bme280_config.configReg);
	BME280_writeByte(BME280_CTRL_HUM_REG, bme280_ctrl_hum);
	BME280_writeByte(BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
}

void BME280_startForcedSensing(void)
{
	bme280_ctrl_meas.mode = BME280_FORCED_MODE;
	BME280_writeByte(BME280_CTRL_MEAS_REG, bme280_ctrl_meas.ctrlMeasReg);
}

u1_t BME280_isMeasuring(void)
{
	bme280_status.statusReg = BME280_getStatus();
	return (bme280_status.measuring);
}

void BME280_readMeasurements(void)
{
	u1_t sensorData[BME280_DATA_FRAME_SIZE];

	BME280_readBlock(BME280_PRESS_MSB_REG, sensorData, BME280_DATA_FRAME_SIZE);

	adc_H = ((u4_t)sensorData[BME280_HUM_MSB] << 8) | sensorData[BME280_HUM_LSB];

	adc_T = ((u4_t)sensorData[BME280_TEMP_MSB] << 12)
	        | (((u4_t)sensorData[BME280_TEMP_LSB] << 4) | ((u4_t)sensorData[BME280_TEMP_XLSB] >> 4));

	adc_P = ((u4_t)sensorData[BME280_PRESS_MSB] << 12)
	        | (((u4_t)sensorData[BME280_PRESS_LSB] << 4) | ((u4_t)sensorData[BME280_PRESS_XLSB] >> 4));
}

float BME280_getTemperature(void)
{
	float temperature = (float)BME280_compensateTemperature() / 100;
	return temperature;
}

float BME280_getPressure(void)
{
	float pressure = (float)BME280_compensatePressure() / 1000;
	return pressure;
}

float BME280_getHumidity(void)
{
	float humidity = (float)BME280_compensateHumidity() / 1024;
	return humidity;
}

u1_t BME280_getStatus(void)
{
	bme280_status.statusReg = BME280_readByte(BME280_STATUS_REG);
	return bme280_status.statusReg;
}

/*
 * Returns temperature in DegC, resolution is 0.01 DegC.
 * Output value of "5123" equals 51.23 DegC.
 */
static long BME280_compensateTemperature(void)
{
	long tempV1, tempV2, t;

	tempV1 = ((((adc_T >> 3) - ((long)calibParam.dig_T1 << 1))) * ((long)calibParam.dig_T2)) >> 11;
	tempV2 = (((((adc_T >> 4) - ((long)calibParam.dig_T1)) * ((adc_T >> 4) - ((long)calibParam.dig_T1))) >> 12)
	          * ((long)calibParam.dig_T3))
	         >> 14;
	t_fine = tempV1 + tempV2;
	t      = (t_fine * 5 + 128) >> 8;

	return t;
}

/*
 * Returns pressure in Pa as unsigned 32 bit integer.
 * Output value of "96386" equals 96386 Pa = 96.386 kPa
 */
static u4_t BME280_compensatePressure(void)
{
	long     pressV1, pressV2;
	u4_t p;

	pressV1 = (((long)t_fine) >> 1) - (long)64000;
	pressV2 = (((pressV1 >> 2) * (pressV1 >> 2)) >> 11) * ((long)calibParam.dig_P6);
	pressV2 = pressV2 + ((pressV1 * ((long)calibParam.dig_P5)) << 1);
	pressV2 = (pressV2 >> 2) + (((long)calibParam.dig_P4) << 16);
	pressV1 = (((calibParam.dig_P3 * (((pressV1 >> 2) * (pressV1 >> 2)) >> 13)) >> 3)
	           + ((((long)calibParam.dig_P2) * pressV1) >> 1))
	          >> 18;
	pressV1 = ((((32768 + pressV1)) * ((long)calibParam.dig_P1)) >> 15);

	if (pressV1 == 0) {
		// avoid exception caused by division by zero
		return 0;
	}

	p = (((u4_t)(((long)1048576) - adc_P) - (pressV2 >> 12))) * 3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((u4_t)pressV1);
	} else {
		p = (p / (u4_t)pressV1) * 2;
	}

	pressV1 = (((long)calibParam.dig_P9) * ((long)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	pressV2 = (((long)(p >> 2)) * ((long)calibParam.dig_P8)) >> 13;
	p       = (u4_t)((long)p + ((pressV1 + pressV2 + calibParam.dig_P7) >> 4));

	return p;
}

/*
 * Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format
 * (22 integer and 10 fractional bits).
 * Output value of "47445" represents 47445/1024 = 46.333 %RH
 */
static u4_t BME280_compensateHumidity(void)
{
	long     humV;
	u4_t h;

	humV = (t_fine - ((long)76800));
	humV = (((((adc_H << 14) - (((long)calibParam.dig_H4) << 20) - (((long)calibParam.dig_H5) * humV)) + ((long)16384))
	         >> 15)
	        * (((((((humV * ((long)calibParam.dig_H6)) >> 10)
	               * (((humV * ((long)calibParam.dig_H3)) >> 11) + ((long)32768)))
	              >> 10)
	             + ((long)2097152))
	                * ((long)calibParam.dig_H2)
	            + 8192)
	           >> 14));
	humV = (humV - (((((humV >> 15) * (humV >> 15)) >> 7) * ((long)calibParam.dig_H1)) >> 4));
	humV = (humV < 0 ? 0 : humV);
	humV = (humV > 419430400 ? 419430400 : humV);

	h = (u4_t)(humV >> 12);
	return h;
}

static u1_t io_initialized = 0;

static void BME280_ioInit(void)
{
    hal_i2c_ioInit(BME280_ADDR);

    io_initialized = 1;
}

void BME280_writeByte(u1_t reg_addr, u1_t data)
{
    if ( !io_initialized )
    {
        BME280_ioInit();
    }
    hal_i2c_writeByte( reg_addr, data );
}

u1_t BME280_readByte(u1_t reg_addr)
{
    u1_t readData;
    
    if ( !io_initialized )
    {
        BME280_ioInit();
    }
    hal_i2c_readBlock(reg_addr, &readData, sizeof(readData));
    
    return readData;
}

void BME280_readBlock(u1_t reg_addr, u1_t *read_buff, u1_t length)
{
    if ( !io_initialized )
    {
        BME280_ioInit();
    }
    hal_i2c_readBlock(reg_addr, read_buff, length);
}
