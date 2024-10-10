/**
 * @brief : BME280 declaration file
 * @author : Mohammed Ubaid
*/

#ifndef H_BME_280
#define H_BME_280

#include <stdint.h>

#define BME280_DEVID_REG 0xD0
#define BME280_CTRL_HUM_REG 0xF2
#define BME280_CTRL_MEAS_REG 0xF4
#define BME280_CONFIG_REG 0xF4
#define BME280_RESET_REG 0xE0

#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_T2 0x8A
#define BME280_REGISTER_DIG_T3 0x8C

#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_P2 0x90
#define BME280_REGISTER_DIG_P3 0x92
#define BME280_REGISTER_DIG_P4 0x94
#define BME280_REGISTER_DIG_P5 0x96
#define BME280_REGISTER_DIG_P6 0x98
#define BME280_REGISTER_DIG_P7 0x9A
#define BME280_REGISTER_DIG_P8 0x9C
#define BME280_REGISTER_DIG_P9 0x9E

#define BME280_REGISTER_DIG_H1 0xA1
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_DIG_H3 0xE3
#define BME280_REGISTER_DIG_H4 0xE4
#define BME280_REGISTER_DIG_H5 0xE5
#define BME280_REGISTER_DIG_H6 0XE7

static const uint8_t CTRL_HUM_ADDR = 0xF2;
static const uint8_t CTRL_MEAS_ADDR = 0xF4;
static const uint8_t CONFIG_ADDR = 0xF5;
static const uint8_t PRESS_ADDR = 0xF7;
static const uint8_t TEMP_ADDR = 0xFA;
static const uint8_t HUM_ADDR = 0xFD;
static const uint8_t TEMP_DIG_ADDR = 0x88;
static const uint8_t PRESS_DIF_ADDR = 0x8E;
static const uint8_t HUM_DIG_ADDR1 = 0xA1;
static const uint8_t HUM_DIG_ADDR2 = 0xE1;
static const uint8_t ID_ADDR = 0xD0;
static const uint8_t RESET_ADDR = 0xE0;

static const uint8_t RESET_VALUE = 0xB6;

static const uint8_t TEMP_DIG_LENGTH = 6;
static const uint8_t PRESS_DIG_LENGTH = 18;
static const uint8_t HUM_DIG_ADDR1_LENGTH = 1;
static const uint8_t HUM_DIG_ADDR2_LENGTH = 7;
static const uint8_t DIG_LENGTH = 32;
static const uint8_t SENSOR_DATA_LENGTH = 8;

struct bme280_Reg_Data
{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;

    int32_t ucomp_temp;
    int32_t ucomp_press;
    int32_t ucomp_hum;

    int32_t comp_temp;
    uint32_t comp_press;
    uint32_t comp_hum;

    int32_t t_fine;

    uint8_t chip_id;
};

#endif