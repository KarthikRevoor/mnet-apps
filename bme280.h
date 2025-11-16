#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
//  BME280 Register Map
// -----------------------------------------------------------------------------
#define BME280_REG_ID                0xD0
#define BME280_REG_RESET             0xE0
#define BME280_REG_CTRL_HUM          0xF2
#define BME280_REG_STATUS            0xF3
#define BME280_REG_CTRL_MEAS         0xF4
#define BME280_REG_CONFIG            0xF5
#define BME280_REG_PRESS_MSB         0xF7
#define BME280_REG_TEMP_MSB          0xFA
#define BME280_REG_HUM_MSB           0xFD

// -----------------------------------------------------------------------------
//  Calibration Data Structure
// -----------------------------------------------------------------------------
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} bme280_calib_t;

// -----------------------------------------------------------------------------
//  Driver Object
// -----------------------------------------------------------------------------
typedef struct {
    uint8_t address;
    bme280_calib_t calib;
    int32_t t_fine;

    // Function pointers that YOU provide
    int (*i2c_write)(uint8_t addr, uint8_t reg, uint8_t value);
    int (*i2c_read)(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);

} bme280_t;

// -----------------------------------------------------------------------------
//  Public API
// -----------------------------------------------------------------------------

// Initialize with user-supplied I2C functions
int bme280_init(bme280_t *dev,
                uint8_t addr,
                int (*i2c_write)(uint8_t, uint8_t, uint8_t),
                int (*i2c_read)(uint8_t, uint8_t, uint8_t*, size_t));

// Sensor readings
float bme280_read_temperature(bme280_t *dev);
float bme280_read_pressure(bme280_t *dev);
float bme280_read_humidity(bme280_t *dev);
float bme280_read_altitude(float pressure_hPa, float seaLevel_hPa);

#endif
