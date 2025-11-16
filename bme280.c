#include "bme280.h"
#include <math.h>
#include <string.h>

// -----------------------------------------------------------------------------
//  Low-Level Helpers
// -----------------------------------------------------------------------------

static uint8_t read8(bme280_t *dev, uint8_t reg)
{
    uint8_t value;
    dev->i2c_read(dev->address, reg, &value, 1);
    return value;
}

static uint16_t read16(bme280_t *dev, uint8_t reg)
{
    uint8_t buf[2];
    dev->i2c_read(dev->address, reg, buf, 2);
    return (buf[1] << 8) | buf[0];
}

static int16_t readS16(bme280_t *dev, uint8_t reg)
{
    return (int16_t)read16(dev, reg);
}

// Humidity special registers
static void read_calibration_humidity(bme280_t *dev)
{
    uint8_t e4 = read8(dev, 0xE4);
    uint8_t e5 = read8(dev, 0xE5);
    uint8_t e6 = read8(dev, 0xE6);

    dev->calib.dig_H4 = (e4 << 4) | (e5 & 0x0F);
    dev->calib.dig_H5 = (e6 << 4) | (e5 >> 4);
}

// -----------------------------------------------------------------------------
//  Read Calibration Data
// -----------------------------------------------------------------------------

static void read_calibration_data(bme280_t *dev)
{
    dev->calib.dig_T1 = read16(dev, 0x88);
    dev->calib.dig_T2 = readS16(dev, 0x8A);
    dev->calib.dig_T3 = readS16(dev, 0x8C);

    dev->calib.dig_P1 = read16(dev, 0x8E);
    dev->calib.dig_P2 = readS16(dev, 0x90);
    dev->calib.dig_P3 = readS16(dev, 0x92);
    dev->calib.dig_P4 = readS16(dev, 0x94);
    dev->calib.dig_P5 = readS16(dev, 0x96);
    dev->calib.dig_P6 = readS16(dev, 0x98);
    dev->calib.dig_P7 = readS16(dev, 0x9A);
    dev->calib.dig_P8 = readS16(dev, 0x9C);
    dev->calib.dig_P9 = readS16(dev, 0x9E);

    dev->calib.dig_H1 = read8(dev, 0xA1);
    dev->calib.dig_H2 = readS16(dev, 0xE1);
    dev->calib.dig_H3 = read8(dev, 0xE3);

    read_calibration_humidity(dev);

    dev->calib.dig_H6 = (int8_t)read8(dev, 0xE7);
}

// -----------------------------------------------------------------------------
//  Initialization
// -----------------------------------------------------------------------------

int bme280_init(bme280_t *dev,
                uint8_t addr,
                int (*i2c_write)(uint8_t, uint8_t, uint8_t),
                int (*i2c_read)(uint8_t, uint8_t, uint8_t*, size_t))
{
    memset(dev, 0, sizeof(bme280_t));

    dev->address = addr;
    dev->i2c_write = i2c_write;
    dev->i2c_read  = i2c_read;

    uint8_t chipid = read8(dev, BME280_REG_ID);
    if (chipid != 0x60)
        return -1;

    // Soft reset
    dev->i2c_write(addr, BME280_REG_RESET, 0xB6);

    // Wait for chip to reset
    for (int i = 0; i < 10; i++) {
        if ((read8(dev, BME280_REG_STATUS) & 0x01) == 0)
            break;
    }

    read_calibration_data(dev);

    // Humidity oversampling x1
    dev->i2c_write(addr, BME280_REG_CTRL_HUM, 0x01);

    // Temp/Pressure oversampling x1, mode normal
    dev->i2c_write(addr, BME280_REG_CTRL_MEAS, 0x27);

    return 0;
}

// -----------------------------------------------------------------------------
//  Temperature
// -----------------------------------------------------------------------------

float bme280_read_temperature(bme280_t *dev)
{
    uint8_t buf[3];
    dev->i2c_read(dev->address, BME280_REG_TEMP_MSB, buf, 3);

    int32_t adc = (((int32_t)buf[0] << 12) |
                   ((int32_t)buf[1] << 4) |
                   ((int32_t)buf[2] >> 4));

    int32_t var1 = ((((adc >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) *
                    dev->calib.dig_T2) >> 11;

    int32_t var2 = (((((adc >> 4) - dev->calib.dig_T1) *
                      ((adc >> 4) - dev->calib.dig_T1)) >> 12) *
                    dev->calib.dig_T3) >> 14;

    dev->t_fine = var1 + var2;

    float T = (dev->t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

// -----------------------------------------------------------------------------
//  Pressure
// -----------------------------------------------------------------------------

float bme280_read_pressure(bme280_t *dev)
{
    bme280_read_temperature(dev); // Updates t_fine

    uint8_t buf[3];
    dev->i2c_read(dev->address, BME280_REG_PRESS_MSB, buf, 3);

    int32_t adc = (((int32_t)buf[0] << 12) |
                   ((int32_t)buf[1] << 4) |
                   ((int32_t)buf[2] >> 4));

    int64_t var1 = (int64_t)dev->t_fine - 128000;
    int64_t var2 = var1 * var1 * dev->calib.dig_P6;
    var2 += (var1 * dev->calib.dig_P5) << 17;
    var2 += ((int64_t)dev->calib.dig_P4) << 35;

    var1 = ((var1 * var1 * dev->calib.dig_P3) >> 8) +
           ((var1 * dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * dev->calib.dig_P1) >> 33;

    if (var1 == 0) return 0; // Avoid div0

    int64_t p = 1048576 - adc;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = (dev->calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (dev->calib.dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);

    return (float)p / 256.0f;
}

// -----------------------------------------------------------------------------
//  Humidity
// -----------------------------------------------------------------------------

float bme280_read_humidity(bme280_t *dev)
{
    bme280_read_temperature(dev); // updates t_fine

    uint8_t buf[2];
    dev->i2c_read(dev->address, BME280_REG_HUM_MSB, buf, 2);

    int32_t adc = (buf[0] << 8) | buf[1];

    int32_t v = adc - ((dev->calib.dig_H4 * 64) +
                       ((dev->calib.dig_H5 / 16384.0f) * dev->t_fine));

    float h = v * (dev->calib.dig_H2 / 65536.0f)
              * (1.0f +
                 (dev->calib.dig_H3 / 67108864.0f * dev->t_fine *
                  (1.0f + (dev->calib.dig_H6 / 67108864.0f * dev->t_fine))));

    if (h > 100.0f) h = 100.0f;
    if (h < 0.0f)   h = 0.0f;

    return h;
}

// -----------------------------------------------------------------------------
//  Altitude (uses pressure)
// -----------------------------------------------------------------------------

float bme280_read_altitude(float pressure_hPa, float seaLevel_hPa)
{
    return 44330.0f * (1.0f - powf(pressure_hPa / seaLevel_hPa, 0.1903f));
}
