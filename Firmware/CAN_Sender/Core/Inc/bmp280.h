/* bmp280.h - Bosch BMP280 / BME280 (temperature + pressure) over I2C, HAL based. */
#ifndef BMP280_H
#define BMP280_H

#include "stm32f4xx_hal.h"

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t addr;            /* HAL address (7-bit << 1) */
  uint8_t  id;              /* chip id: 0x58 = BMP280, 0x60 = BME280 */
  uint8_t  ok;              /* 1 after a successful BMP280_Init */
  int32_t  temp_x100;       /* last temperature, 0.01 degC */
  uint32_t press_pa;        /* last pressure, Pa */
  /* factory calibration, read from the sensor (private) */
  uint16_t dig_T1, dig_P1;
  int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
  int32_t  t_fine;
} BMP280_t;

/* Tries addresses 0x76 and 0x77. Returns 1 on success (dev->ok = 1), 0 if no sensor answers. */
uint8_t BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c);

/* Reads one sample into dev->temp_x100 / dev->press_pa. Returns 1 on success, 0 on I2C error. */
uint8_t BMP280_Read(BMP280_t *dev);

#endif /* BMP280_H */
