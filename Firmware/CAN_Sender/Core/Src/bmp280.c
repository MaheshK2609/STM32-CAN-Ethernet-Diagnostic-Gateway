#include "bmp280.h"

#define REG_ID      0xD0
#define REG_CALIB   0x88
#define REG_CTRL    0xF4
#define REG_CONFIG  0xF5
#define REG_DATA    0xF7
#define I2C_TO_MS   100

static int32_t comp_T(BMP280_t *d, int32_t adc_T)
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)d->dig_T1 << 1))) * ((int32_t)d->dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)d->dig_T1)) * ((adc_T >> 4) - ((int32_t)d->dig_T1))) >> 12) * ((int32_t)d->dig_T3)) >> 14;
  d->t_fine = var1 + var2;
  return (d->t_fine * 5 + 128) >> 8;
}

static uint32_t comp_P(BMP280_t *d, int32_t adc_P)
{
  int32_t var1, var2; uint32_t p;
  var1 = (((int32_t)d->t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)d->dig_P6);
  var2 = var2 + ((var1 * ((int32_t)d->dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)d->dig_P4) << 16);
  var1 = (((d->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)d->dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)d->dig_P1)) >> 15);
  if (var1 == 0) return 0;
  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000) p = (p << 1) / ((uint32_t)var1);
  else p = (p / (uint32_t)var1) * 2;
  var1 = (((int32_t)d->dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)d->dig_P8)) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + d->dig_P7) >> 4));
  return p;
}

uint8_t BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c)
{
  uint8_t id = 0, cal[24], v;
  const uint16_t addrs[2] = { 0x76 << 1, 0x77 << 1 };

  dev->hi2c = hi2c;
  dev->ok = 0;

  for (int i = 0; i < 2; i++)
  {
    id = 0;
    if (HAL_I2C_Mem_Read(hi2c, addrs[i], REG_ID, I2C_MEMADD_SIZE_8BIT, &id, 1, I2C_TO_MS) == HAL_OK
        && (id == 0x58 || id == 0x60))
    {
      dev->addr = addrs[i];
      dev->id = id;
      break;
    }
    id = 0;
  }
  if (id == 0) return 0;

  if (HAL_I2C_Mem_Read(hi2c, dev->addr, REG_CALIB, I2C_MEMADD_SIZE_8BIT, cal, 24, I2C_TO_MS) != HAL_OK) return 0;
  dev->dig_T1 = (uint16_t)(cal[1]  << 8 | cal[0]);
  dev->dig_T2 = (int16_t) (cal[3]  << 8 | cal[2]);
  dev->dig_T3 = (int16_t) (cal[5]  << 8 | cal[4]);
  dev->dig_P1 = (uint16_t)(cal[7]  << 8 | cal[6]);
  dev->dig_P2 = (int16_t) (cal[9]  << 8 | cal[8]);
  dev->dig_P3 = (int16_t) (cal[11] << 8 | cal[10]);
  dev->dig_P4 = (int16_t) (cal[13] << 8 | cal[12]);
  dev->dig_P5 = (int16_t) (cal[15] << 8 | cal[14]);
  dev->dig_P6 = (int16_t) (cal[17] << 8 | cal[16]);
  dev->dig_P7 = (int16_t) (cal[19] << 8 | cal[18]);
  dev->dig_P8 = (int16_t) (cal[21] << 8 | cal[20]);
  dev->dig_P9 = (int16_t) (cal[23] << 8 | cal[22]);

  v = 0x00;  /* config: no filter */
  HAL_I2C_Mem_Write(hi2c, dev->addr, REG_CONFIG, I2C_MEMADD_SIZE_8BIT, &v, 1, I2C_TO_MS);
  v = 0x57;  /* ctrl_meas: temp x2, pressure x16, normal mode */
  HAL_I2C_Mem_Write(hi2c, dev->addr, REG_CTRL, I2C_MEMADD_SIZE_8BIT, &v, 1, I2C_TO_MS);

  dev->ok = 1;
  return 1;
}

uint8_t BMP280_Read(BMP280_t *dev)
{
  uint8_t d[6];
  if (HAL_I2C_Mem_Read(dev->hi2c, dev->addr, REG_DATA, I2C_MEMADD_SIZE_8BIT, d, 6, I2C_TO_MS) != HAL_OK) return 0;
  int32_t adc_P = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adc_T = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
  dev->temp_x100 = comp_T(dev, adc_T);     /* must run first: sets t_fine */
  dev->press_pa  = comp_P(dev, adc_P);
  return 1;
}
