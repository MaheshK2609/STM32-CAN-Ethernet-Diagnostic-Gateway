#include "ds18b20.h"

#define DS_LOW(d)      HAL_GPIO_WritePin((d)->port, (d)->pin, GPIO_PIN_RESET)
#define DS_RELEASE(d)  HAL_GPIO_WritePin((d)->port, (d)->pin, GPIO_PIN_SET)
#define DS_READ(d)     HAL_GPIO_ReadPin((d)->port, (d)->pin)

static void delay_us(DS18B20_t *d, uint16_t us)
{
  __HAL_TIM_SET_COUNTER(d->htim, 0);
  while (__HAL_TIM_GET_COUNTER(d->htim) < us) { }
}

static uint8_t ds_reset(DS18B20_t *d)
{
  uint8_t presence;
  DS_LOW(d);
  delay_us(d, 480);
  __disable_irq();
  DS_RELEASE(d);
  delay_us(d, 70);
  presence = (DS_READ(d) == GPIO_PIN_RESET);
  __enable_irq();
  delay_us(d, 410);
  return presence;
}

static void ds_write_bit(DS18B20_t *d, uint8_t b)
{
  __disable_irq();
  DS_LOW(d);
  if (b) { delay_us(d, 6);  DS_RELEASE(d); delay_us(d, 64); }
  else   { delay_us(d, 60); DS_RELEASE(d); delay_us(d, 10); }
  __enable_irq();
}

static uint8_t ds_read_bit(DS18B20_t *d)
{
  uint8_t b;
  __disable_irq();
  DS_LOW(d);
  delay_us(d, 2);
  DS_RELEASE(d);
  delay_us(d, 8);
  b = (DS_READ(d) == GPIO_PIN_SET);
  delay_us(d, 55);
  __enable_irq();
  return b;
}

static void ds_write_byte(DS18B20_t *d, uint8_t v)
{
  for (int i = 0; i < 8; i++) { ds_write_bit(d, v & 1); v >>= 1; }
}

static uint8_t ds_read_byte(DS18B20_t *d)
{
  uint8_t v = 0;
  for (int i = 0; i < 8; i++) v |= (ds_read_bit(d) << i);
  return v;
}

static uint8_t ds_crc8(const uint8_t *p, uint8_t n)
{
  uint8_t crc = 0;
  while (n--)
  {
    uint8_t b = *p++;
    for (int i = 0; i < 8; i++)
    {
      uint8_t mix = (crc ^ b) & 1;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      b >>= 1;
    }
  }
  return crc;
}

static uint8_t ds_start_conversion(DS18B20_t *d)
{
  if (!ds_reset(d)) return 0;
  ds_write_byte(d, 0xCC);   /* skip ROM */
  ds_write_byte(d, 0x44);   /* convert T */
  return 1;
}

static uint8_t ds_read_temp(DS18B20_t *d)
{
  uint8_t sp[9];
  if (!ds_reset(d)) { d->cntNoPresence++; return 0; }
  ds_write_byte(d, 0xCC);
  ds_write_byte(d, 0xBE);   /* read scratchpad */
  for (int i = 0; i < 9; i++) sp[i] = ds_read_byte(d);
  for (int i = 0; i < 9; i++) d->sp[i] = sp[i];
  if (ds_crc8(sp, 9) != 0) { d->cntCrc++; return 0; }
  int16_t raw = (int16_t)(sp[1] << 8 | sp[0]);
  d->temp_x100 = ((int32_t)raw * 25) / 4;     /* raw is 1/16 degC */
  return 1;
}

void DS18B20_Init(DS18B20_t *dev, GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim)
{
  dev->port = port;
  dev->pin = pin;
  dev->htim = htim;
  dev->ok = 0;
  dev->busy = 0;
  dev->started = 0;
  dev->cntNoPresence = 0;
  dev->cntCrc = 0;
  HAL_TIM_Base_Start(htim);
  DS_RELEASE(dev);
}

void DS18B20_Service(DS18B20_t *dev)
{
  uint32_t now = HAL_GetTick();

  if (!dev->busy)
  {
    if (dev->started && (now - dev->t0) < DS18B20_PERIOD_MS) return;
    dev->started = 1;
    dev->t0 = now;
    if (ds_start_conversion(dev)) dev->busy = 1;
    else { dev->ok = 0; dev->cntNoPresence++; }
  }
  else if ((now - dev->t0) >= DS18B20_CONVERSION_MS)
  {
    uint8_t ok = 0;
    dev->busy = 0;
    for (int i = 0; i < 3 && !ok; i++) ok = ds_read_temp(dev);
    dev->ok = ok;
  }
}
