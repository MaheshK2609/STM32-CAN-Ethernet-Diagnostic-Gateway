/* ds18b20.h - Maxim DS18B20 on a single GPIO (1-Wire bit-bang), non-blocking service.
 *
 * Hardware needs: GPIO pin as open-drain output (initial level high), 4.7 kOhm pull-up to 3.3V,
 * and a timer running at 1 MHz (1 us per count), e.g. TIM3 with prescaler = APB1 timer clock/1e6 - 1.
 */
#ifndef DS18B20_H
#define DS18B20_H

#include "stm32f4xx_hal.h"

#define DS18B20_PERIOD_MS      2000u   /* start a new conversion this often */
#define DS18B20_CONVERSION_MS  800u    /* 12-bit conversion needs up to 750 ms */

typedef struct {
  GPIO_TypeDef      *port;
  uint16_t           pin;
  TIM_HandleTypeDef *htim;       /* 1 MHz free-running timer used for microsecond delays */
  int32_t            temp_x100;  /* last good temperature, 0.01 degC */
  uint8_t            ok;         /* 1 if the last read succeeded */
  uint32_t           cntNoPresence;
  uint32_t           cntCrc;
  uint8_t            sp[9];      /* last scratchpad (for debugging) */
  uint8_t            busy, started;   /* service state (private) */
  uint32_t           t0;
} DS18B20_t;

/* Starts the timer and releases the bus. */
void DS18B20_Init(DS18B20_t *dev, GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim);

/* Call as often as possible from the main loop. Never blocks longer than one 1-Wire transaction. */
void DS18B20_Service(DS18B20_t *dev);

#endif /* DS18B20_H */
