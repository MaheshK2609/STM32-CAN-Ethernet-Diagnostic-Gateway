/* can_protocol.h - CAN message layout shared by sender and receiver.
 * Copy this same file into both projects (Core/Inc). All multi-byte values are little-endian. */
#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

#define VEH_CAN_ID_ENV  0x100u   /* BMP280: temp int16 (0.01 degC) + pressure uint32 (Pa), DLC 6 */
#define VEH_CAN_ID_DS   0x101u   /* DS18B20: temp int16 (0.01 degC), DLC 2 */
#define VEH_CAN_DLC_ENV 6u
#define VEH_CAN_DLC_DS  2u

static inline void veh_pack_env(uint8_t d[6], int32_t temp_x100, uint32_t press_pa)
{
  uint16_t t = (uint16_t)(int16_t)temp_x100;
  d[0] = t & 0xFF;          d[1] = (t >> 8) & 0xFF;
  d[2] = press_pa & 0xFF;   d[3] = (press_pa >> 8) & 0xFF;
  d[4] = (press_pa >> 16) & 0xFF;  d[5] = (press_pa >> 24) & 0xFF;
}

static inline void veh_unpack_env(const uint8_t d[6], int32_t *temp_x100, uint32_t *press_pa)
{
  *temp_x100 = (int16_t)(d[0] | (d[1] << 8));
  *press_pa  = d[2] | (d[3] << 8) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24);
}

static inline void veh_pack_ds(uint8_t d[2], int32_t temp_x100)
{
  uint16_t t = (uint16_t)(int16_t)temp_x100;
  d[0] = t & 0xFF;  d[1] = (t >> 8) & 0xFF;
}

static inline int32_t veh_unpack_ds(const uint8_t d[2])
{
  return (int16_t)(d[0] | (d[1] << 8));
}

#endif /* CAN_PROTOCOL_H */
