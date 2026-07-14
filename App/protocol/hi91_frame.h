#ifndef HI91_FRAME_H
#define HI91_FRAME_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HI91_FRAME_HEADER_SIZE 6U
#define HI91_FRAME_PAYLOAD_SIZE 76U
#define HI91_FRAME_SIZE (HI91_FRAME_HEADER_SIZE + HI91_FRAME_PAYLOAD_SIZE)
#define HI91_FRAME_TAG 0x91U

typedef struct {
    uint16_t main_status;
    int8_t temperature_c;
    float air_pressure_pa;
    uint32_t system_time_ms;
    float acceleration_g[3];
    float angular_rate_deg_s[3];
    float magnetic_field_ut[3];
    float euler_deg[3];
    float quaternion[4];
} hi91_frame_data_t;

/* CRC-16/CCITT with polynomial 0x1021, no reflection, and caller-owned state. */
uint16_t hi91_crc16_ccitt_update(uint16_t current_crc,
                                 const uint8_t *source,
                                 size_t length);

/*
 * Encodes CRC over bytes 0..3, then continues over bytes 6..81. The CRC field
 * at bytes 4..5 is excluded and stored little-endian after the calculation.
 * Returns HI91_FRAME_SIZE on success and zero for a null/short destination.
 */
size_t hi91_frame_encode(uint8_t *destination,
                         size_t capacity,
                         const hi91_frame_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
