#include "hi91_frame.h"

#include <float.h>
#include <string.h>

#define HI91_HEADER_FIRST 0x5AU
#define HI91_HEADER_SECOND 0xA5U
#define HI91_CRC_OFFSET 4U
#define HI91_PAYLOAD_OFFSET HI91_FRAME_HEADER_SIZE

_Static_assert(sizeof(float) == sizeof(uint32_t),
               "HI91 encoding requires a 32-bit float");
_Static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "HI91 encoding requires IEEE-754 binary32 floats");
_Static_assert(HI91_FRAME_SIZE == 82U, "HI91 frame size must remain fixed");

static void write_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

static void write_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
    destination[2] = (uint8_t)(value >> 16U);
    destination[3] = (uint8_t)(value >> 24U);
}

static void write_float_le(uint8_t *destination, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    write_u32_le(destination, bits);
}

uint16_t hi91_crc16_ccitt_update(uint16_t current_crc,
                                 const uint8_t *source,
                                 size_t length)
{
    if ((source == NULL) && (length != 0U))
        return current_crc;

    uint32_t crc = current_crc;
    for (size_t byte_index = 0U; byte_index < length; ++byte_index) {
        crc ^= (uint32_t)source[byte_index] << 8U;
        for (uint32_t bit_index = 0U; bit_index < 8U; ++bit_index) {
            const uint32_t polynomial =
                ((crc & UINT32_C(0x8000)) != 0U) ? UINT32_C(0x1021) : 0U;
            crc = ((crc << 1U) ^ polynomial) & UINT32_C(0xFFFF);
        }
    }

    return (uint16_t)crc;
}

size_t hi91_frame_encode(uint8_t *destination,
                         size_t capacity,
                         const hi91_frame_data_t *data)
{
    if ((destination == NULL) || (data == NULL) ||
        (capacity < HI91_FRAME_SIZE)) {
        return 0U;
    }

    destination[0] = HI91_HEADER_FIRST;
    destination[1] = HI91_HEADER_SECOND;
    write_u16_le(&destination[2], HI91_FRAME_PAYLOAD_SIZE);
    destination[HI91_CRC_OFFSET] = 0U;
    destination[HI91_CRC_OFFSET + 1U] = 0U;

    size_t offset = HI91_PAYLOAD_OFFSET;
    destination[offset++] = HI91_FRAME_TAG;
    write_u16_le(&destination[offset], data->main_status);
    offset += sizeof(uint16_t);
    destination[offset++] = (uint8_t)data->temperature_c;
    write_float_le(&destination[offset], data->air_pressure_pa);
    offset += sizeof(float);
    write_u32_le(&destination[offset], data->system_time_ms);
    offset += sizeof(uint32_t);

    for (size_t axis = 0U; axis < 3U; ++axis) {
        write_float_le(&destination[offset], data->acceleration_g[axis]);
        offset += sizeof(float);
    }
    for (size_t axis = 0U; axis < 3U; ++axis) {
        write_float_le(&destination[offset], data->angular_rate_deg_s[axis]);
        offset += sizeof(float);
    }
    for (size_t axis = 0U; axis < 3U; ++axis) {
        write_float_le(&destination[offset], data->magnetic_field_ut[axis]);
        offset += sizeof(float);
    }
    for (size_t axis = 0U; axis < 3U; ++axis) {
        write_float_le(&destination[offset], data->euler_deg[axis]);
        offset += sizeof(float);
    }
    for (size_t component = 0U; component < 4U; ++component) {
        write_float_le(&destination[offset], data->quaternion[component]);
        offset += sizeof(float);
    }

    if (offset != HI91_FRAME_SIZE)
        return 0U;

    uint16_t crc = hi91_crc16_ccitt_update(0U, destination, HI91_CRC_OFFSET);
    crc = hi91_crc16_ccitt_update(
        crc, &destination[HI91_PAYLOAD_OFFSET], HI91_FRAME_PAYLOAD_SIZE);
    write_u16_le(&destination[HI91_CRC_OFFSET], crc);
    return HI91_FRAME_SIZE;
}
