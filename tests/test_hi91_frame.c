#include "hi91_frame.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int failure_count;

#define TEST_EXPECT(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",           \
                          __FILE__, __LINE__, #condition);                        \
            failure_count++;                                                      \
        }                                                                         \
    } while (0)

static float float_from_bits(uint32_t bits)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint16_t crc16_ccitt_bitwise_reference(uint16_t current_crc,
                                              const uint8_t *source,
                                              size_t length)
{
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

static void test_crc_known_vectors(void)
{
    static const uint8_t check[] = "123456789";
    TEST_EXPECT(hi91_crc16_ccitt_update(0U, check, sizeof(check) - 1U) ==
                UINT16_C(0x31C3));
    TEST_EXPECT(hi91_crc16_ccitt_update(UINT16_C(0xFFFF), check,
                                       sizeof(check) - 1U) ==
                UINT16_C(0x29B1));

    uint16_t crc = hi91_crc16_ccitt_update(0U, check, 4U);
    crc = hi91_crc16_ccitt_update(crc, &check[4], sizeof(check) - 5U);
    TEST_EXPECT(crc == UINT16_C(0x31C3));
    TEST_EXPECT(hi91_crc16_ccitt_update(UINT16_C(0x1234), NULL, 0U) ==
                UINT16_C(0x1234));
    TEST_EXPECT(hi91_crc16_ccitt_update(UINT16_C(0x1234), NULL, 1U) ==
                UINT16_C(0x1234));
}

static void test_crc_all_single_byte_values(void)
{
    static const uint16_t initial_values[] = {
        UINT16_C(0x0000),
        UINT16_C(0xFFFF),
        UINT16_C(0x1D0F),
        UINT16_C(0xA5C3),
    };

    for (size_t initial_index = 0U;
         initial_index < sizeof(initial_values) / sizeof(initial_values[0]);
         ++initial_index) {
        for (uint32_t value = 0U; value <= UINT8_MAX; ++value) {
            const uint8_t byte = (uint8_t)value;
            const uint16_t expected = crc16_ccitt_bitwise_reference(
                initial_values[initial_index], &byte, 1U);
            TEST_EXPECT(hi91_crc16_ccitt_update(initial_values[initial_index],
                                                &byte, 1U) == expected);
        }
    }
}

static void test_crc_segmented_and_long_data(void)
{
    static uint8_t data[4096];
    uint32_t state = UINT32_C(0xC001D00D);
    for (size_t index = 0U; index < sizeof(data); ++index) {
        state = state * UINT32_C(1664525) + UINT32_C(1013904223);
        data[index] = (uint8_t)(state >> 24U);
    }

    const uint16_t expected =
        crc16_ccitt_bitwise_reference(UINT16_C(0xBEEF), data, sizeof(data));
    TEST_EXPECT(expected == UINT16_C(0xCF1E));
    const uint16_t one_shot =
        hi91_crc16_ccitt_update(UINT16_C(0xBEEF), data, sizeof(data));
    TEST_EXPECT(one_shot == expected);

    for (size_t split = 0U; split <= 256U; ++split) {
        uint16_t crc =
            hi91_crc16_ccitt_update(UINT16_C(0xBEEF), data, split);
        crc = hi91_crc16_ccitt_update(crc, &data[split], sizeof(data) - split);
        TEST_EXPECT(crc == one_shot);
    }

    uint16_t chunked = UINT16_C(0xBEEF);
    size_t offset = 0U;
    while (offset < sizeof(data)) {
        size_t chunk_size = ((offset * 17U) % 97U) + 1U;
        if (chunk_size > (sizeof(data) - offset))
            chunk_size = sizeof(data) - offset;
        chunked =
            hi91_crc16_ccitt_update(chunked, &data[offset], chunk_size);
        offset += chunk_size;
    }
    TEST_EXPECT(chunked == one_shot);
}

static void test_vendor_example_exact_frame(void)
{
    static const uint8_t expected[HI91_FRAME_SIZE] = {
        0x5A, 0xA5, 0x4C, 0x00, 0x14, 0xBB, 0x91, 0x08, 0x15, 0x23,
        0x09, 0xA2, 0xC4, 0x47, 0x08, 0x15, 0x1C, 0x00, 0xCC, 0xE8,
        0x61, 0xBE, 0x9A, 0x35, 0x56, 0x3E, 0x65, 0xEA, 0x72, 0x3F,
        0x31, 0xD0, 0x7C, 0xBD, 0x75, 0xDD, 0xC5, 0xBB, 0x6B, 0xD7,
        0x24, 0xBC, 0x89, 0x88, 0xFC, 0x40, 0x01, 0x00, 0x6A, 0x41,
        0xAB, 0x2A, 0x70, 0xC2, 0x96, 0xD4, 0x50, 0x41, 0xED, 0x03,
        0x43, 0x41, 0x41, 0xF4, 0xF4, 0xC2, 0xCC, 0xCA, 0xF8, 0xBE,
        0x73, 0x6A, 0x19, 0xBE, 0xF0, 0x00, 0x1C, 0x3D, 0x8D, 0x37,
        0x5C, 0x3F,
    };
    hi91_frame_data_t data = {0};
    data.main_status = UINT16_C(0x1508);
    data.temperature_c = INT8_C(35);
    data.air_pressure_pa = float_from_bits(UINT32_C(0x47C4A209));
    data.system_time_ms = UINT32_C(0x001C1508);
    data.acceleration_g[0] = float_from_bits(UINT32_C(0xBE61E8CC));
    data.acceleration_g[1] = float_from_bits(UINT32_C(0x3E56359A));
    data.acceleration_g[2] = float_from_bits(UINT32_C(0x3F72EA65));
    data.angular_rate_deg_s[0] = float_from_bits(UINT32_C(0xBD7CD031));
    data.angular_rate_deg_s[1] = float_from_bits(UINT32_C(0xBBC5DD75));
    data.angular_rate_deg_s[2] = float_from_bits(UINT32_C(0xBC24D76B));
    data.magnetic_field_ut[0] = float_from_bits(UINT32_C(0x40FC8889));
    data.magnetic_field_ut[1] = float_from_bits(UINT32_C(0x416A0001));
    data.magnetic_field_ut[2] = float_from_bits(UINT32_C(0xC2702AAB));
    data.euler_deg[0] = float_from_bits(UINT32_C(0x4150D496));
    data.euler_deg[1] = float_from_bits(UINT32_C(0x414303ED));
    data.euler_deg[2] = float_from_bits(UINT32_C(0xC2F4F441));
    data.quaternion[0] = float_from_bits(UINT32_C(0xBEF8CACC));
    data.quaternion[1] = float_from_bits(UINT32_C(0xBE196A73));
    data.quaternion[2] = float_from_bits(UINT32_C(0x3D1C00F0));
    data.quaternion[3] = float_from_bits(UINT32_C(0x3F5C378D));

    uint8_t actual[HI91_FRAME_SIZE];
    memset(actual, 0xA5, sizeof(actual));
    TEST_EXPECT(hi91_frame_encode(actual, sizeof(actual), &data) ==
                HI91_FRAME_SIZE);
    TEST_EXPECT(memcmp(actual, expected, sizeof(expected)) == 0);

    uint16_t crc = hi91_crc16_ccitt_update(0U, actual, 4U);
    crc = hi91_crc16_ccitt_update(crc, &actual[6], HI91_FRAME_PAYLOAD_SIZE);
    TEST_EXPECT(crc == UINT16_C(0xBB14));
    TEST_EXPECT(actual[4] == 0x14U);
    TEST_EXPECT(actual[5] == 0xBBU);
}

static void test_zero_missing_fields_and_bounds(void)
{
    const hi91_frame_data_t data = {0};
    uint8_t frame[HI91_FRAME_SIZE];
    memset(frame, 0xCC, sizeof(frame));

    TEST_EXPECT(hi91_frame_encode(frame, sizeof(frame), &data) ==
                HI91_FRAME_SIZE);
    TEST_EXPECT(frame[0] == 0x5AU && frame[1] == 0xA5U);
    TEST_EXPECT(frame[2] == 0x4CU && frame[3] == 0x00U);
    TEST_EXPECT(frame[6] == HI91_FRAME_TAG);
    for (size_t index = 7U; index < sizeof(frame); ++index)
        TEST_EXPECT(frame[index] == 0U);

    const uint16_t embedded_crc =
        (uint16_t)((uint16_t)frame[4] | ((uint16_t)frame[5] << 8U));
    uint16_t expected_crc = hi91_crc16_ccitt_update(0U, frame, 4U);
    expected_crc = hi91_crc16_ccitt_update(
        expected_crc, &frame[6], HI91_FRAME_PAYLOAD_SIZE);
    TEST_EXPECT(embedded_crc == expected_crc);

    uint8_t too_short[HI91_FRAME_SIZE - 1U];
    memset(too_short, 0x7E, sizeof(too_short));
    TEST_EXPECT(hi91_frame_encode(too_short, sizeof(too_short), &data) == 0U);
    for (size_t index = 0U; index < sizeof(too_short); ++index)
        TEST_EXPECT(too_short[index] == 0x7EU);
    TEST_EXPECT(hi91_frame_encode(NULL, HI91_FRAME_SIZE, &data) == 0U);
    TEST_EXPECT(hi91_frame_encode(frame, sizeof(frame), NULL) == 0U);
}

int main(void)
{
    test_crc_known_vectors();
    test_crc_all_single_byte_values();
    test_crc_segmented_and_long_data();
    test_vendor_example_exact_frame();
    test_zero_missing_fields_and_bounds();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "%u HI91 frame test(s) failed\n", failure_count);
        return EXIT_FAILURE;
    }

    (void)puts("HI91 frame tests passed");
    return EXIT_SUCCESS;
}
