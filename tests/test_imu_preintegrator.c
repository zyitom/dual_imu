#include "imu_preintegrator.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int failure_count;

#define TEST_EXPECT(condition)                                                        \
    do                                                                                \
    {                                                                                 \
        if (!(condition))                                                             \
        {                                                                             \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",               \
                          __FILE__, __LINE__, #condition);                             \
            failure_count++;                                                          \
        }                                                                             \
    } while (0)

static void expect_near(float actual, float expected, float tolerance)
{
    TEST_EXPECT(isfinite(actual));
    TEST_EXPECT(fabsf(actual - expected) <= tolerance);
}

static void test_rotation_vector_to_quaternion(const float rotation[3],
                                               float quaternion[4])
{
    const float angle = sqrtf((rotation[0] * rotation[0]) +
                              (rotation[1] * rotation[1]) +
                              (rotation[2] * rotation[2]));
    const float scale = (angle < 1.0e-8f)
                            ? 0.5f
                            : sinf(0.5f * angle) / angle;
    quaternion[0] = (angle < 1.0e-8f) ? 1.0f : cosf(0.5f * angle);
    quaternion[1] = scale * rotation[0];
    quaternion[2] = scale * rotation[1];
    quaternion[3] = scale * rotation[2];
}

static void test_quaternion_multiply(const float lhs[4],
                                     const float rhs[4],
                                     float result[4])
{
    result[0] = (lhs[0] * rhs[0]) - (lhs[1] * rhs[1]) -
                (lhs[2] * rhs[2]) - (lhs[3] * rhs[3]);
    result[1] = (lhs[0] * rhs[1]) + (lhs[1] * rhs[0]) +
                (lhs[2] * rhs[3]) - (lhs[3] * rhs[2]);
    result[2] = (lhs[0] * rhs[2]) - (lhs[1] * rhs[3]) +
                (lhs[2] * rhs[0]) + (lhs[3] * rhs[1]);
    result[3] = (lhs[0] * rhs[3]) + (lhs[1] * rhs[2]) -
                (lhs[2] * rhs[1]) + (lhs[3] * rhs[0]);
}

static imu_preintegrator_t make_preintegrator(uint64_t epoch_us,
                                               uint32_t max_gap_us)
{
    imu_preintegrator_t preintegrator;
    imu_preintegrator_config_t config;
    imu_preintegrator_default_config(&config);
    config.max_gyro_gap_us = max_gap_us;
    config.max_accel_gap_us = max_gap_us;
    TEST_EXPECT(imu_preintegrator_init(&preintegrator,
                                       &config,
                                       IMU_SOURCE_ICM45686,
                                       epoch_us));
    return preintegrator;
}

static void push_gyro(imu_preintegrator_t *preintegrator,
                      uint64_t timestamp_us,
                      uint32_t sequence,
                      float x,
                      float y,
                      float z,
                      bool valid)
{
    const imu_gyro_sample_t sample = {
        .timestamp_us = timestamp_us,
        .sequence = sequence,
        .gyro_rad_s = {x, y, z},
        .temperature_c = 25.0f,
        .source = IMU_SOURCE_ICM45686,
        .valid = valid,
    };
    TEST_EXPECT(imu_preintegrator_push_gyro(preintegrator, &sample));
}

static void push_accel(imu_preintegrator_t *preintegrator,
                       uint64_t timestamp_us,
                       uint32_t sequence,
                       float x,
                       float y,
                       float z,
                       bool valid)
{
    const imu_accel_sample_t sample = {
        .timestamp_us = timestamp_us,
        .sequence = sequence,
        .accel_mps2 = {x, y, z},
        .temperature_c = 25.0f,
        .source = IMU_SOURCE_ICM45686,
        .valid = valid,
    };
    TEST_EXPECT(imu_preintegrator_push_accel(preintegrator, &sample));
}

static imu_preintegrated_window_t get_window(imu_preintegrator_t *preintegrator,
                                             uint64_t watermark_us)
{
    imu_preintegrated_window_t window;
    memset(&window, 0xA5, sizeof(window));
    TEST_EXPECT(imu_preintegrator_next_window(preintegrator,
                                              watermark_us,
                                              &window) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    return window;
}

static void test_readiness_query_does_not_consume_window(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(0U, 5000U);
    push_gyro(&preintegrator, 0U, 0U, 0.0f, 0.0f, 1.0f, true);
    push_gyro(&preintegrator, 2500U, 1U, 0.0f, 0.0f, 1.0f, true);
    push_accel(&preintegrator, 0U, 0U, 0.0f, 0.0f, -9.80665f, true);
    push_accel(&preintegrator, 2500U, 1U, 0.0f, 0.0f, -9.80665f, true);

    TEST_EXPECT(imu_preintegrator_next_window_ready(&preintegrator, 2500U) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(imu_preintegrator_next_window_ready(&preintegrator, 2500U) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    const imu_preintegrated_window_t window = get_window(&preintegrator, 2500U);
    TEST_EXPECT(window.start_us == 0U);
    TEST_EXPECT(window.end_us == 2500U);
    TEST_EXPECT(imu_preintegrator_next_window_ready(&preintegrator, 2500U) ==
                IMU_PREINTEGRATOR_NOT_READY);
}

static void test_constant_irregular_samples_and_large_timestamp(void)
{
    const uint64_t epoch = (UINT64_C(1) << 53) + 12345U;
    imu_preintegrator_t preintegrator = make_preintegrator(epoch, 5000U);
    const uint32_t offsets[] = {0U, 317U, 811U, 1299U, 2077U, 2671U};

    for (uint32_t index = 0U; index <
                                  (uint32_t)(sizeof(offsets) / sizeof(offsets[0]));
         ++index)
    {
        push_gyro(&preintegrator,
                  epoch + offsets[index],
                  index,
                  1.0f,
                  -2.0f,
                  0.5f,
                  true);
        push_accel(&preintegrator,
                   epoch + offsets[index],
                   index,
                   2.0f,
                   4.0f,
                   -8.0f,
                   true);
    }

    TEST_EXPECT(imu_preintegrator_next_window(&preintegrator, epoch + 2499U,
                                              &(imu_preintegrated_window_t){0}) ==
                IMU_PREINTEGRATOR_NOT_READY);
    const imu_preintegrated_window_t window =
        get_window(&preintegrator, epoch + 2671U);

    TEST_EXPECT(window.start_us == epoch);
    TEST_EXPECT(window.end_us == epoch + 2500U);
    TEST_EXPECT(window.gyro_valid);
    TEST_EXPECT(window.accel_valid);
    TEST_EXPECT(window.flags == IMU_PREINTEGRATOR_FLAG_NONE);
    TEST_EXPECT(window.gyro_coverage_us == 2500U);
    TEST_EXPECT(window.accel_coverage_us == 2500U);
    expect_near(window.delta_angle_rad[0], 0.0025f, 2.0e-7f);
    expect_near(window.delta_angle_rad[1], -0.005f, 2.0e-7f);
    expect_near(window.delta_angle_rad[2], 0.00125f, 2.0e-7f);
    expect_near(window.accel_mean_mps2[0], 2.0f, 2.0e-6f);
    expect_near(window.accel_mean_mps2[1], 4.0f, 2.0e-6f);
    expect_near(window.accel_mean_mps2[2], -8.0f, 2.0e-6f);
    expect_near(window.gyro_mean_rad_s[0], 1.0f, 2.0e-6f);
    expect_near(window.gyro_mean_rad_s[1], -2.0f, 2.0e-6f);
    expect_near(window.gyro_second_moment_rad2_s2[0][0], 1.0f, 3.0e-6f);
    expect_near(window.gyro_second_moment_rad2_s2[0][1], -2.0f, 3.0e-6f);
}

static void test_boundary_interpolation_and_cross_window_conservation(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(1000U, 5000U);
    const uint64_t timestamps[] = {800U, 1700U, 3100U, 4300U, 6200U};

    for (uint32_t index = 0U; index <
                                  (uint32_t)(sizeof(timestamps) /
                                             sizeof(timestamps[0]));
         ++index)
    {
        const float seconds = (float)(timestamps[index] - 800U) * 1.0e-6f;
        const float gyro_x = 2.0f + (100.0f * seconds);
        const float accel_x = 1.0f + (200.0f * seconds);
        push_gyro(&preintegrator, timestamps[index], index, gyro_x, 0.0f, 0.0f,
                  true);
        push_accel(&preintegrator, timestamps[index], index, accel_x, 0.0f, -9.0f,
                   true);
    }

    const imu_preintegrated_window_t first = get_window(&preintegrator, 6200U);
    const imu_preintegrated_window_t second = get_window(&preintegrator, 6200U);
    TEST_EXPECT(first.gyro_valid && second.gyro_valid);
    TEST_EXPECT(first.accel_valid && second.accel_valid);

    /* Integral of 2 + 100*(t-0.0008) from 0.001 to 0.006 seconds. */
    const float expected_total_angle =
        (2.0f * 0.005f) +
        (50.0f * ((0.0052f * 0.0052f) - (0.0002f * 0.0002f)));
    expect_near(first.delta_angle_rad[0] + second.delta_angle_rad[0],
                expected_total_angle,
                3.0e-7f);

    /* Linear time average at each window midpoint. */
    expect_near(first.accel_mean_mps2[0], 1.0f + (200.0f * 0.00145f), 3.0e-5f);
    expect_near(second.accel_mean_mps2[0], 1.0f + (200.0f * 0.00395f), 3.0e-5f);
}

static void test_all_segments_coning_composition(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(0U, 5000U);
    const float rates[4][3] = {
        {100.0f, 0.0f, 0.0f},
        {0.0f, 100.0f, 0.0f},
        {-100.0f, 0.0f, 0.0f},
        {0.0f, -100.0f, 0.0f},
    };
    const uint64_t timestamps[4] = {0U, 700U, 1600U, 2600U};

    for (uint32_t index = 0U; index < 4U; ++index)
    {
        push_gyro(&preintegrator,
                  timestamps[index],
                  index,
                  rates[index][0],
                  rates[index][1],
                  rates[index][2],
                  true);
        push_accel(&preintegrator,
                   timestamps[index],
                   index,
                   0.0f,
                   0.0f,
                   -9.80665f,
                   true);
    }

    const imu_preintegrated_window_t window = get_window(&preintegrator, 2600U);
    TEST_EXPECT(window.gyro_valid);
    TEST_EXPECT(window.gyro_segment_count == 3U);
    /* A component normal to the commanded XY rate plane proves coning survived. */
    TEST_EXPECT(fabsf(window.delta_angle_rad[2]) > 1.0e-4f);
    TEST_EXPECT(fabsf(window.first_half_delta_angle_rad[2]) > 1.0e-5f);
    TEST_EXPECT(fabsf(window.second_half_delta_angle_rad[2]) > 1.0e-5f);
    float first_quaternion[4];
    float second_quaternion[4];
    float composed_quaternion[4];
    test_rotation_vector_to_quaternion(window.first_half_delta_angle_rad,
                                       first_quaternion);
    test_rotation_vector_to_quaternion(window.second_half_delta_angle_rad,
                                       second_quaternion);
    test_quaternion_multiply(first_quaternion, second_quaternion,
                             composed_quaternion);
    const float quaternion_dot = fabsf(
        (composed_quaternion[0] * window.delta_quaternion[0]) +
        (composed_quaternion[1] * window.delta_quaternion[1]) +
        (composed_quaternion[2] * window.delta_quaternion[2]) +
        (composed_quaternion[3] * window.delta_quaternion[3]));
    TEST_EXPECT(quaternion_dot > 0.999999f);
    TEST_EXPECT(fabsf(window.delta_quaternion[0]) <= 1.0f);
    const float quaternion_norm = sqrtf(
        (window.delta_quaternion[0] * window.delta_quaternion[0]) +
        (window.delta_quaternion[1] * window.delta_quaternion[1]) +
        (window.delta_quaternion[2] * window.delta_quaternion[2]) +
        (window.delta_quaternion[3] * window.delta_quaternion[3]));
    expect_near(quaternion_norm, 1.0f, 2.0e-6f);
}

static void test_true_half_windows_follow_linear_ramp(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(0U, 5000U);
    const uint64_t timestamps[3] = {0U, 1250U, 2500U};
    for (uint32_t index = 0U; index < 3U; ++index) {
        const float rate_x = 1000.0f * ((float)timestamps[index] * 1.0e-6f);
        push_gyro(&preintegrator, timestamps[index], index,
                  rate_x, 0.0f, 0.0f, true);
        push_accel(&preintegrator, timestamps[index], index,
                   0.0f, 0.0f, -9.80665f, true);
    }

    const imu_preintegrated_window_t window = get_window(&preintegrator, 2500U);
    expect_near(window.first_half_delta_angle_rad[0], 0.00078125f, 2.0e-7f);
    expect_near(window.second_half_delta_angle_rad[0], 0.00234375f, 2.0e-7f);
    expect_near(window.delta_angle_rad[0], 0.003125f, 2.0e-7f);
    TEST_EXPECT(window.second_half_delta_angle_rad[0] >
                (2.5f * window.first_half_delta_angle_rad[0]));
}

static void test_invalid_coverage_and_stopped_stream(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(0U, 5000U);
    push_gyro(&preintegrator, 0U, 0U, 0.1f, 0.0f, 0.0f, true);
    push_gyro(&preintegrator, 1000U, 1U, 0.1f, 0.0f, 0.0f, false);
    push_gyro(&preintegrator, 3000U, 2U, 0.1f, 0.0f, 0.0f, true);
    /* No accelerometer samples: watermark must still close the window. */

    TEST_EXPECT(imu_preintegrator_next_window(&preintegrator,
                                              7499U,
                                              &(imu_preintegrated_window_t){0}) ==
                IMU_PREINTEGRATOR_NOT_READY);
    const imu_preintegrated_window_t window = get_window(&preintegrator, 7500U);
    TEST_EXPECT(!window.gyro_valid);
    TEST_EXPECT(!window.accel_valid);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_GYRO_INVALID) != 0U);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_GYRO_MISSING) != 0U);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_ACCEL_MISSING) != 0U);
    TEST_EXPECT(window.gyro_coverage_us == 0U);
    TEST_EXPECT(window.accel_coverage_us == 0U);
}

static void test_gap_drop_and_timestamp_diagnostics(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(1000U, 800U);
    push_gyro(&preintegrator, 900U, 10U, 0.0f, 0.0f, 1.0f, true);
    push_gyro(&preintegrator, 1900U, 12U, 0.0f, 0.0f, 1.0f, true);
    push_gyro(&preintegrator, 2900U, 13U, 0.0f, 0.0f, 1.0f, true);
    push_gyro(&preintegrator, 3900U, 14U, 0.0f, 0.0f, 1.0f, true);
    /* A late sample is rejected, then reported on the next closed window. */
    {
        const imu_gyro_sample_t late = {
            .timestamp_us = 2000U,
            .sequence = 99U,
            .gyro_rad_s = {0.0f, 0.0f, 1.0f},
            .source = IMU_SOURCE_ICM45686,
            .valid = true,
        };
        TEST_EXPECT(!imu_preintegrator_push_gyro(&preintegrator, &late));
    }

    push_accel(&preintegrator, 900U, 20U, 0.0f, 0.0f, -9.0f, true);
    push_accel(&preintegrator, 1900U, 22U, 0.0f, 0.0f, -9.0f, true);
    push_accel(&preintegrator, 2900U, 23U, 0.0f, 0.0f, -9.0f, true);
    push_accel(&preintegrator, 3900U, 24U, 0.0f, 0.0f, -9.0f, true);

    const imu_preintegrated_window_t window = get_window(&preintegrator, 3900U);
    TEST_EXPECT(!window.gyro_valid);
    TEST_EXPECT(!window.accel_valid);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_GYRO_GAP) != 0U);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_ACCEL_GAP) != 0U);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_GYRO_SEQUENCE_DROP) != 0U);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_ACCEL_SEQUENCE_DROP) != 0U);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_GYRO_TIMESTAMP_ORDER) != 0U);

    const imu_preintegrator_diagnostics_t *diagnostics =
        imu_preintegrator_get_diagnostics(&preintegrator);
    TEST_EXPECT(diagnostics != NULL);
    TEST_EXPECT(diagnostics->gyro_out_of_order_samples == 1U);
    TEST_EXPECT(diagnostics->gyro_sequence_discontinuities == 1U);
    TEST_EXPECT(diagnostics->accel_sequence_discontinuities == 1U);
    TEST_EXPECT(diagnostics->gyro_estimated_dropped_samples == 1U);
    TEST_EXPECT(diagnostics->accel_estimated_dropped_samples == 1U);
    TEST_EXPECT(diagnostics->maximum_gyro_gap_us == 1000U);
}

static void test_sequence_counter_wrap_is_continuous(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(0U, 5000U);
    const uint32_t first_sequence = UINT32_MAX;
    push_gyro(&preintegrator, 0U, first_sequence, 0.0f, 0.0f, 0.0f, true);
    push_gyro(&preintegrator, 2500U, 0U, 0.0f, 0.0f, 0.0f, true);
    push_accel(&preintegrator, 0U, first_sequence, 0.0f, 0.0f, -9.0f, true);
    push_accel(&preintegrator, 2500U, 0U, 0.0f, 0.0f, -9.0f, true);
    const imu_preintegrated_window_t window = get_window(&preintegrator, 2500U);
    TEST_EXPECT(window.gyro_valid);
    TEST_EXPECT(window.accel_valid);
    TEST_EXPECT(window.flags == IMU_PREINTEGRATOR_FLAG_NONE);
}

static void test_sequence_drop_can_still_propagate_covered_angle(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(0U, 5000U);
    push_gyro(&preintegrator, 0U, 10U, 0.0f, 0.0f, 1.0f, true);
    push_gyro(&preintegrator, 1250U, 12U, 0.0f, 0.0f, 1.0f, true);
    push_gyro(&preintegrator, 2500U, 13U, 0.0f, 0.0f, 1.0f, true);
    push_accel(&preintegrator, 0U, 20U, 0.0f, 0.0f, -9.0f, true);
    push_accel(&preintegrator, 1250U, 21U, 0.0f, 0.0f, -9.0f, true);
    push_accel(&preintegrator, 2500U, 22U, 0.0f, 0.0f, -9.0f, true);

    const imu_preintegrated_window_t window = get_window(&preintegrator, 2500U);
    TEST_EXPECT(!window.gyro_valid);
    TEST_EXPECT(window.gyro_propagation_valid);
    TEST_EXPECT(window.gyro_coverage_us == 2500U);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_GYRO_SEQUENCE_DROP) != 0U);
    expect_near(window.delta_angle_rad[2], 0.0025f, 2.0e-7f);
}

static void test_queue_overflow_is_visible_to_window(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(0U, 5000U);
    for (uint32_t index = 0U; index < IMU_PREINTEGRATOR_QUEUE_CAPACITY; ++index)
    {
        push_gyro(&preintegrator,
                  (uint64_t)index * 100U,
                  index,
                  0.0f,
                  0.0f,
                  0.1f,
                  true);
    }

    const imu_gyro_sample_t overflow = {
        .timestamp_us =
            (uint64_t)IMU_PREINTEGRATOR_QUEUE_CAPACITY * 100U,
        .sequence = IMU_PREINTEGRATOR_QUEUE_CAPACITY,
        .gyro_rad_s = {0.0f, 0.0f, 0.1f},
        .source = IMU_SOURCE_ICM45686,
        .valid = true,
    };
    TEST_EXPECT(!imu_preintegrator_push_gyro(&preintegrator, &overflow));
    push_accel(&preintegrator, 0U, 0U, 0.0f, 0.0f, -9.0f, true);
    push_accel(&preintegrator, 2500U, 1U, 0.0f, 0.0f, -9.0f, true);

    const imu_preintegrated_window_t window = get_window(&preintegrator, 2500U);
    TEST_EXPECT(!window.gyro_valid);
    TEST_EXPECT(window.accel_valid);
    TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_GYRO_QUEUE_OVERFLOW) != 0U);
    TEST_EXPECT(imu_preintegrator_get_diagnostics(&preintegrator)
                    ->gyro_queue_overflows == 1U);
}

static void test_long_gap_cannot_create_propagation(void)
{
    imu_preintegrator_t preintegrator = make_preintegrator(0U, 1250U);
    const uint64_t far_timestamp_us = 100000U;
    push_gyro(&preintegrator, 0U, 0U, 0.0f, 0.0f, 1.0f, true);
    push_gyro(&preintegrator, far_timestamp_us, 1U, 0.0f, 0.0f, 1.0f, true);
    push_accel(&preintegrator, 0U, 0U, 0.0f, 0.0f, -9.80665f, true);
    push_accel(&preintegrator, far_timestamp_us, 1U,
               0.0f, 0.0f, -9.80665f, true);

    for (uint32_t index = 0U; index < 40U; ++index)
    {
        const imu_preintegrated_window_t window =
            get_window(&preintegrator, far_timestamp_us);
        TEST_EXPECT(!window.gyro_valid);
        TEST_EXPECT(!window.gyro_propagation_valid);
        TEST_EXPECT((window.flags & IMU_PREINTEGRATOR_FLAG_GYRO_GAP) != 0U);
    }
}

int main(void)
{
    test_readiness_query_does_not_consume_window();
    test_constant_irregular_samples_and_large_timestamp();
    test_boundary_interpolation_and_cross_window_conservation();
    test_all_segments_coning_composition();
    test_true_half_windows_follow_linear_ramp();
    test_invalid_coverage_and_stopped_stream();
    test_gap_drop_and_timestamp_diagnostics();
    test_sequence_counter_wrap_is_continuous();
    test_sequence_drop_can_still_propagate_covered_angle();
    test_queue_overflow_is_visible_to_window();
    test_long_gap_cannot_create_propagation();

    if (failure_count != 0U)
    {
        (void)fprintf(stderr,
                      "imu_preintegrator: %u test failure(s)\n",
                      failure_count);
        return EXIT_FAILURE;
    }

    (void)puts("imu_preintegrator: all tests passed");
    return EXIT_SUCCESS;
}
