#include "dual_imu_estimator.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_EPOCH_US  (UINT64_C(1000000))
#define TEST_PERIOD_US (2500U)

static unsigned int failure_count;

#define TEST_EXPECT(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",           \
                          __FILE__, __LINE__, #condition);                        \
            failure_count++;                                                      \
        }                                                                         \
    } while (0)

static float quaternion_distance(const float lhs[4], const float rhs[4])
{
    float dot = fabsf((lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) +
                      (lhs[2] * rhs[2]) + (lhs[3] * rhs[3]));
    dot = fminf(1.0f, fmaxf(-1.0f, dot));
    return 2.0f * acosf(dot);
}

static void push_lane_sample_validity(dual_imu_estimator_t *estimator,
                                      imu_source_t source,
                                      uint64_t timestamp_us,
                                      uint32_t sequence,
                                      const float gyro[3],
                                      bool gyro_valid,
                                      bool accel_valid)
{
    const imu_gyro_sample_t gyro_sample = {
        .timestamp_us = timestamp_us,
        .sequence = sequence,
        .gyro_rad_s = {gyro[0], gyro[1], gyro[2]},
        .temperature_c = 25.0f,
        .source = source,
        .valid = gyro_valid,
    };
    const imu_accel_sample_t accel_sample = {
        .timestamp_us = timestamp_us,
        .sequence = sequence,
        .accel_mps2 = {0.0f, 0.0f, -9.80665f},
        .temperature_c = 25.0f,
        .source = source,
        .valid = accel_valid,
    };
    TEST_EXPECT(dual_imu_estimator_push_gyro(estimator, &gyro_sample));
    TEST_EXPECT(dual_imu_estimator_push_accel(estimator, &accel_sample));
}

static void push_lane_sample(dual_imu_estimator_t *estimator,
                             imu_source_t source,
                             uint64_t timestamp_us,
                             uint32_t sequence,
                             const float gyro[3])
{
    push_lane_sample_validity(estimator, source, timestamp_us, sequence,
                              gyro, true, true);
}

static dual_imu_estimator_t make_estimator(void)
{
    dual_imu_estimator_t estimator;
    dual_imu_estimator_config_t config;
    dual_imu_estimator_default_config(&config);
    config.stationary_dwell_windows = 20U;
    config.calibration_accept_windows = 10U;
    TEST_EXPECT(dual_imu_estimator_init(&estimator, &config, TEST_EPOCH_US));
    return estimator;
}

static void test_common_window_is_transactional(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    push_lane_sample(&estimator, IMU_SOURCE_BMI088, TEST_EPOCH_US, 0U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, TEST_EPOCH_US, 0U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                     TEST_EPOCH_US + TEST_PERIOD_US, 1U, zero);

    TEST_EXPECT(dual_imu_estimator_process_next(
                    &estimator, TEST_EPOCH_US + TEST_PERIOD_US, &output) ==
                IMU_PREINTEGRATOR_NOT_READY);
    TEST_EXPECT(estimator.preintegrator[DUAL_IMU_ESTIMATOR_LANE_BMI088]
                    .next_window_start_us == TEST_EPOCH_US);
    TEST_EXPECT(estimator.preintegrator[DUAL_IMU_ESTIMATOR_LANE_ICM45686]
                    .next_window_start_us == TEST_EPOCH_US);

    push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                     TEST_EPOCH_US + TEST_PERIOD_US, 1U, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(
                    &estimator, TEST_EPOCH_US + TEST_PERIOD_US, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.start_us == TEST_EPOCH_US);
    TEST_EXPECT(output.end_us == TEST_EPOCH_US + TEST_PERIOD_US);
}

static void test_stationary_zaru_and_bumpless_failover(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float bmi_bias[3] = {0.010f, -0.006f, 0.008f};
    const float icm_bias[3] = {0.009f, -0.005f, 0.007f};

    dual_imu_estimator_set_stationary_hint(&estimator, true);
    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, bmi_bias);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, icm_bias);
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(&estimator, timestamp, &output);
        if (sample == 0U)
            TEST_EXPECT(result == IMU_PREINTEGRATOR_NOT_READY);
        else
            TEST_EXPECT(result == IMU_PREINTEGRATOR_WINDOW_READY);
    }

    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(output.lane_calibrated[0]);
    TEST_EXPECT(output.lane_calibrated[1]);
    for (size_t axis = 0U; axis < 3U; ++axis) {
        TEST_EXPECT(fabsf(output.lane_bias_rad_s[0][axis] - bmi_bias[axis]) < 5.0e-4f);
        TEST_EXPECT(fabsf(output.lane_bias_rad_s[1][axis] - icm_bias[axis]) < 5.0e-4f);
    }

    const float before_switch[4] = {
        output.quaternion[0], output.quaternion[1],
        output.quaternion[2], output.quaternion[3],
    };
    dual_imu_estimator_set_hard_faults(&estimator,
                                       IMU_SOURCE_ICM45686,
                                       1U);
    const uint32_t sample = 81U;
    const uint64_t timestamp = TEST_EPOCH_US +
                               ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, bmi_bias);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, icm_bias);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_EXPECT(output.selector.selection_changed);
    TEST_EXPECT(quaternion_distance(before_switch, output.quaternion) < 0.001f);
}

static void test_impact_inhibits_accel_update(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    push_lane_sample(&estimator, IMU_SOURCE_BMI088, TEST_EPOCH_US, 0U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, TEST_EPOCH_US, 0U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                     TEST_EPOCH_US + TEST_PERIOD_US, 1U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                     TEST_EPOCH_US + TEST_PERIOD_US, 1U, zero);
    dual_imu_estimator_inhibit_accel_until(
        &estimator, TEST_EPOCH_US + (2U * TEST_PERIOD_US));

    TEST_EXPECT(dual_imu_estimator_process_next(
                    &estimator, TEST_EPOCH_US + TEST_PERIOD_US, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.accel_inhibited);
    TEST_EXPECT(!output.lane_seeded[0]);
    TEST_EXPECT(!output.lane_seeded[1]);
}

static void test_slow_rotation_without_external_hint_is_not_zaru(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float slow_yaw[3] = {0.0f, 0.0f, 0.02f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, slow_yaw);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, slow_yaw);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    TEST_EXPECT(output.stationary_candidate);
    TEST_EXPECT(!output.stationary_confirmed);
    TEST_EXPECT(!output.lane_calibrated[0]);
    TEST_EXPECT(!output.lane_calibrated[1]);
    TEST_EXPECT(fabsf(output.lane_bias_rad_s[0][2]) < 0.002f);
    TEST_EXPECT(fabsf(output.lane_bias_rad_s[1][2]) < 0.002f);
    TEST_EXPECT(fabsf(output.euler_rad[2]) > 0.002f);
}

static void test_impact_pauses_cross_lane_fdi(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float shock_rate[3] = {0.0f, 0.0f, 0.5f};

    for (uint32_t sample = 0U; sample <= 2U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    dual_imu_estimator_inhibit_accel_until(
        &estimator, TEST_EPOCH_US + (20U * TEST_PERIOD_US));
    for (uint32_t sample = 3U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, shock_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(&estimator,
                                                    timestamp,
                                                    &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
    }
    TEST_EXPECT(output.accel_inhibited);
    TEST_EXPECT(!output.selector.residual_valid);
    TEST_EXPECT(output.selector.mismatch_streak == 0U);
}

static void test_accel_fallback_is_explicit(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    push_lane_sample(&estimator, IMU_SOURCE_BMI088, TEST_EPOCH_US, 0U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, TEST_EPOCH_US, 0U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                     TEST_EPOCH_US + TEST_PERIOD_US, 1U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                     TEST_EPOCH_US + TEST_PERIOD_US, 1U, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(
                    &estimator, TEST_EPOCH_US + TEST_PERIOD_US, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);

    const uint64_t timestamp = TEST_EPOCH_US + (2U * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, 2U, zero);
    push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                              timestamp, 2U, zero, true, false);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.specific_force_valid);
    TEST_EXPECT(output.selected_accel_lane == IMU_SELECTOR_LANE_0);
}

static void test_dropout_lane_is_aided_and_recovers_aligned(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float yaw_rate[3] = {0.0f, 0.0f, 1.0f};

    for (uint32_t sample = 0U; sample <= 1U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, yaw_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, yaw_rate);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    for (uint32_t sample = 2U; sample <= 4U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, yaw_rate);
        push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                                  timestamp, sample, yaw_rate,
                                  sample != 2U, true);
        TEST_EXPECT(dual_imu_estimator_process_next(&estimator,
                                                    timestamp,
                                                    &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
    }

    TEST_EXPECT(output.lane_seeded[0] && output.lane_seeded[1]);
    TEST_EXPECT(quaternion_distance(output.lane_quaternion[0],
                                    output.lane_quaternion[1]) < 0.01f);
}

static void test_accel_fault_does_not_switch_healthy_gyro(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    estimator.config.accel_fault_enter_windows = 2U;

    for (uint32_t sample = 0U; sample <= 4U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                                  timestamp, sample, zero, true,
                                  sample < 2U);
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(&estimator, timestamp, &output);
        if (sample == 0U)
            TEST_EXPECT(result == IMU_PREINTEGRATOR_NOT_READY);
        else
            TEST_EXPECT(result == IMU_PREINTEGRATOR_WINDOW_READY);
    }

    TEST_EXPECT(estimator.lane_accel_fault[IMU_SELECTOR_LANE_1]);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_EXPECT(output.lane_accel_aided[IMU_SELECTOR_LANE_1]);
    TEST_EXPECT(output.selected_accel_lane == IMU_SELECTOR_LANE_0);
    TEST_EXPECT(output.output_valid);
}

static void test_switch_alignment_is_bumpless_then_converges(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    for (uint32_t sample = 0U; sample <= 2U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_1);

    const float yaw_error_rad = 0.10f;
    estimator.mekf[IMU_SELECTOR_LANE_1].q[0] = cosf(0.5f * yaw_error_rad);
    estimator.mekf[IMU_SELECTOR_LANE_1].q[1] = 0.0f;
    estimator.mekf[IMU_SELECTOR_LANE_1].q[2] = 0.0f;
    estimator.mekf[IMU_SELECTOR_LANE_1].q[3] = sinf(0.5f * yaw_error_rad);

    uint32_t sample = 3U;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    const float before_switch[4] = {
        output.quaternion[0], output.quaternion[1],
        output.quaternion[2], output.quaternion[3],
    };

    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_ICM45686, 1U);
    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_EXPECT(quaternion_distance(before_switch, output.quaternion) < 1.0e-5f);
    TEST_EXPECT(output.output_alignment_active);

    for (sample = 5U; sample <= 55U; ++sample) {
        timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, zero);
        TEST_EXPECT(dual_imu_estimator_process_next(&estimator,
                                                    timestamp,
                                                    &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
    }
    TEST_EXPECT(!output.output_alignment_active);
    TEST_EXPECT(quaternion_distance(output.quaternion,
                                    output.lane_quaternion[0]) < 1.0e-5f);
}

static void test_gyro_lane_survives_accel_dead_from_start(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    for (uint32_t sample = 0U; sample <= 1U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample_validity(&estimator, IMU_SOURCE_BMI088,
                                  timestamp, sample, zero, true, false);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.lane_seeded[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT(output.lane_accel_aided[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_1);

    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_ICM45686, 1U);
    const uint32_t sample = 2U;
    const uint64_t timestamp = TEST_EPOCH_US +
                               ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample_validity(&estimator, IMU_SOURCE_BMI088,
                              timestamp, sample, zero, true, false);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                     timestamp, sample, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.lane_accel_aided[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT(output.selected_accel_lane == IMU_SELECTOR_LANE_1);
}

int main(void)
{
    test_common_window_is_transactional();
    test_stationary_zaru_and_bumpless_failover();
    test_impact_inhibits_accel_update();
    test_slow_rotation_without_external_hint_is_not_zaru();
    test_impact_pauses_cross_lane_fdi();
    test_accel_fallback_is_explicit();
    test_dropout_lane_is_aided_and_recovers_aligned();
    test_accel_fault_does_not_switch_healthy_gyro();
    test_switch_alignment_is_bumpless_then_converges();
    test_gyro_lane_survives_accel_dead_from_start();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "dual_imu_estimator: %u test failure(s)\n",
                      failure_count);
        return EXIT_FAILURE;
    }
    (void)puts("dual_imu_estimator: all tests passed");
    return EXIT_SUCCESS;
}
