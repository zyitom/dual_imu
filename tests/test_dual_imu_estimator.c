#include "dual_imu_estimator.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_EPOCH_US  (UINT64_C(1000000))
#define TEST_PERIOD_US (2500U)
#define TEST_PI_F      (3.14159265358979323846f)

static unsigned int failure_count;

#define TEST_EXPECT(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",           \
                          __FILE__, __LINE__, #condition);                        \
            failure_count++;                                                      \
        }                                                                         \
    } while (0)

static float degrees_to_radians(float degrees)
{
    return degrees * TEST_PI_F / 180.0f;
}

static float quaternion_distance(const float lhs[4], const float rhs[4])
{
    float dot = fabsf((lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) +
                      (lhs[2] * rhs[2]) + (lhs[3] * rhs[3]));
    dot = fminf(1.0f, fmaxf(-1.0f, dot));
    return 2.0f * acosf(dot);
}

static void push_lane_measurements_validity(
    dual_imu_estimator_t *estimator,
    imu_source_t source,
    uint64_t timestamp_us,
    uint32_t sequence,
    const float gyro[3],
    const float accel[3],
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
        .accel_mps2 = {accel[0], accel[1], accel[2]},
        .temperature_c = 25.0f,
        .source = source,
        .valid = accel_valid,
    };
    TEST_EXPECT(dual_imu_estimator_push_gyro(estimator, &gyro_sample));
    TEST_EXPECT(dual_imu_estimator_push_accel(estimator, &accel_sample));
}

static void push_lane_sample_validity(dual_imu_estimator_t *estimator,
                                      imu_source_t source,
                                      uint64_t timestamp_us,
                                      uint32_t sequence,
                                      const float gyro[3],
                                      bool gyro_valid,
                                      bool accel_valid)
{
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};
    push_lane_measurements_validity(estimator, source, timestamp_us, sequence,
                                    gyro, gravity, gyro_valid, accel_valid);
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
    config.stationary_hint_dwell_windows = 4U;
    config.calibration_accept_windows = 10U;
    TEST_EXPECT(dual_imu_estimator_init(&estimator, &config, TEST_EPOCH_US));
    return estimator;
}

static void process_first_measurement_window(
    dual_imu_estimator_t *estimator,
    const float start_gyro[3],
    const float end_gyro[3],
    const float bmi_accel[3],
    const float icm_accel[3],
    dual_imu_estimator_output_t *output)
{
    memset(output, 0, sizeof(*output));
    for (uint32_t sample = 0U; sample <= 1U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const float *const gyro = (sample == 0U) ? start_gyro : end_gyro;
        push_lane_measurements_validity(estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, gyro, bmi_accel,
                                        true, true);
        push_lane_measurements_validity(estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, gyro, icm_accel,
                                        true, true);
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(estimator, timestamp, output);
        TEST_EXPECT(result == ((sample == 0U)
                                   ? IMU_PREINTEGRATOR_NOT_READY
                                   : IMU_PREINTEGRATOR_WINDOW_READY));
    }
}

static void expect_stationary_rejection(
    const dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    dual_imu_stationary_reject_reason_t reason)
{
    TEST_EXPECT(!output->stationary_candidate);
    TEST_EXPECT(output->stationary_last_reject_reason == reason);
    TEST_EXPECT(estimator->stationary_last_reject_reason == reason);
    TEST_EXPECT(estimator->stationary_reject_count[reason] == 1U);
    TEST_EXPECT(output->stationary_streak == 0U);
    TEST_EXPECT(estimator->stationary_streak == 0U);
}

static void test_stationary_variance_memory_requires_two_windows(void)
{
    dual_imu_estimator_t estimator;
    dual_imu_estimator_config_t config;
    dual_imu_estimator_default_config(&config);
    config.stationary_dwell_windows = 1U;
    config.stationary_hint_dwell_windows = 1U;
    TEST_EXPECT(!dual_imu_estimator_init(&estimator, &config, TEST_EPOCH_US));
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

static void test_internal_stationary_zaru_and_bumpless_failover(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float bmi_bias[3] = {
        degrees_to_radians(0.12f),
        degrees_to_radians(-0.08f),
        degrees_to_radians(0.10f),
    };
    const float icm_bias[3] = {
        degrees_to_radians(0.10f),
        degrees_to_radians(-0.07f),
        degrees_to_radians(0.09f),
    };
    const uint32_t last_stationary_sample = 1600U;

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= last_stationary_sample; ++sample) {
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
    const uint32_t sample = last_stationary_sample + 1U;
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
    expect_stationary_rejection(
        &estimator, &output, DUAL_IMU_STATIONARY_REJECT_INHIBITED);
}

static void test_pre_statistics_stationary_reject_reasons(void)
{
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};
    dual_imu_estimator_output_t output;

    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_BMI088, 1U);
    process_first_measurement_window(&estimator, zero_rate, zero_rate,
                                     gravity, gravity, &output);
    expect_stationary_rejection(
        &estimator, &output, DUAL_IMU_STATIONARY_REJECT_INVALID_OR_FAULT);

    estimator = make_estimator();
    const float bad_norm_accel[3] = {0.0f, 0.0f, -8.0f};
    process_first_measurement_window(&estimator, zero_rate, zero_rate,
                                     bad_norm_accel, bad_norm_accel, &output);
    expect_stationary_rejection(
        &estimator, &output, DUAL_IMU_STATIONARY_REJECT_ACCEL_NORM);

    estimator = make_estimator();
    const float vibration_start[3] = {0.08f, 0.0f, 0.0f};
    const float vibration_end[3] = {-0.08f, 0.0f, 0.0f};
    process_first_measurement_window(&estimator,
                                     vibration_start, vibration_end,
                                     gravity, gravity, &output);
    expect_stationary_rejection(
        &estimator, &output,
        DUAL_IMU_STATIONARY_REJECT_WINDOW_GYRO_VARIANCE);

    estimator = make_estimator();
    const float pair_offset_mps2 = 0.50f;
    const float offset_accel[3] = {
        pair_offset_mps2,
        0.0f,
        -sqrtf((9.80665f * 9.80665f) -
               (pair_offset_mps2 * pair_offset_mps2)),
    };
    process_first_measurement_window(&estimator, zero_rate, zero_rate,
                                     gravity, offset_accel, &output);
    expect_stationary_rejection(
        &estimator, &output, DUAL_IMU_STATIONARY_REJECT_ACCEL_PAIR);
}

static void test_first_accel_direction_outlier_does_not_restart_dwell(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float disturbance_rad = 0.08f;
    const float disturbed_gravity[3] = {
        9.80665f * sinf(disturbance_rad),
        0.0f,
        -9.80665f * cosf(disturbance_rad),
    };
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 20U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const float *const accel = (sample == 0U) ? disturbed_gravity : gravity;
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero_rate, accel,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero_rate, accel,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    TEST_EXPECT(output.stationary_candidate);
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(estimator.stationary_statistics_count ==
                estimator.config.stationary_dwell_windows);
    TEST_EXPECT(estimator.stationary_gravity_reference_valid[0]);
    TEST_EXPECT(estimator.stationary_gravity_reference_valid[1]);
}

static void test_rotation_above_internal_limit_is_not_zaru(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float rotation[3] = {0.0f, 0.0f, 0.08f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, rotation);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, rotation);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    TEST_EXPECT(!output.stationary_candidate);
    TEST_EXPECT(!output.stationary_confirmed);
    TEST_EXPECT(!output.lane_calibrated[0]);
    TEST_EXPECT(!output.lane_calibrated[1]);
    TEST_EXPECT(fabsf(output.lane_bias_rad_s[0][2]) < 0.002f);
    TEST_EXPECT(fabsf(output.lane_bias_rad_s[1][2]) < 0.002f);
    TEST_EXPECT(fabsf(output.euler_rad[2]) > 0.002f);
    TEST_EXPECT(output.stationary_last_reject_reason ==
                DUAL_IMU_STATIONARY_REJECT_INSTANT_RATE);
    TEST_EXPECT(estimator.stationary_reject_count
                    [DUAL_IMU_STATIONARY_REJECT_INSTANT_RATE] > 0U);
}

static void test_two_dps_yaw_rotation_is_not_zaru(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float yaw_rate[3] = {0.0f, 0.0f, degrees_to_radians(2.0f)};
    bool saw_stationary_confirmed = false;

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 200U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, yaw_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, yaw_rate);
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(&estimator, timestamp, &output);
        if ((result == IMU_PREINTEGRATOR_WINDOW_READY) &&
            output.stationary_confirmed)
            saw_stationary_confirmed = true;
    }

    TEST_EXPECT(!saw_stationary_confirmed);
    TEST_EXPECT(!output.lane_calibrated[0]);
    TEST_EXPECT(!output.lane_calibrated[1]);
    TEST_EXPECT(estimator.stationary_streak <
                estimator.config.stationary_dwell_windows);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_update_count == 0U);
        TEST_EXPECT(fabsf(output.lane_bias_rad_s[lane][2]) < 0.002f);
    }
    TEST_EXPECT(output.angular_rate_rad_s[2] > degrees_to_radians(1.9f));
    TEST_EXPECT(output.stationary_last_reject_reason ==
                DUAL_IMU_STATIONARY_REJECT_MEAN_RATE);
    TEST_EXPECT(estimator.stationary_reject_count
                    [DUAL_IMU_STATIONARY_REJECT_MEAN_RATE] > 0U);
}

static void test_external_hint_does_not_bypass_stationary_rate_mean_gate(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float yaw_rate[3] = {0.0f, 0.0f, degrees_to_radians(2.0f)};
    bool saw_stationary_confirmed = false;

    dual_imu_estimator_set_stationary_hint(&estimator, true);
    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 40U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, yaw_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, yaw_rate);
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(&estimator, timestamp, &output);
        if ((result == IMU_PREINTEGRATOR_WINDOW_READY) &&
            output.stationary_confirmed)
            saw_stationary_confirmed = true;
    }

    TEST_EXPECT(!saw_stationary_confirmed);
    TEST_EXPECT(!output.lane_calibrated[0]);
    TEST_EXPECT(!output.lane_calibrated[1]);
    TEST_EXPECT(estimator.mekf[0].diagnostics.zaru_update_count == 0U);
    TEST_EXPECT(estimator.mekf[1].diagnostics.zaru_update_count == 0U);
    TEST_EXPECT(output.stationary_last_reject_reason ==
                DUAL_IMU_STATIONARY_REJECT_MEAN_RATE);
}

static void test_temporal_gyro_variance_breaks_stationary_dwell(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    bool saw_temporal_rejection = false;

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 25U; ++sample) {
        const float rate[3] = {
            0.0f,
            0.0f,
            degrees_to_radians(0.10f * (float)sample),
        };
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, rate);
        if (dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY &&
            !output.stationary_candidate)
            saw_temporal_rejection = true;
    }

    TEST_EXPECT(saw_temporal_rejection);
    TEST_EXPECT(!output.stationary_confirmed);
    TEST_EXPECT(estimator.stationary_streak <
                estimator.config.stationary_dwell_windows);
    TEST_EXPECT(output.stationary_last_reject_reason ==
                DUAL_IMU_STATIONARY_REJECT_TEMPORAL_VARIANCE);
    TEST_EXPECT(estimator.stationary_reject_count
                    [DUAL_IMU_STATIONARY_REJECT_TEMPORAL_VARIANCE] > 0U);
}

static void test_temporal_accel_variance_breaks_stationary_dwell(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    bool saw_temporal_rejection = false;

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 25U; ++sample) {
        const float accel_offset = (sample <= 8U) ? -0.15f : 0.15f;
        const float accel[3] = {0.0f, 0.0f, -9.80665f + accel_offset};
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero_rate, accel,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero_rate, accel,
                                        true, true);
        if (dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY &&
            !output.stationary_candidate)
            saw_temporal_rejection = true;
    }

    TEST_EXPECT(saw_temporal_rejection);
    TEST_EXPECT(!output.stationary_confirmed);
    TEST_EXPECT(estimator.stationary_streak <
                estimator.config.stationary_dwell_windows);
    TEST_EXPECT(output.stationary_last_reject_reason ==
                DUAL_IMU_STATIONARY_REJECT_TEMPORAL_VARIANCE);
    TEST_EXPECT(estimator.stationary_reject_count
                    [DUAL_IMU_STATIONARY_REJECT_TEMPORAL_VARIANCE] > 0U);
}

static void test_gravity_direction_reason_and_streak_diagnostics(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};
    const float tilt_rad = 0.10f;
    const float tilted_gravity[3] = {
        9.80665f * sinf(tilt_rad),
        0.0f,
        -9.80665f * cosf(tilt_rad),
    };

    estimator.config.stationary_accel_temporal_variance_limit_m2_s4 = 100.0f;
    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 20U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero_rate, gravity,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero_rate, gravity,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(output.stationary_streak == 20U);
    TEST_EXPECT(output.stationary_max_streak == 20U);

    bool saw_gravity_rejection = false;
    for (uint32_t sample = 21U; sample <= 40U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero_rate,
                                        tilted_gravity, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero_rate,
                                        tilted_gravity, true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        if (output.stationary_last_reject_reason ==
                DUAL_IMU_STATIONARY_REJECT_GRAVITY_DIRECTION &&
            !output.stationary_candidate) {
            saw_gravity_rejection = true;
            break;
        }
    }

    TEST_EXPECT(saw_gravity_rejection);
    TEST_EXPECT(output.stationary_streak == 0U);
    TEST_EXPECT(output.stationary_max_streak > 20U);
    TEST_EXPECT(output.stationary_max_streak ==
                estimator.stationary_max_streak);
    TEST_EXPECT(estimator.stationary_reject_count
                    [DUAL_IMU_STATIONARY_REJECT_GRAVITY_DIRECTION] == 1U);
}

static void test_motion_after_long_stationary_interval_is_detected(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float moving_rate[3] = {
        0.0f, 0.0f, degrees_to_radians(2.0f),
    };
    bool saw_motion_rejection = false;

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 1000U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero_rate);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(estimator.stationary_statistics_count ==
                estimator.config.stationary_dwell_windows);

    for (uint32_t sample = 1001U; sample <= 1010U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, moving_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, moving_rate);
        if (dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY &&
            !output.stationary_candidate)
            saw_motion_rejection = true;
    }

    TEST_EXPECT(saw_motion_rejection);
    TEST_EXPECT(!output.stationary_confirmed);
    TEST_EXPECT(estimator.stationary_streak <
                estimator.config.stationary_dwell_windows);
}

static void test_external_hint_only_shortens_stationary_dwell(void)
{
    dual_imu_estimator_t unhinted = make_estimator();
    dual_imu_estimator_t hinted = make_estimator();
    dual_imu_estimator_output_t unhinted_output;
    dual_imu_estimator_output_t hinted_output;
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};

    dual_imu_estimator_set_stationary_hint(&hinted, true);
    memset(&unhinted_output, 0, sizeof(unhinted_output));
    memset(&hinted_output, 0, sizeof(hinted_output));
    for (uint32_t sample = 0U; sample <= 15U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&unhinted, IMU_SOURCE_BMI088,
                         timestamp, sample, zero_bias);
        push_lane_sample(&unhinted, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero_bias);
        push_lane_sample(&hinted, IMU_SOURCE_BMI088,
                         timestamp, sample, zero_bias);
        push_lane_sample(&hinted, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero_bias);
        (void)dual_imu_estimator_process_next(
            &unhinted, timestamp, &unhinted_output);
        (void)dual_imu_estimator_process_next(
            &hinted, timestamp, &hinted_output);
    }

    TEST_EXPECT(unhinted_output.stationary_candidate);
    TEST_EXPECT(!unhinted_output.stationary_confirmed);
    TEST_EXPECT(!unhinted_output.lane_calibrated[0]);
    TEST_EXPECT(!unhinted_output.lane_calibrated[1]);
    TEST_EXPECT(hinted_output.stationary_confirmed);
    TEST_EXPECT(hinted_output.lane_calibrated[0]);
    TEST_EXPECT(hinted_output.lane_calibrated[1]);
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

static void test_switch_alignment_preserves_unobservable_yaw(void)
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
    TEST_EXPECT(!output.output_alignment_active);

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
    TEST_EXPECT(quaternion_distance(before_switch, output.quaternion) < 1.0e-5f);
    TEST_EXPECT(quaternion_distance(output.quaternion,
                                    output.lane_quaternion[0]) > 0.09f);
    TEST_EXPECT(fabsf(estimator.output_alignment[1]) < 1.0e-6f);
    TEST_EXPECT(fabsf(estimator.output_alignment[2]) < 1.0e-6f);
    TEST_EXPECT(fabsf(estimator.output_alignment[3]) > 0.04f);
}

static void test_switch_alignment_decays_only_observable_tilt(void)
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
    const float roll_error_rad = 0.05f;
    estimator.mekf[IMU_SELECTOR_LANE_1].q[0] = cosf(0.5f * yaw_error_rad);
    estimator.mekf[IMU_SELECTOR_LANE_1].q[1] = 0.0f;
    estimator.mekf[IMU_SELECTOR_LANE_1].q[2] = 0.0f;
    estimator.mekf[IMU_SELECTOR_LANE_1].q[3] = sinf(0.5f * yaw_error_rad);
    estimator.mekf[IMU_SELECTOR_LANE_0].q[0] = cosf(0.5f * roll_error_rad);
    estimator.mekf[IMU_SELECTOR_LANE_0].q[1] = sinf(0.5f * roll_error_rad);
    estimator.mekf[IMU_SELECTOR_LANE_0].q[2] = 0.0f;
    estimator.mekf[IMU_SELECTOR_LANE_0].q[3] = 0.0f;

    uint32_t sample = 3U;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample_validity(&estimator, IMU_SOURCE_BMI088,
                              timestamp, sample, zero, true, false);
    push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                              timestamp, sample, zero, true, false);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    const float before_switch[4] = {
        output.quaternion[0], output.quaternion[1],
        output.quaternion[2], output.quaternion[3],
    };

    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_ICM45686, 1U);
    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample_validity(&estimator, IMU_SOURCE_BMI088,
                              timestamp, sample, zero, true, false);
    push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                              timestamp, sample, zero, true, false);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_EXPECT(quaternion_distance(before_switch, output.quaternion) < 1.0e-5f);
    TEST_EXPECT(output.output_alignment_active);

    for (sample = 5U; sample <= 55U; ++sample) {
        timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample_validity(&estimator, IMU_SOURCE_BMI088,
                                  timestamp, sample, zero, true, false);
        push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                                  timestamp, sample, zero, true, false);
        TEST_EXPECT(dual_imu_estimator_process_next(&estimator,
                                                    timestamp,
                                                    &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
    }

    TEST_EXPECT(!output.output_alignment_active);
    TEST_EXPECT(fabsf(estimator.output_alignment[1]) < 1.0e-6f);
    TEST_EXPECT(fabsf(estimator.output_alignment[2]) < 1.0e-6f);
    TEST_EXPECT(fabsf(estimator.output_alignment[3]) > 0.04f);
    TEST_EXPECT(quaternion_distance(output.quaternion,
                                    output.lane_quaternion[0]) > 0.09f);
    TEST_EXPECT(fabsf(output.euler_rad[0] - roll_error_rad) < 1.0e-4f);
    TEST_EXPECT(fabsf(output.euler_rad[2] - yaw_error_rad) < 1.0e-4f);
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
    test_stationary_variance_memory_requires_two_windows();
    test_common_window_is_transactional();
    test_internal_stationary_zaru_and_bumpless_failover();
    test_impact_inhibits_accel_update();
    test_pre_statistics_stationary_reject_reasons();
    test_first_accel_direction_outlier_does_not_restart_dwell();
    test_rotation_above_internal_limit_is_not_zaru();
    test_two_dps_yaw_rotation_is_not_zaru();
    test_external_hint_does_not_bypass_stationary_rate_mean_gate();
    test_temporal_gyro_variance_breaks_stationary_dwell();
    test_temporal_accel_variance_breaks_stationary_dwell();
    test_gravity_direction_reason_and_streak_diagnostics();
    test_motion_after_long_stationary_interval_is_detected();
    test_external_hint_only_shortens_stationary_dwell();
    test_impact_pauses_cross_lane_fdi();
    test_accel_fallback_is_explicit();
    test_dropout_lane_is_aided_and_recovers_aligned();
    test_accel_fault_does_not_switch_healthy_gyro();
    test_switch_alignment_preserves_unobservable_yaw();
    test_switch_alignment_decays_only_observable_tilt();
    test_gyro_lane_survives_accel_dead_from_start();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "dual_imu_estimator: %u test failure(s)\n",
                      failure_count);
        return EXIT_FAILURE;
    }
    (void)puts("dual_imu_estimator: all tests passed");
    return EXIT_SUCCESS;
}
