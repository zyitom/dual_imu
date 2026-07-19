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

bool dual_imu_estimator_test_reseed_tilt_preserving_heading(
    attitude_mekf_t *filter,
    const float specific_force_mps2[3],
    const float reference_quaternion[4],
    const float bias_rad_s[3]);

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

static void quaternion_from_roll_yaw(float roll_rad,
                                     float yaw_rad,
                                     float quaternion[4])
{
    const float half_roll = 0.5f * roll_rad;
    const float half_yaw = 0.5f * yaw_rad;
    quaternion[0] = cosf(half_yaw) * cosf(half_roll);
    quaternion[1] = cosf(half_yaw) * sinf(half_roll);
    quaternion[2] = sinf(half_yaw) * sinf(half_roll);
    quaternion[3] = sinf(half_yaw) * cosf(half_roll);
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
    config.stationary_single_lane_dwell_windows = 40U;
    config.stationary_single_lane_hint_dwell_windows = 8U;
    config.attitude_convergence_windows = 4U;
    config.attitude_aiding_timeout_windows = 12U;
    config.post_impact_reacquire_dwell_windows = 20U;
    config.post_impact_reacquire_single_lane_dwell_windows = 40U;
    config.post_impact_gravity_trust_windows = 4U;
    config.calibration_accept_windows = 10U;
    config.calibration_revoke_windows = 4U;
    TEST_EXPECT(dual_imu_estimator_init(&estimator, &config, TEST_EPOCH_US));
    return estimator;
}

static uint32_t confirm_level_stationary(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output,
    uint32_t required_windows)
{
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    memset(output, 0, sizeof(*output));
    for (uint32_t sample = 0U; sample <= required_windows; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero_rate);
        push_lane_sample(estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero_rate);
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(estimator, timestamp, output);
        TEST_EXPECT(result == ((sample == 0U)
                                  ? IMU_PREINTEGRATOR_NOT_READY
                                  : IMU_PREINTEGRATOR_WINDOW_READY));
    }
    TEST_EXPECT(output->stationary_candidate);
    TEST_EXPECT(output->stationary_confirmed);
    return required_windows;
}

static void use_sensitive_disagreement_gate(dual_imu_estimator_t *estimator)
{
    estimator->selector.config.nis_enter_threshold = 1.0e-8f;
    estimator->selector.config.nis_clear_threshold = 5.0e-9f;
}

static void test_disagreement_confirmation_requires_multiple_windows(void)
{
    dual_imu_estimator_t estimator;
    dual_imu_estimator_config_t config;
    dual_imu_estimator_default_config(&config);
    config.impact_gyro_disagreement_confirm_windows = 1U;
    TEST_EXPECT(!dual_imu_estimator_init(&estimator, &config, TEST_EPOCH_US));
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

static void test_external_hard_faults_cannot_inject_filter_fault(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_set_hard_faults(
        &estimator, IMU_SOURCE_BMI088, UINT32_MAX);
    TEST_EXPECT(estimator.hard_fault_flags[IMU_SELECTOR_LANE_0] ==
                (UINT32_MAX & ~(UINT32_C(1) << 31)));
    dual_imu_estimator_set_hard_faults(
        &estimator, IMU_SOURCE_BMI088, 0U);
    TEST_EXPECT(estimator.hard_fault_flags[IMU_SELECTOR_LANE_0] == 0U);
}

static void test_reseed_failure_is_transactional(void)
{
    attitude_mekf_config_t config;
    attitude_mekf_default_config(&config);
    attitude_mekf_t filter;
    TEST_EXPECT(attitude_mekf_init(&filter, &config));
    const float accel[3] = {0.0f, 0.0f, -9.80665f};
    const float reference[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float bias[3] = {0.001f, -0.002f, 0.003f};

    float bad_accel[3] = {NAN, accel[1], accel[2]};
    float bad_reference[4] = {reference[0], NAN, 0.0f, 0.0f};
    float bad_bias[3] = {bias[0], INFINITY, bias[2]};
    attitude_mekf_t before = filter;
    TEST_EXPECT(!dual_imu_estimator_test_reseed_tilt_preserving_heading(
        &filter, bad_accel, reference, bias));
    TEST_EXPECT(memcmp(&filter, &before, sizeof(filter)) == 0);

    TEST_EXPECT(!dual_imu_estimator_test_reseed_tilt_preserving_heading(
        &filter, accel, bad_reference, bias));
    TEST_EXPECT(memcmp(&filter, &before, sizeof(filter)) == 0);

    TEST_EXPECT(!dual_imu_estimator_test_reseed_tilt_preserving_heading(
        &filter, accel, reference, bad_bias));
    TEST_EXPECT(memcmp(&filter, &before, sizeof(filter)) == 0);

    filter.config.covariance_ceiling =
        0.5f * filter.config.covariance_floor;
    before = filter;
    TEST_EXPECT(!dual_imu_estimator_test_reseed_tilt_preserving_heading(
        &filter, accel, reference, bias));
    TEST_EXPECT(memcmp(&filter, &before, sizeof(filter)) == 0);
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
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator, TEST_EPOCH_US,
        TEST_EPOCH_US + (2U * TEST_PERIOD_US)));

    TEST_EXPECT(dual_imu_estimator_process_next(
                    &estimator, TEST_EPOCH_US + TEST_PERIOD_US, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.accel_inhibited);
    TEST_EXPECT(!output.lane_seeded[0]);
    TEST_EXPECT(!output.lane_seeded[1]);
    expect_stationary_rejection(
        &estimator, &output, DUAL_IMU_STATIONARY_REJECT_INHIBITED);
}

static void test_impact_intervals_are_causal_under_backlog(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const uint32_t final_sample = 6U;

    for (uint32_t sample = 0U; sample <= final_sample; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, zero);
    }

    const uint64_t first_start = TEST_EPOCH_US +
                                 (UINT64_C(3) * TEST_PERIOD_US);
    const uint64_t first_end = TEST_EPOCH_US +
                               (UINT64_C(4) * TEST_PERIOD_US);
    const uint64_t second_start = TEST_EPOCH_US +
                                  (UINT64_C(5) * TEST_PERIOD_US);
    const uint64_t second_end = TEST_EPOCH_US +
                                (UINT64_C(6) * TEST_PERIOD_US);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator, second_start, second_end));
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator, first_start, first_end));
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator, first_start + 100U, first_end));
    TEST_EXPECT(estimator.accel_inhibit_interval_count == 2U);
    TEST_EXPECT(estimator.accel_inhibit_intervals[0].start_us == first_start);
    TEST_EXPECT(estimator.accel_inhibit_intervals[0].end_us == first_end);
    TEST_EXPECT(estimator.accel_inhibit_intervals[1].start_us == second_start);
    TEST_EXPECT(estimator.accel_inhibit_intervals[1].end_us == second_end);

    const uint64_t complete_through = TEST_EPOCH_US +
                                      ((uint64_t)final_sample * TEST_PERIOD_US);
    for (uint32_t window = 1U; window <= final_sample; ++window) {
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, complete_through, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        const bool expected_inhibited = (window == 4U) || (window == 6U);
        TEST_EXPECT(output.accel_inhibited == expected_inhibited);
    }
    TEST_EXPECT(estimator.accel_inhibit_interval_count == 0U);
}

static void test_pre_statistics_stationary_reject_reasons(void)
{
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};
    dual_imu_estimator_output_t output;

    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_BMI088, 1U);
    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_ICM45686, 1U);
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

static void test_first_accel_direction_outlier_eventually_recovers_stationary(void)
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
    for (uint32_t sample = 0U; sample <= 200U; ++sample) {
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

static void test_physical_roll_is_not_learned_as_gyro_bias(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float roll_rate_rad_s = degrees_to_radians(1.0f);
    const float gyro[3] = {roll_rate_rad_s, 0.0f, 0.0f};
    bool saw_stationary_confirmed = false;

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 4000U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const float elapsed_s =
            (float)sample * (float)TEST_PERIOD_US * 1.0e-6f;
        const float roll_rad = roll_rate_rad_s * elapsed_s;
        const float accel[3] = {
            0.0f,
            -9.80665f * sinf(roll_rad),
            -9.80665f * cosf(roll_rad),
        };
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, gyro, accel,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, gyro, accel,
                                        true, true);
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(&estimator, timestamp, &output);
        if ((result == IMU_PREINTEGRATOR_WINDOW_READY) &&
            output.stationary_confirmed) {
            saw_stationary_confirmed = true;
        }
    }

    TEST_EXPECT(!saw_stationary_confirmed);
    TEST_EXPECT(!output.stationary_confirmed);
    TEST_EXPECT(!output.lane_calibrated[0]);
    TEST_EXPECT(!output.lane_calibrated[1]);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_update_count == 0U);
        TEST_EXPECT(fabsf(output.lane_bias_rad_s[lane][0]) <
                    degrees_to_radians(0.05f));
    }
    TEST_EXPECT(output.angular_rate_rad_s[0] > degrees_to_radians(0.95f));
    TEST_EXPECT(output.stationary_last_reject_reason ==
                DUAL_IMU_STATIONARY_REJECT_MEAN_RATE);
}

static void test_gravity_aiding_learns_static_observable_bias_before_zaru(void)
{
    dual_imu_estimator_t estimator;
    dual_imu_estimator_config_t config;
    dual_imu_estimator_output_t output;
    const float static_bias[3] = {
        degrees_to_radians(1.0f), 0.0f, 0.0f,
    };
    const float level_accel[3] = {0.0f, 0.0f, -9.80665f};
    bool saw_stationary_confirmed = false;
    bool saw_zaru = false;

    dual_imu_estimator_default_config(&config);
    TEST_EXPECT(dual_imu_estimator_init(&estimator, &config, TEST_EPOCH_US));
    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 6000U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, static_bias,
                                        level_accel, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, static_bias,
                                        level_accel, true, true);
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(&estimator, timestamp, &output);
        if (result == IMU_PREINTEGRATOR_WINDOW_READY) {
            saw_stationary_confirmed = saw_stationary_confirmed ||
                                       output.stationary_confirmed;
            saw_zaru = saw_zaru ||
                (output.zaru_result[0] == ATTITUDE_MEKF_ZARU_ACCEPTED) ||
                (output.zaru_result[1] == ATTITUDE_MEKF_ZARU_ACCEPTED);
        }
    }

    TEST_EXPECT(saw_stationary_confirmed);
    TEST_EXPECT(saw_zaru);
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(output.lane_calibrated[0]);
    TEST_EXPECT(output.lane_calibrated[1]);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(fabsf(output.lane_bias_rad_s[lane][0] - static_bias[0]) <
                    degrees_to_radians(0.05f));
        TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_accept_count > 0U);
    }
    TEST_EXPECT(fabsf(output.angular_rate_rad_s[0]) <
                degrees_to_radians(0.05f));
}

static void test_unobservable_static_yaw_rate_is_not_learned_as_bias(void)
{
    dual_imu_estimator_t estimator;
    dual_imu_estimator_config_t config;
    dual_imu_estimator_output_t output;
    const float yaw_rate[3] = {
        0.0f, 0.0f, degrees_to_radians(1.0f),
    };
    bool saw_stationary_confirmed = false;

    dual_imu_estimator_default_config(&config);
    TEST_EXPECT(dual_imu_estimator_init(&estimator, &config, TEST_EPOCH_US));
    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 4000U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, yaw_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, yaw_rate);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_stationary_confirmed =
            saw_stationary_confirmed || output.stationary_confirmed;
    }

    TEST_EXPECT(!saw_stationary_confirmed);
    TEST_EXPECT(!output.lane_calibrated[0]);
    TEST_EXPECT(!output.lane_calibrated[1]);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_update_count == 0U);
        TEST_EXPECT(fabsf(output.lane_bias_rad_s[lane][2]) <
                    degrees_to_radians(0.05f));
    }
    TEST_EXPECT(output.angular_rate_rad_s[2] > degrees_to_radians(0.95f));
    TEST_EXPECT(output.stationary_last_reject_reason ==
                DUAL_IMU_STATIONARY_REJECT_MEAN_RATE);
}

static void test_zaru_bias_recovery_reseeds_diverged_bias(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    /* Matches the field capture (FIX_PLAN §12.1): the learned yaw bias driven
     * ~5.5 dps off during violent motion while the sensor itself stayed
     * healthy, deadlocking the ZARU residual gate. */
    const float diverged_bias[3] = {0.0f, 0.0f, degrees_to_radians(5.5f)};

    estimator.config.zaru_recovery_reject_windows = 16U;
    uint32_t sample = confirm_level_stationary(
        &estimator, &output,
        4U * estimator.config.stationary_dwell_windows);

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        float quaternion[4];
        TEST_EXPECT(attitude_mekf_get_quaternion(&estimator.mekf[lane],
                                                 quaternion));
        TEST_EXPECT(attitude_mekf_reset(&estimator.mekf[lane], quaternion,
                                        diverged_bias));
    }

    bool saw_reseed[DUAL_IMU_ESTIMATOR_LANE_COUNT] = {false, false};
    bool reconfirmed_after_reseed = false;
    for (uint32_t window = 0U; window < 1200U; ++window) {
        sample++;
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            saw_reseed[lane] = saw_reseed[lane] ||
                               output.lane_bias_recovery_reseeded[lane];
        }
        reconfirmed_after_reseed = reconfirmed_after_reseed ||
            ((saw_reseed[0] || saw_reseed[1]) && output.stationary_confirmed);
    }

    TEST_EXPECT(saw_reseed[0]);
    TEST_EXPECT(saw_reseed[1]);
    TEST_EXPECT(estimator.zaru_bias_recovery_count >= 2U);
    TEST_EXPECT(reconfirmed_after_reseed);
    TEST_EXPECT(output.stationary_confirmed);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(fabsf(output.lane_bias_rad_s[lane][2]) <
                    degrees_to_radians(0.2f));
    }
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(fabsf(output.angular_rate_rad_s[2]) <
                degrees_to_radians(0.2f));
}

static void test_zaru_bias_recovery_rejects_slow_tilt_rotation(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    /* 1.5 dps about body x sits inside the tilt rest tolerance, so only the
     * bias-independent evidence (gravity direction moving across the dwell,
     * accel temporal variance) separates this genuine rotation from a
     * diverged bias. It must never be reseeded into the bias state. */
    const float roll_rate_rad_s = degrees_to_radians(1.5f);
    const float rate[3] = {roll_rate_rad_s, 0.0f, 0.0f};

    /* At 1.5 dps the gravity direction crosses the 0.02-chord stability limit
     * after ~0.77 s = ~307 windows, so an 800-window recovery dwell keeps the
     * production invariant (gravity gate fires well before the reseed dwell
     * elapses) while still running fast as a host test. */
    estimator.config.zaru_recovery_reject_windows = 800U;
    memset(&output, 0, sizeof(output));
    bool saw_confirmed = false;
    for (uint32_t sample = 0U; sample <= 4000U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const float angle = roll_rate_rad_s * (float)sample *
                            ((float)TEST_PERIOD_US * 1.0e-6f);
        const float accel[3] = {
            0.0f,
            -9.80665f * sinf(angle),
            -9.80665f * cosf(angle),
        };
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, rate, accel,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, rate, accel,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_confirmed = saw_confirmed || output.stationary_confirmed;
        TEST_EXPECT(!output.lane_bias_recovery_reseeded[0]);
        TEST_EXPECT(!output.lane_bias_recovery_reseeded[1]);
    }
    TEST_EXPECT(estimator.zaru_bias_recovery_count == 0U);
    TEST_EXPECT(!saw_confirmed);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(fabsf(output.lane_bias_rad_s[lane][0]) <
                    degrees_to_radians(0.1f));
    }
}

static void test_calibration_convergence_is_revoked_after_bias_shift(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float shifted_bias[3] = {
        degrees_to_radians(0.30f), 0.0f, 0.0f,
    };

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.lane_calibrated[0]);
    TEST_EXPECT(output.lane_calibrated[1]);

    bool saw_revoked = false;
    for (uint32_t sample = 81U; sample <= 180U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, shifted_bias);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, shifted_bias);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_revoked = saw_revoked || !output.lane_calibrated[0] ||
                      !output.lane_calibrated[1];
    }

    TEST_EXPECT(saw_revoked);
    TEST_EXPECT(!output.lane_calibrated[0]);
    TEST_EXPECT(!output.lane_calibrated[1]);
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

static void test_stationary_soft_outlier_keeps_status_and_pauses_zaru(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float accel_outlier[3] = {0.0f, 0.0f, -10.80665f};
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};
    (void)confirm_level_stationary(
        &estimator, &output, estimator.config.stationary_dwell_windows);

    uint32_t zaru_before[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float bias_before[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        zaru_before[lane] = estimator.mekf[lane].diagnostics.zaru_update_count;
        TEST_EXPECT(attitude_mekf_get_bias(&estimator.mekf[lane],
                                           bias_before[lane]));
    }

    uint32_t sample = estimator.config.stationary_dwell_windows + 1U;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                    timestamp, sample, zero_rate,
                                    accel_outlier, true, true);
    push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                    timestamp, sample, zero_rate,
                                    accel_outlier, true, true);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!output.stationary_candidate);
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(estimator.stationary_soft_exit_streak == 1U);
    TEST_EXPECT(estimator.stationary_rate_exit_streak == 0U);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_update_count ==
                    zaru_before[lane]);
        for (size_t axis = 0U; axis < 3U; ++axis) {
            TEST_EXPECT(fabsf(estimator.mekf[lane].gyro_bias_rad_s[axis] -
                              bias_before[lane][axis]) < 1.0e-9f);
        }
    }

    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                    timestamp, sample, zero_rate,
                                    gravity, true, true);
    push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                    timestamp, sample, zero_rate,
                                    gravity, true, true);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!output.stationary_candidate);
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(estimator.stationary_soft_exit_streak == 2U);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_update_count ==
                    zaru_before[lane]);
    }

    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                    timestamp, sample, zero_rate,
                                    gravity, true, true);
    push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                    timestamp, sample, zero_rate,
                                    gravity, true, true);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.stationary_candidate);
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(estimator.stationary_soft_exit_streak == 0U);
    TEST_EXPECT(estimator.stationary_statistics_count == 1U);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_update_count ==
                    zaru_before[lane]);
    }
}

static void test_stationary_soft_exit_and_full_reentry_dwell(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float accel_outlier[3] = {0.0f, 0.0f, -10.80665f};
    const uint32_t dwell = estimator.config.stationary_dwell_windows;
    uint32_t sample = confirm_level_stationary(&estimator, &output, dwell);

    for (uint16_t reject = 1U;
         reject <= estimator.config.stationary_soft_exit_windows;
         ++reject) {
        sample++;
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero_rate,
                                        accel_outlier, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero_rate,
                                        accel_outlier, true, true);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(!output.stationary_candidate);
        if (reject < estimator.config.stationary_soft_exit_windows) {
            TEST_EXPECT(output.stationary_confirmed);
            TEST_EXPECT(estimator.stationary_soft_exit_streak == reject);
        } else {
            TEST_EXPECT(!output.stationary_confirmed);
            TEST_EXPECT(!estimator.stationary_confirmed_latched);
        }
    }

    sample++;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                     timestamp, sample, zero_rate);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                     timestamp, sample, zero_rate);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!output.stationary_confirmed);

    for (uint32_t clean = 1U; clean < dwell; ++clean) {
        sample++;
        timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(!output.stationary_confirmed);
    }
    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                     timestamp, sample, zero_rate);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                     timestamp, sample, zero_rate);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output.stationary_confirmed);
}

static void run_dual_stationary_rate_exit_case(float rate_dps)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float rate[3] = {0.0f, 0.0f, degrees_to_radians(rate_dps)};
    uint32_t sample = confirm_level_stationary(
        &estimator, &output, estimator.config.stationary_dwell_windows);

    sample++;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, rate);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, rate);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);

    uint32_t zaru_after_transition[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        zaru_after_transition[lane] =
            estimator.mekf[lane].diagnostics.zaru_update_count;
    }
    bool exited = !output.stationary_confirmed;
    for (uint16_t window = 0U;
         window < estimator.config.stationary_rate_exit_windows;
         ++window) {
        sample++;
        timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        exited = exited || !output.stationary_confirmed;
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_update_count ==
                        zaru_after_transition[lane]);
        }
    }
    TEST_EXPECT(exited);
    TEST_EXPECT(!output.stationary_confirmed);
}

static void test_stationary_dual_lane_slow_rate_exit(void)
{
    run_dual_stationary_rate_exit_case(0.6f);
    run_dual_stationary_rate_exit_case(2.0f);
}

static void test_stationary_single_lane_slow_rate_exit(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, degrees_to_radians(0.6f)};
    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_BMI088, 1U);
    uint32_t sample = confirm_level_stationary(
        &estimator, &output,
        estimator.config.stationary_single_lane_dwell_windows);
    TEST_EXPECT(output.stationary_lane_mask ==
                IMU_SELECTOR_LANE_MASK(IMU_SELECTOR_LANE_1));

    sample++;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, rate);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    const uint32_t zaru_after_transition =
        estimator.mekf[IMU_SELECTOR_LANE_1].diagnostics.zaru_update_count;

    for (uint16_t window = 0U;
         window < estimator.config.stationary_rate_exit_windows;
         ++window) {
        sample++;
        timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(estimator.mekf[IMU_SELECTOR_LANE_1]
                        .diagnostics.zaru_update_count == zaru_after_transition);
    }
    TEST_EXPECT(!output.stationary_confirmed);
}

static void test_stationary_one_lane_rate_disagreement_pauses_zaru(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, degrees_to_radians(0.6f)};
    uint32_t sample = confirm_level_stationary(
        &estimator, &output, estimator.config.stationary_dwell_windows);

    sample++;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, sample, rate);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686, timestamp, sample, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);

    uint32_t zaru_before[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float bias_before[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        zaru_before[lane] = estimator.mekf[lane].diagnostics.zaru_update_count;
        TEST_EXPECT(attitude_mekf_get_bias(&estimator.mekf[lane],
                                           bias_before[lane]));
    }

    for (uint16_t window = 1U;
         window <= estimator.config.stationary_soft_exit_windows;
         ++window) {
        sample++;
        timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            TEST_EXPECT(estimator.mekf[lane].diagnostics.zaru_update_count ==
                        zaru_before[lane]);
            for (size_t axis = 0U; axis < 3U; ++axis) {
                TEST_EXPECT(fabsf(estimator.mekf[lane].gyro_bias_rad_s[axis] -
                                  bias_before[lane][axis]) < 1.0e-9f);
            }
        }
        if (window < estimator.config.stationary_soft_exit_windows)
            TEST_EXPECT(output.stationary_confirmed);
    }
    TEST_EXPECT(!output.stationary_confirmed);
}

static void test_stationary_integrity_events_exit_immediately(void)
{
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};

    dual_imu_estimator_t hard_fault_estimator = make_estimator();
    dual_imu_estimator_output_t hard_fault_output;
    uint32_t sample = confirm_level_stationary(
        &hard_fault_estimator, &hard_fault_output,
        hard_fault_estimator.config.stationary_dwell_windows);
    dual_imu_estimator_set_hard_faults(
        &hard_fault_estimator, IMU_SOURCE_BMI088, 1U);
    TEST_EXPECT(!hard_fault_estimator.stationary_confirmed_latched);
    sample++;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&hard_fault_estimator, IMU_SOURCE_BMI088,
                     timestamp, sample, zero);
    push_lane_sample(&hard_fault_estimator, IMU_SOURCE_ICM45686,
                     timestamp, sample, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(
                    &hard_fault_estimator, timestamp, &hard_fault_output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!hard_fault_output.stationary_confirmed);

    dual_imu_estimator_t invalid_estimator = make_estimator();
    dual_imu_estimator_output_t invalid_output;
    sample = confirm_level_stationary(
        &invalid_estimator, &invalid_output,
        invalid_estimator.config.stationary_dwell_windows) + 1U;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_measurements_validity(&invalid_estimator, IMU_SOURCE_BMI088,
                                    timestamp, sample, zero, gravity,
                                    true, false);
    push_lane_measurements_validity(&invalid_estimator, IMU_SOURCE_ICM45686,
                                    timestamp, sample, zero, gravity,
                                    true, true);
    TEST_EXPECT(dual_imu_estimator_process_next(
                    &invalid_estimator, timestamp, &invalid_output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!invalid_output.stationary_confirmed);

    dual_imu_estimator_t impact_estimator = make_estimator();
    dual_imu_estimator_output_t impact_output;
    sample = confirm_level_stationary(
        &impact_estimator, &impact_output,
        impact_estimator.config.stationary_dwell_windows);
    const uint64_t impact_start = TEST_EPOCH_US +
        ((uint64_t)sample * TEST_PERIOD_US);
    TEST_EXPECT(dual_imu_estimator_notify_impact_interval(
        &impact_estimator, impact_start, impact_start + TEST_PERIOD_US));
    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(&impact_estimator, IMU_SOURCE_BMI088,
                     timestamp, sample, zero);
    push_lane_sample(&impact_estimator, IMU_SOURCE_ICM45686,
                     timestamp, sample, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(
                    &impact_estimator, timestamp, &impact_output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(impact_output.accel_inhibited);
    TEST_EXPECT(!impact_output.stationary_confirmed);
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

static void test_impact_gyro_disagreement_fails_closed(void)
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

    float attitude_before[DUAL_IMU_ESTIMATOR_LANE_COUNT][4];
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(attitude_mekf_get_quaternion(&estimator.mekf[lane],
                                                 attitude_before[lane]));
    }
    const imu_angular_accel_estimator_t angular_accel_before =
        estimator.angular_accel_estimator;

    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator,
        TEST_EPOCH_US + (2U * TEST_PERIOD_US),
        TEST_EPOCH_US + (20U * TEST_PERIOD_US)));
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
    TEST_EXPECT(output.selector.residual_valid);
    TEST_EXPECT(output.selector.mismatch_streak > 0U);
    TEST_EXPECT(output.selector.state == IMU_SELECTOR_AMBIGUOUS);
    TEST_EXPECT(output.rotation_unobserved);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(output.post_impact_reacquire_active);
    TEST_EXPECT(!output.attitude_converged);
    TEST_EXPECT(!output.output_valid);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        float attitude_after[4];
        TEST_EXPECT(attitude_mekf_get_quaternion(&estimator.mekf[lane],
                                                 attitude_after));
        TEST_EXPECT(quaternion_distance(attitude_before[lane],
                                        attitude_after) < 1.0e-6f);
    }
    for (size_t axis = 0U; axis < 3U; ++axis) {
        TEST_EXPECT(estimator.angular_accel_estimator.previous_gyro_rad_s[axis] ==
                    angular_accel_before.previous_gyro_rad_s[axis]);
        TEST_EXPECT(estimator.angular_accel_estimator.angular_accel_rad_s2[axis] ==
                    angular_accel_before.angular_accel_rad_s2[axis]);
    }
    TEST_EXPECT(estimator.angular_accel_estimator.cutoff_hz ==
                angular_accel_before.cutoff_hz);
    TEST_EXPECT(estimator.angular_accel_estimator.limit_rad_s2 ==
                angular_accel_before.limit_rad_s2);
    TEST_EXPECT(estimator.angular_accel_estimator.initialized ==
                angular_accel_before.initialized);
}

static void test_impact_gyro_sample_spike_is_held_until_clear(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float spike[3] = {0.0f, 0.0f, 0.5f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.output_valid);
    use_sensitive_disagreement_gate(&estimator);

    float trusted_lane_quaternion[4];
    float trusted_output_quaternion[4];
    TEST_EXPECT(attitude_mekf_get_quaternion(
        &estimator.mekf[IMU_SELECTOR_LANE_1], trusted_lane_quaternion));
    memcpy(trusted_output_quaternion, estimator.output_quaternion,
           sizeof(trusted_output_quaternion));
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator,
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US),
        TEST_EPOCH_US + (UINT64_C(18) * TEST_PERIOD_US)));

    for (uint32_t sample = 13U; sample <= 15U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const float *const icm_rate = (sample == 13U) ? spike : zero;
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, icm_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);

        if (sample < 15U) {
            TEST_EXPECT(output.selector.residual_valid);
            TEST_EXPECT(output.selector.residual_nis >
                        estimator.selector.config.nis_enter_threshold);
            TEST_EXPECT(estimator.impact_gyro_checkpoint_valid);
            TEST_EXPECT(estimator.impact_gyro_disagreement_streak ==
                        (uint16_t)(sample - 12U));
            TEST_EXPECT(!output.output_valid);
        }
    }

    float lane_quaternion_after[4];
    TEST_EXPECT(output.selector.residual_valid);
    TEST_EXPECT(output.selector.residual_nis <
                estimator.selector.config.nis_clear_threshold);
    TEST_EXPECT(!estimator.impact_gyro_checkpoint_valid);
    TEST_EXPECT(estimator.impact_gyro_disagreement_streak == 0U);
    TEST_EXPECT(!estimator.impact_gyro_disagreement_confirmed);
    TEST_EXPECT(!output.heading_continuity_lost);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(attitude_mekf_get_quaternion(
        &estimator.mekf[IMU_SELECTOR_LANE_1], lane_quaternion_after));
    TEST_EXPECT(quaternion_distance(trusted_lane_quaternion,
                                    lane_quaternion_after) < 1.0e-6f);
    TEST_EXPECT(quaternion_distance(trusted_output_quaternion,
                                    output.quaternion) < 1.0e-6f);
}

static void test_impact_gyro_three_windows_roll_back_to_first_suspect(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float disputed_rate[3] = {0.0f, 0.0f, 0.5f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    use_sensitive_disagreement_gate(&estimator);

    float trusted_lane_quaternion[DUAL_IMU_ESTIMATOR_LANE_COUNT][4];
    float trusted_output_quaternion[4];
    float trusted_output_alignment[4];
    float trusted_covariance[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(attitude_mekf_get_quaternion(
            &estimator.mekf[lane], trusted_lane_quaternion[lane]));
        trusted_covariance[lane] = estimator.mekf[lane].covariance[0][0];
    }
    memcpy(trusted_output_quaternion, estimator.output_quaternion,
           sizeof(trusted_output_quaternion));
    memcpy(trusted_output_alignment, estimator.output_alignment,
           sizeof(trusted_output_alignment));
    const imu_selector_lane_t trusted_previous_lane =
        estimator.previous_selected_lane;
    const uint64_t suspect_start_us =
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator, suspect_start_us,
        TEST_EPOCH_US + (UINT64_C(20) * TEST_PERIOD_US)));

    for (uint32_t sample = 13U; sample <= 15U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, disputed_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(output.selector.residual_valid);
        TEST_EXPECT(output.selector.residual_nis >
                    estimator.selector.config.nis_enter_threshold);
        TEST_EXPECT(!output.output_valid);
        if (sample < 15U) {
            TEST_EXPECT(!output.heading_continuity_lost);
            TEST_EXPECT(!output.post_impact_reacquire_active);
            TEST_EXPECT(!estimator.impact_gyro_disagreement_confirmed);
        }
    }

    TEST_EXPECT(estimator.impact_gyro_disagreement_confirmed);
    TEST_EXPECT(estimator.impact_gyro_disagreement_streak == 3U);
    TEST_EXPECT(estimator.impact_gyro_suspect_start_us == suspect_start_us);
    TEST_EXPECT(output.rotation_unobserved);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(output.post_impact_reacquire_active);
    TEST_EXPECT(estimator.heading_continuity_lost_timestamp_us ==
                suspect_start_us);
    TEST_EXPECT(estimator.previous_selected_lane == trusted_previous_lane);
    TEST_EXPECT(quaternion_distance(trusted_output_quaternion,
                                    estimator.output_quaternion) < 1.0e-6f);
    for (size_t axis = 0U; axis < 4U; ++axis) {
        TEST_EXPECT(fabsf(estimator.output_alignment[axis] -
                          trusted_output_alignment[axis]) < 1.0e-7f);
    }
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        float lane_quaternion_after[4];
        TEST_EXPECT(attitude_mekf_get_quaternion(
            &estimator.mekf[lane], lane_quaternion_after));
        TEST_EXPECT(quaternion_distance(trusted_lane_quaternion[lane],
                                        lane_quaternion_after) < 1.0e-6f);
        TEST_EXPECT(estimator.mekf[lane].covariance[0][0] >
                    trusted_covariance[lane]);
    }
}

static void test_impact_gyro_confirmation_requires_contiguous_evidence(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float disputed_rate[3] = {0.0f, 0.0f, 0.5f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    use_sensitive_disagreement_gate(&estimator);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator,
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US),
        TEST_EPOCH_US + (UINT64_C(24) * TEST_PERIOD_US)));

    uint64_t timestamp = TEST_EPOCH_US + (UINT64_C(13) * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, 13U, zero);
    push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                     timestamp, 13U, disputed_rate);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(estimator.impact_gyro_disagreement_streak == 1U);
    TEST_EXPECT(!output.output_valid);

    timestamp = TEST_EPOCH_US + (UINT64_C(14) * TEST_PERIOD_US);
    push_lane_sample(&estimator, IMU_SOURCE_BMI088, timestamp, 14U, zero);
    push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                              timestamp, 14U, disputed_rate, false, true);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!output.selector.residual_valid);
    TEST_EXPECT(!estimator.impact_gyro_checkpoint_valid);
    TEST_EXPECT(estimator.impact_gyro_disagreement_streak == 0U);

    for (uint32_t sample = 15U; sample <= 18U; ++sample) {
        timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
        const float *const icm_rate = (sample <= 16U) ? disputed_rate : zero;
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, icm_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
    }
    TEST_EXPECT(!estimator.impact_gyro_checkpoint_valid);
    TEST_EXPECT(!estimator.impact_gyro_disagreement_confirmed);
    TEST_EXPECT(!output.heading_continuity_lost);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(output.output_valid);
}

static void test_impact_gyro_confirmation_does_not_cross_inhibit_gaps(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float disputed_rate[3] = {0.0f, 0.0f, 0.5f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    use_sensitive_disagreement_gate(&estimator);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator,
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US),
        TEST_EPOCH_US + (UINT64_C(13) * TEST_PERIOD_US)));
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator,
        TEST_EPOCH_US + (UINT64_C(14) * TEST_PERIOD_US),
        TEST_EPOCH_US + (UINT64_C(15) * TEST_PERIOD_US)));

    for (uint32_t sample = 13U; sample <= 16U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const float *const icm_rate = (sample <= 15U) ? disputed_rate : zero;
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, icm_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        if (sample == 13U || sample == 15U) {
            TEST_EXPECT(output.accel_inhibited);
            TEST_EXPECT(estimator.impact_gyro_disagreement_streak == 1U);
            TEST_EXPECT(!output.output_valid);
        }
        if (sample == 14U)
            TEST_EXPECT(!estimator.impact_gyro_checkpoint_valid);
    }

    TEST_EXPECT(!estimator.impact_gyro_checkpoint_valid);
    TEST_EXPECT(!estimator.impact_gyro_disagreement_confirmed);
    TEST_EXPECT(!output.heading_continuity_lost);
    TEST_EXPECT(!output.post_impact_reacquire_active);
}

static void test_inhibited_known_bad_gyro_does_not_poison_healthy_lane(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float bad_rate[3] = {0.0f, 0.0f, 1.0f};
    const uint32_t sensor_fault = UINT32_C(1) << 6;

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.output_valid);
    use_sensitive_disagreement_gate(&estimator);
    dual_imu_estimator_set_hard_faults(
        &estimator, IMU_SOURCE_ICM45686, sensor_fault);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator,
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US),
        TEST_EPOCH_US + (UINT64_C(24) * TEST_PERIOD_US)));

    for (uint32_t sample = 13U; sample <= 22U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, bad_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(output.accel_inhibited);
        TEST_EXPECT(output.selector.residual_valid);
        TEST_EXPECT(output.selector.hard_fault_mask ==
                    IMU_SELECTOR_LANE_MASK(IMU_SELECTOR_LANE_1));
        TEST_EXPECT((output.selector.isolated_mask &
                     IMU_SELECTOR_LANE_MASK(IMU_SELECTOR_LANE_1)) != 0U);
        TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_0);
        TEST_EXPECT(output.output_valid);
        TEST_EXPECT(!output.rotation_unobserved);
        TEST_EXPECT(!output.heading_continuity_lost);
        TEST_EXPECT(!output.post_impact_reacquire_active);
        TEST_EXPECT(!estimator.impact_gyro_checkpoint_valid);
    }

    TEST_EXPECT((estimator.selector.hard_latched_mask &
                 IMU_SELECTOR_LANE_MASK(IMU_SELECTOR_LANE_1)) != 0U);
    TEST_EXPECT(estimator.impact_gyro_disagreement_streak == 0U);
}

static void test_post_impact_dynamic_quality_reacquires_without_stationary(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float yaw_rate[3] = {0.0f, 0.0f, 1.0f};
    const float level[3] = {0.0f, 0.0f, -9.80665f};
    const float disturbed[3] = {0.0f, 0.0f, -30.0f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.attitude_converged);

    const float initial_roll_error_rad = degrees_to_radians(3.0f);
    float erroneous_quaternion[4];
    quaternion_from_roll_yaw(initial_roll_error_rad, output.euler_rad[2],
                             erroneous_quaternion);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(attitude_mekf_reset(&estimator.mekf[lane],
                                        erroneous_quaternion, zero));
    }
    memcpy(estimator.output_quaternion, erroneous_quaternion,
           sizeof(estimator.output_quaternion));
    estimator.output_alignment[0] = 1.0f;
    estimator.output_alignment[1] = 0.0f;
    estimator.output_alignment[2] = 0.0f;
    estimator.output_alignment[3] = 0.0f;

    const uint64_t impact_start_us =
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US);
    const uint64_t impact_end_us =
        TEST_EPOCH_US + (UINT64_C(16) * TEST_PERIOD_US);
    TEST_EXPECT(dual_imu_estimator_notify_impact_interval(
        &estimator, impact_start_us, impact_end_us));
    for (uint32_t sample = 13U; sample <= 16U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, yaw_rate, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, yaw_rate, level,
                                        true, true);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(output.accel_inhibited);
        TEST_EXPECT(output.post_impact_reacquire_active);
        TEST_EXPECT(!output.attitude_reacquired);
    }
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(estimator.heading_continuity_lost_timestamp_us ==
                impact_start_us);

    float before_recovery_euler[3];
    TEST_EXPECT(attitude_mekf_get_euler(
        &estimator.mekf[IMU_SELECTOR_LANE_0], before_recovery_euler));
    const float before_recovery_roll_error = fabsf(before_recovery_euler[0]);
    TEST_EXPECT(before_recovery_roll_error > degrees_to_radians(2.0f));

    bool inject_disturbance_next = false;
    bool disturbance_done = false;
    bool saw_reacquire = false;
    uint32_t accepted_before_disturbance = 0U;
    uint32_t accepted_after_disturbance = 0U;
    for (uint32_t sample = 17U; sample <= 80U && !saw_reacquire; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const bool injecting_disturbance = inject_disturbance_next;
        const float *const accel = injecting_disturbance ? disturbed : level;
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, yaw_rate, accel,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, yaw_rate, accel,
                                        true, true);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(!output.accel_inhibited);
        TEST_EXPECT(!output.stationary_candidate);
        TEST_EXPECT(!output.stationary_confirmed);

        const bool both_accel_updates_accepted =
            (output.accel_result[0] == ATTITUDE_MEKF_ACCEL_ACCEPTED) &&
            (output.accel_result[1] == ATTITUDE_MEKF_ACCEL_ACCEPTED);
        if (injecting_disturbance) {
            TEST_EXPECT(!both_accel_updates_accepted);
            TEST_EXPECT(estimator.attitude_convergence_streak == 0U);
            TEST_EXPECT(output.post_impact_reacquire_active);
            disturbance_done = true;
            inject_disturbance_next = false;
        } else if (!disturbance_done && both_accel_updates_accepted) {
            accepted_before_disturbance++;
            if (accepted_before_disturbance == 2U)
                inject_disturbance_next = true;
        } else if (disturbance_done && both_accel_updates_accepted) {
            accepted_after_disturbance++;
        }

        if (output.attitude_reacquired) {
            saw_reacquire = true;
            TEST_EXPECT(disturbance_done);
            TEST_EXPECT(accepted_after_disturbance ==
                        estimator.config.attitude_convergence_windows);
        }
    }

    float after_recovery_euler[3];
    TEST_EXPECT(attitude_mekf_get_euler(
        &estimator.mekf[IMU_SELECTOR_LANE_0], after_recovery_euler));
    TEST_EXPECT(accepted_before_disturbance == 2U);
    TEST_EXPECT(disturbance_done);
    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(estimator.heading_continuity_lost_timestamp_us ==
                impact_start_us);
    TEST_EXPECT(fabsf(after_recovery_euler[0]) <
                before_recovery_roll_error);
    TEST_EXPECT(estimator.accel_bad_streak[0] == 0U);
    TEST_EXPECT(estimator.accel_bad_streak[1] == 0U);
}

static void test_accel_only_inhibit_keeps_matching_gyro_attitude_observable(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float yaw_rate[3] = {0.0f, 0.0f, 1.0f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(output.lane_seeded[0]);
    TEST_EXPECT(output.lane_seeded[1]);
    const float before[4] = {
        output.quaternion[0], output.quaternion[1],
        output.quaternion[2], output.quaternion[3],
    };

    const uint64_t inhibit_start_us =
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US);
    const uint64_t inhibit_end_us =
        TEST_EPOCH_US + (UINT64_C(17) * TEST_PERIOD_US);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator, inhibit_start_us, inhibit_end_us));
    for (uint32_t sample = 13U; sample <= 17U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, yaw_rate);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, yaw_rate);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(output.accel_inhibited);
        TEST_EXPECT(output.output_valid);
        TEST_EXPECT(!output.rotation_unobserved);
        TEST_EXPECT(!output.heading_continuity_lost);
        TEST_EXPECT(!output.post_impact_reacquire_active);
        TEST_EXPECT(!output.attitude_reacquired);
    }

    TEST_EXPECT(quaternion_distance(before, output.quaternion) > 0.005f);
    TEST_EXPECT(estimator.post_impact_episode_count == 0U);
    TEST_EXPECT(estimator.post_impact_reacquire_count == 0U);
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
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};
    const float invalid_norm[3] = {0.0f, 0.0f, -20.0f};
    estimator.config.accel_fault_enter_windows = 2U;

    for (uint32_t sample = 0U; sample <= 4U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_measurements_validity(
            &estimator, IMU_SOURCE_ICM45686, timestamp, sample, zero,
            (sample < 2U) ? gravity : invalid_norm, true, true);
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

static void test_faulted_lane_does_not_accuse_its_accelerometer(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float invalid_norm[3] = {0.0f, 0.0f, -20.0f};
    estimator.config.accel_fault_enter_windows = 2U;

    for (uint32_t sample = 0U; sample <= 1U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_ICM45686, 1U);
    for (uint32_t sample = 2U; sample <= 5U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_measurements_validity(
            &estimator, IMU_SOURCE_ICM45686, timestamp, sample, zero,
            invalid_norm, true, true);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
    }

    TEST_EXPECT(!estimator.lane_accel_fault[IMU_SELECTOR_LANE_1]);
    TEST_EXPECT(output.accel_result[IMU_SELECTOR_LANE_0] ==
                ATTITUDE_MEKF_ACCEL_ACCEPTED);
    TEST_EXPECT(output.accel_result[IMU_SELECTOR_LANE_1] ==
                ATTITUDE_MEKF_ACCEL_REJECTED_NORM);
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

static void run_post_impact_reacquire_case(float roll_degrees)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float yaw_rate[3] = {0.0f, 0.0f, 1.0f};
    const float level[3] = {0.0f, 0.0f, -9.80665f};
    const float roll_rad = degrees_to_radians(roll_degrees);
    const float tilted_gravity[3] = {
        0.0f,
        -9.80665f * sinf(roll_rad),
        -9.80665f * cosf(roll_rad),
    };

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(!output.heading_continuity_lost);

    for (uint32_t sample = 13U; sample <= 20U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, yaw_rate, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, yaw_rate, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    const float yaw_before_impact = output.euler_rad[2];
    const float covariance_before = estimator.mekf[0].covariance[0][0];

    const uint64_t inhibit_until = TEST_EPOCH_US +
                                   (UINT64_C(23) * TEST_PERIOD_US);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator,
        TEST_EPOCH_US + (UINT64_C(20) * TEST_PERIOD_US),
        inhibit_until));
    bool saw_unobserved_rotation = false;
    for (uint32_t sample = 21U; sample <= 23U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero,
                                        tilted_gravity, false, false);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero,
                                        tilted_gravity, false, false);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_unobserved_rotation = saw_unobserved_rotation ||
                                  output.rotation_unobserved;
    }
    TEST_EXPECT(saw_unobserved_rotation);
    TEST_EXPECT(estimator.mekf[0].covariance[0][0] > covariance_before);
    TEST_EXPECT(estimator.post_impact_reacquire_active);
    TEST_EXPECT(!estimator.attitude_converged);

    /* The reacquire rewrites the MEKF attitude; the published output must
     * bridge the jump through output_alignment and slew it out instead of
     * stepping. Run enough quiet windows for the worst-case 90 deg tilt to
     * finish slewing at output_alignment_slew_rad_s, and assert per-window
     * output continuity the whole way. */
    bool saw_reacquire = false;
    bool calibration_cleared_at_reacquire = false;
    bool have_previous_output = false;
    float previous_output_quaternion[4];
    float max_output_step_rad = 0.0f;
    for (uint32_t sample = 24U; sample <= 999U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero,
                                        tilted_gravity, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero,
                                        tilted_gravity, true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        if (output.attitude_reacquired && !saw_reacquire) {
            saw_reacquire = true;
            calibration_cleared_at_reacquire =
                !output.lane_calibrated[0] && !output.lane_calibrated[1];
        }
        if (output.output_valid) {
            if (have_previous_output) {
                max_output_step_rad = fmaxf(
                    max_output_step_rad,
                    quaternion_distance(output.quaternion,
                                        previous_output_quaternion));
            }
            memcpy(previous_output_quaternion, output.quaternion,
                   sizeof(previous_output_quaternion));
            have_previous_output = true;
        }
    }

    float expected[4];
    quaternion_from_roll_yaw(roll_rad, yaw_before_impact, expected);
    TEST_EXPECT(saw_reacquire);
    /* No teleports: the body is stationary, so the published attitude may
     * move per window at most by the alignment slew plus small filter
     * corrections, never by the reseeded tilt in one step. */
    TEST_EXPECT(max_output_step_rad < 0.02f);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(output.heading_continuity_lost);
    /* The reacquire could not preserve the pre-impact bias, so calibration
     * must be cleared at that moment; the long stationary settle afterwards
     * may legitimately re-earn it. */
    TEST_EXPECT(calibration_cleared_at_reacquire);
    TEST_EXPECT(quaternion_distance(output.quaternion, expected) < 0.01f);
    TEST_EXPECT(fabsf(output.euler_rad[2] - yaw_before_impact) < 0.005f);
}

static void test_post_impact_reacquires_large_tilt(void)
{
    run_post_impact_reacquire_case(15.0f);
    run_post_impact_reacquire_case(45.0f);
    run_post_impact_reacquire_case(90.0f);
}

static void test_single_healthy_lane_can_confirm_stationary_and_zaru(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    dual_imu_estimator_set_hard_faults(&estimator, IMU_SOURCE_BMI088, 1U);
    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    TEST_EXPECT(output.stationary_candidate);
    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(output.stationary_single_lane);
    TEST_EXPECT(output.stationary_lane_mask ==
                IMU_SELECTOR_LANE_MASK(IMU_SELECTOR_LANE_1));
    TEST_EXPECT(!output.lane_calibrated[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT(output.lane_calibrated[IMU_SELECTOR_LANE_1]);
    TEST_EXPECT(estimator.mekf[IMU_SELECTOR_LANE_1]
                    .diagnostics.zaru_accept_count > 0U);
}

static void test_rotation_unobserved_preserves_learned_bias_state(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.lane_calibrated[0]);
    TEST_EXPECT(output.lane_calibrated[1]);

    for (uint32_t sample = 81U; sample <= 83U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, gravity,
                                        false, false);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, gravity,
                                        false, false);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(output.post_impact_reacquire_active);
    TEST_EXPECT(output.lane_calibrated[0]);
    TEST_EXPECT(output.lane_calibrated[1]);

    bool saw_reacquire = false;
    for (uint32_t sample = 84U; sample <= 130U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_reacquire = saw_reacquire || output.attitude_reacquired;
    }
    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(output.lane_calibrated[0]);
    TEST_EXPECT(output.lane_calibrated[1]);
}

static void test_common_impact_reacquires_tilt_and_marks_heading_uncertain(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float level[3] = {0.0f, 0.0f, -9.80665f};
    const float roll_rad = degrees_to_radians(30.0f);
    const float observed_rate[3] = {
        roll_rad / (3.0f * (float)TEST_PERIOD_US * 1.0e-6f), 0.0f, 0.0f,
    };
    const float tilted_gravity[3] = {
        0.0f,
        -9.80665f * sinf(roll_rad),
        -9.80665f * cosf(roll_rad),
    };

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    TEST_EXPECT(dual_imu_estimator_notify_impact_interval(
        &estimator,
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US),
        TEST_EPOCH_US + (UINT64_C(15) * TEST_PERIOD_US)));
    for (uint32_t sample = 13U; sample <= 15U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, observed_rate,
                                        tilted_gravity, true, false);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, observed_rate,
                                        tilted_gravity, true, false);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        TEST_EXPECT(!output.rotation_unobserved);
    }

    bool saw_reacquire = false;
    for (uint32_t sample = 16U; sample <= 55U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero,
                                        tilted_gravity, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero,
                                        tilted_gravity, true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_reacquire = saw_reacquire || output.attitude_reacquired;
    }
    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(estimator.post_impact_episode_count == 1U);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(fabsf(output.euler_rad[0] - roll_rad) < 0.01f);
}

static void test_common_mode_shock_error_reacquires_level_tilt(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float level[3] = {0.0f, 0.0f, -9.80665f};
    const float false_roll_rate[3] = {
        degrees_to_radians(30.0f) /
            (3.0f * (float)TEST_PERIOD_US * 1.0e-6f),
        0.0f,
        0.0f,
    };

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    TEST_EXPECT(dual_imu_estimator_notify_impact_interval(
        &estimator,
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US),
        TEST_EPOCH_US + (UINT64_C(15) * TEST_PERIOD_US)));
    for (uint32_t sample = 13U; sample <= 15U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, false_roll_rate,
                                        level, true, false);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, false_roll_rate,
                                        level, true, false);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.post_impact_reacquire_active);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(fabsf(output.euler_rad[0]) > degrees_to_radians(10.0f));

    /* Extended past the reacquire so the discontinuity absorbed into
     * output_alignment finishes slewing out of the published attitude. */
    bool saw_reacquire = false;
    for (uint32_t sample = 16U; sample <= 500U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_reacquire = saw_reacquire || output.attitude_reacquired;
    }

    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(fabsf(output.euler_rad[0]) < degrees_to_radians(0.5f));
    TEST_EXPECT(fabsf(output.euler_rad[1]) < degrees_to_radians(0.5f));
}

/* Runs one quiet level window and returns the processed output flags. */
static void run_level_window(dual_imu_estimator_t *estimator,
                             dual_imu_estimator_output_t *output,
                             uint32_t sample)
{
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float level[3] = {0.0f, 0.0f, -9.80665f};
    const uint64_t timestamp = TEST_EPOCH_US +
                               ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_measurements_validity(estimator, IMU_SOURCE_BMI088,
                                    timestamp, sample, zero, level,
                                    true, true);
    push_lane_measurements_validity(estimator, IMU_SOURCE_ICM45686,
                                    timestamp, sample, zero, level,
                                    true, true);
    (void)dual_imu_estimator_process_next(estimator, timestamp, output);
}

/* Injects a common-mode false rotation with NO impact notification and both
 * accels staying valid: no clipping, no impact interval, no disagreement.
 * This is the undetected-shock case that used to deadlock the NIS gate. */
static void inject_undetected_shock(dual_imu_estimator_t *estimator,
                                    dual_imu_estimator_output_t *output,
                                    float error_degrees)
{
    const float level[3] = {0.0f, 0.0f, -9.80665f};
    const float false_roll_rate[3] = {
        degrees_to_radians(error_degrees) /
            (3.0f * (float)TEST_PERIOD_US * 1.0e-6f),
        0.0f,
        0.0f,
    };

    memset(output, 0, sizeof(*output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample)
        run_level_window(estimator, output, sample);
    TEST_EXPECT(output->attitude_converged);

    for (uint32_t sample = 13U; sample <= 15U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, false_roll_rate,
                                        level, true, true);
        push_lane_measurements_validity(estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, false_roll_rate,
                                        level, true, true);
        (void)dual_imu_estimator_process_next(estimator, timestamp, output);
    }
    TEST_EXPECT(!output->post_impact_reacquire_active);
    TEST_EXPECT(fabsf(output->euler_rad[0]) >
                degrees_to_radians(0.6f * error_degrees));
}

static void test_undetected_shock_recovers_through_nis_inflation(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    estimator.config.accel_recovery_stuck_windows = 20U;
    estimator.config.accel_recovery_reseed_windows = 400U;
    estimator.config.accel_recovery_inflation_std_rad =
        degrees_to_radians(1.5f);
    dual_imu_estimator_output_t output;
    /* 15 deg stays inside max_attitude_correction_rad (20 deg), so covariance
     * inflation alone must recover without the reseed escalation. */
    inject_undetected_shock(&estimator, &output, 15.0f);

    /* The gate must still be stuck once the streak threshold is reached:
     * inflation has to earn reconvergence, not a silent early accept. */
    uint32_t sample = 16U;
    for (; sample <= 36U; ++sample)
        run_level_window(&estimator, &output, sample);
    TEST_EXPECT(fabsf(output.euler_rad[0]) > degrees_to_radians(8.0f));

    bool saw_inflating = false;
    bool saw_reseed = false;
    for (; sample <= 400U; ++sample) {
        run_level_window(&estimator, &output, sample);
        saw_inflating = saw_inflating ||
                        output.lane_accel_recovery_inflating[0] ||
                        output.lane_accel_recovery_inflating[1];
        saw_reseed = saw_reseed ||
                     output.lane_accel_recovery_reseeded[0] ||
                     output.lane_accel_recovery_reseeded[1];
    }

    TEST_EXPECT(saw_inflating);
    TEST_EXPECT(!saw_reseed);
    TEST_EXPECT(fabsf(output.euler_rad[0]) < degrees_to_radians(1.0f));
    TEST_EXPECT(fabsf(output.euler_rad[1]) < degrees_to_radians(1.0f));
    /* Rotation stayed observed throughout: inflation must not scramble or
     * flag the heading gauge. */
    TEST_EXPECT(fabsf(output.euler_rad[2]) < degrees_to_radians(5.0f));
    TEST_EXPECT(!output.heading_continuity_lost);
}

static void test_undetected_shock_reseeds_after_inflation_timeout(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    estimator.config.accel_recovery_stuck_windows = 8U;
    estimator.config.accel_recovery_reseed_windows = 24U;
    /* Deliberately too small to reopen the gate before the reseed count. */
    estimator.config.accel_recovery_inflation_std_rad =
        degrees_to_radians(0.02f);
    dual_imu_estimator_output_t output;
    /* 30 deg exceeds max_attitude_correction_rad, so even a reopened NIS gate
     * rejects the correction and only the reseed escalation can recover. */
    inject_undetected_shock(&estimator, &output, 30.0f);

    /* Extended past the reseed escalation so the absorbed 30 deg output
     * discontinuity finishes slewing out at output_alignment_slew_rad_s. */
    bool saw_reseed = false;
    bool saw_reacquire = false;
    for (uint32_t sample = 16U; sample <= 600U; ++sample) {
        run_level_window(&estimator, &output, sample);
        saw_reseed = saw_reseed ||
                     output.lane_accel_recovery_reseeded[0] ||
                     output.lane_accel_recovery_reseeded[1];
        saw_reacquire = saw_reacquire || output.attitude_reacquired;
    }

    TEST_EXPECT(saw_reseed);
    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(fabsf(output.euler_rad[0]) < degrees_to_radians(1.0f));
    TEST_EXPECT(fabsf(output.euler_rad[1]) < degrees_to_radians(1.0f));
    TEST_EXPECT(fabsf(output.euler_rad[2]) < degrees_to_radians(5.0f));
}

static void test_sustained_rotation_contamination_never_forced_into_tilt(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    estimator.config.accel_recovery_stuck_windows = 8U;
    estimator.config.accel_recovery_reseed_windows = 24U;
    dual_imu_estimator_output_t output;
    /* Fast yaw with 3 m/s2 of centripetal contamination: the magnitude stays
     * within the norm tolerance (sqrt(g^2+3^2)-g ~= 0.45 m/s2) but the
     * direction is ~17 deg off gravity, so the NIS gate rejects every window.
     * Those rejections are correct dynamics handling, not a stuck gate; the
     * recovery machinery must hold instead of inflating the covariance open
     * or reseeding tilt toward the contaminated direction. */
    const float yaw_rate[3] = {0.0f, 0.0f, degrees_to_radians(200.0f)};
    const float contaminated[3] = {3.0f, 0.0f, -9.80665f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample)
        run_level_window(&estimator, &output, sample);
    TEST_EXPECT(output.attitude_converged);

    bool saw_inflating = false;
    bool saw_reseed = false;
    for (uint32_t sample = 13U; sample <= 400U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, yaw_rate,
                                        contaminated, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, yaw_rate,
                                        contaminated, true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_inflating = saw_inflating ||
                        output.lane_accel_recovery_inflating[0] ||
                        output.lane_accel_recovery_inflating[1];
        saw_reseed = saw_reseed ||
                     output.lane_accel_recovery_reseeded[0] ||
                     output.lane_accel_recovery_reseeded[1];
    }

    TEST_EXPECT(!saw_inflating);
    TEST_EXPECT(!saw_reseed);
    TEST_EXPECT(estimator.accel_recovery_inflation_count == 0U);
    TEST_EXPECT(estimator.accel_recovery_reseed_count == 0U);
    /* Tilt must remain gyro-propagated and clean through the whole burst. */
    TEST_EXPECT(fabsf(output.euler_rad[0]) < degrees_to_radians(1.0f));
    TEST_EXPECT(fabsf(output.euler_rad[1]) < degrees_to_radians(1.0f));
}

static void test_post_rollback_dynamic_accel_waits_for_gravity_trust(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float disputed_rate[3] = {0.0f, 0.0f, 0.5f};
    const float yaw_rate[3] = {0.0f, 0.0f, degrees_to_radians(200.0f)};
    const float level[3] = {0.0f, 0.0f, -9.80665f};
    const float contaminated[3] = {3.0f, 0.0f, -9.80665f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample)
        run_level_window(&estimator, &output, sample);
    TEST_EXPECT(output.attitude_converged);
    use_sensitive_disagreement_gate(&estimator);

    const uint64_t inhibit_start_us =
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US);
    const uint64_t inhibit_end_us =
        TEST_EPOCH_US + (UINT64_C(18) * TEST_PERIOD_US);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator, inhibit_start_us, inhibit_end_us));

    for (uint32_t sample = 13U; sample <= 15U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, disputed_rate, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.rotation_unobserved);
    TEST_EXPECT(output.post_impact_reacquire_active);
    TEST_EXPECT(!output.output_valid);

    for (uint32_t sample = 16U; sample <= 18U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, yaw_rate,
                                        contaminated, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, yaw_rate,
                                        contaminated, true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        TEST_EXPECT(output.accel_inhibited);
    }

    float previous_output[4];
    memcpy(previous_output, output.quaternion, sizeof(previous_output));
    float maximum_output_step = 0.0f;
    float maximum_tilt = 0.0f;
    for (uint32_t sample = 19U; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, yaw_rate,
                                        contaminated, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, yaw_rate,
                                        contaminated, true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        TEST_EXPECT(!output.accel_inhibited);
        TEST_EXPECT(output.gravity_aiding_inhibited);
        TEST_EXPECT(!output.post_impact_gravity_trusted);
        TEST_EXPECT(output.post_impact_gravity_trust_streak == 0U);
        TEST_EXPECT(output.accel_result[0] ==
                    ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT);
        TEST_EXPECT(output.accel_result[1] ==
                    ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT);
        if (output.output_valid) {
            maximum_output_step = fmaxf(
                maximum_output_step,
                quaternion_distance(previous_output, output.quaternion));
            memcpy(previous_output, output.quaternion,
                   sizeof(previous_output));
            maximum_tilt = fmaxf(maximum_tilt,
                                 fmaxf(fabsf(output.euler_rad[0]),
                                       fabsf(output.euler_rad[1])));
        }
    }
    TEST_EXPECT(maximum_output_step < degrees_to_radians(1.0f));
    TEST_EXPECT(maximum_tilt < degrees_to_radians(1.0f));
    TEST_EXPECT(output.post_impact_reacquire_active);

    bool saw_gravity_trusted = false;
    bool saw_reacquire = false;
    for (uint32_t sample = 81U; sample <= 120U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_gravity_trusted = saw_gravity_trusted ||
                              output.post_impact_gravity_trusted;
        saw_reacquire = saw_reacquire || output.attitude_reacquired;
    }
    TEST_EXPECT(saw_gravity_trusted);
    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(fabsf(output.euler_rad[0]) < degrees_to_radians(1.0f));
    TEST_EXPECT(fabsf(output.euler_rad[1]) < degrees_to_radians(1.0f));
}

static uint32_t induce_dual_filter_fault_during_impact(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output)
{
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(estimator, timestamp, output);
    }
    TEST_EXPECT(output->output_valid);
    TEST_EXPECT(output->attitude_converged);

    TEST_EXPECT(dual_imu_estimator_notify_impact_interval(
        estimator,
        TEST_EPOCH_US + (UINT64_C(12) * TEST_PERIOD_US),
        TEST_EPOCH_US + (UINT64_C(15) * TEST_PERIOD_US)));
    uint32_t sample = 13U;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample(estimator, IMU_SOURCE_BMI088,
                     timestamp, sample, zero);
    push_lane_sample(estimator, IMU_SOURCE_ICM45686,
                     timestamp, sample, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(estimator, timestamp, output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(output->accel_inhibited);
    TEST_EXPECT(output->post_impact_reacquire_active);
    TEST_EXPECT(output->heading_continuity_lost);

    estimator->mekf[IMU_SELECTOR_LANE_0].q[0] = NAN;
    estimator->mekf[IMU_SELECTOR_LANE_1].q[0] = NAN;
    for (sample = 14U; sample <= 15U; ++sample) {
        timestamp = TEST_EPOCH_US +
                    ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        estimator, timestamp, output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        TEST_EXPECT(output->accel_inhibited);
        TEST_EXPECT(!output->attitude_reacquired);
    }

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(!estimator->lane_seeded[lane]);
        TEST_EXPECT((estimator->hard_fault_flags[lane] &
                     (UINT32_C(1) << 31)) != 0U);
        TEST_EXPECT(estimator->filter_fault_window_end_us[lane] ==
                    TEST_EPOCH_US +
                        (UINT64_C(14) * TEST_PERIOD_US));
    }
    TEST_EXPECT(!output->output_valid);
    return sample - 1U;
}

static void test_post_impact_dual_filter_fault_reseeds_from_new_windows(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    memset(&output, 0, sizeof(output));
    uint32_t sample = induce_dual_filter_fault_during_impact(
        &estimator, &output);
    const uint64_t heading_loss_timestamp_us =
        estimator.heading_continuity_lost_timestamp_us;
    estimator.lane_calibrated[IMU_SELECTOR_LANE_0] = true;
    estimator.lane_calibrated[IMU_SELECTOR_LANE_1] = true;
    estimator.mekf[IMU_SELECTOR_LANE_0].gyro_bias_rad_s[0] = NAN;

    const float zero[3] = {0.0f, 0.0f, 0.0f};
    sample++;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample_validity(&estimator, IMU_SOURCE_BMI088,
                              timestamp, sample, zero, true, false);
    push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                              timestamp, sample, zero, true, false);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!output.attitude_reacquired);
    TEST_EXPECT(!estimator.lane_seeded[0]);
    TEST_EXPECT(!estimator.lane_seeded[1]);

    const float not_finite_gyro[3] = {NAN, 0.0f, 0.0f};
    const float recovered_roll_rad = degrees_to_radians(30.0f);
    const float recovered_gravity[3] = {
        0.0f,
        -9.80665f * sinf(recovered_roll_rad),
        -9.80665f * cosf(recovered_roll_rad),
    };
    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_measurements_validity(
        &estimator, IMU_SOURCE_BMI088, timestamp, sample,
        not_finite_gyro, recovered_gravity, true, true);
    push_lane_measurements_validity(
        &estimator, IMU_SOURCE_ICM45686, timestamp, sample,
        not_finite_gyro, recovered_gravity, true, true);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!output.attitude_reacquired);

    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_measurements_validity(
        &estimator, IMU_SOURCE_BMI088, timestamp, sample + 2U,
        zero, recovered_gravity, true, true);
    push_lane_measurements_validity(
        &estimator, IMU_SOURCE_ICM45686, timestamp, sample + 2U,
        zero, recovered_gravity, true, true);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!output.attitude_reacquired);

    /* Extended so the 30 deg discontinuity absorbed at the reacquire slews
     * fully out of the published output before the final expectations. */
    bool saw_reacquire = false;
    bool invalid_bias_calibration_cleared = false;
    bool valid_bias_calibration_preserved = false;
    for (sample++; sample <= 600U; ++sample) {
        timestamp = TEST_EPOCH_US +
                    ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(
            &estimator, IMU_SOURCE_BMI088, timestamp, sample + 2U,
            zero, recovered_gravity, true, true);
        push_lane_measurements_validity(
            &estimator, IMU_SOURCE_ICM45686, timestamp, sample + 2U,
            zero, recovered_gravity, true, true);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        if (output.attitude_reacquired) {
            saw_reacquire = true;
            invalid_bias_calibration_cleared =
                !estimator.lane_calibrated[IMU_SELECTOR_LANE_0] &&
                (estimator.zaru_accept_count[IMU_SELECTOR_LANE_0] <= 1U) &&
                (estimator.zaru_divergence_count[IMU_SELECTOR_LANE_0] == 0U);
            valid_bias_calibration_preserved =
                estimator.lane_calibrated[IMU_SELECTOR_LANE_1];
        }
    }

    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(invalid_bias_calibration_cleared);
    TEST_EXPECT(valid_bias_calibration_preserved);
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.lane_seeded[0]);
    TEST_EXPECT(output.lane_seeded[1]);
    TEST_EXPECT((estimator.hard_fault_flags[0] &
                 (UINT32_C(1) << 31)) == 0U);
    TEST_EXPECT((estimator.hard_fault_flags[1] &
                 (UINT32_C(1) << 31)) == 0U);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(estimator.heading_continuity_lost_timestamp_us ==
                heading_loss_timestamp_us);
    TEST_EXPECT(fabsf(output.euler_rad[0] - recovered_roll_rad) < 0.01f);
    TEST_EXPECT(fabsf(output.euler_rad[1]) < 0.01f);
    TEST_EXPECT(estimator.post_impact_reacquire_count == 1U);
}

static void test_post_impact_recovery_never_clears_sensor_hard_fault(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    memset(&output, 0, sizeof(output));
    uint32_t sample = induce_dual_filter_fault_during_impact(
        &estimator, &output);
    const uint32_t sensor_faults =
        UINT32_MAX & ~(UINT32_C(1) << 31);
    dual_imu_estimator_set_hard_faults(
        &estimator, IMU_SOURCE_BMI088, UINT32_MAX);

    const float zero[3] = {0.0f, 0.0f, 0.0f};
    bool saw_reacquire = false;
    for (sample++; sample <= 140U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        saw_reacquire = saw_reacquire || output.attitude_reacquired;
    }

    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(!output.lane_seeded[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT(output.lane_seeded[IMU_SELECTOR_LANE_1]);
    TEST_EXPECT((estimator.hard_fault_flags[IMU_SELECTOR_LANE_0] &
                 sensor_faults) == sensor_faults);
    TEST_EXPECT((estimator.hard_fault_flags[IMU_SELECTOR_LANE_0] &
                 (UINT32_C(1) << 31)) != 0U);
    TEST_EXPECT(estimator.hard_fault_flags[IMU_SELECTOR_LANE_1] == 0U);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(output.heading_continuity_lost);
}

static void test_attitude_aiding_timeout_is_recoverable(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.attitude_converged);

    for (uint32_t sample = 13U; sample <= 25U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, gravity,
                                        true, false);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, gravity,
                                        true, false);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(!output.attitude_converged);
    TEST_EXPECT(output.attitude_aiding_stale);
    TEST_EXPECT(!output.heading_continuity_lost);
    TEST_EXPECT(!output.post_impact_reacquire_active);

    for (uint32_t sample = 26U; sample <= 31U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(!output.attitude_aiding_stale);
    TEST_EXPECT(!output.heading_continuity_lost);
}

static void test_single_lane_stationary_does_not_zaru_untracked_lane(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float gravity[3] = {0.0f, 0.0f, -9.80665f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, gravity,
                                        true, false);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, gravity,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    TEST_EXPECT(output.stationary_confirmed);
    TEST_EXPECT(output.stationary_lane_mask ==
                IMU_SELECTOR_LANE_MASK(IMU_SELECTOR_LANE_1));
    TEST_EXPECT(estimator.mekf[IMU_SELECTOR_LANE_0]
                    .diagnostics.zaru_update_count == 0U);
    TEST_EXPECT(!output.lane_calibrated[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT(output.lane_calibrated[IMU_SELECTOR_LANE_1]);
}

static void test_lane_filter_fault_recovers_without_blocking_healthy_lane(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    const float preserved_bias[3] = {0.001f, -0.0015f, 0.002f};
    memcpy(estimator.mekf[IMU_SELECTOR_LANE_0].gyro_bias_rad_s,
           preserved_bias, sizeof(preserved_bias));
    estimator.lane_calibrated[IMU_SELECTOR_LANE_0] = true;
    estimator.mekf[IMU_SELECTOR_LANE_0].q[0] = NAN;
    bool saw_invalid_output = false;
    bool checked_recovered_bias = false;
    for (uint32_t sample = 13U; sample <= 70U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_invalid_output = saw_invalid_output || !output.output_valid;
        if (!checked_recovered_bias &&
            attitude_mekf_is_valid(
                &estimator.mekf[IMU_SELECTOR_LANE_0])) {
            for (size_t axis = 0U; axis < 3U; ++axis) {
                TEST_EXPECT(fabsf(output.lane_bias_rad_s[0][axis] -
                                  preserved_bias[axis]) < 1.0e-7f);
            }
            checked_recovered_bias = true;
        }
    }

    TEST_EXPECT(!saw_invalid_output);
    TEST_EXPECT(!output.attitude_reacquired);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(estimator.post_impact_episode_count == 0U);
    TEST_EXPECT(checked_recovered_bias);
    TEST_EXPECT(attitude_mekf_is_valid(
        &estimator.mekf[IMU_SELECTOR_LANE_0]));
    TEST_EXPECT(estimator.lane_seeded[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT((estimator.hard_fault_flags[IMU_SELECTOR_LANE_0] &
                 (UINT32_C(1) << 31)) == 0U);
    TEST_EXPECT(output.output_valid);
}

static uint32_t induce_dual_filter_fault_without_impact(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output)
{
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    memset(output, 0, sizeof(*output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        (void)dual_imu_estimator_process_next(estimator, timestamp, output);
    }
    TEST_EXPECT(output->output_valid);
    TEST_EXPECT(output->attitude_converged);

    estimator->mekf[IMU_SELECTOR_LANE_0].q[0] = NAN;
    estimator->mekf[IMU_SELECTOR_LANE_1].q[0] = NAN;
    const uint32_t fault_sample = 13U;
    const uint64_t fault_timestamp =
        TEST_EPOCH_US + ((uint64_t)fault_sample * TEST_PERIOD_US);
    push_lane_sample(estimator, IMU_SOURCE_BMI088,
                     fault_timestamp, fault_sample, zero);
    push_lane_sample(estimator, IMU_SOURCE_ICM45686,
                     fault_timestamp, fault_sample, zero);
    TEST_EXPECT(dual_imu_estimator_process_next(
                    estimator, fault_timestamp, output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        TEST_EXPECT(!estimator->lane_seeded[lane]);
        TEST_EXPECT((estimator->hard_fault_flags[lane] &
                     (UINT32_C(1) << 31)) != 0U);
        TEST_EXPECT(estimator->filter_fault_window_end_us[lane] ==
                    fault_timestamp);
    }
    TEST_EXPECT(!output->output_valid);
    TEST_EXPECT(!output->post_impact_reacquire_active);
    TEST_EXPECT(!output->heading_continuity_lost);
    return fault_sample;
}

static void test_nonimpact_dual_filter_fault_recovers_on_new_complete_window(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    uint32_t sample = induce_dual_filter_fault_without_impact(
        &estimator, &output);
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const uint64_t filter_fault_timestamp =
        TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    bool saw_both_reseeded = false;
    bool saw_output_recover = false;
    uint64_t first_recovery_timestamp = 0U;
    TEST_EXPECT(estimator.selector.hard_latched_mask == 0U);
    for (sample++; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        if (estimator.lane_seeded[0] && estimator.lane_seeded[1] &&
            ((estimator.hard_fault_flags[0] |
              estimator.hard_fault_flags[1]) == 0U)) {
            saw_both_reseeded = true;
            if (first_recovery_timestamp == 0U)
                first_recovery_timestamp = timestamp;
        }
        saw_output_recover = saw_output_recover || output.output_valid;
        TEST_EXPECT(estimator.selector.hard_latched_mask == 0U);
    }

    TEST_EXPECT(saw_both_reseeded);
    TEST_EXPECT(saw_output_recover);
    TEST_EXPECT(first_recovery_timestamp ==
                filter_fault_timestamp + TEST_PERIOD_US);
    TEST_EXPECT((first_recovery_timestamp - filter_fault_timestamp) <=
                UINT64_C(20000));
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.lane_seeded[0]);
    TEST_EXPECT(output.lane_seeded[1]);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(!output.heading_continuity_lost);
}

static void test_nonimpact_filter_reseed_requires_complete_lane_window(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    uint32_t sample = induce_dual_filter_fault_without_impact(
        &estimator, &output);
    const float zero[3] = {0.0f, 0.0f, 0.0f};

    sample++;
    uint64_t timestamp = TEST_EPOCH_US +
                         ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample_validity(&estimator, IMU_SOURCE_BMI088,
                              timestamp, sample, zero, true, false);
    push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                              timestamp, sample, zero, true, false);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!estimator.lane_seeded[0]);
    TEST_EXPECT(!estimator.lane_seeded[1]);

    sample++;
    timestamp = TEST_EPOCH_US + ((uint64_t)sample * TEST_PERIOD_US);
    push_lane_sample_validity(&estimator, IMU_SOURCE_BMI088,
                              timestamp, sample, zero, false, true);
    push_lane_sample_validity(&estimator, IMU_SOURCE_ICM45686,
                              timestamp, sample, zero, false, true);
    TEST_EXPECT(dual_imu_estimator_process_next(&estimator, timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);
    TEST_EXPECT(!estimator.lane_seeded[0]);
    TEST_EXPECT(!estimator.lane_seeded[1]);

    bool saw_reseed = false;
    for (sample++; sample <= 30U; ++sample) {
        timestamp = TEST_EPOCH_US +
                    ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
        saw_reseed = saw_reseed ||
                     (estimator.lane_seeded[0] &&
                      estimator.lane_seeded[1]);
    }
    TEST_EXPECT(saw_reseed);
}

static void test_nonimpact_filter_reseed_preserves_real_hard_fault(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    uint32_t sample = induce_dual_filter_fault_without_impact(
        &estimator, &output);
    const uint32_t sensor_fault = UINT32_C(1) << 6;
    dual_imu_estimator_set_hard_faults(
        &estimator, IMU_SOURCE_BMI088, sensor_fault);

    const float zero[3] = {0.0f, 0.0f, 0.0f};
    for (sample++; sample <= 80U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_sample(&estimator, IMU_SOURCE_BMI088,
                         timestamp, sample, zero);
        push_lane_sample(&estimator, IMU_SOURCE_ICM45686,
                         timestamp, sample, zero);
        TEST_EXPECT(dual_imu_estimator_process_next(
                        &estimator, timestamp, &output) ==
                    IMU_PREINTEGRATOR_WINDOW_READY);
    }

    TEST_EXPECT(!estimator.lane_seeded[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT((estimator.hard_fault_flags[IMU_SELECTOR_LANE_0] &
                 sensor_fault) != 0U);
    TEST_EXPECT((estimator.hard_fault_flags[IMU_SELECTOR_LANE_0] &
                 (UINT32_C(1) << 31)) != 0U);
    TEST_EXPECT((estimator.selector.hard_latched_mask &
                 IMU_SELECTOR_LANE_MASK(IMU_SELECTOR_LANE_0)) != 0U);
    TEST_EXPECT(estimator.lane_seeded[IMU_SELECTOR_LANE_1]);
    TEST_EXPECT(estimator.hard_fault_flags[IMU_SELECTOR_LANE_1] == 0U);
    TEST_EXPECT(output.output_valid);
}

static void test_selected_icm_filter_fault_reseeds_without_attitude_jump(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float initial_roll_rad = degrees_to_radians(25.0f);
    const float roll_rate_rad_s = degrees_to_radians(5.0f);
    const float gyro[3] = {roll_rate_rad_s, 0.0f, 0.0f};

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 40U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const float elapsed_s =
            (float)sample * (float)TEST_PERIOD_US * 1.0e-6f;
        const float roll_rad = initial_roll_rad +
                               (roll_rate_rad_s * elapsed_s);
        const float accel[3] = {
            0.0f,
            -9.80665f * sinf(roll_rad),
            -9.80665f * cosf(roll_rad),
        };
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, gyro, accel,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, gyro, accel,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }

    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_EXPECT(estimator.lane_seeded[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT(estimator.lane_seeded[IMU_SELECTOR_LANE_1]);
    const float before_fault[4] = {
        output.quaternion[0], output.quaternion[1],
        output.quaternion[2], output.quaternion[3],
    };

    const uint32_t fault_sample = 41U;
    const uint64_t fault_timestamp =
        TEST_EPOCH_US + ((uint64_t)fault_sample * TEST_PERIOD_US);
    TEST_EXPECT(dual_imu_estimator_inhibit_accel_interval(
        &estimator, fault_timestamp - TEST_PERIOD_US, fault_timestamp));
    estimator.mekf[IMU_SELECTOR_LANE_1].q[0] = NAN;
    const float fault_elapsed_s =
        (float)fault_sample * (float)TEST_PERIOD_US * 1.0e-6f;
    const float fault_roll_rad = initial_roll_rad +
                                 (roll_rate_rad_s * fault_elapsed_s);
    const float fault_accel[3] = {
        0.0f,
        -9.80665f * sinf(fault_roll_rad),
        -9.80665f * cosf(fault_roll_rad),
    };
    push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                    fault_timestamp, fault_sample, gyro,
                                    fault_accel, true, true);
    push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                    fault_timestamp, fault_sample, gyro,
                                    fault_accel, true, true);
    TEST_EXPECT(dual_imu_estimator_process_next(
                    &estimator, fault_timestamp, &output) ==
                IMU_PREINTEGRATOR_WINDOW_READY);

    float expected[4];
    quaternion_from_roll_yaw(fault_roll_rad, 0.0f, expected);
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.accel_inhibited);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_EXPECT(output.selector.selection_changed);
    TEST_EXPECT(estimator.lane_seeded[IMU_SELECTOR_LANE_0]);
    TEST_EXPECT(!estimator.lane_seeded[IMU_SELECTOR_LANE_1]);
    TEST_EXPECT(attitude_mekf_is_valid(
        &estimator.mekf[IMU_SELECTOR_LANE_0]));
    TEST_EXPECT(!attitude_mekf_is_valid(
        &estimator.mekf[IMU_SELECTOR_LANE_1]));
    TEST_EXPECT((estimator.hard_fault_flags[IMU_SELECTOR_LANE_1] &
                 (UINT32_C(1) << 31)) != 0U);
    TEST_EXPECT(!output.heading_continuity_lost);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(!output.attitude_reacquired);
    TEST_EXPECT(quaternion_distance(before_fault, output.quaternion) <
                degrees_to_radians(0.5f));
    TEST_EXPECT(quaternion_distance(output.quaternion, expected) <
                degrees_to_radians(0.5f));

    float previous[4] = {
        output.quaternion[0], output.quaternion[1],
        output.quaternion[2], output.quaternion[3],
    };
    bool saw_icm_reseed = false;
    bool saw_preferred_icm_return = false;
    for (uint32_t sample = 42U; sample <= 140U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        const float elapsed_s =
            (float)sample * (float)TEST_PERIOD_US * 1.0e-6f;
        const float roll_rad = initial_roll_rad +
                               (roll_rate_rad_s * elapsed_s);
        const float accel[3] = {
            0.0f,
            -9.80665f * sinf(roll_rad),
            -9.80665f * cosf(roll_rad),
        };
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, gyro, accel,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, gyro, accel,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        TEST_EXPECT(output.output_valid);
        TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_0 ||
                    output.selector.selected_lane == IMU_SELECTOR_LANE_1);
        TEST_EXPECT(estimator.lane_seeded[IMU_SELECTOR_LANE_0]);
        TEST_EXPECT(attitude_mekf_is_valid(
            &estimator.mekf[IMU_SELECTOR_LANE_0]));
        if (estimator.lane_seeded[IMU_SELECTOR_LANE_1] &&
            attitude_mekf_is_valid(
                &estimator.mekf[IMU_SELECTOR_LANE_1]) &&
            ((estimator.hard_fault_flags[IMU_SELECTOR_LANE_1] &
              (UINT32_C(1) << 31)) == 0U)) {
            saw_icm_reseed = true;
        }
        if (output.selector.selected_lane == IMU_SELECTOR_LANE_1)
            saw_preferred_icm_return = true;
        TEST_EXPECT(quaternion_distance(previous, output.quaternion) <
                    degrees_to_radians(0.5f));
        memcpy(previous, output.quaternion, sizeof(previous));
    }

    const float final_elapsed_s =
        140.0f * (float)TEST_PERIOD_US * 1.0e-6f;
    quaternion_from_roll_yaw(initial_roll_rad +
                                 (roll_rate_rad_s * final_elapsed_s),
                             0.0f, expected);
    TEST_EXPECT(saw_icm_reseed);
    TEST_EXPECT(saw_preferred_icm_return);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_EXPECT(quaternion_distance(output.quaternion, expected) <
                degrees_to_radians(0.5f));
}

static void test_single_lane_recovers_first_after_dual_gyro_loss(void)
{
    dual_imu_estimator_t estimator = make_estimator();
    dual_imu_estimator_output_t output;
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    const float yaw_rate[3] = {0.0f, 0.0f, 1.0f};
    const float level[3] = {0.0f, 0.0f, -9.80665f};
    const float recovered_roll_rad = degrees_to_radians(45.0f);
    const float recovered_gravity[3] = {
        0.0f,
        -9.80665f * sinf(recovered_roll_rad),
        -9.80665f * cosf(recovered_roll_rad),
    };

    memset(&output, 0, sizeof(output));
    for (uint32_t sample = 0U; sample <= 12U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    for (uint32_t sample = 13U; sample <= 20U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, yaw_rate, level,
                                        true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, yaw_rate, level,
                                        true, true);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
    }
    const float yaw_before_loss = output.euler_rad[2];

    bool saw_invalid_output = false;
    for (uint32_t sample = 21U; sample <= 23U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero,
                                        recovered_gravity, false, false);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero,
                                        recovered_gravity, false, false);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);
        saw_invalid_output = saw_invalid_output || !output.output_valid;
    }
    TEST_EXPECT(saw_invalid_output);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(output.post_impact_reacquire_active);
    TEST_EXPECT(!output.attitude_converged);

    /* Extended so the 45 deg discontinuity absorbed at the reacquire slews
     * fully out of the published output before the final expectations. */
    bool saw_output_resume_while_reacquiring = false;
    bool saw_reacquire = false;
    bool saw_icm_cross_lane_aiding = false;
    for (uint32_t sample = 24U; sample <= 999U; ++sample) {
        const uint64_t timestamp = TEST_EPOCH_US +
                                   ((uint64_t)sample * TEST_PERIOD_US);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_BMI088,
                                        timestamp, sample, zero,
                                        recovered_gravity, true, true);
        push_lane_measurements_validity(&estimator, IMU_SOURCE_ICM45686,
                                        timestamp, sample, zero,
                                        recovered_gravity, false, false);
        (void)dual_imu_estimator_process_next(&estimator, timestamp, &output);

        if (output.output_valid) {
            TEST_EXPECT(output.selector.selected_lane ==
                        IMU_SELECTOR_LANE_0);
            TEST_EXPECT(output.heading_continuity_lost);
            if (output.post_impact_reacquire_active)
                saw_output_resume_while_reacquiring = true;
        }
        if (output.attitude_reacquired) {
            saw_reacquire = true;
            TEST_EXPECT(output.output_valid);
            TEST_EXPECT(output.lane_seeded[IMU_SELECTOR_LANE_0]);
            TEST_EXPECT(!output.lane_seeded[IMU_SELECTOR_LANE_1]);
        }
        if (saw_reacquire &&
            output.lane_seeded[IMU_SELECTOR_LANE_1] &&
            output.lane_aided_propagation[IMU_SELECTOR_LANE_1]) {
            saw_icm_cross_lane_aiding = true;
        }
    }

    float expected[4];
    quaternion_from_roll_yaw(recovered_roll_rad, yaw_before_loss, expected);
    TEST_EXPECT(saw_output_resume_while_reacquiring);
    TEST_EXPECT(saw_reacquire);
    TEST_EXPECT(saw_icm_cross_lane_aiding);
    TEST_EXPECT(output.output_valid);
    TEST_EXPECT(output.selector.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_EXPECT(!output.lane_window[IMU_SELECTOR_LANE_1].gyro_valid);
    TEST_EXPECT(output.heading_continuity_lost);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(quaternion_distance(output.quaternion, expected) < 0.01f);
}

int main(void)
{
    test_disagreement_confirmation_requires_multiple_windows();
    test_stationary_variance_memory_requires_two_windows();
    test_external_hard_faults_cannot_inject_filter_fault();
    test_reseed_failure_is_transactional();
    test_common_window_is_transactional();
    test_internal_stationary_zaru_and_bumpless_failover();
    test_impact_inhibits_accel_update();
    test_impact_intervals_are_causal_under_backlog();
    test_pre_statistics_stationary_reject_reasons();
    test_first_accel_direction_outlier_eventually_recovers_stationary();
    test_rotation_above_internal_limit_is_not_zaru();
    test_two_dps_yaw_rotation_is_not_zaru();
    test_physical_roll_is_not_learned_as_gyro_bias();
    test_gravity_aiding_learns_static_observable_bias_before_zaru();
    test_unobservable_static_yaw_rate_is_not_learned_as_bias();
    test_zaru_bias_recovery_reseeds_diverged_bias();
    test_zaru_bias_recovery_rejects_slow_tilt_rotation();
    test_calibration_convergence_is_revoked_after_bias_shift();
    test_external_hint_does_not_bypass_stationary_rate_mean_gate();
    test_temporal_gyro_variance_breaks_stationary_dwell();
    test_temporal_accel_variance_breaks_stationary_dwell();
    test_gravity_direction_reason_and_streak_diagnostics();
    test_motion_after_long_stationary_interval_is_detected();
    test_stationary_soft_outlier_keeps_status_and_pauses_zaru();
    test_stationary_soft_exit_and_full_reentry_dwell();
    test_stationary_dual_lane_slow_rate_exit();
    test_stationary_single_lane_slow_rate_exit();
    test_stationary_one_lane_rate_disagreement_pauses_zaru();
    test_stationary_integrity_events_exit_immediately();
    test_external_hint_only_shortens_stationary_dwell();
    test_impact_gyro_disagreement_fails_closed();
    test_impact_gyro_sample_spike_is_held_until_clear();
    test_impact_gyro_three_windows_roll_back_to_first_suspect();
    test_impact_gyro_confirmation_requires_contiguous_evidence();
    test_impact_gyro_confirmation_does_not_cross_inhibit_gaps();
    test_inhibited_known_bad_gyro_does_not_poison_healthy_lane();
    test_post_impact_dynamic_quality_reacquires_without_stationary();
    test_accel_only_inhibit_keeps_matching_gyro_attitude_observable();
    test_accel_fallback_is_explicit();
    test_dropout_lane_is_aided_and_recovers_aligned();
    test_accel_fault_does_not_switch_healthy_gyro();
    test_faulted_lane_does_not_accuse_its_accelerometer();
    test_switch_alignment_preserves_unobservable_yaw();
    test_switch_alignment_decays_only_observable_tilt();
    test_gyro_lane_survives_accel_dead_from_start();
    test_post_impact_reacquires_large_tilt();
    test_single_healthy_lane_can_confirm_stationary_and_zaru();
    test_rotation_unobserved_preserves_learned_bias_state();
    test_common_impact_reacquires_tilt_and_marks_heading_uncertain();
    test_common_mode_shock_error_reacquires_level_tilt();
    test_undetected_shock_recovers_through_nis_inflation();
    test_undetected_shock_reseeds_after_inflation_timeout();
    test_sustained_rotation_contamination_never_forced_into_tilt();
    test_post_rollback_dynamic_accel_waits_for_gravity_trust();
    test_post_impact_dual_filter_fault_reseeds_from_new_windows();
    test_post_impact_recovery_never_clears_sensor_hard_fault();
    test_attitude_aiding_timeout_is_recoverable();
    test_single_lane_stationary_does_not_zaru_untracked_lane();
    test_lane_filter_fault_recovers_without_blocking_healthy_lane();
    test_nonimpact_dual_filter_fault_recovers_on_new_complete_window();
    test_nonimpact_filter_reseed_requires_complete_lane_window();
    test_nonimpact_filter_reseed_preserves_real_hard_fault();
    test_selected_icm_filter_fault_reseeds_without_attitude_jump();
    test_single_lane_recovers_first_after_dual_gyro_loss();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "dual_imu_estimator: %u test failure(s)\n",
                      failure_count);
        return EXIT_FAILURE;
    }
    (void)puts("dual_imu_estimator: all tests passed");
    return EXIT_SUCCESS;
}
