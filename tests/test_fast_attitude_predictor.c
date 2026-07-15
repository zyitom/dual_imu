#include "fast_attitude_predictor.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_EPOCH_US (UINT64_C(1000000))
#define TEST_PI (3.14159265358979323846f)

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
    return degrees * (TEST_PI / 180.0f);
}

static float wrap_pi(float angle)
{
    while (angle > TEST_PI)
        angle -= 2.0f * TEST_PI;
    while (angle < -TEST_PI)
        angle += 2.0f * TEST_PI;
    return angle;
}

static bool nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static float quaternion_dot(const float lhs[4], const float rhs[4])
{
    return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) +
           (lhs[2] * rhs[2]) + (lhs[3] * rhs[3]);
}

static float quaternion_norm(const float quaternion[4])
{
    return sqrtf(quaternion_dot(quaternion, quaternion));
}

static void quaternion_from_euler(float roll,
                                  float pitch,
                                  float yaw,
                                  float quaternion[4])
{
    const float half_roll = 0.5f * roll;
    const float half_pitch = 0.5f * pitch;
    const float half_yaw = 0.5f * yaw;
    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);
    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);

    quaternion[0] = (cy * cp * cr) + (sy * sp * sr);
    quaternion[1] = (cy * cp * sr) - (sy * sp * cr);
    quaternion[2] = (cy * sp * cr) + (sy * cp * sr);
    quaternion[3] = (sy * cp * cr) - (cy * sp * sr);
}

static fast_attitude_predictor_t make_predictor(uint32_t max_gap_us)
{
    fast_attitude_predictor_t predictor;
    fast_attitude_predictor_config_t config;
    fast_attitude_predictor_default_config(&config);
    config.max_gyro_gap_us[IMU_SOURCE_BMI088] = max_gap_us;
    config.max_gyro_gap_us[IMU_SOURCE_ICM45686] = max_gap_us;
    TEST_EXPECT(fast_attitude_predictor_init(&predictor, &config));
    return predictor;
}

static void test_default_gap_limits_allow_at_most_one_missing_sample(void)
{
    fast_attitude_predictor_config_t config;
    fast_attitude_predictor_default_config(&config);
    TEST_EXPECT(config.max_gyro_gap_us[IMU_SOURCE_BMI088] == 1250U);
    TEST_EXPECT(config.max_gyro_gap_us[IMU_SOURCE_ICM45686] == 800U);
}

static fast_attitude_anchor_t make_anchor(uint64_t timestamp_us,
                                          imu_source_t source,
                                          const float quaternion[4],
                                          const float rate[3],
                                          const float bias[3],
                                          bool degraded)
{
    fast_attitude_anchor_t anchor;
    memset(&anchor, 0, sizeof(anchor));
    anchor.timestamp_us = timestamp_us;
    anchor.selected_source = source;
    anchor.degraded = degraded;
    memcpy(anchor.quaternion, quaternion, sizeof(anchor.quaternion));
    memcpy(anchor.gyro_rate_rad_s, rate, sizeof(anchor.gyro_rate_rad_s));
    memcpy(anchor.gyro_bias_rad_s, bias, sizeof(anchor.gyro_bias_rad_s));
    return anchor;
}

static imu_gyro_sample_t make_sample(imu_source_t source,
                                     uint64_t timestamp_us,
                                     uint32_t sequence,
                                     float x,
                                     float y,
                                     float z,
                                     bool valid)
{
    imu_gyro_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    sample.source = source;
    sample.timestamp_us = timestamp_us;
    sample.sequence = sequence;
    sample.gyro_rad_s[0] = x;
    sample.gyro_rad_s[1] = y;
    sample.gyro_rad_s[2] = z;
    sample.valid = valid;
    return sample;
}

static fast_attitude_snapshot_t export_snapshot(
    const fast_attitude_predictor_t *predictor)
{
    fast_attitude_snapshot_t snapshot;
    memset(&snapshot, 0xA5, sizeof(snapshot));
    TEST_EXPECT(fast_attitude_predictor_export_snapshot(predictor,
                                                        &snapshot));
    TEST_EXPECT(snapshot.valid);
    return snapshot;
}

static void expect_yaw_quaternion(const fast_attitude_output_t *output,
                                  float expected_yaw,
                                  float tolerance)
{
    TEST_EXPECT(nearly_equal(output->quaternion[0],
                             cosf(0.5f * expected_yaw), tolerance));
    TEST_EXPECT(nearly_equal(output->quaternion[1], 0.0f, tolerance));
    TEST_EXPECT(nearly_equal(output->quaternion[2], 0.0f, tolerance));
    TEST_EXPECT(nearly_equal(output->quaternion[3],
                             sinf(0.5f * expected_yaw), tolerance));
    TEST_EXPECT(nearly_equal(wrap_pi(output->euler_rad[2]),
                             wrap_pi(expected_yaw), tolerance));
    TEST_EXPECT(nearly_equal(quaternion_norm(output->quaternion), 1.0f, 1.0e-6f));
}

static void test_constant_rate_and_bias(void)
{
    fast_attitude_predictor_t predictor = make_predictor(600U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 1.1f};
    const float bias[3] = {0.0f, 0.0f, 0.1f};
    const fast_attitude_anchor_t anchor =
        make_anchor(TEST_EPOCH_US, IMU_SOURCE_BMI088,
                    identity, rate, bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 1.1f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));

    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 625U, &output));
    expect_yaw_quaternion(&output, 0.000625f, 2.0e-7f);
    TEST_EXPECT(output.anchor_timestamp_us == TEST_EPOCH_US);
    TEST_EXPECT(output.latest_gyro_timestamp_us == TEST_EPOCH_US + 500U);
    TEST_EXPECT(output.prediction_horizon_us == 125U);
    TEST_EXPECT(output.selected_source == IMU_SOURCE_BMI088);
    TEST_EXPECT(nearly_equal(output.gyro_rate_rad_s[0], 0.0f, 1.0e-7f));
    TEST_EXPECT(nearly_equal(output.gyro_rate_rad_s[1], 0.0f, 1.0e-7f));
    TEST_EXPECT(nearly_equal(output.gyro_rate_rad_s[2], 1.0f, 1.0e-7f));
    TEST_EXPECT(output.predicted);
    TEST_EXPECT(!output.degraded);
    TEST_EXPECT(!output.accel_saturation_recent);
    TEST_EXPECT(!output.gyro_saturation_recent);

    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 625U, 2U,
                         0.0f, 0.0f, 1.1f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    fast_attitude_output_t replayed;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 625U, &replayed));
    expect_yaw_quaternion(&replayed, 0.000625f, 2.0e-7f);
    TEST_EXPECT(replayed.prediction_horizon_us == 0U);
}

static void test_linear_ramp_uses_trapezoid(void)
{
    fast_attitude_predictor_t predictor = make_predictor(600U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float anchor_rate[3] = {0.0f, 0.0f, 0.2f};
    const float bias[3] = {0.0f, 0.0f, 0.2f};
    const fast_attitude_anchor_t anchor =
        make_anchor(TEST_EPOCH_US, IMU_SOURCE_ICM45686,
                    identity, anchor_rate, bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_ICM45686, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 0.7f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_ICM45686, TEST_EPOCH_US + 1000U, 2U,
                         0.0f, 0.0f, 1.2f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));

    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1000U, &output));
    expect_yaw_quaternion(&output, 0.0005f, 2.0e-7f);
    TEST_EXPECT(nearly_equal(output.gyro_rate_rad_s[2], 1.0f, 1.0e-7f));
}

static void test_changing_axis_includes_coning_term(void)
{
    fast_attitude_predictor_t predictor = make_predictor(1250U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float anchor_rate[3] = {100.0f, 0.0f, 0.0f};
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const fast_attitude_anchor_t anchor =
        make_anchor(TEST_EPOCH_US, IMU_SOURCE_ICM45686,
                    identity, anchor_rate, zero_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    const imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_ICM45686, TEST_EPOCH_US + 1000U, 1U,
        0.0f, 100.0f, 0.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));

    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1000U, &output));
    TEST_EXPECT(output.quaternion[3] > 4.0e-4f);
    TEST_EXPECT(nearly_equal(output.quaternion[3], 4.1632e-4f, 2.0e-6f));
    TEST_EXPECT(nearly_equal(quaternion_norm(output.quaternion), 1.0f, 1.0e-6f));
}

static void test_body_rate_is_right_multiplied_onto_anchor(void)
{
    fast_attitude_predictor_t predictor = make_predictor(1250U);
    float anchor_quaternion[4];
    quaternion_from_euler(0.0f, 0.0f, degrees_to_radians(90.0f),
                          anchor_quaternion);
    const float rate[3] = {100.0f, 0.0f, 0.0f};
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088,
        anchor_quaternion, rate, zero_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1000U, &output));
    const float anchor_half_angle = degrees_to_radians(45.0f);
    const float delta_half_angle = 0.05f;
    const float expected[4] = {
        cosf(anchor_half_angle) * cosf(delta_half_angle),
        cosf(anchor_half_angle) * sinf(delta_half_angle),
        sinf(anchor_half_angle) * sinf(delta_half_angle),
        sinf(anchor_half_angle) * cosf(delta_half_angle),
    };
    for (size_t index = 0U; index < 4U; ++index)
        TEST_EXPECT(nearly_equal(output.quaternion[index], expected[index], 2.0e-6f));
}

static void test_history_wrap_preserves_recent_replay(void)
{
    fast_attitude_predictor_t predictor = make_predictor(200U);
    for (uint32_t index = 1U; index <= 70U; ++index) {
        const imu_gyro_sample_t sample = make_sample(
            IMU_SOURCE_ICM45686,
            TEST_EPOCH_US + ((uint64_t)index * 100U),
            index, 0.0f, 0.0f, 1.0f, true);
        TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    }
    TEST_EXPECT(predictor.history[IMU_SOURCE_ICM45686].count ==
                FAST_ATTITUDE_PREDICTOR_HISTORY_CAPACITY);

    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 1.0f};
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US + 6500U, IMU_SOURCE_ICM45686,
        identity, rate, zero_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    TEST_EXPECT(predictor.history[IMU_SOURCE_ICM45686].count == 5U);
    TEST_EXPECT(predictor.replay_cache.valid);
    TEST_EXPECT(predictor.replay_cache.coverage_timestamp_us ==
                TEST_EPOCH_US + 7000U);
    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 7000U, &output));
    expect_yaw_quaternion(&output, 0.0005f, 3.0e-7f);
    TEST_EXPECT(output.latest_gyro_timestamp_us == TEST_EPOCH_US + 7000U);
    TEST_EXPECT(output.prediction_horizon_us == 0U);
}

static void test_reanchor_is_idempotent_and_does_not_double_integrate(void)
{
    fast_attitude_predictor_t predictor = make_predictor(600U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float initial_rate[3] = {0.0f, 0.0f, 2.0f};
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    fast_attitude_anchor_t anchor =
        make_anchor(TEST_EPOCH_US, IMU_SOURCE_BMI088,
                    identity, initial_rate, zero_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    for (uint32_t index = 1U; index <= 5U; ++index) {
        const imu_gyro_sample_t sample = make_sample(
            IMU_SOURCE_BMI088, TEST_EPOCH_US + ((uint64_t)index * 500U), index,
            0.0f, 0.0f, 2.0f, true);
        TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    }
    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, TEST_EPOCH_US + 3000U, 6U,
        0.0f, 0.0f, 2.5f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));

    fast_attitude_output_t before_anchor;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 2500U, &before_anchor));
    expect_yaw_quaternion(&before_anchor, 0.005f, 5.0e-7f);

    float corrected_quaternion[4];
    quaternion_from_euler(0.0f, 0.0f, 0.105f, corrected_quaternion);
    const float corrected_rate[3] = {0.0f, 0.0f, 2.5f};
    const float corrected_bias[3] = {0.0f, 0.0f, 0.5f};
    anchor = make_anchor(TEST_EPOCH_US + 2500U, IMU_SOURCE_BMI088,
                         corrected_quaternion, corrected_rate,
                         corrected_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    fast_attitude_output_t first;
    fast_attitude_output_t second;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 3125U, &first));
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 3125U, &second));
    expect_yaw_quaternion(&first, 0.10625f, 8.0e-7f);
    TEST_EXPECT(memcmp(first.quaternion, second.quaternion,
                       sizeof(first.quaternion)) == 0);

    const fast_attitude_anchor_t old_anchor =
        make_anchor(TEST_EPOCH_US + 2000U, IMU_SOURCE_BMI088,
                    identity, initial_rate, zero_bias, false);
    TEST_EXPECT(!fast_attitude_predictor_set_anchor(&predictor, &old_anchor));
    fast_attitude_output_t after_rejected_anchor;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 3125U, &after_rejected_anchor));
    TEST_EXPECT(memcmp(first.quaternion, after_rejected_anchor.quaternion,
                       sizeof(first.quaternion)) == 0);
}

static void test_horizon_and_target_boundaries(void)
{
    fast_attitude_predictor_t predictor = make_predictor(1250U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 1.0f};
    const float bias[3] = {0.0f, 0.0f, 0.0f};
    const fast_attitude_anchor_t anchor =
        make_anchor(TEST_EPOCH_US, IMU_SOURCE_BMI088,
                    identity, rate, bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(&predictor, TEST_EPOCH_US, &output));
    TEST_EXPECT(output.valid);
    TEST_EXPECT(!output.predicted);
    TEST_EXPECT(output.prediction_horizon_us == 0U);
    TEST_EXPECT(memcmp(output.quaternion, identity, sizeof(identity)) == 0);

    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1000U, &output));
    TEST_EXPECT(output.prediction_horizon_us == 1000U);
    expect_yaw_quaternion(&output, 0.001f, 3.0e-7f);

    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1001U, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_HORIZON_EXCEEDED) != 0U);

    const imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1601U, &output));
    TEST_EXPECT(output.latest_gyro_timestamp_us == TEST_EPOCH_US + 500U);
    TEST_EXPECT(output.prediction_horizon_us == 1101U);
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_HORIZON_EXCEEDED) != 0U);
    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US - 1U, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_TARGET_BEFORE_ANCHOR) != 0U);
}

static void test_selected_lane_gaps_are_rejected(void)
{
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 1.0f};
    const float bias[3] = {0.0f, 0.0f, 0.0f};

    fast_attitude_predictor_t predictor = make_predictor(600U);
    fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088, identity, rate, bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1200U, 2U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    fast_attitude_output_t output;
    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1200U, &output));
    TEST_EXPECT((output.integrity_flags & FAST_ATTITUDE_FLAG_GYRO_GAP) != 0U);

    predictor = make_predictor(600U);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1000U, 3U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1000U, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_GYRO_SEQUENCE_GAP) != 0U);

    predictor = make_predictor(600U);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
                         NAN, 0.0f, 1.0f, true);
    TEST_EXPECT(!fast_attitude_predictor_push_gyro(&predictor, &sample));
    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 500U, &output));
    TEST_EXPECT((output.integrity_flags & FAST_ATTITUDE_FLAG_GYRO_INVALID) != 0U);
    TEST_EXPECT((output.integrity_flags & FAST_ATTITUDE_FLAG_NUMERIC) != 0U);

    predictor = make_predictor(600U);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
                         0.0f, 0.0f, 1.0f, false);
    TEST_EXPECT(!fast_attitude_predictor_push_gyro(&predictor, &sample));
    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 500U, &output));
    TEST_EXPECT((output.integrity_flags & FAST_ATTITUDE_FLAG_GYRO_INVALID) != 0U);
    TEST_EXPECT((output.integrity_flags & FAST_ATTITUDE_FLAG_NUMERIC) == 0U);

    predictor = make_predictor(600U);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 400U, 2U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(!fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1000U, 2U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1000U, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_GYRO_TIMESTAMP_ORDER) != 0U);

    predictor = make_predictor(600U);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    sample = make_sample(IMU_SOURCE_ICM45686, TEST_EPOCH_US + 500U, 1U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_ICM45686, TEST_EPOCH_US + 1200U, 2U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 625U, &output));
    TEST_EXPECT(output.integrity_flags == FAST_ATTITUDE_FLAG_NONE);
}

static void test_future_sample_is_not_used_noncausally(void)
{
    fast_attitude_predictor_t predictor = make_predictor(600U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 1.0f};
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088,
        identity, rate, zero_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1000U, 2U,
                         0.0f, 0.0f, 100.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));

    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 625U, &output));
    expect_yaw_quaternion(&output, 0.000625f, 3.0e-7f);
    TEST_EXPECT(output.latest_gyro_timestamp_us == TEST_EPOCH_US + 500U);
    TEST_EXPECT(output.prediction_horizon_us == 125U);
    TEST_EXPECT(nearly_equal(output.gyro_rate_rad_s[2], 1.0f, 1.0e-7f));
}

static void test_lane_switch_and_quaternion_hemisphere_are_continuous(void)
{
    fast_attitude_predictor_t predictor = make_predictor(600U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float icm_rate[3] = {0.0f, 0.0f, 1.1f};
    const float icm_bias[3] = {0.0f, 0.0f, 0.1f};
    fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_ICM45686,
        identity, icm_rate, icm_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_ICM45686, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 1.1f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));

    fast_attitude_output_t before_switch;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 500U, &before_switch));

    float negative_anchor[4];
    for (size_t index = 0U; index < 4U; ++index)
        negative_anchor[index] = -before_switch.quaternion[index];
    const float bmi_rate[3] = {0.0f, 0.0f, 1.5f};
    const float bmi_bias[3] = {0.0f, 0.0f, 0.5f};
    anchor = make_anchor(TEST_EPOCH_US + 500U, IMU_SOURCE_BMI088,
                         negative_anchor, bmi_rate, bmi_bias, true);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    fast_attitude_output_t at_switch;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 500U, &at_switch));
    TEST_EXPECT(quaternion_dot(before_switch.quaternion,
                               at_switch.quaternion) > 0.999999f);
    TEST_EXPECT(at_switch.selected_source == IMU_SOURCE_BMI088);
    TEST_EXPECT(at_switch.degraded);

    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1000U, 1U,
                         0.0f, 0.0f, 1.5f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    fast_attitude_output_t after_switch;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1125U, &after_switch));
    expect_yaw_quaternion(&after_switch, 0.001125f, 8.0e-7f);
}

static void test_euler_singularity_and_yaw_wrapping(void)
{
    fast_attitude_predictor_t predictor = make_predictor(1250U);
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    float quaternion[4];
    quaternion_from_euler(0.0f, degrees_to_radians(84.9f), 0.0f, quaternion);
    fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088, quaternion, zero, zero, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(&predictor, TEST_EPOCH_US, &output));
    TEST_EXPECT(!output.euler_singular);

    quaternion_from_euler(degrees_to_radians(20.0f),
                          degrees_to_radians(-30.0f),
                          degrees_to_radians(40.0f), quaternion);
    anchor = make_anchor(TEST_EPOCH_US + 1U, IMU_SOURCE_BMI088,
                         quaternion, zero, zero, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1U, &output));
    TEST_EXPECT(nearly_equal(output.euler_rad[0], degrees_to_radians(20.0f), 2.0e-6f));
    TEST_EXPECT(nearly_equal(output.euler_rad[1], degrees_to_radians(-30.0f), 2.0e-6f));
    TEST_EXPECT(nearly_equal(output.euler_rad[2], degrees_to_radians(40.0f), 2.0e-6f));

    quaternion_from_euler(0.0f, degrees_to_radians(85.1f), 0.0f, quaternion);
    anchor = make_anchor(TEST_EPOCH_US + 2U, IMU_SOURCE_BMI088,
                         quaternion, zero, zero, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 2U, &output));
    TEST_EXPECT(output.euler_singular);

    quaternion_from_euler(0.0f, degrees_to_radians(-85.1f), 0.0f, quaternion);
    anchor = make_anchor(TEST_EPOCH_US + 3U, IMU_SOURCE_BMI088,
                         quaternion, zero, zero, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 3U, &output));
    TEST_EXPECT(output.euler_singular);

    quaternion_from_euler(0.0f, degrees_to_radians(90.0f), 0.0f, quaternion);
    anchor = make_anchor(TEST_EPOCH_US + 4U, IMU_SOURCE_BMI088,
                         quaternion, zero, zero, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 4U, &output));
    TEST_EXPECT(output.euler_singular);
    TEST_EXPECT(nearly_equal(fabsf(output.euler_rad[1]), 0.5f * TEST_PI, 2.0e-4f));
    TEST_EXPECT(isfinite(output.euler_rad[0]));
    TEST_EXPECT(isfinite(output.euler_rad[1]));
    TEST_EXPECT(isfinite(output.euler_rad[2]));

    quaternion_from_euler(0.0f, 0.0f, degrees_to_radians(179.0f), quaternion);
    const float yaw_rate[3] = {
        0.0f, 0.0f, degrees_to_radians(4.0f) / 0.001f,
    };
    anchor = make_anchor(TEST_EPOCH_US + 10U, IMU_SOURCE_BMI088,
                         quaternion, yaw_rate, zero, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 1010U, &output));
    TEST_EXPECT(nearly_equal(output.euler_rad[2],
                             degrees_to_radians(-177.0f), 2.0e-5f));
}

static void test_anchor_validation_and_reset(void)
{
    fast_attitude_predictor_t predictor = make_predictor(1250U);
    const float non_unit[4] = {2.0f, 0.0f, 0.0f, 0.0f};
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088, non_unit, zero, zero, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(&predictor, TEST_EPOCH_US, &output));
    TEST_EXPECT(nearly_equal(output.quaternion[0], 1.0f, 1.0e-7f));

    const float invalid_quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    anchor = make_anchor(TEST_EPOCH_US + 1U, IMU_SOURCE_BMI088,
                         invalid_quaternion, zero, zero, false);
    TEST_EXPECT(!fast_attitude_predictor_set_anchor(&predictor, &anchor));
    TEST_EXPECT(fast_attitude_predictor_predict(&predictor, TEST_EPOCH_US, &output));
    TEST_EXPECT(output.anchor_timestamp_us == TEST_EPOCH_US);

    fast_attitude_predictor_invalidate_anchor(&predictor);
    anchor = make_anchor(TEST_EPOCH_US - 1U, IMU_SOURCE_BMI088,
                         non_unit, zero, zero, false);
    TEST_EXPECT(!fast_attitude_predictor_set_anchor(&predictor, &anchor));
    TEST_EXPECT(!fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US, &output));
    TEST_EXPECT((output.integrity_flags & FAST_ATTITUDE_FLAG_NO_ANCHOR) != 0U);

    fast_attitude_predictor_reset(&predictor);
    TEST_EXPECT(!fast_attitude_predictor_predict(&predictor, TEST_EPOCH_US, &output));
    TEST_EXPECT((output.integrity_flags & FAST_ATTITUDE_FLAG_NO_ANCHOR) != 0U);
}

static void expect_snapshot_matches_predictor(
    const fast_attitude_predictor_t *predictor,
    const fast_attitude_snapshot_t *snapshot,
    uint64_t output_timestamp_us)
{
    fast_attitude_output_t expected;
    fast_attitude_output_t actual;
    const bool expected_valid = fast_attitude_predictor_predict(
        predictor, output_timestamp_us, &expected);
    const bool actual_valid = fast_attitude_snapshot_predict(
        snapshot, output_timestamp_us, &actual);
    TEST_EXPECT(actual_valid == expected_valid);
    TEST_EXPECT(actual.valid == expected.valid);
    TEST_EXPECT(actual.integrity_flags == expected.integrity_flags);
    TEST_EXPECT(actual.anchor_timestamp_us == expected.anchor_timestamp_us);
    TEST_EXPECT(actual.latest_gyro_timestamp_us ==
                expected.latest_gyro_timestamp_us);
    TEST_EXPECT(actual.output_timestamp_us == expected.output_timestamp_us);
    TEST_EXPECT(actual.prediction_horizon_us == expected.prediction_horizon_us);
    TEST_EXPECT(actual.selected_source == expected.selected_source);
    TEST_EXPECT(actual.predicted == expected.predicted);
    TEST_EXPECT(actual.degraded == expected.degraded);
    TEST_EXPECT(actual.attitude_converged == expected.attitude_converged);
    TEST_EXPECT(actual.post_impact_reacquire_active ==
                expected.post_impact_reacquire_active);
    TEST_EXPECT(actual.attitude_aiding_stale ==
                expected.attitude_aiding_stale);
    TEST_EXPECT(actual.rotation_unobserved == expected.rotation_unobserved);
    TEST_EXPECT(actual.euler_singular == expected.euler_singular);
    if (actual_valid && expected_valid) {
        for (size_t index = 0U; index < 4U; ++index) {
            TEST_EXPECT(nearly_equal(actual.quaternion[index],
                                     expected.quaternion[index], 2.0e-7f));
        }
        for (size_t axis = 0U; axis < 3U; ++axis) {
            TEST_EXPECT(nearly_equal(actual.gyro_rate_rad_s[axis],
                                     expected.gyro_rate_rad_s[axis],
                                     2.0e-7f));
            TEST_EXPECT(nearly_equal(actual.euler_rad[axis],
                                     expected.euler_rad[axis], 2.0e-7f));
        }
    }
}

static void test_snapshot_endpoint_boundaries_are_causal(void)
{
    fast_attitude_predictor_t predictor = make_predictor(600U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float anchor_rate[3] = {0.0f, 0.0f, 0.2f};
    const float bias[3] = {0.0f, 0.0f, 0.2f};
    const fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088, identity, anchor_rate, bias, true);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 1.2f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1000U, 2U,
                         0.0f, 0.0f, 3.2f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));

    const fast_attitude_snapshot_t snapshot = export_snapshot(&predictor);
    TEST_EXPECT(sizeof(snapshot) <= 160U);
    TEST_EXPECT(snapshot.previous_endpoint.timestamp_us ==
                TEST_EPOCH_US + 500U);
    TEST_EXPECT(snapshot.current_endpoint.timestamp_us ==
                TEST_EPOCH_US + 1000U);
    TEST_EXPECT(snapshot.coverage_timestamp_us == TEST_EPOCH_US + 1000U);
    TEST_EXPECT(snapshot.selected_source == IMU_SOURCE_BMI088);
    TEST_EXPECT(snapshot.degraded);
    TEST_EXPECT(!snapshot.blocked);
    TEST_EXPECT(snapshot.max_prediction_horizon_us == 1000U);
    TEST_EXPECT(snapshot.max_gyro_gap_us == 600U);
    TEST_EXPECT(nearly_equal(snapshot.euler_singularity_threshold_rad,
                             degrees_to_radians(85.0f), 1.0e-7f));
    TEST_EXPECT(nearly_equal(snapshot.gyro_bias_rad_s[2], 0.2f, 1.0e-7f));

    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US, &output));
    TEST_EXPECT(!output.predicted);
    TEST_EXPECT(nearly_equal(output.gyro_rate_rad_s[2], 0.0f, 1.0e-7f));

    TEST_EXPECT(!fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US + 250U, &output));
    TEST_EXPECT((output.integrity_flags & FAST_ATTITUDE_FLAG_OUTPUT_STALE) !=
                0U);
    TEST_EXPECT(!fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US - 1U, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_TARGET_BEFORE_ANCHOR) != 0U);

    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 500U);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 750U);
    TEST_EXPECT(fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US + 750U, &output));
    expect_yaw_quaternion(&output, 0.0005f, 3.0e-7f);
    TEST_EXPECT(output.latest_gyro_timestamp_us == TEST_EPOCH_US + 500U);
    TEST_EXPECT(output.prediction_horizon_us == 250U);
    TEST_EXPECT(nearly_equal(output.gyro_rate_rad_s[2], 1.0f, 1.0e-7f));

    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 1000U);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 1125U);
    TEST_EXPECT(fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US + 1125U, &output));
    expect_yaw_quaternion(&output, 0.001625f, 3.0e-7f);
    TEST_EXPECT(output.latest_gyro_timestamp_us == TEST_EPOCH_US + 1000U);
    TEST_EXPECT(output.prediction_horizon_us == 125U);

    fast_attitude_snapshot_t unchanged = snapshot;
    TEST_EXPECT(fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US + 1125U, &output));
    TEST_EXPECT(memcmp(&snapshot, &unchanged, sizeof(snapshot)) == 0);
}

static void test_snapshot_blocked_coverage_and_integrity(void)
{
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 1.0f};
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088,
        identity, rate, zero_bias, false);

    fast_attitude_predictor_t predictor = make_predictor(600U);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1200U, 2U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    fast_attitude_snapshot_t snapshot = export_snapshot(&predictor);
    TEST_EXPECT(snapshot.blocked);
    TEST_EXPECT(snapshot.current_endpoint.timestamp_us ==
                TEST_EPOCH_US + 500U);
    TEST_EXPECT(snapshot.coverage_timestamp_us == TEST_EPOCH_US + 1200U);
    TEST_EXPECT((snapshot.integrity_flags & FAST_ATTITUDE_FLAG_GYRO_GAP) !=
                0U);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 1100U);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 1200U);

    predictor = make_predictor(600U);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1000U, 2U,
                         0.0f, 0.0f, 1.0f, false);
    TEST_EXPECT(!fast_attitude_predictor_push_gyro(&predictor, &sample));
    snapshot = export_snapshot(&predictor);
    TEST_EXPECT(snapshot.blocked);
    TEST_EXPECT(snapshot.coverage_timestamp_us == TEST_EPOCH_US + 1000U);
    TEST_EXPECT((snapshot.integrity_flags & FAST_ATTITUDE_FLAG_GYRO_INVALID) !=
                0U);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 999U);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 1000U);

    predictor = make_predictor(600U);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
                         0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_BMI088, TEST_EPOCH_US + 1000U, 3U,
                         0.0f, 0.0f, 2.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    snapshot = export_snapshot(&predictor);
    TEST_EXPECT(!snapshot.blocked);
    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US + 750U, &output));
    TEST_EXPECT(output.integrity_flags == FAST_ATTITUDE_FLAG_NONE);
    TEST_EXPECT(!fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US + 1000U, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_GYRO_SEQUENCE_GAP) != 0U);
}

static void test_snapshot_invalid_export_and_horizon(void)
{
    fast_attitude_predictor_t predictor;
    memset(&predictor, 0, sizeof(predictor));
    fast_attitude_snapshot_t snapshot;
    TEST_EXPECT(!fast_attitude_predictor_export_snapshot(&predictor,
                                                         &snapshot));
    TEST_EXPECT(!snapshot.valid);
    TEST_EXPECT((snapshot.integrity_flags &
                 FAST_ATTITUDE_FLAG_SNAPSHOT_INVALID) != 0U);
    TEST_EXPECT((snapshot.integrity_flags &
                 FAST_ATTITUDE_FLAG_NOT_INITIALIZED) != 0U);

    fast_attitude_output_t output;
    TEST_EXPECT(!fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_SNAPSHOT_INVALID) != 0U);
    TEST_EXPECT(!fast_attitude_snapshot_predict(NULL, TEST_EPOCH_US, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_SNAPSHOT_INVALID) != 0U);

    predictor = make_predictor(1250U);
    TEST_EXPECT(!fast_attitude_predictor_export_snapshot(&predictor,
                                                         &snapshot));
    TEST_EXPECT(!snapshot.valid);
    TEST_EXPECT((snapshot.integrity_flags & FAST_ATTITUDE_FLAG_NO_ANCHOR) !=
                0U);

    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 1.0f};
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    const fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088,
        identity, rate, zero_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    snapshot = export_snapshot(&predictor);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 1000U);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 1001U);

    snapshot.current_endpoint.timestamp_us =
        snapshot.previous_endpoint.timestamp_us - 1U;
    TEST_EXPECT(!fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US, &output));
    TEST_EXPECT((output.integrity_flags &
                 FAST_ATTITUDE_FLAG_SNAPSHOT_INVALID) != 0U);

    fast_attitude_predictor_invalidate_anchor(&predictor);
    TEST_EXPECT(!fast_attitude_predictor_export_snapshot(&predictor,
                                                         &snapshot));
    TEST_EXPECT(!snapshot.valid);
    TEST_EXPECT(!fast_attitude_predictor_export_snapshot(&predictor, NULL));
    TEST_EXPECT(!fast_attitude_snapshot_predict(&snapshot,
                                                TEST_EPOCH_US, NULL));
}

static void test_snapshot_reanchor_rebuilds_endpoints(void)
{
    fast_attitude_predictor_t predictor = make_predictor(600U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float initial_rate[3] = {0.0f, 0.0f, 1.0f};
    const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
    fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_BMI088,
        identity, initial_rate, zero_bias, false);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));
    imu_gyro_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, TEST_EPOCH_US + 500U, 1U,
        0.0f, 0.0f, 1.0f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    const fast_attitude_snapshot_t old_snapshot = export_snapshot(&predictor);

    sample = make_sample(IMU_SOURCE_ICM45686, TEST_EPOCH_US + 750U, 1U,
                         0.0f, 0.0f, 2.5f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    sample = make_sample(IMU_SOURCE_ICM45686, TEST_EPOCH_US + 1000U, 2U,
                         0.0f, 0.0f, 2.5f, true);
    TEST_EXPECT(fast_attitude_predictor_push_gyro(&predictor, &sample));
    float corrected_quaternion[4];
    quaternion_from_euler(0.0f, 0.0f, 0.1f, corrected_quaternion);
    const float corrected_rate[3] = {0.0f, 0.0f, 2.5f};
    const float corrected_bias[3] = {0.0f, 0.0f, 0.5f};
    anchor = make_anchor(TEST_EPOCH_US + 500U, IMU_SOURCE_ICM45686,
                         corrected_quaternion, corrected_rate,
                         corrected_bias, true);
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    const fast_attitude_snapshot_t snapshot = export_snapshot(&predictor);
    TEST_EXPECT(snapshot.anchor_timestamp_us == TEST_EPOCH_US + 500U);
    TEST_EXPECT(snapshot.previous_endpoint.timestamp_us ==
                TEST_EPOCH_US + 750U);
    TEST_EXPECT(snapshot.current_endpoint.timestamp_us ==
                TEST_EPOCH_US + 1000U);
    TEST_EXPECT(snapshot.selected_source == IMU_SOURCE_ICM45686);
    TEST_EXPECT(snapshot.degraded);
    TEST_EXPECT(nearly_equal(snapshot.gyro_bias_rad_s[2], 0.5f, 1.0e-7f));
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 875U);
    expect_snapshot_matches_predictor(&predictor, &snapshot,
                                      TEST_EPOCH_US + 1125U);

    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_snapshot_predict(
        &snapshot, TEST_EPOCH_US + 1125U, &output));
    expect_yaw_quaternion(&output, 0.10125f, 8.0e-7f);

    TEST_EXPECT(fast_attitude_snapshot_predict(
        &old_snapshot, TEST_EPOCH_US + 625U, &output));
    TEST_EXPECT(output.selected_source == IMU_SOURCE_BMI088);
    expect_yaw_quaternion(&output, 0.000625f, 3.0e-7f);
}

static void test_quality_state_is_anchor_and_snapshot_causal(void)
{
    fast_attitude_predictor_t predictor = make_predictor(600U);
    const float identity[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    fast_attitude_anchor_t anchor = make_anchor(
        TEST_EPOCH_US, IMU_SOURCE_ICM45686, identity, zero, zero, false);
    anchor.quality_flags = FAST_ATTITUDE_QUALITY_ATTITUDE_CONVERGED;
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    const fast_attitude_snapshot_t old_snapshot = export_snapshot(&predictor);
    fast_attitude_output_t output;
    TEST_EXPECT(fast_attitude_predictor_predict(
        &predictor, TEST_EPOCH_US + 100U, &output));
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(!output.attitude_aiding_stale);
    TEST_EXPECT(!output.rotation_unobserved);

    anchor.timestamp_us = TEST_EPOCH_US + 500U;
    anchor.degraded = true;
    anchor.quality_flags =
        FAST_ATTITUDE_QUALITY_POST_IMPACT_REACQUIRE_ACTIVE |
        FAST_ATTITUDE_QUALITY_ATTITUDE_AIDING_STALE |
        FAST_ATTITUDE_QUALITY_ROTATION_UNOBSERVED;
    TEST_EXPECT(fast_attitude_predictor_set_anchor(&predictor, &anchor));

    const fast_attitude_snapshot_t new_snapshot = export_snapshot(&predictor);
    TEST_EXPECT((new_snapshot.quality_flags &
                 FAST_ATTITUDE_QUALITY_ATTITUDE_CONVERGED) == 0U);
    TEST_EXPECT((new_snapshot.quality_flags &
                 FAST_ATTITUDE_QUALITY_POST_IMPACT_REACQUIRE_ACTIVE) != 0U);
    TEST_EXPECT((new_snapshot.quality_flags &
                 FAST_ATTITUDE_QUALITY_ATTITUDE_AIDING_STALE) != 0U);
    TEST_EXPECT((new_snapshot.quality_flags &
                 FAST_ATTITUDE_QUALITY_ROTATION_UNOBSERVED) != 0U);
    TEST_EXPECT(fast_attitude_snapshot_predict(
        &new_snapshot, TEST_EPOCH_US + 600U, &output));
    TEST_EXPECT(!output.attitude_converged);
    TEST_EXPECT(output.post_impact_reacquire_active);
    TEST_EXPECT(output.attitude_aiding_stale);
    TEST_EXPECT(output.rotation_unobserved);

    /* Re-anchoring cannot retroactively change an already published
     * snapshot that may still be feeding an older scheduled frame. */
    TEST_EXPECT(fast_attitude_snapshot_predict(
        &old_snapshot, TEST_EPOCH_US + 100U, &output));
    TEST_EXPECT(output.attitude_converged);
    TEST_EXPECT(!output.post_impact_reacquire_active);
    TEST_EXPECT(!output.attitude_aiding_stale);
    TEST_EXPECT(!output.rotation_unobserved);
}

int main(void)
{
    test_default_gap_limits_allow_at_most_one_missing_sample();
    test_constant_rate_and_bias();
    test_linear_ramp_uses_trapezoid();
    test_changing_axis_includes_coning_term();
    test_body_rate_is_right_multiplied_onto_anchor();
    test_history_wrap_preserves_recent_replay();
    test_reanchor_is_idempotent_and_does_not_double_integrate();
    test_horizon_and_target_boundaries();
    test_selected_lane_gaps_are_rejected();
    test_future_sample_is_not_used_noncausally();
    test_lane_switch_and_quaternion_hemisphere_are_continuous();
    test_euler_singularity_and_yaw_wrapping();
    test_anchor_validation_and_reset();
    test_snapshot_endpoint_boundaries_are_causal();
    test_snapshot_blocked_coverage_and_integrity();
    test_snapshot_invalid_export_and_horizon();
    test_snapshot_reanchor_rebuilds_endpoints();
    test_quality_state_is_anchor_and_snapshot_causal();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "fast_attitude_predictor: %u test failure(s)\n",
                      failure_count);
        return EXIT_FAILURE;
    }
    (void)puts("fast_attitude_predictor: all tests passed");
    return EXIT_SUCCESS;
}
