#include "imu_calibration.h"

#include <math.h>
#include <stdio.h>
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

static void expect_near_tolerance(float actual,
                                  float expected,
                                  float tolerance)
{
    TEST_EXPECT(fabsf(actual - expected) <= tolerance);
}

static void expect_near(float actual, float expected)
{
    expect_near_tolerance(actual, expected, 1.0e-6f);
}

static bool apply_accel(imu_source_t source,
                        uint64_t sample_timestamp_us,
                        float temperature_c,
                        uint64_t temperature_timestamp_us,
                        bool temperature_valid,
                        float accel[3],
                        imu_calibration_apply_info_t *info)
{
    return imu_calibration_apply_accel(source, sample_timestamp_us,
                                       temperature_c,
                                       temperature_timestamp_us,
                                       temperature_valid, accel, info);
}

static bool apply_gyro(imu_source_t source,
                       uint64_t sample_timestamp_us,
                       float temperature_c,
                       uint64_t temperature_timestamp_us,
                       bool temperature_valid,
                       float gyro[3],
                       imu_calibration_apply_info_t *info)
{
    return imu_calibration_apply_gyro(source, sample_timestamp_us,
                                      temperature_c,
                                      temperature_timestamp_us,
                                      temperature_valid, gyro, info);
}

static void test_identity_default(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    TEST_EXPECT(imu_calibration_get(IMU_SOURCE_BMI088, &calibration));
    TEST_EXPECT(imu_calibration_validate(&calibration));
    TEST_EXPECT(!imu_calibration_has_custom(IMU_SOURCE_BMI088));
    TEST_EXPECT(!imu_calibration_temperature_compensation_enabled(
        IMU_SOURCE_BMI088));
    TEST_EXPECT(!imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_BMI088, true));
    TEST_EXPECT(!imu_calibration_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088));
    TEST_EXPECT(!imu_calibration_set_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088, true));

    float accel[3] = {1.0f, -2.0f, 3.0f};
    float gyro[3] = {-0.1f, 0.2f, -0.3f};
    imu_calibration_apply_info_t accel_info;
    imu_calibration_apply_info_t gyro_info;
    TEST_EXPECT(apply_accel(IMU_SOURCE_BMI088, UINT64_C(100), NAN, 0U,
                            false, accel, &accel_info));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, UINT64_C(100), 25.0f,
                           UINT64_C(100), true, gyro, &gyro_info));
    TEST_EXPECT(accel_info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_DISABLED);
    TEST_EXPECT(gyro_info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_DISABLED);
    TEST_EXPECT(!accel_info.correction_slew_limited);
    TEST_EXPECT(!gyro_info.correction_slew_limited);
    expect_near(accel[0], 1.0f);
    expect_near(accel[1], -2.0f);
    expect_near(accel[2], 3.0f);
    expect_near(gyro[0], -0.1f);
    expect_near(gyro[1], 0.2f);
    expect_near(gyro[2], -0.3f);
}

static void test_runtime_enable_is_separate_from_coefficients(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.gyro_correction_matrix[0][0] = 1.1f;
    calibration.gyro_correction_matrix[1][1] = 0.9f;
    calibration.gyro_bias_polynomial_rad_s[0][0] = 0.1f;
    calibration.gyro_bias_polynomial_rad_s[0][1] = 0.01f;
    calibration.accel_bias_polynomial_mps2[2][0] = 0.5f;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_ICM45686, &calibration));
    TEST_EXPECT(imu_calibration_has_custom(IMU_SOURCE_ICM45686));
    TEST_EXPECT(!imu_calibration_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686));

    float gyro[3] = {1.2f, 2.0f, 3.0f};
    float accel[3] = {1.0f, 2.0f, -9.0f};
    imu_calibration_apply_info_t info;
    TEST_EXPECT(apply_gyro(IMU_SOURCE_ICM45686, UINT64_C(1000), 35.0f,
                           UINT64_C(900), true, gyro, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_DISABLED);
    expect_near(gyro[0], 1.21f);
    expect_near(gyro[1], 1.8f);
    expect_near(gyro[2], 3.0f);
    TEST_EXPECT(apply_accel(IMU_SOURCE_ICM45686, UINT64_C(1000), 35.0f,
                            UINT64_C(900), true, accel, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_DISABLED);
    expect_near(accel[2], -9.5f);

    TEST_EXPECT(!imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686, true));
    TEST_EXPECT(!imu_calibration_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686));

#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1
    /* A fresh qualification lifecycle may enable only before either stream. */
    imu_calibration_reset_all();
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_ICM45686, &calibration));
    TEST_EXPECT(imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686, true));
    TEST_EXPECT(imu_calibration_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686));
    gyro[0] = 1.2f;
    gyro[1] = 2.0f;
    gyro[2] = 3.0f;
    TEST_EXPECT(apply_gyro(IMU_SOURCE_ICM45686, UINT64_C(11000), 35.0f,
                           UINT64_C(11000), true, gyro, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    TEST_EXPECT(info.correction_slew_limited);

    TEST_EXPECT(!imu_calibration_set(IMU_SOURCE_ICM45686, &calibration));
    TEST_EXPECT(imu_calibration_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686));
    TEST_EXPECT(imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686, false));
    TEST_EXPECT(!imu_calibration_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686));
#endif
}

#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 0
static void test_production_temperature_inputs_are_bit_equivalent(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.gyro_correction_matrix[0][0] = 1.1f;
    calibration.gyro_correction_matrix[0][1] = 0.1f;
    calibration.gyro_correction_matrix[1][1] = 0.9f;
    calibration.gyro_bias_polynomial_rad_s[0][0] = 0.1f;
    calibration.gyro_bias_polynomial_rad_s[1][0] = -0.2f;
    calibration.gyro_bias_polynomial_rad_s[0][1] = 0.001f;
    calibration.gyro_bias_polynomial_rad_s[1][2] = 0.00001f;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_BMI088, &calibration));

    const float raw[3] = {1.2f, -0.7f, 0.3f};
    float expected[3];
    memcpy(expected, raw, sizeof(expected));
    imu_calibration_apply_info_t info;
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, UINT64_C(1000), 25.0f,
                           UINT64_C(1000), true, expected, &info));
    float unbiased[3];
    float reference[3] = {0.0f};
    for (size_t axis = 0U; axis < 3U; ++axis) {
        unbiased[axis] =
            raw[axis] - calibration.gyro_bias_polynomial_rad_s[axis][0];
    }
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            reference[row] += calibration.gyro_correction_matrix[row][column] *
                              unbiased[column];
        }
    }
    TEST_EXPECT(memcmp(expected, reference, sizeof(expected)) == 0);

    const float temperatures[] = {NAN, -1000.0f, 1000.0f, 35.0f};
    const uint64_t sample_timestamps[] = {
        0U, UINT64_C(1), UINT64_C(3000000), UINT64_MAX,
    };
    const uint64_t temperature_timestamps[] = {
        UINT64_MAX, 0U, UINT64_C(4000000), UINT64_C(1),
    };
    const bool valid[] = {false, true, true, true};
    for (size_t index = 0U; index < 4U; ++index) {
        float actual[3];
        memcpy(actual, raw, sizeof(actual));
        TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088,
                               sample_timestamps[index],
                               temperatures[index],
                               temperature_timestamps[index], valid[index],
                               actual, &info));
        TEST_EXPECT(memcmp(actual, expected, sizeof(actual)) == 0);
        TEST_EXPECT(info.temperature_use ==
                    IMU_CALIBRATION_TEMPERATURE_C0_DISABLED);
        TEST_EXPECT(!info.correction_slew_limited);
    }
}
#else
static float vector_norm(const float vector[3])
{
    return sqrtf((vector[0] * vector[0]) +
                 (vector[1] * vector[1]) +
                 (vector[2] * vector[2]));
}

static void output_to_correction(const float output[3], float correction[3])
{
    for (size_t axis = 0U; axis < 3U; ++axis)
        correction[axis] = -output[axis];
}

static void expect_correction_step_bounded(const float before[3],
                                           const float after[3],
                                           float maximum_step)
{
    float difference[3];
    for (size_t axis = 0U; axis < 3U; ++axis)
        difference[axis] = after[axis] - before[axis];
    TEST_EXPECT(vector_norm(difference) <= maximum_step + 1.0e-6f);
}

static void test_output_frame_vector_slew_and_long_gap_cap(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.gyro_correction_matrix[0][0] = 1.2f;
    calibration.gyro_correction_matrix[1][1] = 0.8f;
    calibration.gyro_correction_matrix[2][2] = 1.1f;
    calibration.gyro_bias_polynomial_rad_s[0][1] = 0.01f;
    calibration.gyro_bias_polynomial_rad_s[1][1] = 0.02f;
    calibration.gyro_bias_polynomial_rad_s[2][1] = 0.02f;
    calibration.maximum_gyro_correction_slew_rad_s2 = 0.01f;
    calibration.accel_bias_polynomial_mps2[0][1] = 0.1f;
    calibration.accel_bias_polynomial_mps2[1][1] = 0.2f;
    calibration.accel_bias_polynomial_mps2[2][1] = 0.2f;
    calibration.maximum_accel_correction_slew_mps3 = 0.2f;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_ICM45686, &calibration));
    TEST_EXPECT(imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686, true));

    imu_calibration_apply_info_t info;
    float output[3] = {0.0f};
    float correction_0[3];
    float correction_1[3];
    float correction_2[3];
    const uint64_t first_us = UINT64_C(1000000);
    TEST_EXPECT(apply_gyro(IMU_SOURCE_ICM45686, first_us, 35.0f,
                           first_us, true, output, &info));
    output_to_correction(output, correction_0);
    expect_near(vector_norm(correction_0), 0.0f);
    TEST_EXPECT(info.correction_slew_limited);

    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_ICM45686, first_us + UINT64_C(10000),
                           35.0f, first_us + UINT64_C(10000), true,
                           output, &info));
    output_to_correction(output, correction_1);
    expect_near_tolerance(vector_norm(correction_1), 0.0001f, 2.0e-7f);

    memset(output, 0, sizeof(output));
    const uint64_t long_gap_us = first_us + UINT64_C(100000000);
    TEST_EXPECT(apply_gyro(IMU_SOURCE_ICM45686, long_gap_us, 35.0f,
                           long_gap_us, true, output, &info));
    output_to_correction(output, correction_2);
    expect_correction_step_bounded(correction_1, correction_2, 0.0001f);
    TEST_EXPECT(info.correction_slew_limited);

    float accel_output[3] = {0.0f};
    float accel_correction[3];
    const uint64_t accel_first_us = UINT64_C(200000000);
    TEST_EXPECT(apply_accel(IMU_SOURCE_ICM45686, accel_first_us, 35.0f,
                            accel_first_us, true, accel_output, &info));
    memset(accel_output, 0, sizeof(accel_output));
    TEST_EXPECT(apply_accel(IMU_SOURCE_ICM45686,
                            accel_first_us + UINT64_C(10000), 35.0f,
                            accel_first_us + UINT64_C(10000), true,
                            accel_output, &info));
    output_to_correction(accel_output, accel_correction);
    expect_near_tolerance(vector_norm(accel_correction), 0.002f, 2.0e-6f);

    imu_calibration_diagnostics_t diagnostics;
    TEST_EXPECT(imu_calibration_get_diagnostics(IMU_SOURCE_ICM45686,
                                                &diagnostics));
    TEST_EXPECT(diagnostics.correction_slew_limited_count
                    [IMU_CALIBRATION_STREAM_GYRO] >= 3U);
    TEST_EXPECT(diagnostics.correction_slew_limited_count
                    [IMU_CALIBRATION_STREAM_ACCEL] >= 2U);
}

static void test_temperature_transitions_are_slewed(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.gyro_bias_polynomial_rad_s[0][0] = 0.1f;
    calibration.gyro_bias_polynomial_rad_s[0][1] = 0.0002f;
    calibration.maximum_temperature_rate_c_per_s = 10.0f;
    calibration.maximum_gyro_correction_slew_rad_s2 =
        IMU_CALIBRATION_MAX_GYRO_CORRECTION_SLEW_RAD_S2;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_BMI088, &calibration));
    TEST_EXPECT(imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_BMI088, true));

    const float maximum_step =
        calibration.maximum_gyro_correction_slew_rad_s2 * 0.01f;
    uint64_t timestamp_us = UINT64_C(3000000);
    imu_calibration_apply_info_t info;
    float output[3] = {0.0f};
    float before[3];
    float after[3];

    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 35.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    TEST_EXPECT(info.correction_slew_limited);
    output_to_correction(output, before);
    expect_near(before[0], 0.1f);
    for (size_t settle = 0U; settle < 2U; ++settle) {
        timestamp_us += UINT64_C(10000);
        memset(output, 0, sizeof(output));
        TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 35.0f,
                               timestamp_us, true, output, &info));
        output_to_correction(output, after);
        expect_correction_step_bounded(before, after, maximum_step);
        memcpy(before, after, sizeof(before));
    }
    expect_near(before[0], 0.102f);

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, NAN,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_INVALID);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    TEST_EXPECT(after[0] > 0.1f);
    memcpy(before, after, sizeof(before));

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 35.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(
        IMU_SOURCE_BMI088, timestamp_us, 35.0f,
        timestamp_us - IMU_CALIBRATION_TEMPERATURE_MAX_AGE_US - 1U,
        true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_STALE);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 35.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 35.0f,
                           timestamp_us + 1U, true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_NONCAUSAL);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 35.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    output_to_correction(output, after);
    memcpy(before, after, sizeof(before));

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 45.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_RATE);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    /* A rejected rate sample must not poison the accepted-temperature state. */
    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 35.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    /* Let the clamped endpoint become rate-plausible without resetting the
     * accepted-temperature history. The correction step remains capped. */
    timestamp_us += UINT64_C(6600000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 150.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_FULL_CLAMPED);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, NAN,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_INVALID);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 150.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_FULL_CLAMPED);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);
    memcpy(before, after, sizeof(before));

    TEST_EXPECT(imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_BMI088, false));
    TEST_EXPECT(!imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_BMI088, true));
    timestamp_us += UINT64_C(10000);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, timestamp_us, 35.0f,
                           timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_DISABLED);
    output_to_correction(output, after);
    expect_correction_step_bounded(before, after, maximum_step);

    imu_calibration_diagnostics_t diagnostics;
    TEST_EXPECT(imu_calibration_get_diagnostics(IMU_SOURCE_BMI088,
                                                &diagnostics));
    TEST_EXPECT(diagnostics.c0_invalid_count == 2U);
    TEST_EXPECT(diagnostics.c0_stale_count == 1U);
    TEST_EXPECT(diagnostics.c0_noncausal_count == 1U);
    TEST_EXPECT(diagnostics.c0_rate_count == 1U);
    TEST_EXPECT(diagnostics.clamped_compensation_count == 2U);
    TEST_EXPECT(diagnostics.temperature_use_transition_count
                    [IMU_CALIBRATION_STREAM_GYRO] >= 10U);
    TEST_EXPECT(diagnostics.correction_slew_limited_count
                    [IMU_CALIBRATION_STREAM_GYRO] > 0U);
    TEST_EXPECT(diagnostics.last_temperature_use_by_stream
                    [IMU_CALIBRATION_STREAM_GYRO] ==
                IMU_CALIBRATION_TEMPERATURE_C0_DISABLED);
}

static void test_accel_and_gyro_temperature_histories_are_independent(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.maximum_temperature_rate_c_per_s = 1.0f;
    calibration.accel_bias_polynomial_mps2[0][1] = 0.01f;
    calibration.gyro_bias_polynomial_rad_s[0][1] = 0.001f;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_BMI088, &calibration));
    TEST_EXPECT(imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_BMI088, true));

    imu_calibration_apply_info_t info;
    float accel[3] = {0.0f};
    TEST_EXPECT(apply_accel(IMU_SOURCE_BMI088, UINT64_C(3000000), 30.0f,
                            UINT64_C(2000000), true, accel, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);

    /* This older observation is still causal and fresh for the gyro stream. */
    float gyro[3] = {0.0f};
    TEST_EXPECT(apply_gyro(IMU_SOURCE_BMI088, UINT64_C(1500000), 25.0f,
                           UINT64_C(1000000), true, gyro, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);

    imu_calibration_diagnostics_t diagnostics;
    TEST_EXPECT(imu_calibration_get_diagnostics(IMU_SOURCE_BMI088,
                                                &diagnostics));
    TEST_EXPECT(diagnostics.c0_noncausal_count == 0U);
    TEST_EXPECT(diagnostics.full_compensation_count == 2U);
    TEST_EXPECT(diagnostics.last_temperature_use_by_stream
                    [IMU_CALIBRATION_STREAM_ACCEL] ==
                IMU_CALIBRATION_TEMPERATURE_FULL);
    TEST_EXPECT(diagnostics.last_temperature_use_by_stream
                    [IMU_CALIBRATION_STREAM_GYRO] ==
                IMU_CALIBRATION_TEMPERATURE_FULL);
}

static void test_held_temperature_age_boundary_and_recovery(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.gyro_bias_polynomial_rad_s[0][1] = 0.001f;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_ICM45686, &calibration));
    TEST_EXPECT(imu_calibration_set_temperature_compensation_enabled(
        IMU_SOURCE_ICM45686, true));

    const uint64_t temperature_timestamp_us = UINT64_C(900000);
    imu_calibration_apply_info_t info;
    float output[3] = {0.0f};
    float correction[3];
    TEST_EXPECT(apply_gyro(IMU_SOURCE_ICM45686, UINT64_C(1000000), 30.0f,
                           temperature_timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    TEST_EXPECT(info.correction_slew_limited);
    output_to_correction(output, correction);
    expect_near(correction[0], 0.0f);

    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(
        IMU_SOURCE_ICM45686,
        temperature_timestamp_us + IMU_CALIBRATION_TEMPERATURE_MAX_AGE_US,
        30.0f, temperature_timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    output_to_correction(output, correction);
    const float before_stale = correction[0];

    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(
        IMU_SOURCE_ICM45686,
        temperature_timestamp_us + IMU_CALIBRATION_TEMPERATURE_MAX_AGE_US +
            UINT64_C(1),
        30.0f, temperature_timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use ==
                IMU_CALIBRATION_TEMPERATURE_C0_STALE);
    output_to_correction(output, correction);
    TEST_EXPECT(correction[0] <= before_stale);

    const uint64_t recovery_timestamp_us =
        temperature_timestamp_us + IMU_CALIBRATION_TEMPERATURE_MAX_AGE_US +
        UINT64_C(10001);
    memset(output, 0, sizeof(output));
    TEST_EXPECT(apply_gyro(IMU_SOURCE_ICM45686, recovery_timestamp_us, 30.0f,
                           recovery_timestamp_us, true, output, &info));
    TEST_EXPECT(info.temperature_use == IMU_CALIBRATION_TEMPERATURE_FULL);
    output_to_correction(output, correction);
    TEST_EXPECT(correction[0] > 0.0f);
}
#endif

static void test_rejects_invalid_sources(void)
{
    imu_calibration_reset_all();
    const imu_source_t invalid = (imu_source_t)-1;
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    imu_calibration_diagnostics_t diagnostics;
    TEST_EXPECT(!imu_calibration_set(invalid, &calibration));
    TEST_EXPECT(!imu_calibration_get(invalid, &calibration));
    TEST_EXPECT(!imu_calibration_has_custom(invalid));
    TEST_EXPECT(!imu_calibration_set_temperature_compensation_enabled(
        invalid, false));
    TEST_EXPECT(!imu_calibration_temperature_compensation_enabled(invalid));
    TEST_EXPECT(!imu_calibration_set_gyro_g_sensitivity_enabled(invalid,
                                                                 false));
    TEST_EXPECT(!imu_calibration_gyro_g_sensitivity_enabled(invalid));
    TEST_EXPECT(!imu_calibration_get_diagnostics(invalid, &diagnostics));

    float vector[3] = {1.0f, 2.0f, 3.0f};
    const float original[3] = {1.0f, 2.0f, 3.0f};
    TEST_EXPECT(!apply_accel(invalid, UINT64_C(100), 25.0f,
                             UINT64_C(100), true, vector, NULL));
    TEST_EXPECT(memcmp(vector, original, sizeof(vector)) == 0);
    TEST_EXPECT(!apply_gyro(invalid, UINT64_C(100), 25.0f,
                            UINT64_C(100), true, vector, NULL));
    TEST_EXPECT(memcmp(vector, original, sizeof(vector)) == 0);
    TEST_EXPECT(!imu_calibration_apply_gyro_g_sensitivity(
        invalid, UINT64_C(100), NULL, vector, NULL));
    TEST_EXPECT(memcmp(vector, original, sizeof(vector)) == 0);
}

#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 0
static void test_g_sensitivity_production_path_is_bit_equivalent(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.gyro_g_sensitivity_rad_s_per_mps2[0][0] = 0.0001f;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_BMI088, &calibration));
    TEST_EXPECT(!imu_calibration_set_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088, true));

    float gyro[3] = {0.0f, -0.0f, 1.25f};
    float original[3];
    memcpy(original, gyro, sizeof(original));
    const imu_calibration_accel_evidence_t evidence = {
        .timestamp_us = UINT64_MAX,
        .accel_body_mps2 = {100.0f, -100.0f, 50.0f},
        .source = IMU_SOURCE_ICM45686,
        .valid = false,
        .saturated = true,
    };
    imu_calibration_g_sensitivity_apply_info_t info;
    TEST_EXPECT(imu_calibration_apply_gyro_g_sensitivity(
        IMU_SOURCE_BMI088, 0U, &evidence, gyro, &info));
    TEST_EXPECT(memcmp(gyro, original, sizeof(gyro)) == 0);
    TEST_EXPECT(info.use == IMU_CALIBRATION_G_SENSITIVITY_DISABLED);
}
#else
static void test_g_sensitivity_zero_matrix_is_bit_equivalent(void)
{
    imu_calibration_reset_all();
    TEST_EXPECT(!imu_calibration_set_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088, true));

    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_BMI088, &calibration));
    TEST_EXPECT(imu_calibration_set_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088, true));

    float gyro[3] = {0.0f, -0.0f, 1.25f};
    float original[3];
    memcpy(original, gyro, sizeof(original));
    imu_calibration_g_sensitivity_apply_info_t info;
    TEST_EXPECT(imu_calibration_apply_gyro_g_sensitivity(
        IMU_SOURCE_BMI088, 0U, NULL, gyro, &info));
    TEST_EXPECT(memcmp(gyro, original, sizeof(gyro)) == 0);
    TEST_EXPECT(info.use == IMU_CALIBRATION_G_SENSITIVITY_ZERO_MATRIX);

    TEST_EXPECT(!imu_calibration_set(IMU_SOURCE_BMI088, &calibration));
    TEST_EXPECT(imu_calibration_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088));
    TEST_EXPECT(imu_calibration_set_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088, false));
}

static void test_g_sensitivity_all_matrix_terms_and_units(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    float coefficient = 0.00001f;
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            calibration.gyro_g_sensitivity_rad_s_per_mps2[row][column] =
                coefficient;
            coefficient += 0.00001f;
        }
    }
    calibration.maximum_g_sensitivity_accel_age_us = 1000U;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_ICM45686, &calibration));
    TEST_EXPECT(imu_calibration_set_gyro_g_sensitivity_enabled(
        IMU_SOURCE_ICM45686, true));

    const imu_calibration_accel_evidence_t evidence = {
        .timestamp_us = UINT64_C(2000000),
        .accel_body_mps2 = {1.0f, 2.0f, 3.0f},
        .source = IMU_SOURCE_ICM45686,
        .valid = true,
        .saturated = false,
    };
    float gyro[3] = {1.0f, 2.0f, 3.0f};
    imu_calibration_g_sensitivity_apply_info_t info;
    TEST_EXPECT(imu_calibration_apply_gyro_g_sensitivity(
        IMU_SOURCE_ICM45686, UINT64_C(2001000), &evidence, gyro, &info));
    expect_near(gyro[0], 1.0f - 0.00014f);
    expect_near(gyro[1], 2.0f - 0.00032f);
    expect_near(gyro[2], 3.0f - 0.00050f);
    TEST_EXPECT(info.use == IMU_CALIBRATION_G_SENSITIVITY_APPLIED);
    TEST_EXPECT(info.accel_age_us == UINT64_C(1000));
}

static void expect_g_sensitivity_rejects_without_modifying(
    uint64_t gyro_timestamp_us,
    const imu_calibration_accel_evidence_t *evidence,
    imu_calibration_g_sensitivity_use_t expected_use)
{
    float gyro[3] = {0.0f, -0.0f, 0.75f};
    float original[3];
    memcpy(original, gyro, sizeof(original));
    imu_calibration_g_sensitivity_apply_info_t info;
    TEST_EXPECT(!imu_calibration_apply_gyro_g_sensitivity(
        IMU_SOURCE_BMI088, gyro_timestamp_us, evidence, gyro, &info));
    TEST_EXPECT(memcmp(gyro, original, sizeof(gyro)) == 0);
    TEST_EXPECT(info.use == expected_use);
}

static void test_g_sensitivity_evidence_gates_and_diagnostics(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.gyro_g_sensitivity_rad_s_per_mps2[0][0] = 0.0001f;
    calibration.maximum_g_sensitivity_accel_age_us = 1000U;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_BMI088, &calibration));
    TEST_EXPECT(imu_calibration_set_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088, true));

    expect_g_sensitivity_rejects_without_modifying(
        UINT64_C(10000), NULL,
        IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL);

    imu_calibration_accel_evidence_t evidence = {
        .timestamp_us = UINT64_C(9000),
        .accel_body_mps2 = {9.80665f, 0.0f, 0.0f},
        .source = IMU_SOURCE_ICM45686,
        .valid = true,
        .saturated = false,
    };
    expect_g_sensitivity_rejects_without_modifying(
        UINT64_C(10000), &evidence,
        IMU_CALIBRATION_G_SENSITIVITY_SOURCE_MISMATCH);

    evidence.source = IMU_SOURCE_BMI088;
    evidence.saturated = true;
    evidence.valid = false;
    expect_g_sensitivity_rejects_without_modifying(
        UINT64_C(10000), &evidence,
        IMU_CALIBRATION_G_SENSITIVITY_SATURATED_ACCEL);

    evidence.saturated = false;
    expect_g_sensitivity_rejects_without_modifying(
        UINT64_C(10000), &evidence,
        IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL);

    evidence.valid = true;
    evidence.accel_body_mps2[1] = NAN;
    expect_g_sensitivity_rejects_without_modifying(
        UINT64_C(10000), &evidence,
        IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL);

    evidence.accel_body_mps2[1] = 0.0f;
    evidence.timestamp_us = UINT64_C(10001);
    expect_g_sensitivity_rejects_without_modifying(
        UINT64_C(10000), &evidence,
        IMU_CALIBRATION_G_SENSITIVITY_NONCAUSAL_ACCEL);

    evidence.timestamp_us = UINT64_C(8999);
    expect_g_sensitivity_rejects_without_modifying(
        UINT64_C(10000), &evidence,
        IMU_CALIBRATION_G_SENSITIVITY_STALE_ACCEL);

    evidence.timestamp_us = UINT64_C(9000);
    float gyro[3] = {0.0f};
    imu_calibration_g_sensitivity_apply_info_t info;
    TEST_EXPECT(imu_calibration_apply_gyro_g_sensitivity(
        IMU_SOURCE_BMI088, UINT64_C(10000), &evidence, gyro, &info));
    TEST_EXPECT(info.use == IMU_CALIBRATION_G_SENSITIVITY_APPLIED);
    TEST_EXPECT(info.accel_age_us == UINT64_C(1000));

    expect_g_sensitivity_rejects_without_modifying(
        0U, &evidence, IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL);

    TEST_EXPECT(imu_calibration_set_gyro_g_sensitivity_enabled(
        IMU_SOURCE_BMI088, false));
    gyro[0] = 0.0f;
    gyro[1] = -0.0f;
    gyro[2] = 0.75f;
    float original[3];
    memcpy(original, gyro, sizeof(original));
    TEST_EXPECT(imu_calibration_apply_gyro_g_sensitivity(
        IMU_SOURCE_BMI088, 0U, NULL, gyro, &info));
    TEST_EXPECT(memcmp(gyro, original, sizeof(gyro)) == 0);
    TEST_EXPECT(info.use == IMU_CALIBRATION_G_SENSITIVITY_DISABLED);

    imu_calibration_diagnostics_t diagnostics;
    TEST_EXPECT(imu_calibration_get_diagnostics(IMU_SOURCE_BMI088,
                                                &diagnostics));
    TEST_EXPECT(diagnostics.g_sensitivity_applied_count == 1U);
    TEST_EXPECT(diagnostics.g_sensitivity_disabled_count == 1U);
    TEST_EXPECT(diagnostics.g_sensitivity_invalid_accel_count == 4U);
    TEST_EXPECT(diagnostics.g_sensitivity_saturated_accel_count == 1U);
    TEST_EXPECT(diagnostics.g_sensitivity_source_mismatch_count == 1U);
    TEST_EXPECT(diagnostics.g_sensitivity_noncausal_accel_count == 1U);
    TEST_EXPECT(diagnostics.g_sensitivity_stale_accel_count == 1U);
}
#endif

static void test_rejects_cubic_internal_extremum(void)
{
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.minimum_temperature_c = 24.0f;
    calibration.reference_temperature_c = 25.0f;
    calibration.maximum_temperature_c = 26.0f;
    calibration.gyro_bias_polynomial_rad_s[0][1] = -15.0f;
    calibration.gyro_bias_polynomial_rad_s[0][3] = 15.0f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));

    calibration.gyro_bias_polynomial_rad_s[0][1] = -10.0f;
    calibration.gyro_bias_polynomial_rad_s[0][3] = 10.0f;
    TEST_EXPECT(imu_calibration_validate(&calibration));
}

static void test_rejects_invalid_matrix_and_slew_limits(void)
{
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.maximum_temperature_rate_c_per_s = 0.0f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));

    imu_calibration_default(&calibration);
    calibration.maximum_accel_correction_slew_mps3 = 0.0f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));
    calibration.maximum_accel_correction_slew_mps3 =
        IMU_CALIBRATION_MAX_ACCEL_CORRECTION_SLEW_MPS3 + 0.01f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));

    imu_calibration_default(&calibration);
    calibration.maximum_gyro_correction_slew_rad_s2 = NAN;
    TEST_EXPECT(!imu_calibration_validate(&calibration));
    calibration.maximum_gyro_correction_slew_rad_s2 =
        IMU_CALIBRATION_MAX_GYRO_CORRECTION_SLEW_RAD_S2 + 0.01f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));

    imu_calibration_default(&calibration);
    calibration.maximum_g_sensitivity_accel_age_us = 0U;
    TEST_EXPECT(!imu_calibration_validate(&calibration));
    calibration.maximum_g_sensitivity_accel_age_us =
        IMU_CALIBRATION_MAX_G_SENSITIVITY_ACCEL_MAX_AGE_US + 1U;
    TEST_EXPECT(!imu_calibration_validate(&calibration));

    imu_calibration_default(&calibration);
    calibration.gyro_g_sensitivity_rad_s_per_mps2[0][0] = NAN;
    TEST_EXPECT(!imu_calibration_validate(&calibration));
    calibration.gyro_g_sensitivity_rad_s_per_mps2[0][0] =
        IMU_CALIBRATION_MAX_G_SENSITIVITY_ROW_NORM_RAD_S_PER_MPS2 +
        0.00001f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));

    imu_calibration_default(&calibration);
    for (size_t axis = 0U; axis < 3U; ++axis)
        calibration.gyro_g_sensitivity_rad_s_per_mps2[axis][axis] = 0.0009f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));

    imu_calibration_default(&calibration);
    calibration.gyro_correction_matrix[0][0] = 0.0f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));

    imu_calibration_default(&calibration);
    calibration.accel_correction_matrix[0][0] = -1.0f;
    TEST_EXPECT(!imu_calibration_validate(&calibration));
    TEST_EXPECT(!imu_calibration_set(IMU_SOURCE_FUSED, &calibration));
}

int main(void)
{
    test_identity_default();
    test_rejects_invalid_sources();
    test_runtime_enable_is_separate_from_coefficients();
#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 0
    test_production_temperature_inputs_are_bit_equivalent();
#else
    test_output_frame_vector_slew_and_long_gap_cap();
    test_temperature_transitions_are_slewed();
    test_accel_and_gyro_temperature_histories_are_independent();
    test_held_temperature_age_boundary_and_recovery();
#endif
#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 0
    test_g_sensitivity_production_path_is_bit_equivalent();
#else
    test_g_sensitivity_zero_matrix_is_bit_equivalent();
    test_g_sensitivity_all_matrix_terms_and_units();
    test_g_sensitivity_evidence_gates_and_diagnostics();
#endif
    test_rejects_cubic_internal_extremum();
    test_rejects_invalid_matrix_and_slew_limits();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "imu_calibration: %u test failure(s)\n",
                      failure_count);
        return 1;
    }
    (void)puts("imu_calibration: all tests passed");
    return 0;
}
