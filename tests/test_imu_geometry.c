#include "imu_geometry.h"

#include <math.h>
#include <stdio.h>

static unsigned int failure_count;

#define TEST_EXPECT(condition)                                                        \
    do                                                                                \
    {                                                                                 \
        if (!(condition))                                                             \
        {                                                                             \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",                 \
                          __FILE__, __LINE__, #condition);                             \
            failure_count++;                                                          \
        }                                                                             \
    } while (0)

static void test_board_geometry(void)
{
    const float delta_y = imu_dm_fc01_midpoint_to_bmi_m[1] -
                          imu_dm_fc01_midpoint_to_icm_m[1];

    TEST_EXPECT(fabsf(delta_y + IMU_DM_FC01_CENTER_DISTANCE_M) < 1.0e-8f);
    TEST_EXPECT(fabsf(imu_dm_fc01_midpoint_to_bmi_m[0]) < 1.0e-8f);
    TEST_EXPECT(fabsf(imu_dm_fc01_midpoint_to_icm_m[2]) < 1.0e-8f);
}

static void test_rigid_body_translation(void)
{
    const float arm[3] = {0.01f, 0.0f, 0.0f};
    const float rate[3] = {0.0f, 0.0f, 10.0f};
    const float angular_accel[3] = {0.0f, 0.0f, 100.0f};
    const float at_sensor[3] = {-1.0f, 1.0f, -9.80665f};
    float at_reference[3];

    TEST_EXPECT(imu_translate_specific_force(at_sensor, rate, angular_accel,
                                             arm, at_reference));
    TEST_EXPECT(fabsf(at_reference[0]) < 1.0e-6f);
    TEST_EXPECT(fabsf(at_reference[1]) < 1.0e-6f);
    TEST_EXPECT(fabsf(at_reference[2] + 9.80665f) < 1.0e-6f);
}

static void test_angular_accel_filter(void)
{
    imu_angular_accel_estimator_t estimator;
    float output[3];
    const float zero_rate[3] = {0.0f, 0.0f, 0.0f};
    const float step_rate[3] = {1.0f, 0.0f, 0.0f};

    imu_angular_accel_estimator_init(&estimator, 20.0f, 5000.0f);
    TEST_EXPECT(imu_angular_accel_estimator_update(&estimator, zero_rate,
                                                   0.0025f, output));
    TEST_EXPECT(fabsf(output[0]) < 1.0e-8f);
    TEST_EXPECT(imu_angular_accel_estimator_update(&estimator, step_rate,
                                                   0.0025f, output));
    TEST_EXPECT(output[0] > 0.0f);
    TEST_EXPECT(output[0] < 400.0f);
}

static void test_zero_mean_vibration_translation(void)
{
    const float amplitude_rad_s = 10.0f;
    const float second_moment[3][3] = {
        {0.5f * amplitude_rad_s * amplitude_rad_s, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
    };
    const float zero_alpha[3] = {0.0f, 0.0f, 0.0f};
    const float reference_force[3] = {0.0f, 0.0f, -9.80665f};
    float translated[2][3];

    const float *const arms[2] = {
        imu_dm_fc01_midpoint_to_bmi_m,
        imu_dm_fc01_midpoint_to_icm_m,
    };
    for (size_t sensor = 0U; sensor < 2U; ++sensor) {
        /* f_sensor = f_reference + E[w x (w x r)]. */
        const float at_sensor[3] = {
            reference_force[0],
            reference_force[1] -
                (second_moment[0][0] * arms[sensor][1]),
            reference_force[2],
        };
        TEST_EXPECT(imu_translate_mean_specific_force(at_sensor,
                                                      &second_moment[0][0],
                                                      zero_alpha,
                                                      arms[sensor],
                                                      translated[sensor]));
        for (size_t axis = 0U; axis < 3U; ++axis)
            TEST_EXPECT(fabsf(translated[sensor][axis] -
                              reference_force[axis]) < 1.0e-6f);
    }
    TEST_EXPECT(fabsf(translated[0][1] - translated[1][1]) < 1.0e-7f);
}

int main(void)
{
    test_board_geometry();
    test_rigid_body_translation();
    test_angular_accel_filter();
    test_zero_mean_vibration_translation();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "imu_geometry: %u test failure(s)\n", failure_count);
        return 1;
    }

    (void)puts("imu_geometry: all tests passed");
    return 0;
}
