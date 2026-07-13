#include "imu_calibration.h"

#include <math.h>
#include <stdio.h>

static unsigned int failure_count;

#define TEST_EXPECT(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",           \
                          __FILE__, __LINE__, #condition);                        \
            failure_count++;                                                      \
        }                                                                         \
    } while (0)

static void expect_near(float actual, float expected)
{
    TEST_EXPECT(fabsf(actual - expected) < 1.0e-6f);
}

static void test_identity_default(void)
{
    imu_calibration_reset_all();
    imu_calibration_t calibration;
    TEST_EXPECT(imu_calibration_get(IMU_SOURCE_BMI088, &calibration));
    TEST_EXPECT(imu_calibration_validate(&calibration));
    TEST_EXPECT(!imu_calibration_has_custom(IMU_SOURCE_BMI088));

    float accel[3] = {1.0f, -2.0f, 3.0f};
    float gyro[3] = {-0.1f, 0.2f, -0.3f};
    TEST_EXPECT(imu_calibration_apply_accel(IMU_SOURCE_BMI088, NAN, accel));
    TEST_EXPECT(imu_calibration_apply_gyro(IMU_SOURCE_BMI088, 25.0f, gyro));
    expect_near(accel[0], 1.0f);
    expect_near(accel[1], -2.0f);
    expect_near(accel[2], 3.0f);
    expect_near(gyro[0], -0.1f);
    expect_near(gyro[1], 0.2f);
    expect_near(gyro[2], -0.3f);
}

static void test_matrix_bias_and_passive_temperature_compensation(void)
{
    imu_calibration_t calibration;
    imu_calibration_default(&calibration);
    calibration.gyro_correction_matrix[0][0] = 1.1f;
    calibration.gyro_correction_matrix[1][1] = 0.9f;
    calibration.gyro_bias_polynomial_rad_s[0][0] = 0.1f;
    calibration.gyro_bias_polynomial_rad_s[0][1] = 0.01f;
    calibration.accel_bias_polynomial_mps2[2][0] = 0.5f;
    TEST_EXPECT(imu_calibration_set(IMU_SOURCE_ICM45686, &calibration));
    TEST_EXPECT(imu_calibration_has_custom(IMU_SOURCE_ICM45686));

    float gyro[3] = {1.2f, 2.0f, 3.0f};
    float accel[3] = {1.0f, 2.0f, -9.0f};
    TEST_EXPECT(imu_calibration_apply_gyro(IMU_SOURCE_ICM45686, 35.0f, gyro));
    TEST_EXPECT(imu_calibration_apply_accel(IMU_SOURCE_ICM45686, 35.0f, accel));
    expect_near(gyro[0], 1.1f);
    expect_near(gyro[1], 1.8f);
    expect_near(gyro[2], 3.0f);
    expect_near(accel[2], -9.5f);
}

static void test_rejects_singular_or_reflected_matrix(void)
{
    imu_calibration_t calibration;
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
    test_matrix_bias_and_passive_temperature_compensation();
    test_rejects_singular_or_reflected_matrix();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "imu_calibration: %u test failure(s)\n",
                      failure_count);
        return 1;
    }
    (void)puts("imu_calibration: all tests passed");
    return 0;
}
