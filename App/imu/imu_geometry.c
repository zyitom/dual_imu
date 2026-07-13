#include "imu_geometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_GEOMETRY_TWO_PI (6.28318530717958647692f)
#define IMU_GEOMETRY_MIN_DT_S (1.0e-5f)
#define IMU_GEOMETRY_MAX_DT_S (0.1f)

const float imu_dm_fc01_midpoint_to_bmi_m[3] = {
    0.0f,
    -0.5f * IMU_DM_FC01_CENTER_DISTANCE_M,
    0.0f,
};

const float imu_dm_fc01_midpoint_to_icm_m[3] = {
    0.0f,
    0.5f * IMU_DM_FC01_CENTER_DISTANCE_M,
    0.0f,
};

const float imu_dm_fc01_reference_to_bmi_m[3] = {
    IMU_REFERENCE_TO_MIDPOINT_X_M,
    IMU_REFERENCE_TO_MIDPOINT_Y_M -
        (0.5f * IMU_DM_FC01_CENTER_DISTANCE_M),
    IMU_REFERENCE_TO_MIDPOINT_Z_M,
};

const float imu_dm_fc01_reference_to_icm_m[3] = {
    IMU_REFERENCE_TO_MIDPOINT_X_M,
    IMU_REFERENCE_TO_MIDPOINT_Y_M +
        (0.5f * IMU_DM_FC01_CENTER_DISTANCE_M),
    IMU_REFERENCE_TO_MIDPOINT_Z_M,
};

static bool vector_is_finite(const float vector[3])
{
    return (vector != NULL) && isfinite(vector[0]) && isfinite(vector[1]) &&
           isfinite(vector[2]);
}

static bool matrix_is_finite(const float *matrix)
{
    if (matrix == NULL)
        return false;
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            if (!isfinite(matrix[(row * 3U) + column]))
                return false;
        }
    }
    return true;
}

static float clamp_symmetric(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static void cross_product(const float lhs[3], const float rhs[3], float output[3])
{
    output[0] = (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]);
    output[1] = (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]);
    output[2] = (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]);
}

void imu_angular_accel_estimator_init(imu_angular_accel_estimator_t *estimator,
                                      float cutoff_hz,
                                      float limit_rad_s2)
{
    if (estimator == NULL)
        return;

    memset(estimator, 0, sizeof(*estimator));
    estimator->cutoff_hz = (isfinite(cutoff_hz) && (cutoff_hz > 0.0f))
                               ? cutoff_hz
                               : 20.0f;
    estimator->limit_rad_s2 = (isfinite(limit_rad_s2) && (limit_rad_s2 > 0.0f))
                                  ? limit_rad_s2
                                  : 5000.0f;
}

bool imu_angular_accel_estimator_update(imu_angular_accel_estimator_t *estimator,
                                       const float gyro_rad_s[3],
                                       float dt_s,
                                       float angular_accel_rad_s2[3])
{
    if ((estimator == NULL) || !vector_is_finite(gyro_rad_s) ||
        (angular_accel_rad_s2 == NULL) || !isfinite(dt_s) ||
        (dt_s < IMU_GEOMETRY_MIN_DT_S) || (dt_s > IMU_GEOMETRY_MAX_DT_S) ||
        !isfinite(estimator->cutoff_hz) || (estimator->cutoff_hz <= 0.0f) ||
        !isfinite(estimator->limit_rad_s2) ||
        (estimator->limit_rad_s2 <= 0.0f))
        return false;

    if (!estimator->initialized) {
        memcpy(estimator->previous_gyro_rad_s, gyro_rad_s,
               sizeof(estimator->previous_gyro_rad_s));
        memset(estimator->angular_accel_rad_s2, 0,
               sizeof(estimator->angular_accel_rad_s2));
        memset(angular_accel_rad_s2, 0, 3U * sizeof(float));
        estimator->initialized = true;
        return true;
    }

    const float gain = 1.0f - expf(-IMU_GEOMETRY_TWO_PI *
                                   estimator->cutoff_hz * dt_s);
    if (!isfinite(gain) || (gain <= 0.0f) || (gain > 1.0f))
        return false;

    for (size_t axis = 0U; axis < 3U; ++axis) {
        const float derivative = clamp_symmetric(
            (gyro_rad_s[axis] - estimator->previous_gyro_rad_s[axis]) / dt_s,
            estimator->limit_rad_s2);
        estimator->angular_accel_rad_s2[axis] +=
            gain * (derivative - estimator->angular_accel_rad_s2[axis]);
        estimator->previous_gyro_rad_s[axis] = gyro_rad_s[axis];
        angular_accel_rad_s2[axis] = estimator->angular_accel_rad_s2[axis];
    }

    return vector_is_finite(angular_accel_rad_s2);
}

bool imu_translate_specific_force(const float sensor_specific_force_mps2[3],
                                  const float angular_rate_rad_s[3],
                                  const float angular_accel_rad_s2[3],
                                  const float reference_to_sensor_m[3],
                                  float reference_specific_force_mps2[3])
{
    if (!vector_is_finite(sensor_specific_force_mps2) ||
        !vector_is_finite(angular_rate_rad_s) ||
        !vector_is_finite(angular_accel_rad_s2) ||
        !vector_is_finite(reference_to_sensor_m) ||
        (reference_specific_force_mps2 == NULL))
        return false;

    float second_moment[3][3];
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            second_moment[row][column] =
                angular_rate_rad_s[row] * angular_rate_rad_s[column];
        }
    }
    return imu_translate_mean_specific_force(sensor_specific_force_mps2,
                                             &second_moment[0][0],
                                             angular_accel_rad_s2,
                                             reference_to_sensor_m,
                                             reference_specific_force_mps2);
}

bool imu_translate_mean_specific_force(
    const float sensor_specific_force_mps2[3],
    const float *angular_rate_second_moment_rad2_s2,
    const float mean_angular_accel_rad_s2[3],
    const float reference_to_sensor_m[3],
    float reference_specific_force_mps2[3])
{
    if (!vector_is_finite(sensor_specific_force_mps2) ||
        !matrix_is_finite(angular_rate_second_moment_rad2_s2) ||
        !vector_is_finite(mean_angular_accel_rad_s2) ||
        !vector_is_finite(reference_to_sensor_m) ||
        (reference_specific_force_mps2 == NULL))
        return false;

    float angular_term[3];
    float centripetal_term[3];
    float translated[3];

    cross_product(mean_angular_accel_rad_s2, reference_to_sensor_m, angular_term);
    const float trace = angular_rate_second_moment_rad2_s2[0] +
                        angular_rate_second_moment_rad2_s2[4] +
                        angular_rate_second_moment_rad2_s2[8];
    for (size_t row = 0U; row < 3U; ++row) {
        centripetal_term[row] = 0.0f;
        for (size_t column = 0U; column < 3U; ++column) {
            float coefficient =
                angular_rate_second_moment_rad2_s2[(row * 3U) + column];
            if (row == column)
                coefficient -= trace;
            centripetal_term[row] +=
                coefficient * reference_to_sensor_m[column];
        }
    }

    for (size_t axis = 0U; axis < 3U; ++axis) {
        translated[axis] = sensor_specific_force_mps2[axis] -
                           angular_term[axis] - centripetal_term[axis];
    }

    if (!vector_is_finite(translated))
        return false;

    memcpy(reference_specific_force_mps2, translated, sizeof(translated));
    return true;
}
