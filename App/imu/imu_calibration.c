#include "imu_calibration.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_CALIBRATION_MIN_MATRIX_ROW_NORM (0.5f)
#define IMU_CALIBRATION_MAX_MATRIX_ROW_NORM (1.5f)
#define IMU_CALIBRATION_MIN_ABS_DETERMINANT  (0.05f)
#define IMU_CALIBRATION_MAX_ACCEL_BIAS_MPS2  (49.03325f)
#define IMU_CALIBRATION_MAX_GYRO_BIAS_RAD_S  (5.0f)

static imu_calibration_t s_calibration[IMU_SOURCE_COUNT];
static bool s_custom[IMU_SOURCE_COUNT];
static bool s_initialized;

static bool source_is_valid(imu_source_t source)
{
    return source < IMU_SOURCE_COUNT;
}

static bool vector_is_finite(const float vector[3])
{
    return (vector != NULL) && isfinite(vector[0]) && isfinite(vector[1]) &&
           isfinite(vector[2]);
}

static float clamp_temperature(const imu_calibration_t *calibration,
                               float temperature_c)
{
    if (!isfinite(temperature_c))
        return calibration->reference_temperature_c;
    if (temperature_c < calibration->minimum_temperature_c)
        return calibration->minimum_temperature_c;
    if (temperature_c > calibration->maximum_temperature_c)
        return calibration->maximum_temperature_c;
    return temperature_c;
}

static float evaluate_polynomial(const float coefficients[4], float delta_t)
{
    return coefficients[0] +
           (delta_t * (coefficients[1] +
                       (delta_t * (coefficients[2] +
                                   (delta_t * coefficients[3])))));
}

static bool polynomial_bias_is_bounded(const float polynomial[3][4],
                                       const imu_calibration_t *calibration,
                                       float maximum_norm)
{
    const float temperatures[3] = {
        calibration->minimum_temperature_c,
        calibration->reference_temperature_c,
        calibration->maximum_temperature_c,
    };
    for (size_t point = 0U; point < 3U; ++point) {
        float bias[3];
        const float delta_t =
            temperatures[point] - calibration->reference_temperature_c;
        for (size_t axis = 0U; axis < 3U; ++axis) {
            for (size_t coefficient = 0U; coefficient < 4U; ++coefficient) {
                if (!isfinite(polynomial[axis][coefficient]))
                    return false;
            }
            bias[axis] = evaluate_polynomial(polynomial[axis], delta_t);
        }
        const float norm = sqrtf((bias[0] * bias[0]) +
                                 (bias[1] * bias[1]) +
                                 (bias[2] * bias[2]));
        if (!isfinite(norm) || (norm > maximum_norm))
            return false;
    }
    return true;
}

static bool matrix_is_valid(const float matrix[3][3])
{
    for (size_t row = 0U; row < 3U; ++row) {
        float norm_sq = 0.0f;
        for (size_t column = 0U; column < 3U; ++column) {
            if (!isfinite(matrix[row][column]))
                return false;
            norm_sq += matrix[row][column] * matrix[row][column];
        }
        const float norm = sqrtf(norm_sq);
        if ((norm < IMU_CALIBRATION_MIN_MATRIX_ROW_NORM) ||
            (norm > IMU_CALIBRATION_MAX_MATRIX_ROW_NORM))
            return false;
    }

    const float determinant =
        (matrix[0][0] * ((matrix[1][1] * matrix[2][2]) -
                         (matrix[1][2] * matrix[2][1]))) -
        (matrix[0][1] * ((matrix[1][0] * matrix[2][2]) -
                         (matrix[1][2] * matrix[2][0]))) +
        (matrix[0][2] * ((matrix[1][0] * matrix[2][1]) -
                         (matrix[1][1] * matrix[2][0])));
    return isfinite(determinant) &&
           (determinant >= IMU_CALIBRATION_MIN_ABS_DETERMINANT);
}

void imu_calibration_default(imu_calibration_t *calibration)
{
    if (calibration == NULL)
        return;
    memset(calibration, 0, sizeof(*calibration));
    for (size_t axis = 0U; axis < 3U; ++axis) {
        calibration->accel_correction_matrix[axis][axis] = 1.0f;
        calibration->gyro_correction_matrix[axis][axis] = 1.0f;
    }
    calibration->reference_temperature_c = 25.0f;
    calibration->minimum_temperature_c = -40.0f;
    calibration->maximum_temperature_c = 100.0f;
}

void imu_calibration_reset_all(void)
{
    for (size_t source = 0U; source < IMU_SOURCE_COUNT; ++source) {
        imu_calibration_default(&s_calibration[source]);
        s_custom[source] = false;
    }
    s_initialized = true;
}

void imu_calibration_initialize(void)
{
    if (!s_initialized)
        imu_calibration_reset_all();
}

bool imu_calibration_validate(const imu_calibration_t *calibration)
{
    if ((calibration == NULL) ||
        !isfinite(calibration->reference_temperature_c) ||
        !isfinite(calibration->minimum_temperature_c) ||
        !isfinite(calibration->maximum_temperature_c) ||
        (calibration->minimum_temperature_c >=
         calibration->maximum_temperature_c) ||
        (calibration->reference_temperature_c <
         calibration->minimum_temperature_c) ||
        (calibration->reference_temperature_c >
         calibration->maximum_temperature_c) ||
        !matrix_is_valid(calibration->accel_correction_matrix) ||
        !matrix_is_valid(calibration->gyro_correction_matrix) ||
        !polynomial_bias_is_bounded(calibration->accel_bias_polynomial_mps2,
                                    calibration,
                                    IMU_CALIBRATION_MAX_ACCEL_BIAS_MPS2) ||
        !polynomial_bias_is_bounded(calibration->gyro_bias_polynomial_rad_s,
                                    calibration,
                                    IMU_CALIBRATION_MAX_GYRO_BIAS_RAD_S))
        return false;
    return true;
}

bool imu_calibration_set(imu_source_t source,
                         const imu_calibration_t *calibration)
{
    imu_calibration_initialize();
    if (!source_is_valid(source) || !imu_calibration_validate(calibration))
        return false;
    s_calibration[source] = *calibration;
    s_custom[source] = true;
    return true;
}

bool imu_calibration_get(imu_source_t source,
                         imu_calibration_t *calibration)
{
    imu_calibration_initialize();
    if (!source_is_valid(source) || (calibration == NULL))
        return false;
    *calibration = s_calibration[source];
    return true;
}

bool imu_calibration_has_custom(imu_source_t source)
{
    imu_calibration_initialize();
    return source_is_valid(source) && s_custom[source];
}

static bool apply_calibration(const float *matrix,
                              const float *polynomial,
                              const imu_calibration_t *calibration,
                              float temperature_c,
                              float vector[3])
{
    if (!vector_is_finite(vector))
        return false;
    const float bounded_temperature =
        clamp_temperature(calibration, temperature_c);
    const float delta_t = bounded_temperature -
                          calibration->reference_temperature_c;
    float unbiased[3];
    float corrected[3] = {0.0f};
    for (size_t axis = 0U; axis < 3U; ++axis) {
        const float bias = evaluate_polynomial(&polynomial[axis * 4U], delta_t);
        unbiased[axis] = vector[axis] - bias;
    }
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column)
            corrected[row] += matrix[(row * 3U) + column] * unbiased[column];
    }
    if (!vector_is_finite(corrected))
        return false;
    memcpy(vector, corrected, sizeof(corrected));
    return true;
}

bool imu_calibration_apply_accel(imu_source_t source,
                                 float temperature_c,
                                 float accel_mps2[3])
{
    imu_calibration_initialize();
    if (!source_is_valid(source))
        return false;
    return apply_calibration(&s_calibration[source]
                                  .accel_correction_matrix[0][0],
                             &s_calibration[source]
                                  .accel_bias_polynomial_mps2[0][0],
                             &s_calibration[source], temperature_c, accel_mps2);
}

bool imu_calibration_apply_gyro(imu_source_t source,
                                float temperature_c,
                                float gyro_rad_s[3])
{
    imu_calibration_initialize();
    if (!source_is_valid(source))
        return false;
    return apply_calibration(&s_calibration[source]
                                  .gyro_correction_matrix[0][0],
                             &s_calibration[source]
                                  .gyro_bias_polynomial_rad_s[0][0],
                             &s_calibration[source], temperature_c, gyro_rad_s);
}
