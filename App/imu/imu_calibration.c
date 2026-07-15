#include "imu_calibration.h"

#include <float.h>
#include <limits.h>
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
static imu_calibration_diagnostics_t s_diagnostics[IMU_SOURCE_COUNT];
static bool s_initialized;

#if (IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1) || \
    (IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1)
static bool
    s_sample_seen[IMU_SOURCE_COUNT][IMU_CALIBRATION_STREAM_COUNT];
#endif

#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1
typedef struct
{
    float accepted_temperature_c;
    uint64_t accepted_temperature_timestamp_us;
    float applied_correction[3];
    uint64_t correction_timestamp_us;
    imu_calibration_temperature_use_t last_temperature_use;
    bool accepted_temperature_valid;
    bool applied_correction_valid;
    bool correction_timestamp_valid;
    bool last_temperature_use_valid;
} imu_calibration_stream_state_t;

static bool s_temperature_compensation_enabled[IMU_SOURCE_COUNT];
static imu_calibration_stream_state_t
    s_stream_state[IMU_SOURCE_COUNT][IMU_CALIBRATION_STREAM_COUNT];
#endif

#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1
static bool s_gyro_g_sensitivity_enabled[IMU_SOURCE_COUNT];
static bool s_gyro_g_sensitivity_nonzero[IMU_SOURCE_COUNT];
#endif

static bool source_is_valid(imu_source_t source)
{
    return (unsigned int)source < (unsigned int)IMU_SOURCE_COUNT;
}

static bool vector_is_finite(const float vector[3])
{
    return (vector != NULL) && isfinite(vector[0]) && isfinite(vector[1]) &&
           isfinite(vector[2]);
}

static void increment_saturating(uint32_t *value)
{
    if (*value != UINT32_MAX)
        (*value)++;
}

#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1
static float evaluate_polynomial(const float coefficients[4], float delta_t)
{
    return coefficients[0] +
           (delta_t * (coefficients[1] +
                       (delta_t * (coefficients[2] +
                                   (delta_t * coefficients[3])))));
}
#endif

static double evaluate_polynomial_double(const float coefficients[4],
                                         double delta_t)
{
    return (double)coefficients[0] +
           (delta_t * ((double)coefficients[1] +
                       (delta_t * ((double)coefficients[2] +
                                   (delta_t * (double)coefficients[3])))));
}

static bool update_polynomial_maximum(const float coefficients[4],
                                      double delta_t,
                                      double *maximum_absolute_value)
{
    if ((coefficients == NULL) || (maximum_absolute_value == NULL) ||
        !isfinite(delta_t))
        return false;

    const double value = evaluate_polynomial_double(coefficients, delta_t);
    const double absolute_value = fabs(value);
    if (!isfinite(absolute_value))
        return false;
    if (absolute_value > *maximum_absolute_value)
        *maximum_absolute_value = absolute_value;
    return true;
}

static bool polynomial_axis_maximum(const float coefficients[4],
                                    double minimum_delta_t,
                                    double maximum_delta_t,
                                    double *maximum_absolute_value)
{
    if ((coefficients == NULL) || (maximum_absolute_value == NULL))
        return false;
    for (size_t coefficient = 0U; coefficient < 4U; ++coefficient) {
        if (!isfinite(coefficients[coefficient]))
            return false;
    }

    double maximum = 0.0;
    if (!update_polynomial_maximum(coefficients, minimum_delta_t, &maximum) ||
        !update_polynomial_maximum(coefficients, maximum_delta_t, &maximum) ||
        !update_polynomial_maximum(coefficients, 0.0, &maximum))
        return false;

    const double derivative_quadratic = 3.0 * (double)coefficients[3];
    const double derivative_linear = 2.0 * (double)coefficients[2];
    const double derivative_constant = (double)coefficients[1];
    if (derivative_quadratic == 0.0) {
        if (derivative_linear != 0.0) {
            const double root = -derivative_constant / derivative_linear;
            if ((root > minimum_delta_t) && (root < maximum_delta_t) &&
                !update_polynomial_maximum(coefficients, root, &maximum))
                return false;
        }
    } else {
        const double discriminant =
            (derivative_linear * derivative_linear) -
            (4.0 * derivative_quadratic * derivative_constant);
        if (discriminant >= 0.0) {
            const double square_root = sqrt(discriminant);
            double roots[2];
            size_t root_count;
            if (square_root == 0.0) {
                roots[0] = -derivative_linear /
                           (2.0 * derivative_quadratic);
                root_count = 1U;
            } else {
                const double q = -0.5 *
                    (derivative_linear +
                     copysign(square_root, derivative_linear));
                roots[0] = q / derivative_quadratic;
                roots[1] = derivative_constant / q;
                root_count = 2U;
            }
            for (size_t root_index = 0U; root_index < root_count;
                 ++root_index) {
                if ((roots[root_index] > minimum_delta_t) &&
                    (roots[root_index] < maximum_delta_t) &&
                    !update_polynomial_maximum(coefficients,
                                               roots[root_index], &maximum))
                    return false;
            }
        }
    }

    *maximum_absolute_value = maximum;
    return true;
}

static bool polynomial_bias_is_bounded(const float polynomial[3][4],
                                       const imu_calibration_t *calibration,
                                       float maximum_norm)
{
    const double minimum_delta_t =
        (double)calibration->minimum_temperature_c -
        (double)calibration->reference_temperature_c;
    const double maximum_delta_t =
        (double)calibration->maximum_temperature_c -
        (double)calibration->reference_temperature_c;
    double maximum_norm_squared = 0.0;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        double maximum_absolute_value;
        if (!polynomial_axis_maximum(polynomial[axis], minimum_delta_t,
                                     maximum_delta_t,
                                     &maximum_absolute_value))
            return false;
        maximum_norm_squared +=
            maximum_absolute_value * maximum_absolute_value;
    }
    return isfinite(maximum_norm_squared) &&
           (maximum_norm_squared <=
            ((double)maximum_norm * (double)maximum_norm));
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

static bool g_sensitivity_matrix_is_valid(const float matrix[3][3])
{
    double frobenius_squared = 0.0;
    for (size_t row = 0U; row < 3U; ++row) {
        double row_squared = 0.0;
        for (size_t column = 0U; column < 3U; ++column) {
            if (!isfinite(matrix[row][column]))
                return false;
            const double value = (double)matrix[row][column];
            row_squared += value * value;
        }
        if (!isfinite(row_squared) ||
            (row_squared >
             ((double)IMU_CALIBRATION_MAX_G_SENSITIVITY_ROW_NORM_RAD_S_PER_MPS2 *
              (double)IMU_CALIBRATION_MAX_G_SENSITIVITY_ROW_NORM_RAD_S_PER_MPS2))) {
            return false;
        }
        frobenius_squared += row_squared;
    }
    return isfinite(frobenius_squared) &&
           (frobenius_squared <=
            ((double)IMU_CALIBRATION_MAX_G_SENSITIVITY_FROBENIUS_NORM_RAD_S_PER_MPS2 *
             (double)IMU_CALIBRATION_MAX_G_SENSITIVITY_FROBENIUS_NORM_RAD_S_PER_MPS2));
}

#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1
static bool g_sensitivity_matrix_is_nonzero(const float matrix[3][3])
{
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            if (matrix[row][column] != 0.0f)
                return true;
        }
    }
    return false;
}
#endif

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
    calibration->maximum_temperature_rate_c_per_s = 5.0f;
    calibration->maximum_accel_correction_slew_mps3 =
        IMU_CALIBRATION_DEFAULT_ACCEL_CORRECTION_SLEW_MPS3;
    calibration->maximum_gyro_correction_slew_rad_s2 =
        IMU_CALIBRATION_DEFAULT_GYRO_CORRECTION_SLEW_RAD_S2;
    calibration->maximum_g_sensitivity_accel_age_us =
        IMU_CALIBRATION_DEFAULT_G_SENSITIVITY_ACCEL_MAX_AGE_US;
}

void imu_calibration_reset_all(void)
{
    for (size_t source = 0U; source < IMU_SOURCE_COUNT; ++source) {
        imu_calibration_default(&s_calibration[source]);
        s_custom[source] = false;
        memset(&s_diagnostics[source], 0, sizeof(s_diagnostics[source]));
#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1
        s_temperature_compensation_enabled[source] = false;
        memset(&s_stream_state[source], 0, sizeof(s_stream_state[source]));
#endif
#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1
        s_gyro_g_sensitivity_enabled[source] = false;
        s_gyro_g_sensitivity_nonzero[source] = false;
#endif
    }
#if (IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1) || \
    (IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1)
    memset(s_sample_seen, 0, sizeof(s_sample_seen));
#endif
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
        !isfinite(calibration->maximum_temperature_rate_c_per_s) ||
        !isfinite(calibration->maximum_accel_correction_slew_mps3) ||
        !isfinite(calibration->maximum_gyro_correction_slew_rad_s2) ||
        (calibration->maximum_temperature_rate_c_per_s <= 0.0f) ||
        (calibration->maximum_accel_correction_slew_mps3 <= 0.0f) ||
        (calibration->maximum_accel_correction_slew_mps3 >
         IMU_CALIBRATION_MAX_ACCEL_CORRECTION_SLEW_MPS3) ||
        (calibration->maximum_gyro_correction_slew_rad_s2 <= 0.0f) ||
        (calibration->maximum_gyro_correction_slew_rad_s2 >
         IMU_CALIBRATION_MAX_GYRO_CORRECTION_SLEW_RAD_S2) ||
        (calibration->maximum_g_sensitivity_accel_age_us == 0U) ||
        (calibration->maximum_g_sensitivity_accel_age_us >
         IMU_CALIBRATION_MAX_G_SENSITIVITY_ACCEL_MAX_AGE_US) ||
        (calibration->minimum_temperature_c >=
         calibration->maximum_temperature_c) ||
        (calibration->reference_temperature_c <
         calibration->minimum_temperature_c) ||
        (calibration->reference_temperature_c >
         calibration->maximum_temperature_c) ||
        !matrix_is_valid(calibration->accel_correction_matrix) ||
        !matrix_is_valid(calibration->gyro_correction_matrix) ||
        !g_sensitivity_matrix_is_valid(
            calibration->gyro_g_sensitivity_rad_s_per_mps2) ||
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
#if (IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1) || \
    (IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1)
    for (size_t stream = 0U; stream < IMU_CALIBRATION_STREAM_COUNT; ++stream) {
        if (s_sample_seen[source][stream])
            return false;
    }
#endif
    s_calibration[source] = *calibration;
    s_custom[source] = true;
    memset(&s_diagnostics[source], 0, sizeof(s_diagnostics[source]));
#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1
    s_temperature_compensation_enabled[source] = false;
    memset(&s_stream_state[source], 0, sizeof(s_stream_state[source]));
#endif
#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1
    s_gyro_g_sensitivity_enabled[source] = false;
    s_gyro_g_sensitivity_nonzero[source] =
        g_sensitivity_matrix_is_nonzero(
            calibration->gyro_g_sensitivity_rad_s_per_mps2);
#endif
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

bool imu_calibration_set_temperature_compensation_enabled(imu_source_t source,
                                                          bool enabled)
{
    imu_calibration_initialize();
    if (!source_is_valid(source))
        return false;
#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 0
    if (enabled)
        return false;
    s_diagnostics[source].temperature_compensation_enabled = false;
    return true;
#else
    if (enabled) {
        if (!s_custom[source])
            return false;
        for (size_t stream = 0U; stream < IMU_CALIBRATION_STREAM_COUNT;
             ++stream) {
            if (s_sample_seen[source][stream])
                return false;
        }
    }
    s_temperature_compensation_enabled[source] = enabled;
    for (size_t stream = 0U; stream < IMU_CALIBRATION_STREAM_COUNT; ++stream)
        s_stream_state[source][stream].accepted_temperature_valid = false;
    s_diagnostics[source].temperature_compensation_enabled = enabled;
    return true;
#endif
}

bool imu_calibration_temperature_compensation_enabled(imu_source_t source)
{
    imu_calibration_initialize();
#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 0
    (void)source;
    return false;
#else
    return source_is_valid(source) &&
           s_temperature_compensation_enabled[source];
#endif
}

bool imu_calibration_set_gyro_g_sensitivity_enabled(imu_source_t source,
                                                    bool enabled)
{
    imu_calibration_initialize();
    if (!source_is_valid(source))
        return false;
#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 0
    if (enabled)
        return false;
    s_diagnostics[source].gyro_g_sensitivity_enabled = false;
    return true;
#else
    if (enabled) {
        if (!s_custom[source])
            return false;
        for (size_t stream = 0U; stream < IMU_CALIBRATION_STREAM_COUNT;
             ++stream) {
            if (s_sample_seen[source][stream])
                return false;
        }
    }
    s_gyro_g_sensitivity_enabled[source] = enabled;
    s_diagnostics[source].gyro_g_sensitivity_enabled = enabled;
    return true;
#endif
}

bool imu_calibration_gyro_g_sensitivity_enabled(imu_source_t source)
{
    imu_calibration_initialize();
#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 0
    (void)source;
    return false;
#else
    return source_is_valid(source) &&
           s_gyro_g_sensitivity_enabled[source];
#endif
}

bool imu_calibration_get_diagnostics(
    imu_source_t source,
    imu_calibration_diagnostics_t *diagnostics)
{
    imu_calibration_initialize();
    if (!source_is_valid(source) || (diagnostics == NULL))
        return false;
    *diagnostics = s_diagnostics[source];
#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 0
    diagnostics->temperature_compensation_enabled = false;
#else
    diagnostics->temperature_compensation_enabled =
        s_temperature_compensation_enabled[source];
#endif
#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 0
    diagnostics->gyro_g_sensitivity_enabled = false;
#else
    diagnostics->gyro_g_sensitivity_enabled =
        s_gyro_g_sensitivity_enabled[source];
#endif
    return true;
}

#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1
static imu_calibration_temperature_use_t select_temperature_use(
    imu_calibration_stream_state_t *stream_state,
    const imu_calibration_t *calibration,
    bool compensation_enabled,
    uint64_t sample_timestamp_us,
    float temperature_c,
    uint64_t temperature_timestamp_us,
    bool temperature_valid,
    float *delta_t)
{
    *delta_t = 0.0f;
    if (!compensation_enabled)
        return IMU_CALIBRATION_TEMPERATURE_C0_DISABLED;
    if (!temperature_valid || !isfinite(temperature_c) ||
        (sample_timestamp_us == 0U) || (temperature_timestamp_us == 0U))
        return IMU_CALIBRATION_TEMPERATURE_C0_INVALID;
    if (temperature_timestamp_us > sample_timestamp_us)
        return IMU_CALIBRATION_TEMPERATURE_C0_NONCAUSAL;
    if ((sample_timestamp_us - temperature_timestamp_us) >
        IMU_CALIBRATION_TEMPERATURE_MAX_AGE_US)
        return IMU_CALIBRATION_TEMPERATURE_C0_STALE;

    float bounded_temperature = temperature_c;
    imu_calibration_temperature_use_t use =
        IMU_CALIBRATION_TEMPERATURE_FULL;
    if (bounded_temperature < calibration->minimum_temperature_c) {
        bounded_temperature = calibration->minimum_temperature_c;
        use = IMU_CALIBRATION_TEMPERATURE_FULL_CLAMPED;
    } else if (bounded_temperature > calibration->maximum_temperature_c) {
        bounded_temperature = calibration->maximum_temperature_c;
        use = IMU_CALIBRATION_TEMPERATURE_FULL_CLAMPED;
    }

    if (stream_state->accepted_temperature_valid) {
        const uint64_t previous_timestamp_us =
            stream_state->accepted_temperature_timestamp_us;
        if (temperature_timestamp_us < previous_timestamp_us)
            return IMU_CALIBRATION_TEMPERATURE_C0_NONCAUSAL;
        if (temperature_timestamp_us == previous_timestamp_us) {
            if (bounded_temperature != stream_state->accepted_temperature_c)
                return IMU_CALIBRATION_TEMPERATURE_C0_RATE;
        } else {
            const double dt_s =
                (double)(temperature_timestamp_us - previous_timestamp_us) *
                1.0e-6;
            const double rate =
                fabs((double)bounded_temperature -
                     (double)stream_state->accepted_temperature_c) /
                dt_s;
            if (!isfinite(rate) ||
                (rate >
                 (double)calibration->maximum_temperature_rate_c_per_s)) {
                return IMU_CALIBRATION_TEMPERATURE_C0_RATE;
            }
        }
    }

    const double delta = (double)bounded_temperature -
                         (double)calibration->reference_temperature_c;
    if (!isfinite(delta) || (fabs(delta) > FLT_MAX))
        return IMU_CALIBRATION_TEMPERATURE_C0_INVALID;

    stream_state->accepted_temperature_c = bounded_temperature;
    stream_state->accepted_temperature_timestamp_us =
        temperature_timestamp_us;
    stream_state->accepted_temperature_valid = true;
    *delta_t = (float)delta;
    return use;
}
#endif

static void record_temperature_use(
    imu_calibration_diagnostics_t *diagnostics,
    imu_calibration_stream_t stream,
    imu_calibration_temperature_use_t use)
{
    diagnostics->last_temperature_use = use;
    diagnostics->last_temperature_use_by_stream[stream] = use;
    switch (use) {
    case IMU_CALIBRATION_TEMPERATURE_FULL:
        increment_saturating(&diagnostics->full_compensation_count);
        break;
    case IMU_CALIBRATION_TEMPERATURE_FULL_CLAMPED:
        increment_saturating(&diagnostics->full_compensation_count);
        increment_saturating(&diagnostics->clamped_compensation_count);
        break;
    case IMU_CALIBRATION_TEMPERATURE_C0_INVALID:
        increment_saturating(&diagnostics->c0_invalid_count);
        break;
    case IMU_CALIBRATION_TEMPERATURE_C0_STALE:
        increment_saturating(&diagnostics->c0_stale_count);
        break;
    case IMU_CALIBRATION_TEMPERATURE_C0_NONCAUSAL:
        increment_saturating(&diagnostics->c0_noncausal_count);
        break;
    case IMU_CALIBRATION_TEMPERATURE_C0_RATE:
        increment_saturating(&diagnostics->c0_rate_count);
        break;
    case IMU_CALIBRATION_TEMPERATURE_C0_DISABLED:
    default:
        increment_saturating(&diagnostics->c0_disabled_count);
        break;
    }
}

static void record_g_sensitivity_use(
    imu_calibration_diagnostics_t *diagnostics,
    imu_calibration_g_sensitivity_apply_info_t *info,
    imu_calibration_g_sensitivity_use_t use,
    uint64_t accel_age_us)
{
    diagnostics->last_g_sensitivity_use = use;
    if (info != NULL) {
        info->use = use;
        info->accel_age_us = accel_age_us;
    }
    switch (use) {
    case IMU_CALIBRATION_G_SENSITIVITY_APPLIED:
        increment_saturating(&diagnostics->g_sensitivity_applied_count);
        break;
    case IMU_CALIBRATION_G_SENSITIVITY_ZERO_MATRIX:
        increment_saturating(&diagnostics->g_sensitivity_zero_matrix_count);
        break;
    case IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL:
        increment_saturating(&diagnostics->g_sensitivity_invalid_accel_count);
        break;
    case IMU_CALIBRATION_G_SENSITIVITY_SATURATED_ACCEL:
        increment_saturating(
            &diagnostics->g_sensitivity_saturated_accel_count);
        break;
    case IMU_CALIBRATION_G_SENSITIVITY_SOURCE_MISMATCH:
        increment_saturating(
            &diagnostics->g_sensitivity_source_mismatch_count);
        break;
    case IMU_CALIBRATION_G_SENSITIVITY_NONCAUSAL_ACCEL:
        increment_saturating(
            &diagnostics->g_sensitivity_noncausal_accel_count);
        break;
    case IMU_CALIBRATION_G_SENSITIVITY_STALE_ACCEL:
        increment_saturating(&diagnostics->g_sensitivity_stale_accel_count);
        break;
    case IMU_CALIBRATION_G_SENSITIVITY_DISABLED:
    default:
        increment_saturating(&diagnostics->g_sensitivity_disabled_count);
        break;
    }
}

#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1
static bool transform_vector(const float *matrix,
                             const float input[3],
                             float output[3])
{
    memset(output, 0, sizeof(float) * 3U);
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column)
            output[row] += matrix[(row * 3U) + column] * input[column];
    }
    return vector_is_finite(output);
}

static bool correction_difference_norm(const float target[3],
                                       const float applied[3],
                                       float difference[3],
                                       float *norm)
{
    float norm_squared = 0.0f;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        difference[axis] = target[axis] - applied[axis];
        norm_squared += difference[axis] * difference[axis];
    }
    *norm = sqrtf(norm_squared);
    return vector_is_finite(difference) && isfinite(*norm);
}

static bool slew_correction(imu_calibration_stream_state_t *stream_state,
                            const float c0_correction[3],
                            const float target_correction[3],
                            uint64_t sample_timestamp_us,
                            float maximum_slew_per_s,
                            bool *limited)
{
    *limited = false;
    if (!stream_state->applied_correction_valid) {
        memcpy(stream_state->applied_correction, c0_correction,
               sizeof(stream_state->applied_correction));
        stream_state->applied_correction_valid = true;
    }

    float difference[3];
    float difference_norm;
    if (!correction_difference_norm(target_correction,
                                    stream_state->applied_correction,
                                    difference, &difference_norm)) {
        return false;
    }

    uint64_t step_us = 0U;
    if (stream_state->correction_timestamp_valid &&
        (sample_timestamp_us > stream_state->correction_timestamp_us)) {
        step_us = sample_timestamp_us -
                  stream_state->correction_timestamp_us;
        if (step_us > IMU_CALIBRATION_CORRECTION_SLEW_MAX_STEP_US)
            step_us = IMU_CALIBRATION_CORRECTION_SLEW_MAX_STEP_US;
    }

    const float maximum_step =
        maximum_slew_per_s * ((float)step_us * 1.0e-6f);
    if (difference_norm > maximum_step) {
        *limited = difference_norm > 0.0f;
        if (maximum_step > 0.0f) {
            const float scale = maximum_step / difference_norm;
            for (size_t axis = 0U; axis < 3U; ++axis)
                stream_state->applied_correction[axis] +=
                    difference[axis] * scale;
        }
    } else {
        memcpy(stream_state->applied_correction, target_correction,
               sizeof(stream_state->applied_correction));
    }

    if ((sample_timestamp_us != 0U) &&
        (!stream_state->correction_timestamp_valid ||
         (sample_timestamp_us > stream_state->correction_timestamp_us))) {
        stream_state->correction_timestamp_us = sample_timestamp_us;
        stream_state->correction_timestamp_valid = true;
    }
    return vector_is_finite(stream_state->applied_correction);
}
#endif

static bool apply_calibration(imu_source_t source,
                              imu_calibration_stream_t stream,
                              const float *matrix,
                              const float *polynomial,
                              const imu_calibration_t *calibration,
                              float maximum_correction_slew_per_s,
                              uint64_t sample_timestamp_us,
                              float temperature_c,
                              uint64_t temperature_timestamp_us,
                              bool temperature_valid,
                              float vector[3],
                              imu_calibration_apply_info_t *info,
                              imu_calibration_diagnostics_t *diagnostics)
{
#if (IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1) || \
    (IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1)
    s_sample_seen[source][stream] = true;
#endif
    if (!vector_is_finite(vector)) {
        increment_saturating(&diagnostics->apply_failure_count);
        return false;
    }
#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 0
    (void)source;
    (void)calibration;
    (void)maximum_correction_slew_per_s;
    (void)sample_timestamp_us;
    (void)temperature_c;
    (void)temperature_timestamp_us;
    (void)temperature_valid;
    record_temperature_use(diagnostics, stream,
                           IMU_CALIBRATION_TEMPERATURE_C0_DISABLED);
    diagnostics->correction_slew_active[stream] = false;
    if (info != NULL) {
        info->temperature_use = IMU_CALIBRATION_TEMPERATURE_C0_DISABLED;
        info->correction_slew_limited = false;
    }

    /* Keep the qualified production operation order: M * (raw - c0). */
    float unbiased[3];
    float corrected[3] = {0.0f};
    for (size_t axis = 0U; axis < 3U; ++axis) {
        const float bias = polynomial[axis * 4U];
        unbiased[axis] = vector[axis] - bias;
    }
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column)
            corrected[row] += matrix[(row * 3U) + column] * unbiased[column];
    }
#else
    imu_calibration_stream_state_t *const committed_state =
        &s_stream_state[source][stream];
    imu_calibration_stream_state_t next_state = *committed_state;
    float delta_t;
    const imu_calibration_temperature_use_t temperature_use =
        select_temperature_use(&next_state, calibration,
                               s_temperature_compensation_enabled[source],
                               sample_timestamp_us, temperature_c,
                               temperature_timestamp_us, temperature_valid,
                               &delta_t);
    record_temperature_use(diagnostics, stream, temperature_use);
    if (info != NULL) {
        info->temperature_use = temperature_use;
        info->correction_slew_limited = false;
    }

    float target_bias[3];
    float c0_bias[3];
    for (size_t axis = 0U; axis < 3U; ++axis) {
        target_bias[axis] =
            evaluate_polynomial(&polynomial[axis * 4U], delta_t);
        c0_bias[axis] = polynomial[axis * 4U];
    }
    float transformed[3];
    float target_correction[3];
    float c0_correction[3];
    if (!transform_vector(matrix, vector, transformed) ||
        !transform_vector(matrix, target_bias, target_correction) ||
        !transform_vector(matrix, c0_bias, c0_correction)) {
        increment_saturating(&diagnostics->apply_failure_count);
        return false;
    }

    bool slew_limited;
    if (!slew_correction(&next_state, c0_correction, target_correction,
                         sample_timestamp_us, maximum_correction_slew_per_s,
                         &slew_limited)) {
        increment_saturating(&diagnostics->apply_failure_count);
        return false;
    }

    float corrected[3];
    for (size_t axis = 0U; axis < 3U; ++axis)
        corrected[axis] = transformed[axis] -
                          next_state.applied_correction[axis];
#endif
    if (!vector_is_finite(corrected)) {
        increment_saturating(&diagnostics->apply_failure_count);
        return false;
    }
#if IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1
    if (committed_state->last_temperature_use_valid &&
        (committed_state->last_temperature_use != temperature_use)) {
        increment_saturating(
            &diagnostics->temperature_use_transition_count[stream]);
    }
    next_state.last_temperature_use = temperature_use;
    next_state.last_temperature_use_valid = true;
    *committed_state = next_state;
    diagnostics->correction_slew_active[stream] = slew_limited;
    if (slew_limited) {
        increment_saturating(
            &diagnostics->correction_slew_limited_count[stream]);
    }
    if (info != NULL)
        info->correction_slew_limited = slew_limited;
#endif
    memcpy(vector, corrected, sizeof(corrected));
    return true;
}

bool imu_calibration_apply_accel(imu_source_t source,
                                 uint64_t sample_timestamp_us,
                                 float temperature_c,
                                 uint64_t temperature_timestamp_us,
                                 bool temperature_valid,
                                 float accel_mps2[3],
                                 imu_calibration_apply_info_t *info)
{
    imu_calibration_initialize();
    if (!source_is_valid(source))
        return false;
    return apply_calibration(source,
                             IMU_CALIBRATION_STREAM_ACCEL,
                             &s_calibration[source]
                                  .accel_correction_matrix[0][0],
                             &s_calibration[source]
                                  .accel_bias_polynomial_mps2[0][0],
                             &s_calibration[source],
                             s_calibration[source]
                                  .maximum_accel_correction_slew_mps3,
                             sample_timestamp_us, temperature_c,
                             temperature_timestamp_us, temperature_valid,
                             accel_mps2, info, &s_diagnostics[source]);
}

bool imu_calibration_apply_gyro(imu_source_t source,
                                uint64_t sample_timestamp_us,
                                float temperature_c,
                                uint64_t temperature_timestamp_us,
                                bool temperature_valid,
                                float gyro_rad_s[3],
                                imu_calibration_apply_info_t *info)
{
    imu_calibration_initialize();
    if (!source_is_valid(source))
        return false;
    return apply_calibration(source,
                             IMU_CALIBRATION_STREAM_GYRO,
                             &s_calibration[source]
                                  .gyro_correction_matrix[0][0],
                             &s_calibration[source]
                                  .gyro_bias_polynomial_rad_s[0][0],
                             &s_calibration[source],
                             s_calibration[source]
                                  .maximum_gyro_correction_slew_rad_s2,
                             sample_timestamp_us, temperature_c,
                             temperature_timestamp_us, temperature_valid,
                             gyro_rad_s, info, &s_diagnostics[source]);
}

bool imu_calibration_apply_gyro_g_sensitivity(
    imu_source_t source,
    uint64_t gyro_timestamp_us,
    const imu_calibration_accel_evidence_t *accel_evidence,
    float gyro_rad_s[3],
    imu_calibration_g_sensitivity_apply_info_t *info)
{
    imu_calibration_initialize();
    if (!source_is_valid(source))
        return false;
#if (IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE == 1) || \
    (IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 1)
    s_sample_seen[source][IMU_CALIBRATION_STREAM_GYRO] = true;
#endif
    if (!vector_is_finite(gyro_rad_s))
        return false;

    imu_calibration_diagnostics_t *const diagnostics =
        &s_diagnostics[source];
#if IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE == 0
    (void)gyro_timestamp_us;
    (void)accel_evidence;
    record_g_sensitivity_use(diagnostics, info,
                             IMU_CALIBRATION_G_SENSITIVITY_DISABLED, 0U);
    return true;
#else
    if (!s_gyro_g_sensitivity_enabled[source]) {
        record_g_sensitivity_use(diagnostics, info,
                                 IMU_CALIBRATION_G_SENSITIVITY_DISABLED, 0U);
        return true;
    }
    if (!s_gyro_g_sensitivity_nonzero[source]) {
        record_g_sensitivity_use(
            diagnostics, info, IMU_CALIBRATION_G_SENSITIVITY_ZERO_MATRIX, 0U);
        return true;
    }
    if (accel_evidence == NULL) {
        record_g_sensitivity_use(
            diagnostics, info, IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL,
            0U);
        return false;
    }
    if (accel_evidence->source != source) {
        record_g_sensitivity_use(
            diagnostics, info,
            IMU_CALIBRATION_G_SENSITIVITY_SOURCE_MISMATCH, 0U);
        return false;
    }
    if (accel_evidence->saturated) {
        record_g_sensitivity_use(
            diagnostics, info,
            IMU_CALIBRATION_G_SENSITIVITY_SATURATED_ACCEL, 0U);
        return false;
    }
    if (!accel_evidence->valid || (gyro_timestamp_us == 0U) ||
        (accel_evidence->timestamp_us == 0U) ||
        !vector_is_finite(accel_evidence->accel_body_mps2)) {
        record_g_sensitivity_use(
            diagnostics, info, IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL,
            0U);
        return false;
    }
    if (accel_evidence->timestamp_us > gyro_timestamp_us) {
        record_g_sensitivity_use(
            diagnostics, info,
            IMU_CALIBRATION_G_SENSITIVITY_NONCAUSAL_ACCEL, 0U);
        return false;
    }

    const uint64_t accel_age_us =
        gyro_timestamp_us - accel_evidence->timestamp_us;
    if (accel_age_us >
        s_calibration[source].maximum_g_sensitivity_accel_age_us) {
        record_g_sensitivity_use(
            diagnostics, info, IMU_CALIBRATION_G_SENSITIVITY_STALE_ACCEL,
            accel_age_us);
        return false;
    }

    float correction[3] = {0.0f};
    float corrected[3];
    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            correction[row] +=
                s_calibration[source]
                    .gyro_g_sensitivity_rad_s_per_mps2[row][column] *
                accel_evidence->accel_body_mps2[column];
        }
        corrected[row] = gyro_rad_s[row] - correction[row];
    }
    if (!vector_is_finite(correction) || !vector_is_finite(corrected)) {
        increment_saturating(&diagnostics->apply_failure_count);
        record_g_sensitivity_use(
            diagnostics, info, IMU_CALIBRATION_G_SENSITIVITY_INVALID_ACCEL,
            accel_age_us);
        return false;
    }

    memcpy(gyro_rad_s, corrected, sizeof(corrected));
    record_g_sensitivity_use(diagnostics, info,
                             IMU_CALIBRATION_G_SENSITIVITY_APPLIED,
                             accel_age_us);
    return true;
#endif
}
