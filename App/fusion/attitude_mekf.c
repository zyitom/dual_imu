#include "attitude_mekf.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ATTITUDE_MEKF_PI_F                 (3.14159265358979323846f)
#define ATTITUDE_MEKF_DEFAULT_GRAVITY      (9.80665f)
#define ATTITUDE_MEKF_DEFAULT_NIS_GATE_99  (9.21034037f)
#define ATTITUDE_MEKF_DEFAULT_NIS_GATE_3D  (11.34486673f)
#define ATTITUDE_MEKF_QUATERNION_NORM_MIN  (1.0e-12f)
#define ATTITUDE_MEKF_SYMMETRY_TOLERANCE   (1.0e-5f)

static float mekf_clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static bool mekf_vector_is_finite(const float vector[ATTITUDE_MEKF_VECTOR_DIM])
{
    return (vector != NULL) && isfinite(vector[0]) && isfinite(vector[1]) &&
           isfinite(vector[2]);
}

static float mekf_vector_norm(const float vector[ATTITUDE_MEKF_VECTOR_DIM])
{
    return sqrtf((vector[0] * vector[0]) + (vector[1] * vector[1]) +
                 (vector[2] * vector[2]));
}

static float mekf_dot(const float lhs[ATTITUDE_MEKF_VECTOR_DIM],
                      const float rhs[ATTITUDE_MEKF_VECTOR_DIM])
{
    return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) + (lhs[2] * rhs[2]);
}

static void mekf_cross(const float lhs[ATTITUDE_MEKF_VECTOR_DIM],
                       const float rhs[ATTITUDE_MEKF_VECTOR_DIM],
                       float output[ATTITUDE_MEKF_VECTOR_DIM])
{
    output[0] = (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]);
    output[1] = (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]);
    output[2] = (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]);
}

static bool mekf_normalize_vector(float vector[ATTITUDE_MEKF_VECTOR_DIM])
{
    const float norm = mekf_vector_norm(vector);
    if ((!isfinite(norm)) || (norm < ATTITUDE_MEKF_QUATERNION_NORM_MIN))
    {
        return false;
    }

    const float inverse_norm = 1.0f / norm;
    vector[0] *= inverse_norm;
    vector[1] *= inverse_norm;
    vector[2] *= inverse_norm;
    return mekf_vector_is_finite(vector);
}

static void mekf_identity_quaternion(float quaternion[4])
{
    quaternion[0] = 1.0f;
    quaternion[1] = 0.0f;
    quaternion[2] = 0.0f;
    quaternion[3] = 0.0f;
}

static bool mekf_normalize_quaternion(float quaternion[4])
{
    const float norm_sq = (quaternion[0] * quaternion[0]) +
                          (quaternion[1] * quaternion[1]) +
                          (quaternion[2] * quaternion[2]) +
                          (quaternion[3] * quaternion[3]);
    if ((!isfinite(norm_sq)) || (norm_sq < ATTITUDE_MEKF_QUATERNION_NORM_MIN))
    {
        return false;
    }

    const float inverse_norm = 1.0f / sqrtf(norm_sq);
    for (size_t index = 0U; index < 4U; ++index)
    {
        quaternion[index] *= inverse_norm;
        if (!isfinite(quaternion[index]))
        {
            return false;
        }
    }
    return true;
}

static void mekf_quaternion_multiply(const float lhs[4],
                                     const float rhs[4],
                                     float output[4])
{
    output[0] = (lhs[0] * rhs[0]) - (lhs[1] * rhs[1]) -
                (lhs[2] * rhs[2]) - (lhs[3] * rhs[3]);
    output[1] = (lhs[0] * rhs[1]) + (lhs[1] * rhs[0]) +
                (lhs[2] * rhs[3]) - (lhs[3] * rhs[2]);
    output[2] = (lhs[0] * rhs[2]) - (lhs[1] * rhs[3]) +
                (lhs[2] * rhs[0]) + (lhs[3] * rhs[1]);
    output[3] = (lhs[0] * rhs[3]) + (lhs[1] * rhs[2]) -
                (lhs[2] * rhs[1]) + (lhs[3] * rhs[0]);
}

static bool mekf_rotation_vector_to_quaternion(
    const float rotation_vector[ATTITUDE_MEKF_VECTOR_DIM],
    float quaternion[4])
{
    const float angle_sq = mekf_dot(rotation_vector, rotation_vector);
    if ((!isfinite(angle_sq)) || (angle_sq < 0.0f))
    {
        return false;
    }

    float vector_scale;
    if (angle_sq < 1.0e-8f)
    {
        quaternion[0] = 1.0f - (0.125f * angle_sq);
        vector_scale = 0.5f - (angle_sq / 48.0f);
    }
    else
    {
        const float angle = sqrtf(angle_sq);
        const float half_angle = 0.5f * angle;
        quaternion[0] = cosf(half_angle);
        vector_scale = sinf(half_angle) / angle;
    }

    quaternion[1] = vector_scale * rotation_vector[0];
    quaternion[2] = vector_scale * rotation_vector[1];
    quaternion[3] = vector_scale * rotation_vector[2];
    return mekf_normalize_quaternion(quaternion);
}

static void mekf_skew(const float vector[ATTITUDE_MEKF_VECTOR_DIM],
                      float matrix[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM])
{
    matrix[0][0] = 0.0f;
    matrix[0][1] = -vector[2];
    matrix[0][2] = vector[1];
    matrix[1][0] = vector[2];
    matrix[1][1] = 0.0f;
    matrix[1][2] = -vector[0];
    matrix[2][0] = -vector[1];
    matrix[2][1] = vector[0];
    matrix[2][2] = 0.0f;
}

static void mekf_matrix6_identity(
    float matrix[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM])
{
    memset(matrix, 0, sizeof(float) * ATTITUDE_MEKF_STATE_DIM * ATTITUDE_MEKF_STATE_DIM);
    for (size_t index = 0U; index < ATTITUDE_MEKF_STATE_DIM; ++index)
    {
        matrix[index][index] = 1.0f;
    }
}

static bool mekf_matrix3_is_zero(
    const float matrix[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM])
{
    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            if ((!isfinite(matrix[row][column])) || (matrix[row][column] != 0.0f))
            {
                return false;
            }
        }
    }
    return true;
}

static bool mekf_matrix3_is_spd(
    const float matrix[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM])
{
    float lower[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM] = {{0.0f}};

    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            if (!isfinite(matrix[row][column]))
            {
                return false;
            }
            const float scale = fmaxf(1.0e-12f,
                                      fmaxf(fabsf(matrix[row][column]),
                                            fabsf(matrix[column][row])));
            if (fabsf(matrix[row][column] - matrix[column][row]) >
                (ATTITUDE_MEKF_SYMMETRY_TOLERANCE * scale))
            {
                return false;
            }
        }
    }

    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column <= row; ++column)
        {
            float value = matrix[row][column];
            for (size_t inner = 0U; inner < column; ++inner)
            {
                value -= lower[row][inner] * lower[column][inner];
            }

            if (row == column)
            {
                if ((!isfinite(value)) || (value <= 0.0f))
                {
                    return false;
                }
                lower[row][column] = sqrtf(value);
            }
            else
            {
                lower[row][column] = value / lower[column][column];
                if (!isfinite(lower[row][column]))
                {
                    return false;
                }
            }
        }
    }
    return true;
}

static bool mekf_matrix3_inverse_spd(
    const float matrix[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM],
    float inverse[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM])
{
    float lower[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM] = {{0.0f}};

    if (!mekf_matrix3_is_spd(matrix))
    {
        return false;
    }

    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column <= row; ++column)
        {
            float value = matrix[row][column];
            for (size_t inner = 0U; inner < column; ++inner)
            {
                value -= lower[row][inner] * lower[column][inner];
            }
            if (row == column)
            {
                lower[row][column] = sqrtf(value);
            }
            else
            {
                lower[row][column] = value / lower[column][column];
            }
        }
    }

    memset(inverse, 0, sizeof(float) * ATTITUDE_MEKF_VECTOR_DIM *
                               ATTITUDE_MEKF_VECTOR_DIM);
    for (size_t right_hand_side = 0U; right_hand_side < ATTITUDE_MEKF_VECTOR_DIM;
         ++right_hand_side)
    {
        float forward[ATTITUDE_MEKF_VECTOR_DIM] = {0.0f};
        float solution[ATTITUDE_MEKF_VECTOR_DIM] = {0.0f};
        for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
        {
            float value = (row == right_hand_side) ? 1.0f : 0.0f;
            for (size_t column = 0U; column < row; ++column)
            {
                value -= lower[row][column] * forward[column];
            }
            forward[row] = value / lower[row][row];
        }
        for (size_t reverse = ATTITUDE_MEKF_VECTOR_DIM; reverse-- > 0U;)
        {
            float value = forward[reverse];
            for (size_t column = reverse + 1U; column < ATTITUDE_MEKF_VECTOR_DIM;
                 ++column)
            {
                value -= lower[column][reverse] * solution[column];
            }
            solution[reverse] = value / lower[reverse][reverse];
        }
        for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
        {
            inverse[row][right_hand_side] = solution[row];
            if (!isfinite(inverse[row][right_hand_side]))
            {
                return false;
            }
        }
    }

    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = row + 1U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            const float symmetric = 0.5f * (inverse[row][column] +
                                             inverse[column][row]);
            inverse[row][column] = symmetric;
            inverse[column][row] = symmetric;
        }
    }
    return true;
}

static bool mekf_matrix6_is_spd(
    const float matrix[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM])
{
    float lower[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};

    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column <= row; ++column)
        {
            float value = matrix[row][column];
            if (!isfinite(value))
            {
                return false;
            }
            for (size_t inner = 0U; inner < column; ++inner)
            {
                value -= lower[row][inner] * lower[column][inner];
            }

            if (row == column)
            {
                if ((!isfinite(value)) || (value <= 0.0f))
                {
                    return false;
                }
                lower[row][column] = sqrtf(value);
            }
            else
            {
                lower[row][column] = value / lower[column][column];
                if (!isfinite(lower[row][column]))
                {
                    return false;
                }
            }
        }
    }
    return true;
}

static bool mekf_condition_covariance(
    const attitude_mekf_config_t *config,
    float covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM])
{
    float maximum_diagonal = config->covariance_floor;

    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = row; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            if ((!isfinite(covariance[row][column])) ||
                (!isfinite(covariance[column][row])))
            {
                return false;
            }
            const float symmetric = 0.5f * (covariance[row][column] +
                                             covariance[column][row]);
            covariance[row][column] = symmetric;
            covariance[column][row] = symmetric;
        }

        if (covariance[row][row] < config->covariance_floor)
        {
            covariance[row][row] = config->covariance_floor;
        }
        if (covariance[row][row] > config->covariance_ceiling)
        {
            return false;
        }
        maximum_diagonal = fmaxf(maximum_diagonal, covariance[row][row]);
    }

    if (mekf_matrix6_is_spd(
            (const float (*)[ATTITUDE_MEKF_STATE_DIM])covariance))
    {
        return true;
    }

    float jitter = fmaxf(config->covariance_floor, maximum_diagonal * 1.0e-7f);
    for (size_t attempt = 0U; attempt < 4U; ++attempt)
    {
        for (size_t index = 0U; index < ATTITUDE_MEKF_STATE_DIM; ++index)
        {
            covariance[index][index] += jitter;
            if (covariance[index][index] > config->covariance_ceiling)
            {
                return false;
            }
        }
        if (mekf_matrix6_is_spd(
                (const float (*)[ATTITUDE_MEKF_STATE_DIM])covariance))
        {
            return true;
        }
        jitter *= 10.0f;
    }
    return false;
}

static bool mekf_config_is_valid(const attitude_mekf_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    const bool matrices_valid = mekf_matrix3_is_spd(config->gyro_noise_psd) &&
                                (mekf_matrix3_is_zero(config->gyro_bias_rw_psd) ||
                                 mekf_matrix3_is_spd(config->gyro_bias_rw_psd)) &&
                                mekf_matrix3_is_spd(config->accel_direction_covariance);
    if (!matrices_valid)
    {
        return false;
    }

    return isfinite(config->initial_attitude_std_rad) &&
           (config->initial_attitude_std_rad > 0.0f) &&
           isfinite(config->initial_bias_std_rad_s) &&
           (config->initial_bias_std_rad_s > 0.0f) &&
           isfinite(config->standard_gravity_mps2) &&
           (config->standard_gravity_mps2 > 1.0f) &&
           isfinite(config->accel_norm_soft_deviation_mps2) &&
           (config->accel_norm_soft_deviation_mps2 > 0.0f) &&
           isfinite(config->accel_norm_hard_deviation_mps2) &&
           (config->accel_norm_hard_deviation_mps2 >=
            config->accel_norm_soft_deviation_mps2) &&
           isfinite(config->accel_variance_scale_max) &&
           (config->accel_variance_scale_max >= 1.0f) &&
           isfinite(config->accel_nis_gate) && (config->accel_nis_gate > 0.0f) &&
           isfinite(config->zaru_nis_gate) && (config->zaru_nis_gate > 0.0f) &&
           isfinite(config->max_accel_correction_step_rad) &&
           (config->max_accel_correction_step_rad > 0.0f) &&
           (config->max_accel_correction_step_rad <=
            config->max_attitude_correction_rad) &&
           isfinite(config->max_attitude_correction_rad) &&
           (config->max_attitude_correction_rad > 0.0f) &&
           (config->max_attitude_correction_rad <= ATTITUDE_MEKF_PI_F) &&
           isfinite(config->max_abs_bias_rad_s) &&
           (config->max_abs_bias_rad_s > 0.0f) && isfinite(config->min_dt_s) &&
           (config->min_dt_s > 0.0f) && isfinite(config->max_dt_s) &&
           (config->max_dt_s >= config->min_dt_s) &&
           isfinite(config->covariance_floor) && (config->covariance_floor > 0.0f) &&
           isfinite(config->covariance_ceiling) &&
           (config->covariance_ceiling > config->covariance_floor);
}

static void mekf_reset_covariance(attitude_mekf_t *filter)
{
    memset(filter->covariance, 0, sizeof(filter->covariance));
    const float attitude_variance = filter->config.initial_attitude_std_rad *
                                    filter->config.initial_attitude_std_rad;
    const float bias_variance = filter->config.initial_bias_std_rad_s *
                                filter->config.initial_bias_std_rad_s;
    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
    {
        filter->covariance[axis][axis] = attitude_variance;
        filter->covariance[axis + ATTITUDE_MEKF_VECTOR_DIM]
                          [axis + ATTITUDE_MEKF_VECTOR_DIM] = bias_variance;
    }
}

static attitude_mekf_accel_result_t mekf_record_accel_result(
    attitude_mekf_t *filter,
    attitude_mekf_accel_result_t result)
{
    filter->diagnostics.last_accel_result = result;
    switch (result)
    {
        case ATTITUDE_MEKF_ACCEL_ACCEPTED:
            filter->diagnostics.accel_accept_count++;
            break;
        case ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT:
            filter->diagnostics.accel_invalid_reject_count++;
            break;
        case ATTITUDE_MEKF_ACCEL_REJECTED_NORM:
            filter->diagnostics.accel_norm_reject_count++;
            break;
        case ATTITUDE_MEKF_ACCEL_REJECTED_NIS:
            filter->diagnostics.accel_nis_reject_count++;
            break;
        case ATTITUDE_MEKF_ACCEL_REJECTED_CORRECTION:
            filter->diagnostics.accel_correction_reject_count++;
            break;
        case ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE:
        default:
            filter->diagnostics.numeric_fault_count++;
            break;
    }
    return result;
}

static attitude_mekf_zaru_result_t mekf_record_zaru_result(
    attitude_mekf_t *filter,
    attitude_mekf_zaru_result_t result)
{
    filter->diagnostics.last_zaru_result = result;
    switch (result)
    {
        case ATTITUDE_MEKF_ZARU_ACCEPTED:
            filter->diagnostics.zaru_accept_count++;
            break;
        case ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT:
            filter->diagnostics.zaru_invalid_reject_count++;
            break;
        case ATTITUDE_MEKF_ZARU_REJECTED_NIS:
            filter->diagnostics.zaru_nis_reject_count++;
            break;
        case ATTITUDE_MEKF_ZARU_REJECTED_CORRECTION:
            filter->diagnostics.zaru_correction_reject_count++;
            break;
        case ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE:
        default:
            filter->diagnostics.numeric_fault_count++;
            break;
    }
    return result;
}

void attitude_mekf_default_config(attitude_mekf_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));

    const float gyro_noise_density = 0.02f * ATTITUDE_MEKF_PI_F / 180.0f;
    const float bias_rw_density = 0.002f * ATTITUDE_MEKF_PI_F / 180.0f;
    const float accel_direction_std = 1.0f * ATTITUDE_MEKF_PI_F / 180.0f;
    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
    {
        config->gyro_noise_psd[axis][axis] = gyro_noise_density * gyro_noise_density;
        config->gyro_bias_rw_psd[axis][axis] = bias_rw_density * bias_rw_density;
        config->accel_direction_covariance[axis][axis] =
            accel_direction_std * accel_direction_std;
    }

    config->initial_attitude_std_rad = 10.0f * ATTITUDE_MEKF_PI_F / 180.0f;
    config->initial_bias_std_rad_s = 5.0f * ATTITUDE_MEKF_PI_F / 180.0f;
    config->standard_gravity_mps2 = ATTITUDE_MEKF_DEFAULT_GRAVITY;
    config->accel_norm_soft_deviation_mps2 = 0.35f;
    config->accel_norm_hard_deviation_mps2 = 3.0f;
    config->accel_variance_scale_max = 100.0f;
    config->accel_nis_gate = ATTITUDE_MEKF_DEFAULT_NIS_GATE_99;
    config->zaru_nis_gate = ATTITUDE_MEKF_DEFAULT_NIS_GATE_3D;
    config->max_accel_correction_step_rad =
        0.5f * ATTITUDE_MEKF_PI_F / 180.0f;
    config->max_attitude_correction_rad = 20.0f * ATTITUDE_MEKF_PI_F / 180.0f;
    config->max_abs_bias_rad_s = 1.0f;
    config->min_dt_s = 0.0001f;
    config->max_dt_s = 0.02f;
    config->covariance_floor = 1.0e-12f;
    config->covariance_ceiling = 1.0e4f;
}

bool attitude_mekf_init(attitude_mekf_t *filter,
                        const attitude_mekf_config_t *config)
{
    if (filter == NULL)
    {
        return false;
    }

    attitude_mekf_config_t selected_config;
    if (config == NULL)
    {
        attitude_mekf_default_config(&selected_config);
    }
    else
    {
        selected_config = *config;
    }
    if (!mekf_config_is_valid(&selected_config))
    {
        memset(filter, 0, sizeof(*filter));
        return false;
    }

    memset(filter, 0, sizeof(*filter));
    filter->config = selected_config;
    mekf_identity_quaternion(filter->q);
    mekf_reset_covariance(filter);
    filter->diagnostics.last_accel_result = ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT;
    filter->diagnostics.last_zaru_result = ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT;
    filter->initialized = mekf_condition_covariance(&filter->config, filter->covariance);
    return filter->initialized;
}

bool attitude_mekf_reset(attitude_mekf_t *filter,
                         const float quaternion_body_to_world[4],
                         const float gyro_bias_rad_s[ATTITUDE_MEKF_VECTOR_DIM])
{
    if ((filter == NULL) || (!filter->initialized) ||
        (quaternion_body_to_world == NULL) ||
        (!mekf_vector_is_finite(gyro_bias_rad_s)))
    {
        return false;
    }

    float normalized_quaternion[4] = {
        quaternion_body_to_world[0],
        quaternion_body_to_world[1],
        quaternion_body_to_world[2],
        quaternion_body_to_world[3],
    };
    if (!mekf_normalize_quaternion(normalized_quaternion))
    {
        return false;
    }
    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
    {
        if (fabsf(gyro_bias_rad_s[axis]) > filter->config.max_abs_bias_rad_s)
        {
            return false;
        }
    }

    memcpy(filter->q, normalized_quaternion, sizeof(filter->q));
    memcpy(filter->gyro_bias_rad_s, gyro_bias_rad_s, sizeof(filter->gyro_bias_rad_s));
    memset(&filter->diagnostics, 0, sizeof(filter->diagnostics));
    filter->diagnostics.last_accel_result = ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT;
    filter->diagnostics.last_zaru_result = ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT;
    mekf_reset_covariance(filter);
    return mekf_condition_covariance(&filter->config, filter->covariance);
}

bool attitude_mekf_seed_from_accel(attitude_mekf_t *filter,
                                   const float specific_force_mps2[ATTITUDE_MEKF_VECTOR_DIM],
                                   float yaw_rad,
                                   const float gyro_bias_rad_s[ATTITUDE_MEKF_VECTOR_DIM])
{
    if ((filter == NULL) || (!filter->initialized) ||
        (!mekf_vector_is_finite(specific_force_mps2)) || (!isfinite(yaw_rad)) ||
        (!mekf_vector_is_finite(gyro_bias_rad_s)))
    {
        return false;
    }

    const float norm = mekf_vector_norm(specific_force_mps2);
    if ((!isfinite(norm)) ||
        (fabsf(norm - filter->config.standard_gravity_mps2) >
         filter->config.accel_norm_hard_deviation_mps2))
    {
        return false;
    }

    const float down_body[ATTITUDE_MEKF_VECTOR_DIM] = {
        -specific_force_mps2[0] / norm,
        -specific_force_mps2[1] / norm,
        -specific_force_mps2[2] / norm,
    };
    const float roll = atan2f(down_body[1], down_body[2]);
    const float pitch = atan2f(-down_body[0],
                               sqrtf((down_body[1] * down_body[1]) +
                                     (down_body[2] * down_body[2])));
    const float half_roll = 0.5f * roll;
    const float half_pitch = 0.5f * pitch;
    const float half_yaw = 0.5f * yaw_rad;
    const float cosine_roll = cosf(half_roll);
    const float sine_roll = sinf(half_roll);
    const float cosine_pitch = cosf(half_pitch);
    const float sine_pitch = sinf(half_pitch);
    const float cosine_yaw = cosf(half_yaw);
    const float sine_yaw = sinf(half_yaw);
    const float quaternion[4] = {
        (cosine_yaw * cosine_pitch * cosine_roll) +
            (sine_yaw * sine_pitch * sine_roll),
        (cosine_yaw * cosine_pitch * sine_roll) -
            (sine_yaw * sine_pitch * cosine_roll),
        (cosine_yaw * sine_pitch * cosine_roll) +
            (sine_yaw * cosine_pitch * sine_roll),
        (sine_yaw * cosine_pitch * cosine_roll) -
            (cosine_yaw * sine_pitch * sine_roll),
    };
    return attitude_mekf_reset(filter, quaternion, gyro_bias_rad_s);
}

bool attitude_mekf_is_valid(const attitude_mekf_t *filter)
{
    if ((filter == NULL) || (!filter->initialized) ||
        (!mekf_config_is_valid(&filter->config)) ||
        (!mekf_vector_is_finite(filter->gyro_bias_rad_s)))
    {
        return false;
    }

    const float quaternion_norm_sq = (filter->q[0] * filter->q[0]) +
                                     (filter->q[1] * filter->q[1]) +
                                     (filter->q[2] * filter->q[2]) +
                                     (filter->q[3] * filter->q[3]);
    if ((!isfinite(quaternion_norm_sq)) ||
        (fabsf(quaternion_norm_sq - 1.0f) > 1.0e-3f))
    {
        return false;
    }

    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
    {
        if (fabsf(filter->gyro_bias_rad_s[axis]) > filter->config.max_abs_bias_rad_s)
        {
            return false;
        }
    }

    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        if ((!isfinite(filter->covariance[row][row])) ||
            (filter->covariance[row][row] < filter->config.covariance_floor) ||
            (filter->covariance[row][row] > filter->config.covariance_ceiling))
        {
            return false;
        }
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            if ((!isfinite(filter->covariance[row][column])) ||
                (fabsf(filter->covariance[row][column] -
                       filter->covariance[column][row]) >
                 ATTITUDE_MEKF_SYMMETRY_TOLERANCE *
                     fmaxf(1.0e-12f,
                           fmaxf(fabsf(filter->covariance[row][column]),
                                 fabsf(filter->covariance[column][row])))))
            {
                return false;
            }
        }
    }
    return mekf_matrix6_is_spd(filter->covariance);
}

bool attitude_mekf_propagate_delta(attitude_mekf_t *filter,
                                   const float delta_angle_rad[ATTITUDE_MEKF_VECTOR_DIM],
                                   float dt_s)
{
    if ((filter == NULL) || (!filter->initialized))
    {
        return false;
    }
    if (!attitude_mekf_is_valid(filter))
    {
        filter->diagnostics.propagation_reject_count++;
        filter->diagnostics.numeric_fault_count++;
        return false;
    }
    if ((!mekf_vector_is_finite(delta_angle_rad)) || (!isfinite(dt_s)) ||
        (dt_s < filter->config.min_dt_s) || (dt_s > filter->config.max_dt_s))
    {
        filter->diagnostics.propagation_reject_count++;
        return false;
    }

    float corrected_delta[ATTITUDE_MEKF_VECTOR_DIM];
    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
    {
        corrected_delta[axis] = delta_angle_rad[axis] -
                                (filter->gyro_bias_rad_s[axis] * dt_s);
    }

    float delta_quaternion[4];
    float propagated_quaternion[4];
    if ((!mekf_rotation_vector_to_quaternion(corrected_delta, delta_quaternion)))
    {
        filter->diagnostics.numeric_fault_count++;
        return false;
    }
    mekf_quaternion_multiply(filter->q, delta_quaternion, propagated_quaternion);
    if (!mekf_normalize_quaternion(propagated_quaternion))
    {
        filter->diagnostics.numeric_fault_count++;
        return false;
    }

    float delta_skew[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];
    float delta_skew_sq[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM] = {{0.0f}};
    mekf_skew(corrected_delta, delta_skew);
    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_VECTOR_DIM; ++inner)
            {
                delta_skew_sq[row][column] += delta_skew[row][inner] *
                                              delta_skew[inner][column];
            }
        }
    }

    float transition[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    mekf_matrix6_identity(transition);
    const float corrected_angle_sq = mekf_dot(corrected_delta, corrected_delta);
    float sine_over_angle;
    float one_minus_cosine_over_angle_sq;
    float angle_minus_sine_over_angle_cubed;
    if (corrected_angle_sq < 1.0e-6f)
    {
        const float corrected_angle_fourth = corrected_angle_sq * corrected_angle_sq;
        sine_over_angle = 1.0f - (corrected_angle_sq / 6.0f) +
                          (corrected_angle_fourth / 120.0f);
        one_minus_cosine_over_angle_sq =
            0.5f - (corrected_angle_sq / 24.0f) +
            (corrected_angle_fourth / 720.0f);
        angle_minus_sine_over_angle_cubed =
            (1.0f / 6.0f) - (corrected_angle_sq / 120.0f) +
            (corrected_angle_fourth / 5040.0f);
    }
    else
    {
        const float corrected_angle = sqrtf(corrected_angle_sq);
        sine_over_angle = sinf(corrected_angle) / corrected_angle;
        one_minus_cosine_over_angle_sq =
            (1.0f - cosf(corrected_angle)) / corrected_angle_sq;
        angle_minus_sine_over_angle_cubed =
            (corrected_angle - sinf(corrected_angle)) /
            (corrected_angle_sq * corrected_angle);
    }
    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            const float identity = (row == column) ? 1.0f : 0.0f;
            transition[row][column] =
                identity - (sine_over_angle * delta_skew[row][column]) +
                (one_minus_cosine_over_angle_sq * delta_skew_sq[row][column]);
            transition[row][column + ATTITUDE_MEKF_VECTOR_DIM] =
                -dt_s * (identity -
                         (one_minus_cosine_over_angle_sq * delta_skew[row][column]) +
                         (angle_minus_sine_over_angle_cubed *
                          delta_skew_sq[row][column]));
        }
    }

    float intermediate[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    float propagated_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] =
        {{0.0f}};
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                intermediate[row][column] += transition[row][inner] *
                                             filter->covariance[inner][column];
            }
        }
    }
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                propagated_covariance[row][column] += intermediate[row][inner] *
                                                      transition[column][inner];
            }
        }
    }

    const float dt_sq = dt_s * dt_s;
    const float dt_cubed = dt_sq * dt_s;
    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            const float bias_psd = filter->config.gyro_bias_rw_psd[row][column];
            propagated_covariance[row][column] +=
                (filter->config.gyro_noise_psd[row][column] * dt_s) +
                (bias_psd * dt_cubed / 3.0f);
            propagated_covariance[row][column + ATTITUDE_MEKF_VECTOR_DIM] -=
                bias_psd * dt_sq * 0.5f;
            propagated_covariance[row + ATTITUDE_MEKF_VECTOR_DIM][column] -=
                bias_psd * dt_sq * 0.5f;
            propagated_covariance[row + ATTITUDE_MEKF_VECTOR_DIM]
                                 [column + ATTITUDE_MEKF_VECTOR_DIM] +=
                bias_psd * dt_s;
        }
    }

    if (!mekf_condition_covariance(&filter->config, propagated_covariance))
    {
        filter->diagnostics.numeric_fault_count++;
        return false;
    }

    memcpy(filter->q, propagated_quaternion, sizeof(filter->q));
    memcpy(filter->covariance, propagated_covariance, sizeof(filter->covariance));
    filter->diagnostics.propagation_count++;
    return true;
}

bool attitude_mekf_mark_rotation_unobserved(attitude_mekf_t *filter,
                                            float rotation_std_rad)
{
    if ((filter == NULL) || !filter->initialized ||
        !attitude_mekf_is_valid(filter) || !isfinite(rotation_std_rad) ||
        (rotation_std_rad <= 0.0f)) {
        return false;
    }

    const float added_variance = rotation_std_rad * rotation_std_rad;
    if (!isfinite(added_variance))
        return false;

    float inflated[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    memcpy(inflated, filter->covariance, sizeof(inflated));
    const float maximum_attitude_variance =
        0.5f * filter->config.covariance_ceiling;
    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis) {
        inflated[axis][axis] = fminf(maximum_attitude_variance,
                                     inflated[axis][axis] + added_variance);
    }
    if (!mekf_condition_covariance(&filter->config, inflated)) {
        filter->diagnostics.numeric_fault_count++;
        return false;
    }

    memcpy(filter->covariance, inflated, sizeof(filter->covariance));
    if (filter->diagnostics.unobserved_rotation_count < UINT32_MAX)
        filter->diagnostics.unobserved_rotation_count++;
    return true;
}

attitude_mekf_accel_result_t attitude_mekf_update_accel(
    attitude_mekf_t *filter,
    const float specific_force_mps2[ATTITUDE_MEKF_VECTOR_DIM],
    float variance_scale)
{
    if ((filter == NULL) || (!filter->initialized))
    {
        return ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT;
    }

    filter->diagnostics.accel_update_count++;
    filter->diagnostics.last_accel_nis = 0.0f;
    filter->diagnostics.last_accel_variance_scale = variance_scale;
    if (!attitude_mekf_is_valid(filter))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }
    if ((!mekf_vector_is_finite(specific_force_mps2)) || (!isfinite(variance_scale)) ||
        (variance_scale < 1.0f))
    {
        return mekf_record_accel_result(filter,
                                        ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT);
    }

    const float accel_norm = mekf_vector_norm(specific_force_mps2);
    filter->diagnostics.last_accel_norm_mps2 = accel_norm;
    if ((!isfinite(accel_norm)) ||
        (fabsf(accel_norm - filter->config.standard_gravity_mps2) >
         filter->config.accel_norm_hard_deviation_mps2))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_REJECTED_NORM);
    }

    float measured_direction[ATTITUDE_MEKF_VECTOR_DIM] = {
        specific_force_mps2[0] / accel_norm,
        specific_force_mps2[1] / accel_norm,
        specific_force_mps2[2] / accel_norm,
    };
    const float quaternion_w = filter->q[0];
    const float quaternion_x = filter->q[1];
    const float quaternion_y = filter->q[2];
    const float quaternion_z = filter->q[3];
    float predicted_direction[ATTITUDE_MEKF_VECTOR_DIM] = {
        -2.0f * ((quaternion_x * quaternion_z) -
                 (quaternion_w * quaternion_y)),
        -2.0f * ((quaternion_w * quaternion_x) +
                 (quaternion_y * quaternion_z)),
        -((quaternion_w * quaternion_w) - (quaternion_x * quaternion_x) -
          (quaternion_y * quaternion_y) + (quaternion_z * quaternion_z)),
    };
    if ((!mekf_normalize_vector(measured_direction)) ||
        (!mekf_normalize_vector(predicted_direction)))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }

    const float reference[ATTITUDE_MEKF_VECTOR_DIM] = {
        (fabsf(predicted_direction[2]) > 0.8f) ? 1.0f : 0.0f,
        0.0f,
        (fabsf(predicted_direction[2]) > 0.8f) ? 0.0f : 1.0f,
    };
    float tangent[2][ATTITUDE_MEKF_VECTOR_DIM];
    mekf_cross(predicted_direction, reference, tangent[0]);
    if (!mekf_normalize_vector(tangent[0]))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }
    mekf_cross(predicted_direction, tangent[0], tangent[1]);
    if (!mekf_normalize_vector(tangent[1]))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }

    float predicted_skew[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];
    float observation[2][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    float residual[2] = {0.0f, 0.0f};
    mekf_skew(predicted_direction, predicted_skew);
    for (size_t row = 0U; row < 2U; ++row)
    {
        residual[row] = mekf_dot(tangent[row], measured_direction) -
                        mekf_dot(tangent[row], predicted_direction);
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_VECTOR_DIM; ++inner)
            {
                observation[row][column] += tangent[row][inner] *
                                             predicted_skew[inner][column];
            }
        }
    }

    const float norm_deviation = fabsf(accel_norm - filter->config.standard_gravity_mps2);
    const float soft_ratio = norm_deviation /
                             filter->config.accel_norm_soft_deviation_mps2;
    const float adaptive_scale = 1.0f + (soft_ratio * soft_ratio);
    const float combined_scale = mekf_clampf(adaptive_scale * variance_scale,
                                             1.0f,
                                             filter->config.accel_variance_scale_max);
    filter->diagnostics.last_accel_variance_scale = combined_scale;

    float measurement_covariance[2][2] = {{0.0f}};
    for (size_t row = 0U; row < 2U; ++row)
    {
        for (size_t column = 0U; column < 2U; ++column)
        {
            for (size_t first = 0U; first < ATTITUDE_MEKF_VECTOR_DIM; ++first)
            {
                for (size_t second = 0U; second < ATTITUDE_MEKF_VECTOR_DIM; ++second)
                {
                    measurement_covariance[row][column] +=
                        tangent[row][first] *
                        filter->config.accel_direction_covariance[first][second] *
                        tangent[column][second];
                }
            }
            measurement_covariance[row][column] *= combined_scale;
        }
    }

    float covariance_observation_transpose[ATTITUDE_MEKF_STATE_DIM][2] = {{0.0f}};
    float innovation_covariance[2][2] = {
        {measurement_covariance[0][0], measurement_covariance[0][1]},
        {measurement_covariance[1][0], measurement_covariance[1][1]},
    };
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < 2U; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                covariance_observation_transpose[row][column] +=
                    filter->covariance[row][inner] * observation[column][inner];
            }
        }
    }
    for (size_t row = 0U; row < 2U; ++row)
    {
        for (size_t column = 0U; column < 2U; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                innovation_covariance[row][column] +=
                    observation[row][inner] * covariance_observation_transpose[inner][column];
            }
        }
    }

    const float determinant =
        (innovation_covariance[0][0] * innovation_covariance[1][1]) -
        (innovation_covariance[0][1] * innovation_covariance[1][0]);
    if ((!isfinite(determinant)) || (determinant <= 0.0f))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }
    const float inverse_determinant = 1.0f / determinant;
    const float inverse_innovation[2][2] = {
        {innovation_covariance[1][1] * inverse_determinant,
         -innovation_covariance[0][1] * inverse_determinant},
        {-innovation_covariance[1][0] * inverse_determinant,
         innovation_covariance[0][0] * inverse_determinant},
    };
    const float weighted_residual[2] = {
        (inverse_innovation[0][0] * residual[0]) +
            (inverse_innovation[0][1] * residual[1]),
        (inverse_innovation[1][0] * residual[0]) +
            (inverse_innovation[1][1] * residual[1]),
    };
    float nis = (residual[0] * weighted_residual[0]) +
                (residual[1] * weighted_residual[1]);
    if ((!isfinite(nis)) || (nis < -1.0e-5f))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }
    nis = fmaxf(nis, 0.0f);
    filter->diagnostics.last_accel_nis = nis;
    if (nis > filter->config.accel_nis_gate)
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_REJECTED_NIS);
    }

    float gain[ATTITUDE_MEKF_STATE_DIM][2] = {{0.0f}};
    float correction[ATTITUDE_MEKF_STATE_DIM] = {0.0f};
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < 2U; ++column)
        {
            gain[row][column] =
                (covariance_observation_transpose[row][0] *
                 inverse_innovation[0][column]) +
                (covariance_observation_transpose[row][1] *
                 inverse_innovation[1][column]);
        }
    }

    /* Gravity cannot observe a right attitude error or gyro bias parallel to
     * its body-frame direction. Keep both gain blocks in the tangent plane so
     * noise cannot inject either yaw or gravity-axis bias through covariance
     * cross-correlation. The Joseph update below uses this same gain. */
    for (size_t block = 0U;
         block < ATTITUDE_MEKF_STATE_DIM;
         block += ATTITUDE_MEKF_VECTOR_DIM)
    {
        for (size_t column = 0U; column < 2U; ++column)
        {
            float parallel_gain = 0.0f;
            for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
            {
                parallel_gain += predicted_direction[axis] *
                                 gain[block + axis][column];
            }
            for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
            {
                gain[block + axis][column] -=
                    predicted_direction[axis] * parallel_gain;
            }
        }
    }

    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        correction[row] = (gain[row][0] * residual[0]) +
                          (gain[row][1] * residual[1]);
        if (!isfinite(correction[row]))
        {
            return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
        }
    }

    const float attitude_correction_norm = mekf_vector_norm(correction);
    if ((!isfinite(attitude_correction_norm)) ||
        (attitude_correction_norm > filter->config.max_attitude_correction_rad))
    {
        return mekf_record_accel_result(filter,
                                        ATTITUDE_MEKF_ACCEL_REJECTED_CORRECTION);
    }

    if (attitude_correction_norm >
        filter->config.max_accel_correction_step_rad)
    {
        const float gain_scale =
            filter->config.max_accel_correction_step_rad /
            attitude_correction_norm;
        for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
        {
            correction[row] *= gain_scale;
            for (size_t column = 0U; column < 2U; ++column)
            {
                gain[row][column] *= gain_scale;
            }
        }
    }

    float corrected_bias[ATTITUDE_MEKF_VECTOR_DIM];
    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
    {
        corrected_bias[axis] = filter->gyro_bias_rad_s[axis] +
                               correction[axis + ATTITUDE_MEKF_VECTOR_DIM];
        if ((!isfinite(corrected_bias[axis])) ||
            (fabsf(corrected_bias[axis]) > filter->config.max_abs_bias_rad_s))
        {
            return mekf_record_accel_result(filter,
                                            ATTITUDE_MEKF_ACCEL_REJECTED_CORRECTION);
        }
    }

    float correction_quaternion[4];
    float corrected_quaternion[4];
    if (!mekf_rotation_vector_to_quaternion(correction, correction_quaternion))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }
    mekf_quaternion_multiply(filter->q, correction_quaternion, corrected_quaternion);
    if (!mekf_normalize_quaternion(corrected_quaternion))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }

    float joseph_factor[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    mekf_matrix6_identity(joseph_factor);
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < 2U; ++inner)
            {
                joseph_factor[row][column] -= gain[row][inner] *
                                              observation[inner][column];
            }
        }
    }

    float factor_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    float joseph_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                factor_covariance[row][column] += joseph_factor[row][inner] *
                                                  filter->covariance[inner][column];
            }
        }
    }
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                joseph_covariance[row][column] += factor_covariance[row][inner] *
                                                  joseph_factor[column][inner];
            }
            for (size_t first = 0U; first < 2U; ++first)
            {
                for (size_t second = 0U; second < 2U; ++second)
                {
                    joseph_covariance[row][column] +=
                        gain[row][first] * measurement_covariance[first][second] *
                        gain[column][second];
                }
            }
        }
    }

    float reset_jacobian[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    float correction_skew[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];
    mekf_matrix6_identity(reset_jacobian);
    mekf_skew(correction, correction_skew);
    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            reset_jacobian[row][column] -= 0.5f * correction_skew[row][column];
        }
    }

    float reset_intermediate[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    float corrected_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                reset_intermediate[row][column] += reset_jacobian[row][inner] *
                                                   joseph_covariance[inner][column];
            }
        }
    }
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                corrected_covariance[row][column] += reset_intermediate[row][inner] *
                                                     reset_jacobian[column][inner];
            }
        }
    }
    if (!mekf_condition_covariance(&filter->config, corrected_covariance))
    {
        return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE);
    }

    memcpy(filter->q, corrected_quaternion, sizeof(filter->q));
    memcpy(filter->gyro_bias_rad_s, corrected_bias, sizeof(filter->gyro_bias_rad_s));
    memcpy(filter->covariance, corrected_covariance, sizeof(filter->covariance));
    return mekf_record_accel_result(filter, ATTITUDE_MEKF_ACCEL_ACCEPTED);
}

static attitude_mekf_zaru_result_t mekf_update_zero_rate(
    attitude_mekf_t *filter,
    const float gyro_rad_s[ATTITUDE_MEKF_VECTOR_DIM],
    const float measurement_covariance[ATTITUDE_MEKF_VECTOR_DIM]
                                      [ATTITUDE_MEKF_VECTOR_DIM],
    bool bounded,
    float target_residual_limit_rad_s,
    float max_bias_correction_rad_s,
    float bounded_target_rad_s[ATTITUDE_MEKF_VECTOR_DIM])
{
    if ((filter == NULL) || (!filter->initialized))
    {
        return ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT;
    }

    filter->diagnostics.zaru_update_count++;
    filter->diagnostics.last_zaru_nis = 0.0f;
    if (!attitude_mekf_is_valid(filter))
    {
        return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE);
    }
    if ((!mekf_vector_is_finite(gyro_rad_s)) || (measurement_covariance == NULL) ||
        (!mekf_matrix3_is_spd(measurement_covariance)) ||
        (bounded &&
         ((!isfinite(target_residual_limit_rad_s)) ||
          (target_residual_limit_rad_s <= 0.0f) ||
          (!isfinite(max_bias_correction_rad_s)) ||
          (max_bias_correction_rad_s <= 0.0f))))
    {
        return mekf_record_zaru_result(filter,
                                       ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT);
    }

    float target_rad_s[ATTITUDE_MEKF_VECTOR_DIM];
    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
    {
        if (bounded)
        {
            const float residual = gyro_rad_s[axis] -
                                   filter->gyro_bias_rad_s[axis];
            target_rad_s[axis] = filter->gyro_bias_rad_s[axis] +
                                 fmaxf(-target_residual_limit_rad_s,
                                       fminf(target_residual_limit_rad_s,
                                             residual));
        }
        else
        {
            target_rad_s[axis] = gyro_rad_s[axis];
        }
    }
    if (bounded_target_rad_s != NULL)
    {
        memcpy(bounded_target_rad_s, target_rad_s, sizeof(target_rad_s));
    }

    float residual[ATTITUDE_MEKF_VECTOR_DIM];
    float innovation_covariance[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];
    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        residual[row] = target_rad_s[row] - filter->gyro_bias_rad_s[row];
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            innovation_covariance[row][column] =
                filter->covariance[row + ATTITUDE_MEKF_VECTOR_DIM]
                                  [column + ATTITUDE_MEKF_VECTOR_DIM] +
                measurement_covariance[row][column];
        }
    }

    float inverse_innovation[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];
    if (!mekf_matrix3_inverse_spd(
            (const float (*)[ATTITUDE_MEKF_VECTOR_DIM])innovation_covariance,
            inverse_innovation))
    {
        return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE);
    }

    float weighted_residual[ATTITUDE_MEKF_VECTOR_DIM] = {0.0f};
    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            weighted_residual[row] += inverse_innovation[row][column] *
                                      residual[column];
        }
    }
    float nis = mekf_dot(residual, weighted_residual);
    if ((!isfinite(nis)) || (nis < -1.0e-5f))
    {
        return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE);
    }
    nis = fmaxf(nis, 0.0f);
    filter->diagnostics.last_zaru_nis = nis;
    if (nis > filter->config.zaru_nis_gate)
    {
        return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_REJECTED_NIS);
    }

    float covariance_observation_transpose[ATTITUDE_MEKF_STATE_DIM]
                                          [ATTITUDE_MEKF_VECTOR_DIM];
    float gain[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_VECTOR_DIM] = {{0.0f}};
    float correction[ATTITUDE_MEKF_STATE_DIM] = {0.0f};
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            covariance_observation_transpose[row][column] =
                filter->covariance[row][column + ATTITUDE_MEKF_VECTOR_DIM];
        }
    }
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_VECTOR_DIM; ++inner)
            {
                gain[row][column] +=
                    covariance_observation_transpose[row][inner] *
                    inverse_innovation[inner][column];
            }
            correction[row] += gain[row][column] * residual[column];
        }
        if (!isfinite(correction[row]))
        {
            return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE);
        }
    }

    if (bounded)
    {
        float bias_correction_sq = 0.0f;
        for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
        {
            const float bias_correction =
                correction[axis + ATTITUDE_MEKF_VECTOR_DIM];
            bias_correction_sq += bias_correction * bias_correction;
        }
        if (!isfinite(bias_correction_sq))
        {
            return mekf_record_zaru_result(filter,
                                           ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE);
        }

        const float bias_correction_norm = sqrtf(fmaxf(0.0f,
                                                       bias_correction_sq));
        if (bias_correction_norm > max_bias_correction_rad_s)
        {
            const float gain_scale =
                max_bias_correction_rad_s / bias_correction_norm;
            for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
            {
                correction[row] *= gain_scale;
                for (size_t column = 0U;
                     column < ATTITUDE_MEKF_VECTOR_DIM;
                     ++column)
                {
                    gain[row][column] *= gain_scale;
                }
            }
        }
    }

    const float attitude_correction_norm = mekf_vector_norm(correction);
    if ((!isfinite(attitude_correction_norm)) ||
        (attitude_correction_norm > filter->config.max_attitude_correction_rad))
    {
        return mekf_record_zaru_result(filter,
                                       ATTITUDE_MEKF_ZARU_REJECTED_CORRECTION);
    }

    float corrected_bias[ATTITUDE_MEKF_VECTOR_DIM];
    for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
    {
        corrected_bias[axis] = filter->gyro_bias_rad_s[axis] +
                               correction[axis + ATTITUDE_MEKF_VECTOR_DIM];
        if ((!isfinite(corrected_bias[axis])) ||
            (fabsf(corrected_bias[axis]) > filter->config.max_abs_bias_rad_s))
        {
            return mekf_record_zaru_result(filter,
                                           ATTITUDE_MEKF_ZARU_REJECTED_CORRECTION);
        }
    }

    float correction_quaternion[4];
    float corrected_quaternion[4];
    if (!mekf_rotation_vector_to_quaternion(correction, correction_quaternion))
    {
        return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE);
    }
    mekf_quaternion_multiply(filter->q, correction_quaternion, corrected_quaternion);
    if (!mekf_normalize_quaternion(corrected_quaternion))
    {
        return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE);
    }

    float joseph_factor[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    mekf_matrix6_identity(joseph_factor);
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t axis = 0U; axis < ATTITUDE_MEKF_VECTOR_DIM; ++axis)
        {
            joseph_factor[row][axis + ATTITUDE_MEKF_VECTOR_DIM] -= gain[row][axis];
        }
    }

    float factor_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    float joseph_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                factor_covariance[row][column] += joseph_factor[row][inner] *
                                                  filter->covariance[inner][column];
            }
        }
    }
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                joseph_covariance[row][column] += factor_covariance[row][inner] *
                                                  joseph_factor[column][inner];
            }
            for (size_t first = 0U; first < ATTITUDE_MEKF_VECTOR_DIM; ++first)
            {
                for (size_t second = 0U; second < ATTITUDE_MEKF_VECTOR_DIM; ++second)
                {
                    joseph_covariance[row][column] +=
                        gain[row][first] * measurement_covariance[first][second] *
                        gain[column][second];
                }
            }
        }
    }

    float reset_jacobian[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    float correction_skew[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];
    mekf_matrix6_identity(reset_jacobian);
    mekf_skew(correction, correction_skew);
    for (size_t row = 0U; row < ATTITUDE_MEKF_VECTOR_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_VECTOR_DIM; ++column)
        {
            reset_jacobian[row][column] -= 0.5f * correction_skew[row][column];
        }
    }

    float reset_intermediate[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    float corrected_covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM] = {{0.0f}};
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                reset_intermediate[row][column] += reset_jacobian[row][inner] *
                                                   joseph_covariance[inner][column];
            }
        }
    }
    for (size_t row = 0U; row < ATTITUDE_MEKF_STATE_DIM; ++row)
    {
        for (size_t column = 0U; column < ATTITUDE_MEKF_STATE_DIM; ++column)
        {
            for (size_t inner = 0U; inner < ATTITUDE_MEKF_STATE_DIM; ++inner)
            {
                corrected_covariance[row][column] += reset_intermediate[row][inner] *
                                                     reset_jacobian[column][inner];
            }
        }
    }
    if (!mekf_condition_covariance(&filter->config, corrected_covariance))
    {
        return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE);
    }

    memcpy(filter->q, corrected_quaternion, sizeof(filter->q));
    memcpy(filter->gyro_bias_rad_s, corrected_bias, sizeof(filter->gyro_bias_rad_s));
    memcpy(filter->covariance, corrected_covariance, sizeof(filter->covariance));
    return mekf_record_zaru_result(filter, ATTITUDE_MEKF_ZARU_ACCEPTED);
}

attitude_mekf_zaru_result_t attitude_mekf_update_zero_rate(
    attitude_mekf_t *filter,
    const float gyro_rad_s[ATTITUDE_MEKF_VECTOR_DIM],
    const float measurement_covariance[ATTITUDE_MEKF_VECTOR_DIM]
                                      [ATTITUDE_MEKF_VECTOR_DIM])
{
    return mekf_update_zero_rate(filter, gyro_rad_s, measurement_covariance,
                                 false, 0.0f, 0.0f, NULL);
}

attitude_mekf_zaru_result_t attitude_mekf_update_zero_rate_bounded(
    attitude_mekf_t *filter,
    const float gyro_rad_s[ATTITUDE_MEKF_VECTOR_DIM],
    const float measurement_covariance[ATTITUDE_MEKF_VECTOR_DIM]
                                      [ATTITUDE_MEKF_VECTOR_DIM],
    float target_residual_limit_rad_s,
    float max_bias_correction_rad_s,
    float bounded_target_rad_s[ATTITUDE_MEKF_VECTOR_DIM])
{
    return mekf_update_zero_rate(filter, gyro_rad_s, measurement_covariance,
                                 true, target_residual_limit_rad_s,
                                 max_bias_correction_rad_s,
                                 bounded_target_rad_s);
}

bool attitude_mekf_get_quaternion(const attitude_mekf_t *filter, float quaternion[4])
{
    if ((quaternion == NULL) || (!attitude_mekf_is_valid(filter)))
    {
        return false;
    }
    memcpy(quaternion, filter->q, sizeof(filter->q));
    return true;
}

bool attitude_mekf_get_euler(const attitude_mekf_t *filter,
                             float euler_rad[ATTITUDE_MEKF_VECTOR_DIM])
{
    if ((euler_rad == NULL) || (!attitude_mekf_is_valid(filter)))
    {
        return false;
    }

    const float sin_roll_cos_pitch =
        2.0f * ((filter->q[0] * filter->q[1]) + (filter->q[2] * filter->q[3]));
    const float cos_roll_cos_pitch =
        1.0f - (2.0f * ((filter->q[1] * filter->q[1]) +
                        (filter->q[2] * filter->q[2])));
    const float sin_pitch = mekf_clampf(
        2.0f * ((filter->q[0] * filter->q[2]) -
                (filter->q[3] * filter->q[1])),
        -1.0f,
        1.0f);
    const float sin_yaw_cos_pitch =
        2.0f * ((filter->q[0] * filter->q[3]) + (filter->q[1] * filter->q[2]));
    const float cos_yaw_cos_pitch =
        1.0f - (2.0f * ((filter->q[2] * filter->q[2]) +
                        (filter->q[3] * filter->q[3])));

    euler_rad[0] = atan2f(sin_roll_cos_pitch, cos_roll_cos_pitch);
    euler_rad[1] = asinf(sin_pitch);
    euler_rad[2] = atan2f(sin_yaw_cos_pitch, cos_yaw_cos_pitch);
    return mekf_vector_is_finite(euler_rad);
}

bool attitude_mekf_get_bias(const attitude_mekf_t *filter,
                            float gyro_bias_rad_s[ATTITUDE_MEKF_VECTOR_DIM])
{
    if ((gyro_bias_rad_s == NULL) || (!attitude_mekf_is_valid(filter)))
    {
        return false;
    }
    memcpy(gyro_bias_rad_s, filter->gyro_bias_rad_s, sizeof(filter->gyro_bias_rad_s));
    return true;
}
