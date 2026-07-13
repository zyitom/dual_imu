#include "imu_preintegrator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_PREINTEGRATOR_GYRO_BAD_FLAGS                                      \
    (IMU_PREINTEGRATOR_FLAG_GYRO_MISSING |                                   \
     IMU_PREINTEGRATOR_FLAG_GYRO_INVALID |                                   \
     IMU_PREINTEGRATOR_FLAG_GYRO_GAP |                                       \
     IMU_PREINTEGRATOR_FLAG_GYRO_SEQUENCE_DROP |                             \
     IMU_PREINTEGRATOR_FLAG_GYRO_TIMESTAMP_ORDER |                           \
     IMU_PREINTEGRATOR_FLAG_GYRO_QUEUE_OVERFLOW |                            \
     IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC)

#define IMU_PREINTEGRATOR_ACCEL_BAD_FLAGS                                     \
    (IMU_PREINTEGRATOR_FLAG_ACCEL_MISSING |                                  \
     IMU_PREINTEGRATOR_FLAG_ACCEL_INVALID |                                  \
     IMU_PREINTEGRATOR_FLAG_ACCEL_GAP |                                      \
     IMU_PREINTEGRATOR_FLAG_ACCEL_SEQUENCE_DROP |                            \
     IMU_PREINTEGRATOR_FLAG_ACCEL_TIMESTAMP_ORDER |                          \
     IMU_PREINTEGRATOR_FLAG_ACCEL_QUEUE_OVERFLOW |                           \
     IMU_PREINTEGRATOR_FLAG_ACCEL_NUMERIC)

#define IMU_PREINTEGRATOR_GYRO_PROPAGATION_BAD_FLAGS                          \
    (IMU_PREINTEGRATOR_FLAG_GYRO_MISSING |                                   \
     IMU_PREINTEGRATOR_FLAG_GYRO_INVALID |                                   \
     IMU_PREINTEGRATOR_FLAG_GYRO_GAP |                                       \
     IMU_PREINTEGRATOR_FLAG_GYRO_TIMESTAMP_ORDER |                           \
     IMU_PREINTEGRATOR_FLAG_GYRO_QUEUE_OVERFLOW |                            \
     IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC)

#define IMU_PREINTEGRATOR_US_TO_S (1.0e-6f)
#define IMU_PREINTEGRATOR_NORM_MIN (1.0e-12f)

static bool preintegrator_source_is_valid(imu_source_t source)
{
    return (source == IMU_SOURCE_BMI088) || (source == IMU_SOURCE_ICM45686);
}

static bool preintegrator_vector_is_finite(const float vector[3])
{
    return (vector != NULL) && isfinite(vector[0]) && isfinite(vector[1]) &&
           isfinite(vector[2]);
}

static void preintegrator_cross(const float lhs[3],
                                const float rhs[3],
                                float result[3])
{
    result[0] = (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]);
    result[1] = (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]);
    result[2] = (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]);
}

static bool preintegrator_normalize_quaternion(float quaternion[4])
{
    const float norm_sq = (quaternion[0] * quaternion[0]) +
                          (quaternion[1] * quaternion[1]) +
                          (quaternion[2] * quaternion[2]) +
                          (quaternion[3] * quaternion[3]);
    if ((!isfinite(norm_sq)) || (norm_sq < IMU_PREINTEGRATOR_NORM_MIN))
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

static bool preintegrator_rotation_vector_to_quaternion(
    const float rotation_vector[3],
    float quaternion[4])
{
    const float angle_sq = (rotation_vector[0] * rotation_vector[0]) +
                           (rotation_vector[1] * rotation_vector[1]) +
                           (rotation_vector[2] * rotation_vector[2]);
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
    return preintegrator_normalize_quaternion(quaternion);
}

static void preintegrator_quaternion_multiply(const float lhs[4],
                                              const float rhs[4],
                                              float result[4])
{
    result[0] = (lhs[0] * rhs[0]) - (lhs[1] * rhs[1]) -
                (lhs[2] * rhs[2]) - (lhs[3] * rhs[3]);
    result[1] = (lhs[0] * rhs[1]) + (lhs[1] * rhs[0]) +
                (lhs[2] * rhs[3]) - (lhs[3] * rhs[2]);
    result[2] = (lhs[0] * rhs[2]) - (lhs[1] * rhs[3]) +
                (lhs[2] * rhs[0]) + (lhs[3] * rhs[1]);
    result[3] = (lhs[0] * rhs[3]) + (lhs[1] * rhs[2]) -
                (lhs[2] * rhs[1]) + (lhs[3] * rhs[0]);
}

static bool preintegrator_quaternion_to_rotation_vector(
    const float input[4],
    float rotation_vector[3])
{
    float quaternion[4] = {input[0], input[1], input[2], input[3]};
    if (!preintegrator_normalize_quaternion(quaternion))
    {
        return false;
    }

    /* The 2.5 ms operating window is far below pi even at the sensor FSR. */
    if (quaternion[0] < 0.0f)
    {
        for (size_t index = 0U; index < 4U; ++index)
        {
            quaternion[index] = -quaternion[index];
        }
    }

    const float vector_norm = sqrtf((quaternion[1] * quaternion[1]) +
                                    (quaternion[2] * quaternion[2]) +
                                    (quaternion[3] * quaternion[3]));
    float scale;
    if (vector_norm < 1.0e-7f)
    {
        scale = 2.0f;
    }
    else
    {
        scale = 2.0f * atan2f(vector_norm, quaternion[0]) / vector_norm;
    }

    rotation_vector[0] = scale * quaternion[1];
    rotation_vector[1] = scale * quaternion[2];
    rotation_vector[2] = scale * quaternion[3];
    return preintegrator_vector_is_finite(rotation_vector);
}

static bool preintegrator_compose_linear_rate_interval(
    const float left_rate[3],
    const float right_rate[3],
    uint64_t duration_us,
    float accumulated_quaternion[4])
{
    if ((duration_us == 0U) ||
        !preintegrator_vector_is_finite(left_rate) ||
        !preintegrator_vector_is_finite(right_rate) ||
        (accumulated_quaternion == NULL))
        return false;

    const float dt_s = (float)duration_us * IMU_PREINTEGRATOR_US_TO_S;
    float cross_rate[3];
    float interval_delta[3];
    preintegrator_cross(left_rate, right_rate, cross_rate);
    for (size_t axis = 0U; axis < IMU_PREINTEGRATOR_VECTOR_DIM; ++axis)
    {
        interval_delta[axis] =
            (0.5f * (left_rate[axis] + right_rate[axis]) * dt_s) +
            ((dt_s * dt_s / 12.0f) * cross_rate[axis]);
    }

    float interval_quaternion[4];
    float composed[4];
    if (!preintegrator_rotation_vector_to_quaternion(interval_delta,
                                                      interval_quaternion))
        return false;
    preintegrator_quaternion_multiply(accumulated_quaternion,
                                      interval_quaternion,
                                      composed);
    if (!preintegrator_normalize_quaternion(composed))
        return false;
    memcpy(accumulated_quaternion, composed, sizeof(composed));
    return true;
}

static uint16_t preintegrator_queue_index(uint16_t head, uint16_t offset)
{
    return (uint16_t)((head + offset) % IMU_PREINTEGRATOR_QUEUE_CAPACITY);
}

static void preintegrator_lerp_vector(const float lhs[3],
                                      const float rhs[3],
                                      double fraction,
                                      float result[3])
{
    const float ratio = (float)fraction;
    for (size_t axis = 0U; axis < 3U; ++axis)
    {
        result[axis] = lhs[axis] + (ratio * (rhs[axis] - lhs[axis]));
    }
}

static uint32_t preintegrator_sequence_flags(uint32_t previous,
                                             uint32_t current,
                                             uint32_t flag,
                                             uint64_t *discontinuities,
                                             uint64_t *estimated_drops)
{
    const uint32_t delta = current - previous;
    if (delta == 1U)
    {
        return 0U;
    }

    (*discontinuities)++;
    if ((delta > 1U) && (delta < 0x80000000U))
    {
        *estimated_drops += (uint64_t)(delta - 1U);
    }
    return flag;
}

static imu_preintegrator_gyro_node_t *preintegrator_gyro_node(
    imu_preintegrator_t *preintegrator,
    uint16_t offset)
{
    return &preintegrator->gyro_queue[
        preintegrator_queue_index(preintegrator->gyro_head, offset)];
}

static const imu_preintegrator_gyro_node_t *preintegrator_const_gyro_node(
    const imu_preintegrator_t *preintegrator,
    uint16_t offset)
{
    return &preintegrator->gyro_queue[
        preintegrator_queue_index(preintegrator->gyro_head, offset)];
}

static imu_preintegrator_accel_node_t *preintegrator_accel_node(
    imu_preintegrator_t *preintegrator,
    uint16_t offset)
{
    return &preintegrator->accel_queue[
        preintegrator_queue_index(preintegrator->accel_head, offset)];
}

static const imu_preintegrator_accel_node_t *preintegrator_const_accel_node(
    const imu_preintegrator_t *preintegrator,
    uint16_t offset)
{
    return &preintegrator->accel_queue[
        preintegrator_queue_index(preintegrator->accel_head, offset)];
}

void imu_preintegrator_default_config(imu_preintegrator_config_t *config)
{
    if (config != NULL)
    {
        config->window_us = IMU_PREINTEGRATOR_DEFAULT_WINDOW_US;
        /* Safe during 400 Hz bring-up; tighten after selecting final ODRs. */
        config->max_gyro_gap_us = 5000U;
        config->max_accel_gap_us = 5000U;
    }
}

bool imu_preintegrator_init(imu_preintegrator_t *preintegrator,
                            const imu_preintegrator_config_t *config,
                            imu_source_t source,
                            uint64_t epoch_us)
{
    if ((preintegrator == NULL) || (config == NULL) ||
        (!preintegrator_source_is_valid(source)) || (config->window_us == 0U) ||
        ((UINT64_MAX - epoch_us) < config->window_us))
    {
        return false;
    }

    memset(preintegrator, 0, sizeof(*preintegrator));
    preintegrator->config = *config;
    preintegrator->source = source;
    preintegrator->next_window_start_us = epoch_us;
    preintegrator->initialized = true;
    return true;
}

void imu_preintegrator_reset(imu_preintegrator_t *preintegrator,
                             uint64_t epoch_us)
{
    if ((preintegrator == NULL) || (!preintegrator->initialized) ||
        ((UINT64_MAX - epoch_us) < preintegrator->config.window_us))
    {
        return;
    }

    const imu_preintegrator_config_t config = preintegrator->config;
    const imu_source_t source = preintegrator->source;
    (void)imu_preintegrator_init(preintegrator, &config, source, epoch_us);
}

bool imu_preintegrator_push_gyro(imu_preintegrator_t *preintegrator,
                                 const imu_gyro_sample_t *sample)
{
    if ((preintegrator == NULL) || (!preintegrator->initialized) ||
        (sample == NULL) || (sample->source != preintegrator->source))
    {
        return false;
    }

    uint32_t interval_flags = preintegrator->pending_gyro_flags;
    if (preintegrator->gyro_count > 0U)
    {
        const imu_preintegrator_gyro_node_t *previous =
            preintegrator_const_gyro_node(preintegrator,
                                          (uint16_t)(preintegrator->gyro_count - 1U));
        if (sample->timestamp_us <= previous->sample.timestamp_us)
        {
            preintegrator->diagnostics.gyro_out_of_order_samples++;
            preintegrator->pending_gyro_flags |=
                IMU_PREINTEGRATOR_FLAG_GYRO_TIMESTAMP_ORDER;
            return false;
        }

        const uint64_t gap_us = sample->timestamp_us - previous->sample.timestamp_us;
        if (gap_us > preintegrator->diagnostics.maximum_gyro_gap_us)
        {
            preintegrator->diagnostics.maximum_gyro_gap_us = gap_us;
        }
        if ((preintegrator->config.max_gyro_gap_us != 0U) &&
            (gap_us > preintegrator->config.max_gyro_gap_us))
        {
            interval_flags |= IMU_PREINTEGRATOR_FLAG_GYRO_GAP;
            preintegrator->diagnostics.gyro_gap_count++;
        }
        interval_flags |= preintegrator_sequence_flags(
            previous->sample.sequence,
            sample->sequence,
            IMU_PREINTEGRATOR_FLAG_GYRO_SEQUENCE_DROP,
            &preintegrator->diagnostics.gyro_sequence_discontinuities,
            &preintegrator->diagnostics.gyro_estimated_dropped_samples);
    }

    if (preintegrator->gyro_count >= IMU_PREINTEGRATOR_QUEUE_CAPACITY)
    {
        preintegrator->diagnostics.gyro_queue_overflows++;
        preintegrator->pending_gyro_flags |=
            IMU_PREINTEGRATOR_FLAG_GYRO_QUEUE_OVERFLOW;
        return false;
    }

    const bool numeric_valid = preintegrator_vector_is_finite(sample->gyro_rad_s);
    if (!sample->valid)
    {
        interval_flags |= IMU_PREINTEGRATOR_FLAG_GYRO_INVALID;
        preintegrator->diagnostics.gyro_invalid_samples++;
    }
    if (!numeric_valid)
    {
        interval_flags |= IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC |
                          IMU_PREINTEGRATOR_FLAG_GYRO_INVALID;
        if (sample->valid)
        {
            preintegrator->diagnostics.gyro_invalid_samples++;
        }
    }

    imu_preintegrator_gyro_node_t *node =
        preintegrator_gyro_node(preintegrator, preintegrator->gyro_count);
    node->sample = *sample;
    node->sample.valid = sample->valid && numeric_valid;
    node->interval_flags = interval_flags;
    preintegrator->gyro_count++;
    preintegrator->pending_gyro_flags = 0U;
    preintegrator->diagnostics.gyro_samples_accepted++;
    return true;
}

bool imu_preintegrator_push_accel(imu_preintegrator_t *preintegrator,
                                  const imu_accel_sample_t *sample)
{
    if ((preintegrator == NULL) || (!preintegrator->initialized) ||
        (sample == NULL) || (sample->source != preintegrator->source))
    {
        return false;
    }

    uint32_t interval_flags = preintegrator->pending_accel_flags;
    if (preintegrator->accel_count > 0U)
    {
        const imu_preintegrator_accel_node_t *previous =
            preintegrator_const_accel_node(
                preintegrator, (uint16_t)(preintegrator->accel_count - 1U));
        if (sample->timestamp_us <= previous->sample.timestamp_us)
        {
            preintegrator->diagnostics.accel_out_of_order_samples++;
            preintegrator->pending_accel_flags |=
                IMU_PREINTEGRATOR_FLAG_ACCEL_TIMESTAMP_ORDER;
            return false;
        }

        const uint64_t gap_us = sample->timestamp_us - previous->sample.timestamp_us;
        if (gap_us > preintegrator->diagnostics.maximum_accel_gap_us)
        {
            preintegrator->diagnostics.maximum_accel_gap_us = gap_us;
        }
        if ((preintegrator->config.max_accel_gap_us != 0U) &&
            (gap_us > preintegrator->config.max_accel_gap_us))
        {
            interval_flags |= IMU_PREINTEGRATOR_FLAG_ACCEL_GAP;
            preintegrator->diagnostics.accel_gap_count++;
        }
        interval_flags |= preintegrator_sequence_flags(
            previous->sample.sequence,
            sample->sequence,
            IMU_PREINTEGRATOR_FLAG_ACCEL_SEQUENCE_DROP,
            &preintegrator->diagnostics.accel_sequence_discontinuities,
            &preintegrator->diagnostics.accel_estimated_dropped_samples);
    }

    if (preintegrator->accel_count >= IMU_PREINTEGRATOR_QUEUE_CAPACITY)
    {
        preintegrator->diagnostics.accel_queue_overflows++;
        preintegrator->pending_accel_flags |=
            IMU_PREINTEGRATOR_FLAG_ACCEL_QUEUE_OVERFLOW;
        return false;
    }

    const bool numeric_valid = preintegrator_vector_is_finite(sample->accel_mps2);
    if (!sample->valid)
    {
        interval_flags |= IMU_PREINTEGRATOR_FLAG_ACCEL_INVALID;
        preintegrator->diagnostics.accel_invalid_samples++;
    }
    if (!numeric_valid)
    {
        interval_flags |= IMU_PREINTEGRATOR_FLAG_ACCEL_NUMERIC |
                          IMU_PREINTEGRATOR_FLAG_ACCEL_INVALID;
        if (sample->valid)
        {
            preintegrator->diagnostics.accel_invalid_samples++;
        }
    }

    imu_preintegrator_accel_node_t *node =
        preintegrator_accel_node(preintegrator, preintegrator->accel_count);
    node->sample = *sample;
    node->sample.valid = sample->valid && numeric_valid;
    node->interval_flags = interval_flags;
    preintegrator->accel_count++;
    preintegrator->pending_accel_flags = 0U;
    preintegrator->diagnostics.accel_samples_accepted++;
    return true;
}

static void preintegrator_integrate_gyro(imu_preintegrator_t *preintegrator,
                                        uint64_t start_us,
                                        uint64_t end_us,
                                        imu_preintegrated_window_t *window)
{
    float accumulated_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float first_half_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float second_half_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float rate_integral[IMU_PREINTEGRATOR_VECTOR_DIM] = {0.0f};
    float second_moment_integral[IMU_PREINTEGRATOR_VECTOR_DIM]
                                [IMU_PREINTEGRATOR_VECTOR_DIM] = {{0.0f}};
    uint64_t coverage_us = 0U;
    bool have_start_rate = false;
    const uint64_t midpoint_us = start_us + ((end_us - start_us) / 2U);

    for (uint16_t offset = 1U; offset < preintegrator->gyro_count; ++offset)
    {
        const imu_preintegrator_gyro_node_t *left =
            preintegrator_const_gyro_node(preintegrator, (uint16_t)(offset - 1U));
        const imu_preintegrator_gyro_node_t *right =
            preintegrator_const_gyro_node(preintegrator, offset);
        const uint64_t left_us = left->sample.timestamp_us;
        const uint64_t right_us = right->sample.timestamp_us;

        if ((right_us <= start_us) || (left_us >= end_us))
        {
            continue;
        }

        const uint64_t clipped_left_us = (left_us > start_us) ? left_us : start_us;
        const uint64_t clipped_right_us = (right_us < end_us) ? right_us : end_us;
        if (clipped_right_us <= clipped_left_us)
        {
            continue;
        }

        window->flags |= right->interval_flags;
        if ((!left->sample.valid) || (!right->sample.valid))
        {
            window->flags |= IMU_PREINTEGRATOR_FLAG_GYRO_INVALID;
            if ((!preintegrator_vector_is_finite(left->sample.gyro_rad_s)) ||
                (!preintegrator_vector_is_finite(right->sample.gyro_rad_s)))
            {
                window->flags |= IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC;
            }
            continue;
        }

        const double interval_us = (double)(right_us - left_us);
        const double left_ratio = (double)(clipped_left_us - left_us) / interval_us;
        const double right_ratio = (double)(clipped_right_us - left_us) / interval_us;
        float left_rate[3];
        float right_rate[3];
        preintegrator_lerp_vector(left->sample.gyro_rad_s,
                                  right->sample.gyro_rad_s,
                                  left_ratio,
                                  left_rate);
        preintegrator_lerp_vector(left->sample.gyro_rad_s,
                                  right->sample.gyro_rad_s,
                                  right_ratio,
                                  right_rate);

        const uint64_t duration_us = clipped_right_us - clipped_left_us;
        const float dt_s = (float)duration_us * IMU_PREINTEGRATOR_US_TO_S;
        if (!preintegrator_compose_linear_rate_interval(
                left_rate, right_rate, duration_us, accumulated_quaternion))
        {
            window->flags |= IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC;
            continue;
        }

        bool half_composed;
        if (clipped_right_us <= midpoint_us)
        {
            half_composed = preintegrator_compose_linear_rate_interval(
                left_rate, right_rate, duration_us, first_half_quaternion);
        }
        else if (clipped_left_us >= midpoint_us)
        {
            half_composed = preintegrator_compose_linear_rate_interval(
                left_rate, right_rate, duration_us, second_half_quaternion);
        }
        else
        {
            float midpoint_rate[3];
            const double midpoint_ratio =
                (double)(midpoint_us - clipped_left_us) /
                (double)duration_us;
            preintegrator_lerp_vector(left_rate, right_rate,
                                      midpoint_ratio, midpoint_rate);
            half_composed = preintegrator_compose_linear_rate_interval(
                                left_rate, midpoint_rate,
                                midpoint_us - clipped_left_us,
                                first_half_quaternion) &&
                            preintegrator_compose_linear_rate_interval(
                                midpoint_rate, right_rate,
                                clipped_right_us - midpoint_us,
                                second_half_quaternion);
        }
        if (!half_composed)
        {
            window->flags |= IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC;
            coverage_us = 0U;
            break;
        }

        for (size_t axis = 0U; axis < 3U; ++axis)
        {
            rate_integral[axis] +=
                0.5f * (left_rate[axis] + right_rate[axis]) * dt_s;
        }
        for (size_t row = 0U; row < IMU_PREINTEGRATOR_VECTOR_DIM; ++row)
        {
            for (size_t column = 0U;
                 column < IMU_PREINTEGRATOR_VECTOR_DIM;
                 ++column)
            {
                second_moment_integral[row][column] +=
                    (dt_s / 6.0f) *
                    ((2.0f * left_rate[row] * left_rate[column]) +
                     (left_rate[row] * right_rate[column]) +
                     (right_rate[row] * left_rate[column]) +
                     (2.0f * right_rate[row] * right_rate[column]));
            }
        }

        if (!have_start_rate)
        {
            memcpy(window->gyro_start_rad_s, left_rate,
                   sizeof(window->gyro_start_rad_s));
            have_start_rate = true;
        }
        memcpy(window->gyro_end_rad_s, right_rate,
               sizeof(window->gyro_end_rad_s));
        coverage_us += duration_us;
        if (window->gyro_segment_count != UINT16_MAX)
        {
            window->gyro_segment_count++;
        }
    }

    memcpy(window->delta_quaternion,
           accumulated_quaternion,
           sizeof(window->delta_quaternion));
    if (!preintegrator_quaternion_to_rotation_vector(accumulated_quaternion,
                                                     window->delta_angle_rad))
    {
        window->flags |= IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC;
        memset(window->delta_angle_rad, 0, sizeof(window->delta_angle_rad));
    }
    if (!preintegrator_quaternion_to_rotation_vector(
            first_half_quaternion, window->first_half_delta_angle_rad) ||
        !preintegrator_quaternion_to_rotation_vector(
            second_half_quaternion, window->second_half_delta_angle_rad))
    {
        window->flags |= IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC;
        memset(window->first_half_delta_angle_rad, 0,
               sizeof(window->first_half_delta_angle_rad));
        memset(window->second_half_delta_angle_rad, 0,
               sizeof(window->second_half_delta_angle_rad));
    }

    const uint64_t window_duration_us = end_us - start_us;
    window->gyro_coverage_us = (coverage_us > UINT32_MAX)
                                   ? UINT32_MAX
                                   : (uint32_t)coverage_us;
    if (coverage_us != window_duration_us)
    {
        window->flags |= IMU_PREINTEGRATOR_FLAG_GYRO_MISSING;
    }
    if (coverage_us != 0U)
    {
        const float inverse_coverage_s =
            1.0f / ((float)coverage_us * IMU_PREINTEGRATOR_US_TO_S);
        for (size_t row = 0U; row < IMU_PREINTEGRATOR_VECTOR_DIM; ++row)
        {
            window->gyro_mean_rad_s[row] =
                rate_integral[row] * inverse_coverage_s;
            for (size_t column = 0U;
                 column < IMU_PREINTEGRATOR_VECTOR_DIM;
                 ++column)
            {
                window->gyro_second_moment_rad2_s2[row][column] =
                    second_moment_integral[row][column] * inverse_coverage_s;
            }
        }
    }
}

static void preintegrator_integrate_accel(imu_preintegrator_t *preintegrator,
                                         uint64_t start_us,
                                         uint64_t end_us,
                                         imu_preintegrated_window_t *window)
{
    float integral[3] = {0.0f, 0.0f, 0.0f};
    uint64_t coverage_us = 0U;

    for (uint16_t offset = 1U; offset < preintegrator->accel_count; ++offset)
    {
        const imu_preintegrator_accel_node_t *left =
            preintegrator_const_accel_node(preintegrator,
                                           (uint16_t)(offset - 1U));
        const imu_preintegrator_accel_node_t *right =
            preintegrator_const_accel_node(preintegrator, offset);
        const uint64_t left_us = left->sample.timestamp_us;
        const uint64_t right_us = right->sample.timestamp_us;

        if ((right_us <= start_us) || (left_us >= end_us))
        {
            continue;
        }

        const uint64_t clipped_left_us = (left_us > start_us) ? left_us : start_us;
        const uint64_t clipped_right_us = (right_us < end_us) ? right_us : end_us;
        if (clipped_right_us <= clipped_left_us)
        {
            continue;
        }

        window->flags |= right->interval_flags;
        if ((!left->sample.valid) || (!right->sample.valid))
        {
            window->flags |= IMU_PREINTEGRATOR_FLAG_ACCEL_INVALID;
            if ((!preintegrator_vector_is_finite(left->sample.accel_mps2)) ||
                (!preintegrator_vector_is_finite(right->sample.accel_mps2)))
            {
                window->flags |= IMU_PREINTEGRATOR_FLAG_ACCEL_NUMERIC;
            }
            continue;
        }

        const double interval_us = (double)(right_us - left_us);
        const double left_ratio = (double)(clipped_left_us - left_us) / interval_us;
        const double right_ratio = (double)(clipped_right_us - left_us) / interval_us;
        float left_accel[3];
        float right_accel[3];
        preintegrator_lerp_vector(left->sample.accel_mps2,
                                  right->sample.accel_mps2,
                                  left_ratio,
                                  left_accel);
        preintegrator_lerp_vector(left->sample.accel_mps2,
                                  right->sample.accel_mps2,
                                  right_ratio,
                                  right_accel);

        const uint64_t duration_us = clipped_right_us - clipped_left_us;
        const float dt_s = (float)duration_us * IMU_PREINTEGRATOR_US_TO_S;
        for (size_t axis = 0U; axis < 3U; ++axis)
        {
            integral[axis] += 0.5f * (left_accel[axis] + right_accel[axis]) * dt_s;
        }
        coverage_us += duration_us;
        if (window->accel_segment_count != UINT16_MAX)
        {
            window->accel_segment_count++;
        }
    }

    if (coverage_us > 0U)
    {
        const float inverse_duration =
            1.0f / ((float)coverage_us * IMU_PREINTEGRATOR_US_TO_S);
        for (size_t axis = 0U; axis < 3U; ++axis)
        {
            window->accel_mean_mps2[axis] = integral[axis] * inverse_duration;
        }
    }

    const uint64_t window_duration_us = end_us - start_us;
    window->accel_coverage_us = (coverage_us > UINT32_MAX)
                                    ? UINT32_MAX
                                    : (uint32_t)coverage_us;
    if (coverage_us != window_duration_us)
    {
        window->flags |= IMU_PREINTEGRATOR_FLAG_ACCEL_MISSING;
    }
}

static void preintegrator_prune_gyro(imu_preintegrator_t *preintegrator,
                                    uint64_t end_us)
{
    uint16_t retain_offset = 0U;
    bool found = false;
    for (uint16_t offset = 0U; offset < preintegrator->gyro_count; ++offset)
    {
        if (preintegrator_const_gyro_node(preintegrator, offset)->sample.timestamp_us <=
            end_us)
        {
            retain_offset = offset;
            found = true;
        }
        else
        {
            break;
        }
    }

    if (found && (retain_offset > 0U))
    {
        preintegrator->gyro_head = preintegrator_queue_index(
            preintegrator->gyro_head, retain_offset);
        preintegrator->gyro_count =
            (uint16_t)(preintegrator->gyro_count - retain_offset);
    }
}

static void preintegrator_prune_accel(imu_preintegrator_t *preintegrator,
                                     uint64_t end_us)
{
    uint16_t retain_offset = 0U;
    bool found = false;
    for (uint16_t offset = 0U; offset < preintegrator->accel_count; ++offset)
    {
        if (preintegrator_const_accel_node(preintegrator, offset)->sample.timestamp_us <=
            end_us)
        {
            retain_offset = offset;
            found = true;
        }
        else
        {
            break;
        }
    }

    if (found && (retain_offset > 0U))
    {
        preintegrator->accel_head = preintegrator_queue_index(
            preintegrator->accel_head, retain_offset);
        preintegrator->accel_count =
            (uint16_t)(preintegrator->accel_count - retain_offset);
    }
}

static bool preintegrator_gyro_can_close(
    const imu_preintegrator_t *preintegrator,
    uint64_t end_us,
    uint64_t complete_through_us)
{
    if (preintegrator->gyro_count > 0U)
    {
        const imu_preintegrator_gyro_node_t *last =
            preintegrator_const_gyro_node(
                preintegrator, (uint16_t)(preintegrator->gyro_count - 1U));
        if (last->sample.timestamp_us >= end_us)
        {
            return true;
        }
    }

    const uint32_t grace_us = (preintegrator->config.max_gyro_gap_us != 0U)
                                  ? preintegrator->config.max_gyro_gap_us
                                  : preintegrator->config.window_us;
    return (complete_through_us >= end_us) &&
           ((complete_through_us - end_us) >= grace_us);
}

static bool preintegrator_accel_can_close(
    const imu_preintegrator_t *preintegrator,
    uint64_t end_us,
    uint64_t complete_through_us)
{
    if (preintegrator->accel_count > 0U)
    {
        const imu_preintegrator_accel_node_t *last =
            preintegrator_const_accel_node(
                preintegrator, (uint16_t)(preintegrator->accel_count - 1U));
        if (last->sample.timestamp_us >= end_us)
        {
            return true;
        }
    }

    const uint32_t grace_us = (preintegrator->config.max_accel_gap_us != 0U)
                                  ? preintegrator->config.max_accel_gap_us
                                  : preintegrator->config.window_us;
    return (complete_through_us >= end_us) &&
           ((complete_through_us - end_us) >= grace_us);
}

imu_preintegrator_result_t imu_preintegrator_next_window_ready(
    const imu_preintegrator_t *preintegrator,
    uint64_t complete_through_us)
{
    if ((preintegrator == NULL) || (!preintegrator->initialized) ||
        ((UINT64_MAX - preintegrator->next_window_start_us) <
         preintegrator->config.window_us))
    {
        return IMU_PREINTEGRATOR_ERROR;
    }

    const uint64_t start_us = preintegrator->next_window_start_us;
    const uint64_t end_us = start_us + preintegrator->config.window_us;
    if ((complete_through_us < end_us) ||
        (!preintegrator_gyro_can_close(preintegrator, end_us,
                                       complete_through_us)) ||
        (!preintegrator_accel_can_close(preintegrator, end_us,
                                        complete_through_us)))
    {
        return IMU_PREINTEGRATOR_NOT_READY;
    }
    return IMU_PREINTEGRATOR_WINDOW_READY;
}

imu_preintegrator_result_t imu_preintegrator_next_window(
    imu_preintegrator_t *preintegrator,
    uint64_t complete_through_us,
    imu_preintegrated_window_t *window)
{
    if ((preintegrator == NULL) || (!preintegrator->initialized) ||
        (window == NULL) ||
        ((UINT64_MAX - preintegrator->next_window_start_us) <
         preintegrator->config.window_us))
    {
        return IMU_PREINTEGRATOR_ERROR;
    }

    const imu_preintegrator_result_t readiness =
        imu_preintegrator_next_window_ready(preintegrator, complete_through_us);
    if (readiness != IMU_PREINTEGRATOR_WINDOW_READY)
    {
        return readiness;
    }

    const uint64_t start_us = preintegrator->next_window_start_us;
    const uint64_t end_us = start_us + preintegrator->config.window_us;
    memset(window, 0, sizeof(*window));
    window->source = preintegrator->source;
    window->start_us = start_us;
    window->end_us = end_us;
    window->delta_quaternion[0] = 1.0f;
    window->flags = preintegrator->pending_gyro_flags |
                    preintegrator->pending_accel_flags;
    preintegrator->pending_gyro_flags = 0U;
    preintegrator->pending_accel_flags = 0U;

    preintegrator_integrate_gyro(preintegrator, start_us, end_us, window);
    preintegrator_integrate_accel(preintegrator, start_us, end_us, window);

    window->gyro_propagation_valid =
        (window->flags & IMU_PREINTEGRATOR_GYRO_PROPAGATION_BAD_FLAGS) == 0U;
    window->gyro_valid =
        (window->flags & IMU_PREINTEGRATOR_GYRO_BAD_FLAGS) == 0U;
    window->accel_valid =
        (window->flags & IMU_PREINTEGRATOR_ACCEL_BAD_FLAGS) == 0U;

    preintegrator->diagnostics.windows_produced++;
    if (!window->gyro_valid)
    {
        preintegrator->diagnostics.gyro_invalid_windows++;
    }
    if (!window->accel_valid)
    {
        preintegrator->diagnostics.accel_invalid_windows++;
    }

    preintegrator_prune_gyro(preintegrator, end_us);
    preintegrator_prune_accel(preintegrator, end_us);
    preintegrator->next_window_start_us = end_us;
    return IMU_PREINTEGRATOR_WINDOW_READY;
}

const imu_preintegrator_diagnostics_t *imu_preintegrator_get_diagnostics(
    const imu_preintegrator_t *preintegrator)
{
    return ((preintegrator != NULL) && preintegrator->initialized)
               ? &preintegrator->diagnostics
               : NULL;
}
