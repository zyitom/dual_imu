#include "fast_attitude_predictor.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define FAST_ATTITUDE_US_TO_S (1.0e-6f)
#define FAST_ATTITUDE_DEG_TO_RAD (0.017453292519943295f)
#define FAST_ATTITUDE_HALF_PI (1.57079632679489661923f)

static bool vector_is_finite(const float vector[3])
{
    return isfinite(vector[0]) && isfinite(vector[1]) && isfinite(vector[2]);
}

static bool quaternion_is_finite(const float quaternion[4])
{
    return isfinite(quaternion[0]) && isfinite(quaternion[1]) &&
           isfinite(quaternion[2]) && isfinite(quaternion[3]);
}

static bool quaternion_normalize(float quaternion[4])
{
    const float norm_sq = (quaternion[0] * quaternion[0]) +
                          (quaternion[1] * quaternion[1]) +
                          (quaternion[2] * quaternion[2]) +
                          (quaternion[3] * quaternion[3]);
    if (!isfinite(norm_sq) || (norm_sq < 1.0e-12f))
        return false;

    const float inverse_norm = 1.0f / sqrtf(norm_sq);
    for (size_t index = 0U; index < 4U; ++index)
        quaternion[index] *= inverse_norm;
    return true;
}

static float quaternion_dot(const float lhs[4], const float rhs[4])
{
    return (lhs[0] * rhs[0]) + (lhs[1] * rhs[1]) +
           (lhs[2] * rhs[2]) + (lhs[3] * rhs[3]);
}

static void quaternion_multiply(const float lhs[4],
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

static bool rotation_vector_to_quaternion(const float rotation[3],
                                          float quaternion[4])
{
    const float angle_sq = (rotation[0] * rotation[0]) +
                           (rotation[1] * rotation[1]) +
                           (rotation[2] * rotation[2]);
    if (!isfinite(angle_sq))
        return false;

    float scale;
    if (angle_sq < 1.0e-8f) {
        quaternion[0] = 1.0f - (0.125f * angle_sq);
        scale = 0.5f - (angle_sq / 48.0f);
    } else {
        const float angle = sqrtf(angle_sq);
        quaternion[0] = cosf(0.5f * angle);
        scale = sinf(0.5f * angle) / angle;
    }
    quaternion[1] = scale * rotation[0];
    quaternion[2] = scale * rotation[1];
    quaternion[3] = scale * rotation[2];
    return quaternion_normalize(quaternion);
}

static void vector_cross(const float lhs[3],
                         const float rhs[3],
                         float result[3])
{
    result[0] = (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]);
    result[1] = (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]);
    result[2] = (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]);
}

static bool compose_linear_rate_interval(float quaternion[4],
                                         const float left_rate[3],
                                         const float right_rate[3],
                                         uint64_t duration_us)
{
    if ((duration_us == 0U) || !vector_is_finite(left_rate) ||
        !vector_is_finite(right_rate))
        return duration_us == 0U;

    const float dt_s = (float)duration_us * FAST_ATTITUDE_US_TO_S;
    float cross_rate[3];
    float delta_angle[3];
    vector_cross(left_rate, right_rate, cross_rate);
    for (size_t axis = 0U; axis < 3U; ++axis) {
        delta_angle[axis] =
            (0.5f * (left_rate[axis] + right_rate[axis]) * dt_s) +
            ((dt_s * dt_s / 12.0f) * cross_rate[axis]);
    }

    float delta_quaternion[4];
    float composed[4];
    if (!rotation_vector_to_quaternion(delta_angle, delta_quaternion))
        return false;
    quaternion_multiply(quaternion, delta_quaternion, composed);
    if (!quaternion_normalize(composed))
        return false;
    memcpy(quaternion, composed, sizeof(composed));
    return true;
}

static void quaternion_to_euler(const float quaternion[4], float euler[3])
{
    const float w = quaternion[0];
    const float x = quaternion[1];
    const float y = quaternion[2];
    const float z = quaternion[3];
    const float sin_pitch = fmaxf(
        -1.0f,
        fminf(1.0f, 2.0f * ((w * y) - (z * x))));

    euler[0] = atan2f(2.0f * ((w * x) + (y * z)),
                      1.0f - (2.0f * ((x * x) + (y * y))));
    euler[1] = asinf(sin_pitch);
    euler[2] = atan2f(2.0f * ((w * z) + (x * y)),
                      1.0f - (2.0f * ((y * y) + (z * z))));
}

static bool config_is_valid(const fast_attitude_predictor_config_t *config)
{
    if ((config == NULL) || (config->max_prediction_horizon_us == 0U) ||
        !isfinite(config->euler_singularity_threshold_rad) ||
        (config->euler_singularity_threshold_rad <= 0.0f) ||
        (config->euler_singularity_threshold_rad >= FAST_ATTITUDE_HALF_PI))
        return false;

    for (size_t source = 0U; source < IMU_SOURCE_COUNT; ++source) {
        if (config->max_gyro_gap_us[source] == 0U)
            return false;
    }
    return true;
}

static uint16_t history_index(const fast_attitude_gyro_history_t *history,
                              uint16_t offset)
{
    return (uint16_t)((history->head + offset) %
                      FAST_ATTITUDE_PREDICTOR_HISTORY_CAPACITY);
}

static const fast_attitude_gyro_node_t *history_node(
    const fast_attitude_gyro_history_t *history,
    uint16_t offset)
{
    return &history->nodes[history_index(history, offset)];
}

static void history_discard_through(fast_attitude_gyro_history_t *history,
                                    uint64_t timestamp_us)
{
    while ((history->count != 0U) &&
           (history_node(history, 0U)->sample.timestamp_us <= timestamp_us)) {
        history->head = history_index(history, 1U);
        history->count--;
    }
}

static void replay_cache_begin(fast_attitude_predictor_t *predictor)
{
    fast_attitude_replay_cache_t *const cache = &predictor->replay_cache;
    memset(cache, 0, sizeof(*cache));
    memcpy(cache->quaternion, predictor->anchor.quaternion,
           sizeof(cache->quaternion));
    for (size_t axis = 0U; axis < 3U; ++axis) {
        cache->gyro_rate_rad_s[axis] =
            predictor->anchor.gyro_rate_rad_s[axis] -
            predictor->anchor.gyro_bias_rad_s[axis];
    }
    cache->timestamp_us = predictor->anchor.timestamp_us;
    memcpy(cache->previous_quaternion, cache->quaternion,
           sizeof(cache->previous_quaternion));
    memcpy(cache->previous_gyro_rate_rad_s, cache->gyro_rate_rad_s,
           sizeof(cache->previous_gyro_rate_rad_s));
    cache->previous_timestamp_us = cache->timestamp_us;
    cache->coverage_timestamp_us = predictor->anchor.timestamp_us;
    cache->valid = true;
    if (!vector_is_finite(cache->gyro_rate_rad_s)) {
        cache->integrity_flags = FAST_ATTITUDE_FLAG_NUMERIC;
        cache->blocked = true;
    }
}

static void replay_cache_append(fast_attitude_predictor_t *predictor,
                                const fast_attitude_gyro_node_t *node)
{
    fast_attitude_replay_cache_t *const cache = &predictor->replay_cache;
    if (!cache->valid || cache->blocked ||
        (node->sample.timestamp_us <= predictor->anchor.timestamp_us))
        return;

    cache->coverage_timestamp_us = node->sample.timestamp_us;
    cache->integrity_flags |= node->interval_flags;
    if (!node->sample.valid) {
        cache->integrity_flags |= FAST_ATTITUDE_FLAG_GYRO_INVALID;
        cache->blocked = true;
        return;
    }

    const uint64_t interval_us = node->sample.timestamp_us -
                                 cache->timestamp_us;
    if (interval_us > predictor->config.max_gyro_gap_us[
                          predictor->anchor.selected_source]) {
        cache->integrity_flags |= FAST_ATTITUDE_FLAG_GYRO_GAP;
        cache->blocked = true;
        return;
    }

    float right_rate[3];
    for (size_t axis = 0U; axis < 3U; ++axis) {
        right_rate[axis] = node->sample.gyro_rad_s[axis] -
                           predictor->anchor.gyro_bias_rad_s[axis];
    }
    float next_quaternion[4];
    memcpy(next_quaternion, cache->quaternion, sizeof(next_quaternion));
    if (!compose_linear_rate_interval(next_quaternion,
                                      cache->gyro_rate_rad_s,
                                      right_rate,
                                      interval_us)) {
        cache->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
        cache->blocked = true;
        return;
    }

    memcpy(cache->previous_quaternion, cache->quaternion,
           sizeof(cache->previous_quaternion));
    memcpy(cache->previous_gyro_rate_rad_s, cache->gyro_rate_rad_s,
           sizeof(cache->previous_gyro_rate_rad_s));
    cache->previous_timestamp_us = cache->timestamp_us;
    cache->previous_integrity_flags = cache->endpoint_integrity_flags;
    memcpy(cache->quaternion, next_quaternion, sizeof(cache->quaternion));
    memcpy(cache->gyro_rate_rad_s, right_rate,
           sizeof(cache->gyro_rate_rad_s));
    cache->timestamp_us = node->sample.timestamp_us;
    cache->endpoint_integrity_flags = cache->integrity_flags;
}

static void replay_cache_rebuild(fast_attitude_predictor_t *predictor)
{
    replay_cache_begin(predictor);
    const fast_attitude_gyro_history_t *const history =
        &predictor->history[predictor->anchor.selected_source];
    for (uint16_t offset = 0U; offset < history->count; ++offset)
        replay_cache_append(predictor, history_node(history, offset));
}

void fast_attitude_predictor_default_config(
    fast_attitude_predictor_config_t *config)
{
    if (config == NULL)
        return;

    memset(config, 0, sizeof(*config));
    config->max_prediction_horizon_us = 1000U;
    config->max_gyro_gap_us[IMU_SOURCE_BMI088] = 1250U;
    config->max_gyro_gap_us[IMU_SOURCE_ICM45686] = 800U;
    config->euler_singularity_threshold_rad =
        85.0f * FAST_ATTITUDE_DEG_TO_RAD;
}

bool fast_attitude_predictor_init(
    fast_attitude_predictor_t *predictor,
    const fast_attitude_predictor_config_t *config)
{
    if ((predictor == NULL) || !config_is_valid(config))
        return false;

    memset(predictor, 0, sizeof(*predictor));
    predictor->config = *config;
    predictor->initialized = true;
    return true;
}

void fast_attitude_predictor_reset(fast_attitude_predictor_t *predictor)
{
    if (predictor == NULL)
        return;

    const fast_attitude_predictor_config_t config = predictor->config;
    const bool initialized = predictor->initialized;
    memset(predictor, 0, sizeof(*predictor));
    predictor->config = config;
    predictor->initialized = initialized;
}

void fast_attitude_predictor_invalidate_anchor(
    fast_attitude_predictor_t *predictor)
{
    if (predictor == NULL)
        return;
    predictor->anchor_valid = false;
}

bool fast_attitude_predictor_push_gyro(
    fast_attitude_predictor_t *predictor,
    const imu_gyro_sample_t *sample)
{
    if ((predictor == NULL) || !predictor->initialized || (sample == NULL) ||
        (sample->source >= IMU_SOURCE_COUNT))
        return false;

    fast_attitude_gyro_history_t *const history =
        &predictor->history[sample->source];
    const bool numeric_valid = vector_is_finite(sample->gyro_rad_s);
    uint32_t interval_flags = history->pending_flags;

    if (history->have_last_sample) {
        if (sample->timestamp_us <= history->last_timestamp_us) {
            history->pending_flags |=
                FAST_ATTITUDE_FLAG_GYRO_TIMESTAMP_ORDER;
            return false;
        }
        if ((uint32_t)(sample->sequence - history->last_sequence) != 1U)
            interval_flags |= FAST_ATTITUDE_FLAG_GYRO_SEQUENCE_GAP;
        if ((sample->timestamp_us - history->last_timestamp_us) >
            predictor->config.max_gyro_gap_us[sample->source])
            interval_flags |= FAST_ATTITUDE_FLAG_GYRO_GAP;
    }
    if (!sample->valid)
        interval_flags |= FAST_ATTITUDE_FLAG_GYRO_INVALID;
    if (!numeric_valid)
        interval_flags |= FAST_ATTITUDE_FLAG_GYRO_INVALID |
                          FAST_ATTITUDE_FLAG_NUMERIC;

    uint16_t insert_offset = history->count;
    if (history->count == FAST_ATTITUDE_PREDICTOR_HISTORY_CAPACITY) {
        history->head = history_index(history, 1U);
        insert_offset--;
    } else {
        history->count++;
    }

    fast_attitude_gyro_node_t *const node =
        &history->nodes[history_index(history, insert_offset)];
    node->sample = *sample;
    node->sample.valid = sample->valid && numeric_valid;
    node->interval_flags = interval_flags;
    history->last_timestamp_us = sample->timestamp_us;
    history->last_sequence = sample->sequence;
    history->have_last_sample = true;
    history->pending_flags = 0U;
    if (predictor->anchor_valid &&
        (sample->source == predictor->anchor.selected_source)) {
        replay_cache_append(predictor, node);
    }
    return node->sample.valid;
}

bool fast_attitude_predictor_set_anchor(
    fast_attitude_predictor_t *predictor,
    const fast_attitude_anchor_t *anchor)
{
    if ((predictor == NULL) || !predictor->initialized || (anchor == NULL) ||
        (anchor->selected_source >= IMU_SOURCE_COUNT) ||
        !vector_is_finite(anchor->gyro_rate_rad_s) ||
        !vector_is_finite(anchor->gyro_bias_rad_s))
        return false;

    fast_attitude_anchor_t normalized = *anchor;
    if (!quaternion_normalize(normalized.quaternion))
        return false;
    if (predictor->history_floor_valid &&
        (normalized.timestamp_us < predictor->history_floor_timestamp_us))
        return false;
    if (predictor->anchor_valid &&
        (quaternion_dot(normalized.quaternion,
                        predictor->anchor.quaternion) < 0.0f)) {
        for (size_t index = 0U; index < 4U; ++index)
            normalized.quaternion[index] = -normalized.quaternion[index];
    }

    for (size_t source = 0U; source < IMU_SOURCE_COUNT; ++source) {
        history_discard_through(&predictor->history[source],
                                normalized.timestamp_us);
    }
    predictor->history_floor_timestamp_us = normalized.timestamp_us;
    predictor->history_floor_valid = true;
    predictor->anchor = normalized;
    predictor->anchor_valid = true;
    replay_cache_rebuild(predictor);
    return true;
}

static void initialize_output(const fast_attitude_predictor_t *predictor,
                              uint64_t output_timestamp_us,
                              fast_attitude_output_t *output)
{
    memset(output, 0, sizeof(*output));
    output->output_timestamp_us = output_timestamp_us;
    output->selected_source = IMU_SOURCE_FUSED;
    output->integrity_flags = FAST_ATTITUDE_FLAG_NONE;

    if ((predictor == NULL) || !predictor->initialized) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_NOT_INITIALIZED;
        return;
    }
    if (!predictor->anchor_valid) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_NO_ANCHOR;
        return;
    }

    output->anchor_timestamp_us = predictor->anchor.timestamp_us;
    output->latest_gyro_timestamp_us = predictor->anchor.timestamp_us;
    output->selected_source = predictor->anchor.selected_source;
    output->degraded = predictor->anchor.degraded;
    output->attitude_converged =
        (predictor->anchor.quality_flags &
         FAST_ATTITUDE_QUALITY_ATTITUDE_CONVERGED) != 0U;
    output->post_impact_reacquire_active =
        (predictor->anchor.quality_flags &
         FAST_ATTITUDE_QUALITY_POST_IMPACT_REACQUIRE_ACTIVE) != 0U;
    output->attitude_aiding_stale =
        (predictor->anchor.quality_flags &
         FAST_ATTITUDE_QUALITY_ATTITUDE_AIDING_STALE) != 0U;
    output->rotation_unobserved =
        (predictor->anchor.quality_flags &
         FAST_ATTITUDE_QUALITY_ROTATION_UNOBSERVED) != 0U;
    memcpy(output->quaternion, predictor->anchor.quaternion,
           sizeof(output->quaternion));
}

bool fast_attitude_predictor_predict(
    const fast_attitude_predictor_t *predictor,
    uint64_t output_timestamp_us,
    fast_attitude_output_t *output)
{
    if (output == NULL)
        return false;
    initialize_output(predictor, output_timestamp_us, output);
    if ((predictor == NULL) || !predictor->initialized ||
        !predictor->anchor_valid)
        return false;
    if (output_timestamp_us < predictor->anchor.timestamp_us) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_TARGET_BEFORE_ANCHOR;
        return false;
    }

    float current_rate[3];
    uint64_t current_timestamp_us;
    const fast_attitude_replay_cache_t *const cache =
        &predictor->replay_cache;
    if (cache->valid &&
        (output_timestamp_us >= cache->coverage_timestamp_us)) {
        memcpy(output->quaternion, cache->quaternion,
               sizeof(output->quaternion));
        memcpy(current_rate, cache->gyro_rate_rad_s,
               sizeof(current_rate));
        current_timestamp_us = cache->timestamp_us;
        output->latest_gyro_timestamp_us = current_timestamp_us;
        output->integrity_flags |= cache->integrity_flags;
    } else {
        for (size_t axis = 0U; axis < 3U; ++axis) {
            current_rate[axis] = predictor->anchor.gyro_rate_rad_s[axis] -
                                 predictor->anchor.gyro_bias_rad_s[axis];
        }
        if (!vector_is_finite(current_rate)) {
            output->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
            return false;
        }

        current_timestamp_us = predictor->anchor.timestamp_us;
        const fast_attitude_gyro_history_t *const history =
            &predictor->history[predictor->anchor.selected_source];
        for (uint16_t offset = 0U; offset < history->count; ++offset) {
            const fast_attitude_gyro_node_t *const node =
                history_node(history, offset);
            if (node->sample.timestamp_us > output_timestamp_us)
                break;

            output->integrity_flags |= node->interval_flags;
            if (!node->sample.valid) {
                output->integrity_flags |= FAST_ATTITUDE_FLAG_GYRO_INVALID;
                break;
            }
            const uint64_t interval_us =
                node->sample.timestamp_us - current_timestamp_us;
            if (interval_us > predictor->config.max_gyro_gap_us[
                                  predictor->anchor.selected_source]) {
                output->integrity_flags |= FAST_ATTITUDE_FLAG_GYRO_GAP;
                break;
            }

            float right_rate[3];
            for (size_t axis = 0U; axis < 3U; ++axis) {
                right_rate[axis] = node->sample.gyro_rad_s[axis] -
                                   predictor->anchor.gyro_bias_rad_s[axis];
            }
            if (!compose_linear_rate_interval(output->quaternion,
                                              current_rate,
                                              right_rate,
                                              interval_us)) {
                output->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
                break;
            }
            memcpy(current_rate, right_rate, sizeof(current_rate));
            current_timestamp_us = node->sample.timestamp_us;
            output->latest_gyro_timestamp_us = current_timestamp_us;
        }
    }

    if (output->integrity_flags != FAST_ATTITUDE_FLAG_NONE)
        return false;

    memcpy(output->gyro_rate_rad_s, current_rate,
           sizeof(output->gyro_rate_rad_s));

    const uint64_t horizon_us = output_timestamp_us - current_timestamp_us;
    output->prediction_horizon_us =
        (horizon_us > UINT32_MAX) ? UINT32_MAX : (uint32_t)horizon_us;
    if (horizon_us > predictor->config.max_prediction_horizon_us) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_HORIZON_EXCEEDED;
        return false;
    }
    if (!compose_linear_rate_interval(output->quaternion,
                                      current_rate,
                                      current_rate,
                                      horizon_us)) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
        return false;
    }

    quaternion_to_euler(output->quaternion, output->euler_rad);
    if (!vector_is_finite(output->euler_rad)) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
        return false;
    }
    output->predicted = output_timestamp_us > predictor->anchor.timestamp_us;
    output->euler_singular =
        fabsf(output->euler_rad[1]) >=
        predictor->config.euler_singularity_threshold_rad;
    output->valid = true;
    return true;
}

static void initialize_snapshot(fast_attitude_snapshot_t *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->selected_source = IMU_SOURCE_FUSED;
    snapshot->integrity_flags = FAST_ATTITUDE_FLAG_SNAPSHOT_INVALID;
}

static bool replay_cache_is_exportable(
    const fast_attitude_predictor_t *predictor)
{
    const fast_attitude_replay_cache_t *const cache =
        &predictor->replay_cache;
    return cache->valid &&
           (cache->previous_timestamp_us >= predictor->anchor.timestamp_us) &&
           (cache->timestamp_us >= cache->previous_timestamp_us) &&
           (cache->coverage_timestamp_us >= cache->timestamp_us) &&
           quaternion_is_finite(cache->previous_quaternion) &&
           quaternion_is_finite(cache->quaternion) &&
           vector_is_finite(cache->previous_gyro_rate_rad_s) &&
           vector_is_finite(cache->gyro_rate_rad_s);
}

static void export_snapshot_endpoint(
    fast_attitude_snapshot_endpoint_t *endpoint,
    uint64_t timestamp_us,
    const float quaternion[4],
    const float gyro_rate_rad_s[3],
    uint32_t integrity_flags)
{
    endpoint->timestamp_us = timestamp_us;
    memcpy(endpoint->quaternion, quaternion, sizeof(endpoint->quaternion));
    memcpy(endpoint->gyro_rate_rad_s, gyro_rate_rad_s,
           sizeof(endpoint->gyro_rate_rad_s));
    endpoint->integrity_flags = integrity_flags;
}

bool fast_attitude_predictor_export_snapshot(
    const fast_attitude_predictor_t *predictor,
    fast_attitude_snapshot_t *snapshot)
{
    if (snapshot == NULL)
        return false;

    initialize_snapshot(snapshot);
    if ((predictor == NULL) || !predictor->initialized) {
        snapshot->integrity_flags |= FAST_ATTITUDE_FLAG_NOT_INITIALIZED;
        return false;
    }
    if (!predictor->anchor_valid) {
        snapshot->integrity_flags |= FAST_ATTITUDE_FLAG_NO_ANCHOR;
        return false;
    }
    if (!replay_cache_is_exportable(predictor))
        return false;

    const fast_attitude_replay_cache_t *const cache =
        &predictor->replay_cache;
    export_snapshot_endpoint(&snapshot->previous_endpoint,
                             cache->previous_timestamp_us,
                             cache->previous_quaternion,
                             cache->previous_gyro_rate_rad_s,
                             cache->previous_integrity_flags);
    export_snapshot_endpoint(&snapshot->current_endpoint,
                             cache->timestamp_us,
                             cache->quaternion,
                             cache->gyro_rate_rad_s,
                             cache->endpoint_integrity_flags);
    snapshot->anchor_timestamp_us = predictor->anchor.timestamp_us;
    snapshot->coverage_timestamp_us = cache->coverage_timestamp_us;
    memcpy(snapshot->anchor_quaternion, predictor->anchor.quaternion,
           sizeof(snapshot->anchor_quaternion));
    memcpy(snapshot->anchor_gyro_rate_rad_s,
           predictor->anchor.gyro_rate_rad_s,
           sizeof(snapshot->anchor_gyro_rate_rad_s));
    memcpy(snapshot->gyro_bias_rad_s, predictor->anchor.gyro_bias_rad_s,
           sizeof(snapshot->gyro_bias_rad_s));
    snapshot->integrity_flags = cache->integrity_flags;
    snapshot->max_prediction_horizon_us =
        predictor->config.max_prediction_horizon_us;
    snapshot->max_gyro_gap_us = predictor->config.max_gyro_gap_us[
        predictor->anchor.selected_source];
    snapshot->euler_singularity_threshold_rad =
        predictor->config.euler_singularity_threshold_rad;
    snapshot->selected_source = predictor->anchor.selected_source;
    snapshot->quality_flags = predictor->anchor.quality_flags;
    snapshot->degraded = predictor->anchor.degraded;
    snapshot->blocked = cache->blocked;
    snapshot->valid = true;
    return true;
}

static bool snapshot_is_usable(const fast_attitude_snapshot_t *snapshot)
{
    return (snapshot != NULL) && snapshot->valid &&
           (snapshot->selected_source < IMU_SOURCE_COUNT) &&
           (snapshot->max_prediction_horizon_us != 0U) &&
           (snapshot->max_gyro_gap_us != 0U) &&
           isfinite(snapshot->euler_singularity_threshold_rad) &&
           (snapshot->euler_singularity_threshold_rad > 0.0f) &&
           (snapshot->euler_singularity_threshold_rad <
            FAST_ATTITUDE_HALF_PI) &&
           (snapshot->previous_endpoint.timestamp_us >=
            snapshot->anchor_timestamp_us) &&
           (snapshot->current_endpoint.timestamp_us >=
            snapshot->previous_endpoint.timestamp_us) &&
           (snapshot->coverage_timestamp_us >=
            snapshot->current_endpoint.timestamp_us) &&
           quaternion_is_finite(snapshot->anchor_quaternion) &&
           quaternion_is_finite(snapshot->previous_endpoint.quaternion) &&
           quaternion_is_finite(snapshot->current_endpoint.quaternion) &&
           vector_is_finite(snapshot->anchor_gyro_rate_rad_s) &&
           vector_is_finite(snapshot->gyro_bias_rad_s) &&
           vector_is_finite(snapshot->previous_endpoint.gyro_rate_rad_s) &&
           vector_is_finite(snapshot->current_endpoint.gyro_rate_rad_s);
}

static bool initialize_snapshot_output(
    const fast_attitude_snapshot_t *snapshot,
    uint64_t output_timestamp_us,
    fast_attitude_output_t *output)
{
    memset(output, 0, sizeof(*output));
    output->output_timestamp_us = output_timestamp_us;
    output->selected_source = IMU_SOURCE_FUSED;
    if (!snapshot_is_usable(snapshot)) {
        output->integrity_flags = FAST_ATTITUDE_FLAG_SNAPSHOT_INVALID;
        if (snapshot != NULL)
            output->integrity_flags |= snapshot->integrity_flags;
        return false;
    }

    output->anchor_timestamp_us = snapshot->anchor_timestamp_us;
    output->latest_gyro_timestamp_us = snapshot->anchor_timestamp_us;
    output->selected_source = snapshot->selected_source;
    output->degraded = snapshot->degraded;
    output->attitude_converged =
        (snapshot->quality_flags &
         FAST_ATTITUDE_QUALITY_ATTITUDE_CONVERGED) != 0U;
    output->post_impact_reacquire_active =
        (snapshot->quality_flags &
         FAST_ATTITUDE_QUALITY_POST_IMPACT_REACQUIRE_ACTIVE) != 0U;
    output->attitude_aiding_stale =
        (snapshot->quality_flags &
         FAST_ATTITUDE_QUALITY_ATTITUDE_AIDING_STALE) != 0U;
    output->rotation_unobserved =
        (snapshot->quality_flags &
         FAST_ATTITUDE_QUALITY_ROTATION_UNOBSERVED) != 0U;
    memcpy(output->quaternion, snapshot->anchor_quaternion,
           sizeof(output->quaternion));
    return true;
}

static uint32_t saturate_u64_to_u32(uint64_t value)
{
    return (value > UINT32_MAX) ? UINT32_MAX : (uint32_t)value;
}

static bool snapshot_finalize_output(
    const fast_attitude_snapshot_t *snapshot,
    fast_attitude_output_t *output)
{
    if (output->integrity_flags != FAST_ATTITUDE_FLAG_NONE)
        return false;

    quaternion_to_euler(output->quaternion, output->euler_rad);
    if (!vector_is_finite(output->euler_rad)) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
        return false;
    }
    output->predicted =
        output->output_timestamp_us > snapshot->anchor_timestamp_us;
    output->euler_singular =
        fabsf(output->euler_rad[1]) >=
        snapshot->euler_singularity_threshold_rad;
    output->valid = true;
    return true;
}

static bool snapshot_check_horizon(
    const fast_attitude_snapshot_t *snapshot,
    uint64_t horizon_us,
    fast_attitude_output_t *output)
{
    output->prediction_horizon_us = saturate_u64_to_u32(horizon_us);
    if (horizon_us <= snapshot->max_prediction_horizon_us)
        return true;
    output->integrity_flags |= FAST_ATTITUDE_FLAG_HORIZON_EXCEEDED;
    return false;
}

bool fast_attitude_snapshot_predict(
    const fast_attitude_snapshot_t *snapshot,
    uint64_t output_timestamp_us,
    fast_attitude_output_t *output)
{
    if (output == NULL)
        return false;
    if (!initialize_snapshot_output(snapshot, output_timestamp_us, output))
        return false;
    if (output_timestamp_us < snapshot->anchor_timestamp_us) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_TARGET_BEFORE_ANCHOR;
        return false;
    }

    if (output_timestamp_us == snapshot->anchor_timestamp_us) {
        for (size_t axis = 0U; axis < 3U; ++axis) {
            output->gyro_rate_rad_s[axis] =
                snapshot->anchor_gyro_rate_rad_s[axis] -
                snapshot->gyro_bias_rad_s[axis];
        }
        if (!vector_is_finite(output->gyro_rate_rad_s)) {
            output->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
            return false;
        }
        return snapshot_finalize_output(snapshot, output);
    }

    const fast_attitude_snapshot_endpoint_t *const previous =
        &snapshot->previous_endpoint;
    const fast_attitude_snapshot_endpoint_t *const current =
        &snapshot->current_endpoint;
    if (output_timestamp_us < previous->timestamp_us) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_OUTPUT_STALE;
        return false;
    }

    if (output_timestamp_us == previous->timestamp_us) {
        memcpy(output->quaternion, previous->quaternion,
               sizeof(output->quaternion));
        memcpy(output->gyro_rate_rad_s, previous->gyro_rate_rad_s,
               sizeof(output->gyro_rate_rad_s));
        output->latest_gyro_timestamp_us = previous->timestamp_us;
        output->integrity_flags |= previous->integrity_flags;
        return snapshot_finalize_output(snapshot, output);
    }

    if (output_timestamp_us < current->timestamp_us) {
        /* The current endpoint is in the future for this output epoch. Keep
         * the public predictor's causal ZOH semantics. */
        const uint64_t partial_us = output_timestamp_us -
                                    previous->timestamp_us;
        output->latest_gyro_timestamp_us = previous->timestamp_us;
        output->integrity_flags |= previous->integrity_flags;
        memcpy(output->quaternion, previous->quaternion,
               sizeof(output->quaternion));
        memcpy(output->gyro_rate_rad_s, previous->gyro_rate_rad_s,
               sizeof(output->gyro_rate_rad_s));
        if (output->integrity_flags != FAST_ATTITUDE_FLAG_NONE)
            return false;
        if (!snapshot_check_horizon(snapshot, partial_us, output))
            return false;
        if (!compose_linear_rate_interval(output->quaternion,
                                          previous->gyro_rate_rad_s,
                                          previous->gyro_rate_rad_s,
                                          partial_us)) {
            output->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
            return false;
        }
        return snapshot_finalize_output(snapshot, output);
    }

    memcpy(output->quaternion, current->quaternion,
           sizeof(output->quaternion));
    memcpy(output->gyro_rate_rad_s, current->gyro_rate_rad_s,
           sizeof(output->gyro_rate_rad_s));
    output->latest_gyro_timestamp_us = current->timestamp_us;
    output->integrity_flags |= current->integrity_flags;
    if (snapshot->blocked &&
        (output_timestamp_us >= snapshot->coverage_timestamp_us)) {
        output->integrity_flags |= snapshot->integrity_flags;
    }
    if (output->integrity_flags != FAST_ATTITUDE_FLAG_NONE)
        return false;

    const uint64_t horizon_us = output_timestamp_us - current->timestamp_us;
    if (!snapshot_check_horizon(snapshot, horizon_us, output))
        return false;
    if (!compose_linear_rate_interval(output->quaternion,
                                      current->gyro_rate_rad_s,
                                      current->gyro_rate_rad_s,
                                      horizon_us)) {
        output->integrity_flags |= FAST_ATTITUDE_FLAG_NUMERIC;
        return false;
    }
    return snapshot_finalize_output(snapshot, output);
}
