#include "imu_causal_accel_history.h"

#include <limits.h>
#include <math.h>
#include <string.h>

static void increment_saturating(uint32_t *value)
{
    if (*value != UINT32_MAX)
        (*value)++;
}

static bool vector_is_finite(const float vector[3])
{
    return isfinite(vector[0]) && isfinite(vector[1]) &&
           isfinite(vector[2]);
}

static void clear_samples(imu_causal_accel_history_t *history,
                          imu_causal_accel_reset_reason_t reason,
                          uint64_t invalidation_timestamp_us,
                          bool invalidation_timestamp_valid)
{
    history->count = 0U;
    history->newest_timestamp_us = 0U;
    history->newest_timestamp_valid = false;
    history->last_reset_reason = reason;
    history->invalidation_timestamp_us = invalidation_timestamp_us;
    history->invalidation_timestamp_valid =
        invalidation_timestamp_valid && (invalidation_timestamp_us != 0U);
}

bool imu_causal_accel_history_init(imu_causal_accel_history_t *history,
                                   imu_source_t source,
                                   uint32_t maximum_sample_gap_us,
                                   uint32_t maximum_sample_age_us)
{
    if ((history == NULL) || (source >= IMU_SOURCE_COUNT) ||
        (maximum_sample_gap_us == 0U) || (maximum_sample_age_us == 0U)) {
        return false;
    }
    memset(history, 0, sizeof(*history));
    history->source = source;
    history->maximum_sample_gap_us = maximum_sample_gap_us;
    history->maximum_sample_age_us = maximum_sample_age_us;
    history->initialized = true;
    return true;
}

void imu_causal_accel_history_reset(imu_causal_accel_history_t *history)
{
    if ((history == NULL) || !history->initialized)
        return;
    clear_samples(history, IMU_CAUSAL_ACCEL_RESET_EXPLICIT, 0U, false);
    increment_saturating(&history->diagnostics.explicit_reset_count);
}

static void invalidate_sample(imu_causal_accel_history_t *history,
                              const imu_accel_sample_t *sample,
                              imu_causal_accel_reset_reason_t reason)
{
    const bool timestamp_valid = (sample != NULL) &&
                                 (sample->timestamp_us != 0U);
    const uint64_t timestamp_us = timestamp_valid ? sample->timestamp_us : 0U;
    clear_samples(history, reason, timestamp_us, timestamp_valid);
    if (reason == IMU_CAUSAL_ACCEL_RESET_SATURATED) {
        increment_saturating(&history->diagnostics.saturated_reset_count);
    } else {
        increment_saturating(&history->diagnostics.invalid_reset_count);
    }
}

static size_t oldest_sample_index(const imu_causal_accel_history_t *history)
{
    size_t oldest = 0U;
    for (size_t index = 1U; index < history->count; ++index) {
        if (history->samples[index].timestamp_us <
            history->samples[oldest].timestamp_us) {
            oldest = index;
        }
    }
    return oldest;
}

bool imu_causal_accel_history_push(imu_causal_accel_history_t *history,
                                   const imu_accel_sample_t *sample,
                                   bool timestamp_trusted,
                                   bool saturated)
{
    if ((history == NULL) || !history->initialized)
        return false;
    if ((sample != NULL) && (sample->source != history->source))
        return false;
    if (saturated) {
        invalidate_sample(history, sample,
                          IMU_CAUSAL_ACCEL_RESET_SATURATED);
        return false;
    }
    if ((sample == NULL) || !timestamp_trusted || !sample->valid ||
        (sample->timestamp_us == 0U) ||
        !vector_is_finite(sample->accel_mps2)) {
        invalidate_sample(history, sample, IMU_CAUSAL_ACCEL_RESET_INVALID);
        return false;
    }
    if (history->invalidation_timestamp_valid &&
        (sample->timestamp_us <= history->invalidation_timestamp_us)) {
        increment_saturating(
            &history->diagnostics.invalidation_barrier_reject_count);
        return false;
    }

    if (history->newest_timestamp_valid &&
        (sample->timestamp_us > history->newest_timestamp_us) &&
        ((sample->timestamp_us - history->newest_timestamp_us) >
         history->maximum_sample_gap_us)) {
        clear_samples(history, IMU_CAUSAL_ACCEL_RESET_GAP,
                      history->newest_timestamp_us, true);
        increment_saturating(&history->diagnostics.gap_reset_count);
    }

    for (size_t index = 0U; index < history->count; ++index) {
        if (history->samples[index].timestamp_us == sample->timestamp_us) {
            history->samples[index] = *sample;
            increment_saturating(
                &history->diagnostics.duplicate_replacement_count);
            history->last_reset_reason = IMU_CAUSAL_ACCEL_RESET_NONE;
            history->invalidation_timestamp_valid = false;
            return true;
        }
    }

    if (history->newest_timestamp_valid &&
        (sample->timestamp_us < history->newest_timestamp_us)) {
        increment_saturating(&history->diagnostics.out_of_order_count);
    }

    if (history->count < IMU_CAUSAL_ACCEL_HISTORY_CAPACITY) {
        history->samples[history->count++] = *sample;
    } else {
        const size_t oldest = oldest_sample_index(history);
        if (sample->timestamp_us < history->samples[oldest].timestamp_us) {
            increment_saturating(&history->diagnostics.too_old_drop_count);
            return false;
        }
        history->samples[oldest] = *sample;
    }

    if (!history->newest_timestamp_valid ||
        (sample->timestamp_us > history->newest_timestamp_us)) {
        history->newest_timestamp_us = sample->timestamp_us;
        history->newest_timestamp_valid = true;
    }
    history->last_reset_reason = IMU_CAUSAL_ACCEL_RESET_NONE;
    history->invalidation_timestamp_valid = false;
    increment_saturating(&history->diagnostics.accepted_count);
    return true;
}

static imu_causal_accel_query_status_t empty_query_status(
    const imu_causal_accel_history_t *history)
{
    if (history->last_reset_reason == IMU_CAUSAL_ACCEL_RESET_SATURATED)
        return IMU_CAUSAL_ACCEL_QUERY_SATURATED;
    if ((history->last_reset_reason == IMU_CAUSAL_ACCEL_RESET_INVALID) ||
        (history->last_reset_reason == IMU_CAUSAL_ACCEL_RESET_EXPLICIT) ||
        (history->last_reset_reason == IMU_CAUSAL_ACCEL_RESET_GAP)) {
        return IMU_CAUSAL_ACCEL_QUERY_INVALIDATED;
    }
    return IMU_CAUSAL_ACCEL_QUERY_EMPTY;
}

bool imu_causal_accel_history_query(
    imu_causal_accel_history_t *history,
    imu_source_t source,
    uint64_t gyro_timestamp_us,
    imu_causal_accel_query_result_t *result)
{
    if (result == NULL)
        return false;
    memset(result, 0, sizeof(*result));
    result->status = IMU_CAUSAL_ACCEL_QUERY_INVALID_ARGUMENT;
    if ((history == NULL) || !history->initialized ||
        (gyro_timestamp_us == 0U)) {
        return false;
    }
    if (source != history->source) {
        result->status = IMU_CAUSAL_ACCEL_QUERY_SOURCE_MISMATCH;
        increment_saturating(
            &history->diagnostics.query_source_mismatch_count);
        return true;
    }
    if (history->count == 0U) {
        result->status = empty_query_status(history);
        increment_saturating(&history->diagnostics.query_empty_count);
        return true;
    }

    size_t causal_index = 0U;
    size_t future_index = 0U;
    bool causal_found = false;
    bool future_found = false;
    for (size_t index = 0U; index < history->count; ++index) {
        const uint64_t timestamp_us = history->samples[index].timestamp_us;
        if (timestamp_us <= gyro_timestamp_us) {
            if (!causal_found ||
                (timestamp_us > history->samples[causal_index].timestamp_us)) {
                causal_index = index;
                causal_found = true;
            }
        } else if (!future_found ||
                   (timestamp_us < history->samples[future_index].timestamp_us)) {
            future_index = index;
            future_found = true;
        }
    }

    if (!causal_found) {
        result->status = IMU_CAUSAL_ACCEL_QUERY_FUTURE_ONLY;
        if (future_found)
            result->sample = history->samples[future_index];
        increment_saturating(&history->diagnostics.query_future_count);
        return true;
    }

    result->sample = history->samples[causal_index];
    result->age_us = gyro_timestamp_us - result->sample.timestamp_us;
    if (result->age_us > history->maximum_sample_age_us) {
        result->status = IMU_CAUSAL_ACCEL_QUERY_STALE;
        increment_saturating(&history->diagnostics.query_stale_count);
    } else {
        result->status = IMU_CAUSAL_ACCEL_QUERY_OK;
        increment_saturating(&history->diagnostics.query_ok_count);
    }
    return true;
}
