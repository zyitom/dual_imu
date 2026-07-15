#include "imu_motion_guard.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

typedef enum
{
    IMU_MOTION_GUARD_TIMESTAMP_FIRST,
    IMU_MOTION_GUARD_TIMESTAMP_CONTIGUOUS,
    IMU_MOTION_GUARD_TIMESTAMP_FORWARD_GAP,
    IMU_MOTION_GUARD_TIMESTAMP_NONMONOTONIC
} imu_motion_guard_timestamp_result_t;

static void increment_saturated(uint32_t *counter)
{
    if (*counter < UINT32_MAX)
        (*counter)++;
}

static bool finite_positive(float value)
{
    return isfinite(value) && (value > 0.0f);
}

static bool config_is_valid(const imu_motion_guard_config_t *config)
{
    return (config != NULL) && (config->gyro_debounce_us != 0U) &&
           (config->common_impact_window_us != 0U) &&
           (config->gyro_hard_latch_suppression_us != 0U) &&
           (config->accel_inhibit_us != 0U) &&
           (config->max_sample_gap_us != 0U) &&
           (config->accel_disturbance_coincidence_us != 0U) &&
           (config->accel_disturbance_quiet_us != 0U) &&
           (config->accel_disturbance_max_sample_gap_us != 0U) &&
           finite_positive(
               config->accel_disturbance_norm_excursion_mps2) &&
           finite_positive(config->accel_disturbance_delta_mps2) &&
           finite_positive(config->accel_disturbance_jerk_mps3) &&
           finite_positive(config->accel_disturbance_severe_delta_mps2) &&
           finite_positive(config->accel_disturbance_severe_norm_mps2) &&
           finite_positive(
               config->accel_disturbance_rearm_norm_excursion_mps2) &&
           finite_positive(config->accel_disturbance_rearm_delta_mps2) &&
           (config->accel_disturbance_rearm_norm_excursion_mps2 <
            config->accel_disturbance_norm_excursion_mps2) &&
           (config->accel_disturbance_rearm_delta_mps2 <
            config->accel_disturbance_delta_mps2) &&
           (config->accel_disturbance_severe_delta_mps2 >
            config->accel_disturbance_delta_mps2) &&
           (config->accel_disturbance_severe_norm_mps2 >
            config->accel_disturbance_norm_excursion_mps2);
}

static bool window_is_active(bool valid,
                             uint64_t event_timestamp_us,
                             uint64_t duration_us,
                             uint64_t now_us)
{
    if (!valid || (now_us < event_timestamp_us))
        return false;

    return (now_us - event_timestamp_us) < duration_us;
}

static void record_latest_event(uint64_t timestamp_us,
                                uint64_t *latest_timestamp_us,
                                bool *seen)
{
    if (!*seen || (timestamp_us > *latest_timestamp_us))
        *latest_timestamp_us = timestamp_us;
    *seen = true;
}

static imu_motion_guard_timestamp_result_t observe_timestamp(
    imu_motion_guard_timestamp_state_t *state,
    uint64_t timestamp_us,
    uint64_t max_gap_us,
    uint32_t *nonmonotonic_count,
    uint32_t *gap_count)
{
    if (!state->initialized)
    {
        state->last_timestamp_us = timestamp_us;
        state->initialized = true;
        return IMU_MOTION_GUARD_TIMESTAMP_FIRST;
    }

    const uint64_t previous_timestamp_us = state->last_timestamp_us;
    if (timestamp_us <= previous_timestamp_us)
    {
        increment_saturated(nonmonotonic_count);
        return IMU_MOTION_GUARD_TIMESTAMP_NONMONOTONIC;
    }

    /* A forward discontinuity becomes the new trusted anchor, but the sample
     * at the discontinuity remains isolated from both neighboring runs. */
    state->last_timestamp_us = timestamp_us;
    if ((timestamp_us - previous_timestamp_us) > max_gap_us)
    {
        increment_saturated(gap_count);
        return IMU_MOTION_GUARD_TIMESTAMP_FORWARD_GAP;
    }

    return IMU_MOTION_GUARD_TIMESTAMP_CONTIGUOUS;
}

static uint64_t absolute_timestamp_delta(uint64_t first_us,
                                         uint64_t second_us)
{
    return (first_us >= second_us) ? (first_us - second_us)
                                   : (second_us - first_us);
}

static uint64_t saturating_deadline(uint64_t timestamp_us,
                                    uint64_t duration_us)
{
    return (duration_us > (UINT64_MAX - timestamp_us))
               ? UINT64_MAX
               : timestamp_us + duration_us;
}

static bool saturation_window_contains(
    const imu_motion_guard_saturation_window_t *window,
    uint64_t timestamp_us)
{
    return window->valid && (timestamp_us >= window->start_us) &&
           (timestamp_us <= window->end_us);
}

static bool saturation_windows_touch_or_overlap(
    const imu_motion_guard_saturation_window_t *earlier,
    const imu_motion_guard_saturation_window_t *later)
{
    if (later->start_us <= earlier->end_us)
        return true;
    return (earlier->end_us != UINT64_MAX) &&
           (later->start_us == (earlier->end_us + 1U));
}

static bool saturation_window_precedes(
    const imu_motion_guard_saturation_window_t *first,
    const imu_motion_guard_saturation_window_t *second)
{
    return (first->start_us < second->start_us) ||
           ((first->start_us == second->start_us) &&
            (first->end_us < second->end_us));
}

static void record_saturation_window(
    imu_motion_guard_saturation_window_t
        history[IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT],
    uint64_t timestamp_us)
{
    const imu_motion_guard_saturation_window_t event = {
        .start_us = timestamp_us,
        .end_us = saturating_deadline(
            timestamp_us,
            IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US - 1U),
        .valid = true,
    };

    imu_motion_guard_saturation_window_t candidates
        [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT + 1U];
    size_t candidate_count = 0U;
    for (size_t index = 0U;
         index < IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT; ++index) {
        if (history[index].valid)
            candidates[candidate_count++] = history[index];
    }
    candidates[candidate_count++] = event;

    for (size_t index = 1U; index < candidate_count; ++index) {
        const imu_motion_guard_saturation_window_t key = candidates[index];
        size_t position = index;
        while ((position > 0U) &&
               saturation_window_precedes(&key,
                                          &candidates[position - 1U])) {
            candidates[position] = candidates[position - 1U];
            position--;
        }
        candidates[position] = key;
    }

    imu_motion_guard_saturation_window_t merged
        [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT + 1U];
    size_t merged_count = 0U;
    for (size_t index = 0U; index < candidate_count; ++index) {
        if ((merged_count == 0U) ||
            !saturation_windows_touch_or_overlap(
                &merged[merged_count - 1U], &candidates[index])) {
            merged[merged_count++] = candidates[index];
        } else if (candidates[index].end_us > merged[merged_count - 1U].end_us) {
            merged[merged_count - 1U].end_us = candidates[index].end_us;
        }
    }

    memset(history, 0,
           sizeof(*history) * IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT);
    const size_t retained_count =
        (merged_count < IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT)
            ? merged_count
            : IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT;
    for (size_t index = 0U; index < retained_count; ++index) {
        history[index] = merged[merged_count - 1U - index];
    }
}

static bool saturation_history_contains(
    const imu_motion_guard_saturation_window_t
        history[IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT],
    uint64_t timestamp_us)
{
    for (size_t index = 0U;
         index < IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT; ++index) {
        if (saturation_window_contains(&history[index], timestamp_us))
            return true;
    }
    return false;
}

static void record_common_impact(imu_motion_guard_t *guard,
                                 uint64_t timestamp_us)
{
    if (!guard->common_impact_valid) {
        guard->common_impact_start_us = timestamp_us;
        guard->last_common_impact_us = timestamp_us;
        guard->common_impact_valid = true;
        increment_saturated(&guard->counters.common_impact_episode_count);
        return;
    }

    const uint64_t current_end_us = saturating_deadline(
        guard->last_common_impact_us, guard->config.accel_inhibit_us);
    bool same_episode;
    if (timestamp_us >= guard->common_impact_start_us) {
        same_episode = (current_end_us == UINT64_MAX) ||
                       (timestamp_us < current_end_us);
    } else {
        const uint64_t candidate_end_us = saturating_deadline(
            timestamp_us, guard->config.accel_inhibit_us);
        same_episode = candidate_end_us > guard->common_impact_start_us;
    }

    if (same_episode) {
        if (timestamp_us > guard->last_common_impact_us)
            guard->last_common_impact_us = timestamp_us;
        return;
    }

    /* A disjoint observation older than the active episode cannot replace the
     * latest causal interval. It is retained only by saturation history. */
    if (timestamp_us <= guard->last_common_impact_us)
        return;

    guard->common_impact_start_us = timestamp_us;
    guard->last_common_impact_us = timestamp_us;
    increment_saturated(&guard->counters.common_impact_episode_count);
}

static void record_accel_disturbance(imu_motion_guard_t *guard,
                                     uint64_t start_us,
                                     uint64_t evidence_us)
{
    if (start_us > evidence_us) {
        const uint64_t swap = start_us;
        start_us = evidence_us;
        evidence_us = swap;
    }

    const uint64_t previous_end_us = saturating_deadline(
        guard->last_accel_disturbance_us, guard->config.accel_inhibit_us);
    const bool same_episode = guard->accel_disturbance_valid &&
                              (start_us < previous_end_us);
    if (!same_episode) {
        increment_saturated(
            &guard->counters.accel_disturbance_episode_count);
        guard->accel_disturbance_start_us = start_us;
        guard->last_accel_disturbance_us = evidence_us;
    } else {
        bool extended = false;
        if (start_us < guard->accel_disturbance_start_us) {
            guard->accel_disturbance_start_us = start_us;
            extended = true;
        }
        if (evidence_us > guard->last_accel_disturbance_us) {
            guard->last_accel_disturbance_us = evidence_us;
            extended = true;
        }
        if (extended) {
            increment_saturated(
                &guard->counters.accel_disturbance_extension_count);
        }
    }
    guard->accel_disturbance_valid = true;
}

static bool vector_is_finite(const float vector[3])
{
    return (vector != NULL) && isfinite(vector[0]) && isfinite(vector[1]) &&
           isfinite(vector[2]);
}

static float vector_norm(const float vector[3])
{
    return sqrtf((vector[0] * vector[0]) +
                 (vector[1] * vector[1]) +
                 (vector[2] * vector[2]));
}

static float vector_distance(const float first[3], const float second[3])
{
    const float delta[3] = {
        first[0] - second[0],
        first[1] - second[1],
        first[2] - second[2],
    };
    return vector_norm(delta);
}

static void reset_accel_subrange_lane(
    imu_motion_guard_lane_state_t *lane_state)
{
    memset(lane_state->previous_accel_mps2, 0,
           sizeof(lane_state->previous_accel_mps2));
    lane_state->previous_accel_timestamp_us = 0U;
    lane_state->accel_disturbance_candidate_us = 0U;
    lane_state->accel_disturbance_candidate_latest_us = 0U;
    lane_state->accel_disturbance_quiet_start_us = 0U;
    lane_state->previous_accel_valid = false;
    lane_state->accel_disturbance_candidate_valid = false;
    lane_state->accel_disturbance_quiet_active = false;
}

static void reset_accel_subrange_evidence(imu_motion_guard_t *guard,
                                          uint8_t lane)
{
    reset_accel_subrange_lane(&guard->lane[lane]);
}

static void store_accel_subrange_baseline(
    imu_motion_guard_lane_state_t *lane_state,
    uint64_t timestamp_us,
    const float accel_mps2[3])
{
    memcpy(lane_state->previous_accel_mps2, accel_mps2,
           sizeof(lane_state->previous_accel_mps2));
    lane_state->previous_accel_timestamp_us = timestamp_us;
    lane_state->previous_accel_valid = true;
}

static void disarm_accel_disturbance_pair(imu_motion_guard_t *guard)
{
    for (uint8_t lane = 0U; lane < IMU_MOTION_GUARD_LANE_COUNT; ++lane) {
        imu_motion_guard_lane_state_t *const lane_state = &guard->lane[lane];
        lane_state->accel_disturbance_candidate_valid = false;
        lane_state->accel_disturbance_candidate_armed = false;
        lane_state->accel_disturbance_rearm_required = true;
        lane_state->accel_disturbance_quiet_active = false;
        lane_state->accel_disturbance_quiet_start_us = 0U;
    }
}

static void try_rearm_accel_disturbance_pair(imu_motion_guard_t *guard,
                                             uint64_t timestamp_us)
{
    for (uint8_t lane = 0U; lane < IMU_MOTION_GUARD_LANE_COUNT; ++lane) {
        const imu_motion_guard_lane_state_t *const lane_state =
            &guard->lane[lane];
        if (!lane_state->accel_disturbance_rearm_required ||
            !lane_state->accel_disturbance_quiet_active ||
            !lane_state->previous_accel_valid ||
            (absolute_timestamp_delta(
                 timestamp_us, lane_state->previous_accel_timestamp_us) >
             guard->config.accel_disturbance_max_sample_gap_us) ||
            (timestamp_us < lane_state->accel_disturbance_quiet_start_us) ||
            ((timestamp_us - lane_state->accel_disturbance_quiet_start_us) <
             guard->config.accel_disturbance_quiet_us)) {
            return;
        }
    }

    for (uint8_t lane = 0U; lane < IMU_MOTION_GUARD_LANE_COUNT; ++lane) {
        imu_motion_guard_lane_state_t *const lane_state = &guard->lane[lane];
        lane_state->accel_disturbance_candidate_valid = false;
        lane_state->accel_disturbance_candidate_armed = true;
        lane_state->accel_disturbance_rearm_required = false;
        lane_state->accel_disturbance_quiet_active = false;
        lane_state->accel_disturbance_quiet_start_us = 0U;
    }
}

static bool record_accel_subrange_candidate(imu_motion_guard_t *guard,
                                            uint8_t lane,
                                            uint64_t timestamp_us,
                                            bool severe)
{
    imu_motion_guard_lane_state_t *const lane_state = &guard->lane[lane];
    increment_saturated(
        &guard->counters.accel_subrange_candidate_count[lane]);
    lane_state->accel_disturbance_quiet_active = false;
    lane_state->accel_disturbance_quiet_start_us = 0U;

    if (severe) {
        increment_saturated(
            &guard->counters.accel_subrange_severe_observation_count[lane]);
        disarm_accel_disturbance_pair(guard);
        record_accel_disturbance(guard, timestamp_us, timestamp_us);
        record_common_impact(guard, timestamp_us);
        return true;
    }

    if (!lane_state->accel_disturbance_candidate_valid ||
        (timestamp_us < lane_state->accel_disturbance_candidate_latest_us) ||
        ((timestamp_us - lane_state->accel_disturbance_candidate_latest_us) >
         guard->config.accel_disturbance_coincidence_us)) {
        lane_state->accel_disturbance_candidate_us = timestamp_us;
    }
    lane_state->accel_disturbance_candidate_latest_us = timestamp_us;
    lane_state->accel_disturbance_candidate_valid = true;
    const uint8_t other_lane = (uint8_t)(lane ^ 1U);
    imu_motion_guard_lane_state_t *const other_state =
        &guard->lane[other_lane];
    if (!other_state->accel_disturbance_candidate_valid ||
        (absolute_timestamp_delta(
             lane_state->accel_disturbance_candidate_latest_us,
             other_state->accel_disturbance_candidate_latest_us) >
         guard->config.accel_disturbance_coincidence_us)) {
        return false;
    }

    const uint64_t episode_start_us =
        (lane_state->accel_disturbance_candidate_us <=
         other_state->accel_disturbance_candidate_us)
            ? lane_state->accel_disturbance_candidate_us
            : other_state->accel_disturbance_candidate_us;
    const uint64_t latest_evidence_us =
        (lane_state->accel_disturbance_candidate_latest_us >=
         other_state->accel_disturbance_candidate_latest_us)
            ? lane_state->accel_disturbance_candidate_latest_us
            : other_state->accel_disturbance_candidate_latest_us;
    increment_saturated(
        &guard->counters.accel_subrange_common_observation_count);
    disarm_accel_disturbance_pair(guard);
    record_accel_disturbance(guard, episode_start_us, latest_evidence_us);
    return true;
}

static void observe_accel_subrange(
    imu_motion_guard_t *guard,
    uint8_t lane,
    uint64_t evidence_timestamp_us,
    imu_motion_guard_timestamp_result_t timestamp_result,
    bool saturated,
    bool evidence_trusted,
    const float accel_mps2[3])
{
    imu_motion_guard_lane_state_t *const lane_state = &guard->lane[lane];
    if (!evidence_trusted || saturated || !vector_is_finite(accel_mps2) ||
        (timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_FORWARD_GAP) ||
        (timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_NONMONOTONIC)) {
        reset_accel_subrange_evidence(guard, lane);
        if ((timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_FORWARD_GAP) ||
            (timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_NONMONOTONIC)) {
            guard->lane[lane ^ 1U].accel_disturbance_candidate_valid = false;
        }
        return;
    }

    if ((timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_FIRST) ||
        !lane_state->previous_accel_valid) {
        reset_accel_subrange_evidence(guard, lane);
        store_accel_subrange_baseline(lane_state, evidence_timestamp_us,
                                      accel_mps2);
        if (!lane_state->accel_disturbance_rearm_required)
            lane_state->accel_disturbance_candidate_armed = true;
        return;
    }

    if (evidence_timestamp_us <= lane_state->previous_accel_timestamp_us) {
        reset_accel_subrange_evidence(guard, lane);
        return;
    }
    const uint64_t delta_time_us =
        evidence_timestamp_us - lane_state->previous_accel_timestamp_us;
    if (delta_time_us >
        guard->config.accel_disturbance_max_sample_gap_us) {
        increment_saturated(
            &guard->counters.accel_subrange_gap_count[lane]);
        reset_accel_subrange_evidence(guard, lane);
        return;
    }
    const float delta_mps2 =
        vector_distance(accel_mps2, lane_state->previous_accel_mps2);
    const float previous_norm_mps2 =
        vector_norm(lane_state->previous_accel_mps2);
    const float norm_mps2 = vector_norm(accel_mps2);
    const float norm_excursion_mps2 =
        fabsf(norm_mps2 - IMU_MOTION_GUARD_GRAVITY_MPS2);
    const float previous_norm_excursion_mps2 =
        fabsf(previous_norm_mps2 - IMU_MOTION_GUARD_GRAVITY_MPS2);
    const float edge_norm_excursion_mps2 =
        (norm_excursion_mps2 >= previous_norm_excursion_mps2)
            ? norm_excursion_mps2
            : previous_norm_excursion_mps2;
    const float jerk_mps3 =
        delta_mps2 / ((float)delta_time_us * 1.0e-6f);
    store_accel_subrange_baseline(lane_state, evidence_timestamp_us,
                                  accel_mps2);
    if (!isfinite(delta_mps2) || !isfinite(edge_norm_excursion_mps2) ||
        !isfinite(jerk_mps3)) {
        reset_accel_subrange_evidence(guard, lane);
        return;
    }

    const bool severe_delta =
        delta_mps2 >= guard->config.accel_disturbance_severe_delta_mps2;
    const bool severe =
        severe_delta ||
        (norm_mps2 >= guard->config.accel_disturbance_severe_norm_mps2);
    const bool moderate =
        (edge_norm_excursion_mps2 >=
         guard->config.accel_disturbance_norm_excursion_mps2) &&
        ((delta_mps2 >= guard->config.accel_disturbance_delta_mps2) ||
         (jerk_mps3 >= guard->config.accel_disturbance_jerk_mps3));
    if (!lane_state->accel_disturbance_candidate_armed) {
        /* A new full-scale subrange edge extends protection even before the
         * quiet rearm. Constant high-g has zero delta and cannot refresh it. */
        if (severe_delta) {
            (void)record_accel_subrange_candidate(
                guard, lane, evidence_timestamp_us, true);
            return;
        }
        if (moderate) {
            (void)record_accel_subrange_candidate(
                guard, lane, evidence_timestamp_us, false);
            return;
        }
        const bool quiet =
            (norm_excursion_mps2 <= guard->config
                 .accel_disturbance_rearm_norm_excursion_mps2) &&
            (delta_mps2 <=
             guard->config.accel_disturbance_rearm_delta_mps2);
        if (!quiet) {
            lane_state->accel_disturbance_quiet_active = false;
            lane_state->accel_disturbance_quiet_start_us = 0U;
            return;
        }
        if (!lane_state->accel_disturbance_quiet_active) {
            lane_state->accel_disturbance_quiet_active = true;
            lane_state->accel_disturbance_quiet_start_us =
                evidence_timestamp_us;
            return;
        }
        try_rearm_accel_disturbance_pair(guard, evidence_timestamp_us);
        return;
    }

    if (severe || moderate) {
        (void)record_accel_subrange_candidate(
            guard, lane, evidence_timestamp_us, severe);
    }
}

static void reset_gyro_streak(imu_motion_guard_lane_state_t *lane)
{
    lane->gyro_streak_start_us = 0U;
    lane->gyro_streak_active = false;
    lane->gyro_hard_fault_active = false;
}

void imu_motion_guard_default_config(imu_motion_guard_config_t *config)
{
    if (config == NULL)
        return;

    config->gyro_debounce_us = IMU_MOTION_GUARD_DEFAULT_GYRO_DEBOUNCE_US;
    config->common_impact_window_us =
        IMU_MOTION_GUARD_DEFAULT_COMMON_IMPACT_WINDOW_US;
    config->gyro_hard_latch_suppression_us =
        IMU_MOTION_GUARD_DEFAULT_GYRO_SUPPRESSION_US;
    config->accel_inhibit_us = IMU_MOTION_GUARD_DEFAULT_ACCEL_INHIBIT_US;
    config->max_sample_gap_us =
        IMU_MOTION_GUARD_DEFAULT_MAX_SAMPLE_GAP_US;
    config->accel_disturbance_coincidence_us =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_COINCIDENCE_US;
    config->accel_disturbance_quiet_us =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_QUIET_US;
    config->accel_disturbance_max_sample_gap_us =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_MAX_GAP_US;
    config->accel_disturbance_norm_excursion_mps2 =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_NORM_EXCURSION_MPS2;
    config->accel_disturbance_delta_mps2 =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_DELTA_MPS2;
    config->accel_disturbance_jerk_mps3 =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_JERK_MPS3;
    config->accel_disturbance_severe_delta_mps2 =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_SEVERE_DELTA_MPS2;
    config->accel_disturbance_severe_norm_mps2 =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_SEVERE_NORM_MPS2;
    config->accel_disturbance_rearm_norm_excursion_mps2 =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_REARM_NORM_MPS2;
    config->accel_disturbance_rearm_delta_mps2 =
        IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_REARM_DELTA_MPS2;
}

bool imu_motion_guard_init(imu_motion_guard_t *guard,
                           const imu_motion_guard_config_t *config)
{
    if (guard == NULL)
        return false;

    imu_motion_guard_config_t selected_config;
    if (config == NULL)
        imu_motion_guard_default_config(&selected_config);
    else
        selected_config = *config;

    if (!config_is_valid(&selected_config))
    {
        memset(guard, 0, sizeof(*guard));
        return false;
    }

    memset(guard, 0, sizeof(*guard));
    guard->config = selected_config;
    guard->initialized = true;
    return true;
}

bool imu_motion_guard_observe_accel(imu_motion_guard_t *guard,
                                    uint8_t lane,
                                    uint64_t timestamp_us,
                                    bool saturated)
{
    return imu_motion_guard_observe_accel_sample(
        guard, lane, timestamp_us, saturated, timestamp_us, false, NULL);
}

bool imu_motion_guard_observe_accel_sample(
    imu_motion_guard_t *guard,
    uint8_t lane,
    uint64_t timestamp_us,
    bool saturated,
    uint64_t evidence_timestamp_us,
    bool evidence_trusted,
    const float accel_mps2[3])
{
    if ((guard == NULL) || !guard->initialized ||
        (lane >= IMU_MOTION_GUARD_LANE_COUNT))
    {
        return false;
    }

    increment_saturated(&guard->counters.accel_observation_count[lane]);
    const imu_motion_guard_timestamp_result_t timestamp_result =
        observe_timestamp(
        &guard->lane[lane].accel_timestamp,
        timestamp_us,
        guard->config.max_sample_gap_us,
        &guard->counters.accel_nonmonotonic_count[lane],
        &guard->counters.accel_gap_count[lane]);
    imu_motion_guard_lane_state_t *const lane_state = &guard->lane[lane];
    const bool timestamp_nonmonotonic =
        timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_NONMONOTONIC;
    const bool timestamp_boundary = timestamp_nonmonotonic ||
        (timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_FORWARD_GAP);
    if (timestamp_boundary)
        lane_state->accel_saturation_evidence_active = false;

    const bool saturation_started =
        saturated && !lane_state->accel_saturation_evidence_active &&
        !timestamp_nonmonotonic;
    if (!timestamp_nonmonotonic)
        lane_state->accel_saturated = saturated;
    if (!saturated && !timestamp_nonmonotonic)
        lane_state->accel_saturation_evidence_active = false;

    if (saturated)
    {
        increment_saturated(&guard->counters.accel_saturation_count[lane]);
    }
    if (saturation_started)
    {
        lane_state->accel_saturation_evidence_active = true;
        increment_saturated(&guard->counters.accel_impact_observation_count);
    }
    if (saturated && !timestamp_nonmonotonic) {
        record_saturation_window(guard->accel_saturation_history,
                                 timestamp_us);
        record_latest_event(timestamp_us,
                            &guard->last_accel_saturation_us,
                            &guard->accel_saturation_seen);
        /* Keep the inhibit deadline behind the most recent clipped sample.
         * The episode counter still advances only at saturation_started. */
        record_common_impact(guard, timestamp_us);
    }
    observe_accel_subrange(guard, lane, evidence_timestamp_us, timestamp_result,
                           saturated, evidence_trusted, accel_mps2);
    return true;
}

void imu_motion_guard_reset_accel_subrange_evidence(
    imu_motion_guard_t *guard,
    uint8_t lane)
{
    if ((guard == NULL) || !guard->initialized ||
        (lane >= IMU_MOTION_GUARD_LANE_COUNT)) {
        return;
    }
    reset_accel_subrange_evidence(guard, lane);
}

bool imu_motion_guard_observe_gyro(imu_motion_guard_t *guard,
                                   uint8_t lane,
                                   uint64_t timestamp_us,
                                   bool saturated)
{
    if ((guard == NULL) || !guard->initialized ||
        (lane >= IMU_MOTION_GUARD_LANE_COUNT))
    {
        return false;
    }

    imu_motion_guard_lane_state_t *const lane_state = &guard->lane[lane];
    increment_saturated(&guard->counters.gyro_observation_count[lane]);
    const imu_motion_guard_timestamp_result_t timestamp_result =
        observe_timestamp(
            &lane_state->gyro_timestamp,
            timestamp_us,
            guard->config.max_sample_gap_us,
            &guard->counters.gyro_nonmonotonic_count[lane],
            &guard->counters.gyro_gap_count[lane]);
    const bool timestamp_nonmonotonic =
        timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_NONMONOTONIC;
    const bool timestamp_boundary = timestamp_nonmonotonic ||
        (timestamp_result == IMU_MOTION_GUARD_TIMESTAMP_FORWARD_GAP);
    if (!timestamp_nonmonotonic)
        lane_state->gyro_saturated = saturated;

    if (saturated && !timestamp_nonmonotonic) {
        record_saturation_window(guard->gyro_saturation_history,
                                 timestamp_us);
        record_latest_event(timestamp_us,
                            &guard->last_gyro_saturation_us,
                            &guard->gyro_saturation_seen);
    }

    if (timestamp_boundary)
    {
        reset_gyro_streak(lane_state);
        lane_state->last_gyro_saturation_valid = false;
        if (saturated)
            increment_saturated(&guard->counters.gyro_saturation_count[lane]);
        return true;
    }

    if (!saturated)
    {
        reset_gyro_streak(lane_state);
        return true;
    }

    increment_saturated(&guard->counters.gyro_saturation_count[lane]);
    if (!lane_state->gyro_streak_active)
    {
        lane_state->gyro_streak_start_us = timestamp_us;
        lane_state->gyro_streak_active = true;
    }
    else if (!lane_state->gyro_hard_fault_active &&
             (timestamp_us >= lane_state->gyro_streak_start_us) &&
             ((timestamp_us - lane_state->gyro_streak_start_us) >=
              guard->config.gyro_debounce_us))
    {
        lane_state->gyro_hard_fault_active = true;
        increment_saturated(
            &guard->counters.gyro_hard_fault_assertion_count[lane]);
    }

    const uint8_t other_lane = (uint8_t)(lane ^ 1U);
    const imu_motion_guard_lane_state_t *const other_state =
        &guard->lane[other_lane];
    if (other_state->last_gyro_saturation_valid &&
        (absolute_timestamp_delta(timestamp_us,
                                  other_state->last_gyro_saturation_us) <=
         guard->config.common_impact_window_us))
    {
        const uint64_t common_timestamp_us =
            (timestamp_us >= other_state->last_gyro_saturation_us)
                ? timestamp_us
                : other_state->last_gyro_saturation_us;
        increment_saturated(
            &guard->counters.gyro_common_impact_observation_count);
        record_common_impact(guard, common_timestamp_us);
    }

    lane_state->last_gyro_saturation_us = timestamp_us;
    lane_state->last_gyro_saturation_valid = true;
    return true;
}

bool imu_motion_guard_gyro_hard_fault_active(const imu_motion_guard_t *guard,
                                             uint8_t lane)
{
    return (guard != NULL) && guard->initialized &&
           (lane < IMU_MOTION_GUARD_LANE_COUNT) &&
           guard->lane[lane].gyro_hard_fault_active;
}

bool imu_motion_guard_accel_saturated(const imu_motion_guard_t *guard,
                                      uint8_t lane)
{
    return (guard != NULL) && guard->initialized &&
           (lane < IMU_MOTION_GUARD_LANE_COUNT) &&
           guard->lane[lane].accel_saturated;
}

bool imu_motion_guard_gyro_saturated(const imu_motion_guard_t *guard,
                                     uint8_t lane)
{
    return (guard != NULL) && guard->initialized &&
           (lane < IMU_MOTION_GUARD_LANE_COUNT) &&
           guard->lane[lane].gyro_saturated;
}

bool imu_motion_guard_accel_saturation_recent(
    const imu_motion_guard_t *guard,
    uint64_t now_us,
    uint64_t hold_us)
{
    return (guard != NULL) && guard->initialized &&
           window_is_active(guard->accel_saturation_seen,
                            guard->last_accel_saturation_us,
                            hold_us,
                            now_us);
}

bool imu_motion_guard_gyro_saturation_recent(
    const imu_motion_guard_t *guard,
    uint64_t now_us,
    uint64_t hold_us)
{
    return (guard != NULL) && guard->initialized &&
           window_is_active(guard->gyro_saturation_seen,
                            guard->last_gyro_saturation_us,
                            hold_us,
                            now_us);
}

bool imu_motion_guard_accel_saturation_status_at(
    const imu_motion_guard_t *guard,
    uint64_t timestamp_us)
{
    return (guard != NULL) && guard->initialized &&
           saturation_history_contains(guard->accel_saturation_history,
                                       timestamp_us);
}

bool imu_motion_guard_gyro_saturation_status_at(
    const imu_motion_guard_t *guard,
    uint64_t timestamp_us)
{
    return (guard != NULL) && guard->initialized &&
           saturation_history_contains(guard->gyro_saturation_history,
                                       timestamp_us);
}

bool imu_motion_guard_export_saturation_snapshot(
    const imu_motion_guard_t *guard,
    imu_motion_guard_saturation_snapshot_t *snapshot)
{
    if (snapshot == NULL)
        return false;

    memset(snapshot, 0, sizeof(*snapshot));
    if ((guard == NULL) || !guard->initialized)
        return false;

    memcpy(snapshot->accel_saturation_history,
           guard->accel_saturation_history,
           sizeof(snapshot->accel_saturation_history));
    memcpy(snapshot->gyro_saturation_history,
           guard->gyro_saturation_history,
           sizeof(snapshot->gyro_saturation_history));
    snapshot->valid = true;
    return true;
}

bool imu_motion_guard_snapshot_accel_saturation_status_at(
    const imu_motion_guard_saturation_snapshot_t *snapshot,
    uint64_t timestamp_us)
{
    return (snapshot != NULL) && snapshot->valid &&
           saturation_history_contains(snapshot->accel_saturation_history,
                                       timestamp_us);
}

bool imu_motion_guard_snapshot_gyro_saturation_status_at(
    const imu_motion_guard_saturation_snapshot_t *snapshot,
    uint64_t timestamp_us)
{
    return (snapshot != NULL) && snapshot->valid &&
           saturation_history_contains(snapshot->gyro_saturation_history,
                                       timestamp_us);
}

uint8_t imu_motion_guard_gyro_hard_fault_mask(
    const imu_motion_guard_t *guard)
{
    if ((guard == NULL) || !guard->initialized)
        return 0U;

    uint8_t mask = 0U;
    for (uint8_t lane = 0U; lane < IMU_MOTION_GUARD_LANE_COUNT; ++lane)
    {
        if (guard->lane[lane].gyro_hard_fault_active)
            mask |= (uint8_t)(1U << lane);
    }
    return mask;
}

bool imu_motion_guard_gyro_hard_latch_suppressed(
    const imu_motion_guard_t *guard,
    uint64_t now_us)
{
    return (guard != NULL) && guard->initialized &&
           window_is_active(
               guard->common_impact_valid,
               guard->last_common_impact_us,
               guard->config.gyro_hard_latch_suppression_us,
               now_us);
}

bool imu_motion_guard_accel_inhibited(const imu_motion_guard_t *guard,
                                      uint64_t now_us)
{
    if ((guard == NULL) || !guard->initialized)
        return false;

    return window_is_active(guard->common_impact_valid,
                            guard->last_common_impact_us,
                            guard->config.accel_inhibit_us,
                            now_us) ||
           imu_motion_guard_accel_disturbance_inhibited(guard, now_us);
}

bool imu_motion_guard_accel_disturbance_inhibited(
    const imu_motion_guard_t *guard,
    uint64_t now_us)
{
    return (guard != NULL) && guard->initialized &&
           window_is_active(guard->accel_disturbance_valid,
                            guard->last_accel_disturbance_us,
                            guard->config.accel_inhibit_us,
                            now_us);
}

uint64_t imu_motion_guard_accel_disturbance_until_us(
    const imu_motion_guard_t *guard)
{
    if ((guard == NULL) || !guard->initialized ||
        !guard->accel_disturbance_valid) {
        return 0U;
    }
    return saturating_deadline(guard->last_accel_disturbance_us,
                               guard->config.accel_inhibit_us);
}

bool imu_motion_guard_accel_disturbance_interval(
    const imu_motion_guard_t *guard,
    uint64_t *start_us,
    uint64_t *end_us)
{
    if ((guard == NULL) || !guard->initialized ||
        !guard->accel_disturbance_valid || (start_us == NULL) ||
        (end_us == NULL)) {
        return false;
    }

    *start_us = guard->accel_disturbance_start_us;
    *end_us = saturating_deadline(guard->last_accel_disturbance_us,
                                  guard->config.accel_inhibit_us);
    return *start_us < *end_us;
}

uint64_t imu_motion_guard_accel_inhibit_until_us(
    const imu_motion_guard_t *guard)
{
    if ((guard == NULL) || !guard->initialized ||
        !guard->common_impact_valid) {
        return 0U;
    }
    return saturating_deadline(guard->last_common_impact_us,
                               guard->config.accel_inhibit_us);
}

bool imu_motion_guard_accel_inhibit_interval(
    const imu_motion_guard_t *guard,
    uint64_t *start_us,
    uint64_t *end_us)
{
    if ((guard == NULL) || !guard->initialized ||
        !guard->common_impact_valid || (start_us == NULL) || (end_us == NULL)) {
        return false;
    }

    *start_us = guard->common_impact_start_us;
    *end_us = saturating_deadline(guard->last_common_impact_us,
                                  guard->config.accel_inhibit_us);
    return *start_us < *end_us;
}

void imu_motion_guard_get_diagnostics(
    const imu_motion_guard_t *guard,
    uint64_t now_us,
    imu_motion_guard_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
        return;

    memset(diagnostics, 0, sizeof(*diagnostics));
    if ((guard == NULL) || !guard->initialized)
        return;

    diagnostics->counters = guard->counters;
    memcpy(diagnostics->accel_saturation_history,
           guard->accel_saturation_history,
           sizeof(diagnostics->accel_saturation_history));
    memcpy(diagnostics->gyro_saturation_history,
           guard->gyro_saturation_history,
           sizeof(diagnostics->gyro_saturation_history));
    for (uint8_t lane = 0U; lane < IMU_MOTION_GUARD_LANE_COUNT; ++lane)
    {
        diagnostics->gyro_streak_start_us[lane] =
            guard->lane[lane].gyro_streak_start_us;
        diagnostics->lane_last_gyro_saturation_us[lane] =
            guard->lane[lane].last_gyro_saturation_us;
        diagnostics->gyro_streak_active[lane] =
            guard->lane[lane].gyro_streak_active;
    }
    diagnostics->common_impact_start_us = guard->common_impact_start_us;
    diagnostics->last_common_impact_us = guard->last_common_impact_us;
    diagnostics->accel_disturbance_start_us =
        guard->accel_disturbance_start_us;
    diagnostics->last_accel_disturbance_us =
        guard->last_accel_disturbance_us;
    diagnostics->last_accel_saturation_us =
        guard->last_accel_saturation_us;
    diagnostics->last_gyro_saturation_us =
        guard->last_gyro_saturation_us;
    diagnostics->gyro_hard_fault_mask =
        imu_motion_guard_gyro_hard_fault_mask(guard);
    diagnostics->gyro_hard_latch_suppressed =
        imu_motion_guard_gyro_hard_latch_suppressed(guard, now_us);
    diagnostics->accel_inhibited =
        imu_motion_guard_accel_inhibited(guard, now_us);
    diagnostics->accel_disturbance_inhibited =
        imu_motion_guard_accel_disturbance_inhibited(guard, now_us);
    diagnostics->accel_saturation_seen = guard->accel_saturation_seen;
    diagnostics->gyro_saturation_seen = guard->gyro_saturation_seen;
    diagnostics->common_impact_valid = guard->common_impact_valid;
    diagnostics->accel_disturbance_valid = guard->accel_disturbance_valid;
    diagnostics->initialized = true;
}
