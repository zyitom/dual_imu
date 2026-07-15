#ifndef IMU_MOTION_GUARD_H
#define IMU_MOTION_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_MOTION_GUARD_LANE_COUNT 2U

#define IMU_MOTION_GUARD_DEFAULT_GYRO_DEBOUNCE_US UINT64_C(3000)
#define IMU_MOTION_GUARD_DEFAULT_COMMON_IMPACT_WINDOW_US UINT64_C(3000)
#define IMU_MOTION_GUARD_DEFAULT_GYRO_SUPPRESSION_US UINT64_C(20000)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_INHIBIT_US UINT64_C(100000)
#define IMU_MOTION_GUARD_DEFAULT_MAX_SAMPLE_GAP_US UINT64_C(2000)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_COINCIDENCE_US \
    UINT64_C(3000)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_QUIET_US \
    UINT64_C(20000)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_MAX_GAP_US \
    UINT64_C(1000)
#define IMU_MOTION_GUARD_GRAVITY_MPS2 9.80665f
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_NORM_EXCURSION_MPS2 \
    (0.5f * IMU_MOTION_GUARD_GRAVITY_MPS2)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_DELTA_MPS2 \
    (0.2f * IMU_MOTION_GUARD_GRAVITY_MPS2)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_JERK_MPS3 \
    (50.0f * IMU_MOTION_GUARD_GRAVITY_MPS2)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_SEVERE_DELTA_MPS2 \
    (8.0f * IMU_MOTION_GUARD_GRAVITY_MPS2)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_SEVERE_NORM_MPS2 \
    (8.0f * IMU_MOTION_GUARD_GRAVITY_MPS2)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_REARM_NORM_MPS2 \
    (0.25f * IMU_MOTION_GUARD_GRAVITY_MPS2)
#define IMU_MOTION_GUARD_DEFAULT_ACCEL_DISTURBANCE_REARM_DELTA_MPS2 \
    (0.05f * IMU_MOTION_GUARD_GRAVITY_MPS2)
#define IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US UINT64_C(2000000)
#define IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT 3U

static inline bool imu_motion_guard_sample_can_update_frozen_monitor(
    bool sample_valid,
    bool timestamp_trusted,
    bool saturated)
{
    return sample_valid && timestamp_trusted && !saturated;
}

typedef struct
{
    uint64_t gyro_debounce_us;
    uint64_t common_impact_window_us;
    uint64_t gyro_hard_latch_suppression_us;
    uint64_t accel_inhibit_us;
    uint64_t max_sample_gap_us;
    uint64_t accel_disturbance_coincidence_us;
    uint64_t accel_disturbance_quiet_us;
    uint64_t accel_disturbance_max_sample_gap_us;
    float accel_disturbance_norm_excursion_mps2;
    float accel_disturbance_delta_mps2;
    float accel_disturbance_jerk_mps3;
    float accel_disturbance_severe_delta_mps2;
    float accel_disturbance_severe_norm_mps2;
    float accel_disturbance_rearm_norm_excursion_mps2;
    float accel_disturbance_rearm_delta_mps2;
} imu_motion_guard_config_t;

typedef struct
{
    uint64_t last_timestamp_us;
    bool initialized;
} imu_motion_guard_timestamp_state_t;

typedef struct
{
    /* Inclusive coverage interval used by epoch-causal status encoding. */
    uint64_t start_us;
    uint64_t end_us;
    bool valid;
} imu_motion_guard_saturation_window_t;

/* Main publishes this immutable snapshot before an ISR evaluates a scheduled
 * output epoch. It contains no pointer back to the mutable guard state. */
typedef struct
{
    imu_motion_guard_saturation_window_t accel_saturation_history
        [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT];
    imu_motion_guard_saturation_window_t gyro_saturation_history
        [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT];
    bool valid;
} imu_motion_guard_saturation_snapshot_t;

typedef struct
{
    imu_motion_guard_timestamp_state_t accel_timestamp;
    imu_motion_guard_timestamp_state_t gyro_timestamp;
    uint64_t gyro_streak_start_us;
    uint64_t last_gyro_saturation_us;
    float previous_accel_mps2[3];
    uint64_t previous_accel_timestamp_us;
    uint64_t accel_disturbance_candidate_us;
    uint64_t accel_disturbance_candidate_latest_us;
    uint64_t accel_disturbance_quiet_start_us;
    bool gyro_streak_active;
    bool gyro_hard_fault_active;
    bool last_gyro_saturation_valid;
    bool accel_saturation_evidence_active;
    bool previous_accel_valid;
    bool accel_disturbance_candidate_valid;
    bool accel_disturbance_candidate_armed;
    bool accel_disturbance_rearm_required;
    bool accel_disturbance_quiet_active;
    bool accel_saturated;
    bool gyro_saturated;
} imu_motion_guard_lane_state_t;

typedef struct
{
    uint32_t accel_observation_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t gyro_observation_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t accel_saturation_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t gyro_saturation_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t accel_nonmonotonic_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t gyro_nonmonotonic_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t accel_gap_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t gyro_gap_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t gyro_hard_fault_assertion_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t accel_impact_observation_count;
    uint32_t gyro_common_impact_observation_count;
    uint32_t common_impact_episode_count;
    uint32_t accel_subrange_candidate_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t accel_subrange_gap_count[IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t accel_subrange_common_observation_count;
    uint32_t accel_subrange_severe_observation_count
        [IMU_MOTION_GUARD_LANE_COUNT];
    uint32_t accel_disturbance_episode_count;
    uint32_t accel_disturbance_extension_count;
} imu_motion_guard_counters_t;

typedef struct
{
    imu_motion_guard_config_t config;
    imu_motion_guard_lane_state_t lane[IMU_MOTION_GUARD_LANE_COUNT];
    imu_motion_guard_counters_t counters;
    imu_motion_guard_saturation_window_t accel_saturation_history
        [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT];
    imu_motion_guard_saturation_window_t gyro_saturation_history
        [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT];
    uint64_t last_accel_saturation_us;
    uint64_t last_gyro_saturation_us;
    uint64_t common_impact_start_us;
    uint64_t last_common_impact_us;
    uint64_t accel_disturbance_start_us;
    uint64_t last_accel_disturbance_us;
    bool accel_saturation_seen;
    bool gyro_saturation_seen;
    bool common_impact_valid;
    bool accel_disturbance_valid;
    bool initialized;
} imu_motion_guard_t;

typedef struct
{
    imu_motion_guard_counters_t counters;
    uint64_t gyro_streak_start_us[IMU_MOTION_GUARD_LANE_COUNT];
    uint64_t lane_last_gyro_saturation_us[IMU_MOTION_GUARD_LANE_COUNT];
    imu_motion_guard_saturation_window_t accel_saturation_history
        [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT];
    imu_motion_guard_saturation_window_t gyro_saturation_history
        [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT];
    uint64_t last_accel_saturation_us;
    uint64_t last_gyro_saturation_us;
    uint64_t common_impact_start_us;
    uint64_t last_common_impact_us;
    uint64_t accel_disturbance_start_us;
    uint64_t last_accel_disturbance_us;
    uint8_t gyro_hard_fault_mask;
    bool gyro_streak_active[IMU_MOTION_GUARD_LANE_COUNT];
    bool gyro_hard_latch_suppressed;
    bool accel_inhibited;
    bool accel_disturbance_inhibited;
    bool accel_saturation_seen;
    bool gyro_saturation_seen;
    bool common_impact_valid;
    bool accel_disturbance_valid;
    bool initialized;
} imu_motion_guard_diagnostics_t;

void imu_motion_guard_default_config(imu_motion_guard_config_t *config);

/* Passing NULL for config selects imu_motion_guard_default_config(). */
bool imu_motion_guard_init(imu_motion_guard_t *guard,
                           const imu_motion_guard_config_t *config);

/*
 * The return value reports whether the observation was accepted by this API;
 * it is not sample validity. A saturated sample must still be marked invalid
 * by the caller before it reaches fusion.
 */
bool imu_motion_guard_observe_accel(imu_motion_guard_t *guard,
                                    uint8_t lane,
                                    uint64_t timestamp_us,
                                    bool saturated);
/* Subrange evidence uses a calibrated common-frame vector only when trusted.
 * Severe edges are rotation-integrity common impacts; moderate coincident
 * edges remain accel-only disturbances. Neither changes saturation history.
 * The legacy API above supplies no subrange evidence and remains useful for
 * saturation-only callers. */
bool imu_motion_guard_observe_accel_sample(
    imu_motion_guard_t *guard,
    uint8_t lane,
    uint64_t timestamp_us,
    bool saturated,
    uint64_t evidence_timestamp_us,
    bool evidence_trusted,
    const float accel_mps2[3]);
void imu_motion_guard_reset_accel_subrange_evidence(
    imu_motion_guard_t *guard,
    uint8_t lane);
bool imu_motion_guard_observe_gyro(imu_motion_guard_t *guard,
                                   uint8_t lane,
                                   uint64_t timestamp_us,
                                   bool saturated);

bool imu_motion_guard_gyro_hard_fault_active(const imu_motion_guard_t *guard,
                                             uint8_t lane);
bool imu_motion_guard_accel_saturated(const imu_motion_guard_t *guard,
                                      uint8_t lane);
bool imu_motion_guard_gyro_saturated(const imu_motion_guard_t *guard,
                                     uint8_t lane);
/* Recent windows aggregate both lanes. A strictly increasing forward gap
 * resets continuity-dependent evidence but still records current clipping;
 * equal/backward timestamps cannot refresh the event time. The window is
 * half-open [event, event + hold). */
bool imu_motion_guard_accel_saturation_recent(
    const imu_motion_guard_t *guard,
    uint64_t now_us,
    uint64_t hold_us);
bool imu_motion_guard_gyro_saturation_recent(
    const imu_motion_guard_t *guard,
    uint64_t now_us,
    uint64_t hold_us);
bool imu_motion_guard_accel_saturation_status_at(
    const imu_motion_guard_t *guard,
    uint64_t timestamp_us);
bool imu_motion_guard_gyro_saturation_status_at(
    const imu_motion_guard_t *guard,
    uint64_t timestamp_us);
bool imu_motion_guard_export_saturation_snapshot(
    const imu_motion_guard_t *guard,
    imu_motion_guard_saturation_snapshot_t *snapshot);
bool imu_motion_guard_snapshot_accel_saturation_status_at(
    const imu_motion_guard_saturation_snapshot_t *snapshot,
    uint64_t timestamp_us);
bool imu_motion_guard_snapshot_gyro_saturation_status_at(
    const imu_motion_guard_saturation_snapshot_t *snapshot,
    uint64_t timestamp_us);
uint8_t imu_motion_guard_gyro_hard_fault_mask(
    const imu_motion_guard_t *guard);
bool imu_motion_guard_gyro_hard_latch_suppressed(
    const imu_motion_guard_t *guard,
    uint64_t now_us);
bool imu_motion_guard_accel_inhibited(const imu_motion_guard_t *guard,
                                      uint64_t now_us);
bool imu_motion_guard_accel_disturbance_inhibited(
    const imu_motion_guard_t *guard,
    uint64_t now_us);
uint64_t imu_motion_guard_accel_disturbance_until_us(
    const imu_motion_guard_t *guard);
bool imu_motion_guard_accel_disturbance_interval(
    const imu_motion_guard_t *guard,
    uint64_t *start_us,
    uint64_t *end_us);
/* These legacy interval accessors describe rotation-integrity common impacts,
 * not accel-only subrange disturbances. The start remains fixed for an episode
 * while new clipping evidence extends the half-open end. */
uint64_t imu_motion_guard_accel_inhibit_until_us(
    const imu_motion_guard_t *guard);
bool imu_motion_guard_accel_inhibit_interval(
    const imu_motion_guard_t *guard,
    uint64_t *start_us,
    uint64_t *end_us);

void imu_motion_guard_get_diagnostics(
    const imu_motion_guard_t *guard,
    uint64_t now_us,
    imu_motion_guard_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
