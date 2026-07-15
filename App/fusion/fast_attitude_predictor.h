#ifndef FAST_ATTITUDE_PREDICTOR_H
#define FAST_ATTITUDE_PREDICTOR_H

#include "imu_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FAST_ATTITUDE_PREDICTOR_HISTORY_CAPACITY (64U)

typedef enum
{
    FAST_ATTITUDE_FLAG_NONE = 0U,
    FAST_ATTITUDE_FLAG_NOT_INITIALIZED = (1U << 0),
    FAST_ATTITUDE_FLAG_NO_ANCHOR = (1U << 1),
    FAST_ATTITUDE_FLAG_TARGET_BEFORE_ANCHOR = (1U << 2),
    FAST_ATTITUDE_FLAG_GYRO_INVALID = (1U << 3),
    FAST_ATTITUDE_FLAG_GYRO_GAP = (1U << 4),
    FAST_ATTITUDE_FLAG_GYRO_SEQUENCE_GAP = (1U << 5),
    FAST_ATTITUDE_FLAG_GYRO_TIMESTAMP_ORDER = (1U << 6),
    FAST_ATTITUDE_FLAG_HORIZON_EXCEEDED = (1U << 7),
    FAST_ATTITUDE_FLAG_NUMERIC = (1U << 8),
    FAST_ATTITUDE_FLAG_DEADLINE_MISS = (1U << 9),
    FAST_ATTITUDE_FLAG_OUTPUT_STALE = (1U << 10),
    FAST_ATTITUDE_FLAG_SNAPSHOT_INVALID = (1U << 11)
} fast_attitude_flag_t;

typedef enum
{
    FAST_ATTITUDE_QUALITY_ATTITUDE_CONVERGED = (1U << 0),
    FAST_ATTITUDE_QUALITY_POST_IMPACT_REACQUIRE_ACTIVE = (1U << 1),
    FAST_ATTITUDE_QUALITY_ATTITUDE_AIDING_STALE = (1U << 2),
    FAST_ATTITUDE_QUALITY_ROTATION_UNOBSERVED = (1U << 3)
} fast_attitude_quality_flag_t;

typedef struct
{
    uint32_t max_prediction_horizon_us;
    uint32_t max_gyro_gap_us[IMU_SOURCE_COUNT];
    float euler_singularity_threshold_rad;
} fast_attitude_predictor_config_t;

typedef struct
{
    uint64_t timestamp_us;
    float quaternion[4];
    float gyro_rate_rad_s[3];
    float gyro_bias_rad_s[3];
    imu_source_t selected_source;
    uint8_t quality_flags;
    bool degraded;
} fast_attitude_anchor_t;

typedef struct
{
    float quaternion[4];
    float euler_rad[3];
    float gyro_rate_rad_s[3];
    uint64_t anchor_timestamp_us;
    uint64_t latest_gyro_timestamp_us;
    uint64_t output_timestamp_us;
    uint64_t publish_timestamp_us;
    uint32_t sequence;
    uint32_t prediction_horizon_us;
    uint32_t publish_latency_us;
    uint32_t integrity_flags;
    imu_source_t selected_source;
    bool predicted;
    bool degraded;
    bool attitude_converged;
    bool post_impact_reacquire_active;
    bool attitude_aiding_stale;
    bool rotation_unobserved;
    bool deadline_miss;
    bool euler_singular;
    bool accel_saturation_recent;
    bool gyro_saturation_recent;
    bool valid;
} fast_attitude_output_t;

typedef struct
{
    imu_gyro_sample_t sample;
    uint32_t interval_flags;
} fast_attitude_gyro_node_t;

typedef struct
{
    fast_attitude_gyro_node_t nodes[FAST_ATTITUDE_PREDICTOR_HISTORY_CAPACITY];
    uint16_t head;
    uint16_t count;
    uint32_t pending_flags;
    uint64_t last_timestamp_us;
    uint32_t last_sequence;
    bool have_last_sample;
} fast_attitude_gyro_history_t;

typedef struct
{
    float quaternion[4];
    float gyro_rate_rad_s[3];
    float previous_quaternion[4];
    float previous_gyro_rate_rad_s[3];
    uint64_t timestamp_us;
    uint64_t previous_timestamp_us;
    uint64_t coverage_timestamp_us;
    uint32_t integrity_flags;
    uint32_t endpoint_integrity_flags;
    uint32_t previous_integrity_flags;
    bool blocked;
    bool valid;
} fast_attitude_replay_cache_t;

typedef struct
{
    uint64_t timestamp_us;
    float quaternion[4];
    float gyro_rate_rad_s[3];
    uint32_t integrity_flags;
} fast_attitude_snapshot_endpoint_t;

/* Snapshot endpoint rates are bias-corrected and the value is read-only after
 * publication to an ISR. */
typedef struct
{
    fast_attitude_snapshot_endpoint_t previous_endpoint;
    fast_attitude_snapshot_endpoint_t current_endpoint;
    uint64_t anchor_timestamp_us;
    uint64_t coverage_timestamp_us;
    float anchor_quaternion[4];
    float anchor_gyro_rate_rad_s[3];
    float gyro_bias_rad_s[3];
    uint32_t integrity_flags;
    uint32_t max_prediction_horizon_us;
    uint32_t max_gyro_gap_us;
    float euler_singularity_threshold_rad;
    imu_source_t selected_source;
    uint8_t quality_flags;
    bool degraded;
    bool blocked;
    bool valid;
} fast_attitude_snapshot_t;

typedef struct
{
    fast_attitude_predictor_config_t config;
    fast_attitude_gyro_history_t history[IMU_SOURCE_COUNT];
    fast_attitude_anchor_t anchor;
    fast_attitude_replay_cache_t replay_cache;
    uint64_t history_floor_timestamp_us;
    bool history_floor_valid;
    bool anchor_valid;
    bool initialized;
} fast_attitude_predictor_t;

void fast_attitude_predictor_default_config(
    fast_attitude_predictor_config_t *config);

bool fast_attitude_predictor_init(
    fast_attitude_predictor_t *predictor,
    const fast_attitude_predictor_config_t *config);

void fast_attitude_predictor_reset(fast_attitude_predictor_t *predictor);

void fast_attitude_predictor_invalidate_anchor(
    fast_attitude_predictor_t *predictor);

bool fast_attitude_predictor_push_gyro(
    fast_attitude_predictor_t *predictor,
    const imu_gyro_sample_t *sample);

bool fast_attitude_predictor_set_anchor(
    fast_attitude_predictor_t *predictor,
    const fast_attitude_anchor_t *anchor);

bool fast_attitude_predictor_predict(
    const fast_attitude_predictor_t *predictor,
    uint64_t output_timestamp_us,
    fast_attitude_output_t *output);

/* A non-null destination is always overwritten. Failure leaves an explicit
 * invalid snapshot with reason flags suitable for publication. */
bool fast_attitude_predictor_export_snapshot(
    const fast_attitude_predictor_t *predictor,
    fast_attitude_snapshot_t *snapshot);

/* Only the most recent causal interval is retained. Targets older than its
 * previous endpoint (except the exact anchor) report OUTPUT_STALE. */
bool fast_attitude_snapshot_predict(
    const fast_attitude_snapshot_t *snapshot,
    uint64_t output_timestamp_us,
    fast_attitude_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
