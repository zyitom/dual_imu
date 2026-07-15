#ifndef IMU_CAUSAL_ACCEL_HISTORY_H
#define IMU_CAUSAL_ACCEL_HISTORY_H

#include "imu_types.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CAUSAL_ACCEL_HISTORY_CAPACITY (16U)

typedef enum
{
    IMU_CAUSAL_ACCEL_RESET_NONE = 0,
    IMU_CAUSAL_ACCEL_RESET_EXPLICIT,
    IMU_CAUSAL_ACCEL_RESET_INVALID,
    IMU_CAUSAL_ACCEL_RESET_SATURATED,
    IMU_CAUSAL_ACCEL_RESET_GAP
} imu_causal_accel_reset_reason_t;

typedef enum
{
    IMU_CAUSAL_ACCEL_QUERY_OK = 0,
    IMU_CAUSAL_ACCEL_QUERY_EMPTY,
    IMU_CAUSAL_ACCEL_QUERY_INVALIDATED,
    IMU_CAUSAL_ACCEL_QUERY_SATURATED,
    IMU_CAUSAL_ACCEL_QUERY_FUTURE_ONLY,
    IMU_CAUSAL_ACCEL_QUERY_STALE,
    IMU_CAUSAL_ACCEL_QUERY_SOURCE_MISMATCH,
    IMU_CAUSAL_ACCEL_QUERY_INVALID_ARGUMENT
} imu_causal_accel_query_status_t;

typedef struct
{
    uint32_t accepted_count;
    uint32_t duplicate_replacement_count;
    uint32_t out_of_order_count;
    uint32_t too_old_drop_count;
    uint32_t invalid_reset_count;
    uint32_t saturated_reset_count;
    uint32_t gap_reset_count;
    uint32_t explicit_reset_count;
    uint32_t invalidation_barrier_reject_count;
    uint32_t query_ok_count;
    uint32_t query_future_count;
    uint32_t query_stale_count;
    uint32_t query_empty_count;
    uint32_t query_source_mismatch_count;
} imu_causal_accel_history_diagnostics_t;

typedef struct
{
    imu_causal_accel_query_status_t status;
    imu_accel_sample_t sample;
    uint64_t age_us;
} imu_causal_accel_query_result_t;

typedef struct
{
    imu_accel_sample_t samples[IMU_CAUSAL_ACCEL_HISTORY_CAPACITY];
    imu_causal_accel_history_diagnostics_t diagnostics;
    uint64_t newest_timestamp_us;
    uint64_t invalidation_timestamp_us;
    uint32_t maximum_sample_gap_us;
    uint32_t maximum_sample_age_us;
    size_t count;
    imu_source_t source;
    imu_causal_accel_reset_reason_t last_reset_reason;
    bool newest_timestamp_valid;
    bool invalidation_timestamp_valid;
    bool initialized;
} imu_causal_accel_history_t;

bool imu_causal_accel_history_init(imu_causal_accel_history_t *history,
                                   imu_source_t source,
                                   uint32_t maximum_sample_gap_us,
                                   uint32_t maximum_sample_age_us);
void imu_causal_accel_history_reset(imu_causal_accel_history_t *history);
bool imu_causal_accel_history_push(imu_causal_accel_history_t *history,
                                   const imu_accel_sample_t *sample,
                                   bool timestamp_trusted,
                                   bool saturated);
bool imu_causal_accel_history_query(
    imu_causal_accel_history_t *history,
    imu_source_t source,
    uint64_t gyro_timestamp_us,
    imu_causal_accel_query_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
