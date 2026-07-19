#ifndef IMU_PREINTEGRATOR_H
#define IMU_PREINTEGRATOR_H

#include "imu_types.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_PREINTEGRATOR_VECTOR_DIM       (3U)
#define IMU_PREINTEGRATOR_QUEUE_CAPACITY   (128U)
#define IMU_PREINTEGRATOR_DEFAULT_WINDOW_US (2500U)

typedef enum
{
    IMU_PREINTEGRATOR_NOT_READY = 0,
    IMU_PREINTEGRATOR_WINDOW_READY,
    IMU_PREINTEGRATOR_ERROR
} imu_preintegrator_result_t;

typedef enum
{
    IMU_PREINTEGRATOR_FLAG_NONE = 0U,
    IMU_PREINTEGRATOR_FLAG_GYRO_MISSING = (1U << 0),
    IMU_PREINTEGRATOR_FLAG_GYRO_INVALID = (1U << 1),
    IMU_PREINTEGRATOR_FLAG_GYRO_GAP = (1U << 2),
    IMU_PREINTEGRATOR_FLAG_GYRO_SEQUENCE_DROP = (1U << 3),
    IMU_PREINTEGRATOR_FLAG_GYRO_TIMESTAMP_ORDER = (1U << 4),
    IMU_PREINTEGRATOR_FLAG_GYRO_QUEUE_OVERFLOW = (1U << 5),
    IMU_PREINTEGRATOR_FLAG_GYRO_NUMERIC = (1U << 6),
    IMU_PREINTEGRATOR_FLAG_ACCEL_MISSING = (1U << 8),
    IMU_PREINTEGRATOR_FLAG_ACCEL_INVALID = (1U << 9),
    IMU_PREINTEGRATOR_FLAG_ACCEL_GAP = (1U << 10),
    IMU_PREINTEGRATOR_FLAG_ACCEL_SEQUENCE_DROP = (1U << 11),
    IMU_PREINTEGRATOR_FLAG_ACCEL_TIMESTAMP_ORDER = (1U << 12),
    IMU_PREINTEGRATOR_FLAG_ACCEL_QUEUE_OVERFLOW = (1U << 13),
    IMU_PREINTEGRATOR_FLAG_ACCEL_NUMERIC = (1U << 14)
} imu_preintegrator_flag_t;

typedef struct
{
    uint32_t window_us;

    /* Zero disables the corresponding timestamp-gap check. */
    uint32_t max_gyro_gap_us;
    uint32_t max_accel_gap_us;
} imu_preintegrator_config_t;

typedef struct
{
    imu_source_t source;
    uint64_t start_us;
    uint64_t end_us;

    /* Net body rotation over the window, including all coning terms. */
    float delta_angle_rad[IMU_PREINTEGRATOR_VECTOR_DIM];
    float delta_quaternion[4];
    float first_half_delta_angle_rad[IMU_PREINTEGRATOR_VECTOR_DIM];
    float second_half_delta_angle_rad[IMU_PREINTEGRATOR_VECTOR_DIM];

    /*
     * Time statistics of the calibrated gyro samples before MEKF bias
     * subtraction.  The second moment retains zero-mean angular vibration for
     * rigid-body lever-arm compensation; squaring gyro_mean_rad_s would lose it.
     */
    float gyro_mean_rad_s[IMU_PREINTEGRATOR_VECTOR_DIM];
    float gyro_start_rad_s[IMU_PREINTEGRATOR_VECTOR_DIM];
    float gyro_end_rad_s[IMU_PREINTEGRATOR_VECTOR_DIM];
    float gyro_second_moment_rad2_s2[IMU_PREINTEGRATOR_VECTOR_DIM]
                                      [IMU_PREINTEGRATOR_VECTOR_DIM];

    /* Trapezoidal time average over valid accelerometer coverage. */
    float accel_mean_mps2[IMU_PREINTEGRATOR_VECTOR_DIM];

    uint32_t gyro_coverage_us;
    uint32_t accel_coverage_us;
    uint16_t gyro_segment_count;
    uint16_t accel_segment_count;
    uint32_t flags;
    bool gyro_propagation_valid;
    bool gyro_valid;
    bool accel_valid;
} imu_preintegrated_window_t;

typedef struct
{
    uint64_t windows_produced;
    uint64_t gyro_invalid_windows;
    uint64_t accel_invalid_windows;

    uint64_t gyro_samples_accepted;
    uint64_t accel_samples_accepted;
    uint64_t gyro_invalid_samples;
    uint64_t accel_invalid_samples;
    uint64_t gyro_out_of_order_samples;
    uint64_t accel_out_of_order_samples;
    uint64_t gyro_sequence_discontinuities;
    uint64_t accel_sequence_discontinuities;
    uint64_t gyro_estimated_dropped_samples;
    uint64_t accel_estimated_dropped_samples;
    uint64_t gyro_gap_count;
    uint64_t accel_gap_count;
    uint64_t gyro_queue_overflows;
    uint64_t accel_queue_overflows;
    uint64_t maximum_gyro_gap_us;
    uint64_t maximum_accel_gap_us;
} imu_preintegrator_diagnostics_t;

/* Public for static allocation; callers must treat queue nodes as private. */
typedef struct
{
    imu_gyro_sample_t sample;
    uint32_t interval_flags;
} imu_preintegrator_gyro_node_t;

typedef struct
{
    imu_accel_sample_t sample;
    uint32_t interval_flags;
} imu_preintegrator_accel_node_t;

typedef struct
{
    imu_preintegrator_config_t config;
    imu_source_t source;
    uint64_t next_window_start_us;

    imu_preintegrator_gyro_node_t gyro_queue[IMU_PREINTEGRATOR_QUEUE_CAPACITY];
    imu_preintegrator_accel_node_t accel_queue[IMU_PREINTEGRATOR_QUEUE_CAPACITY];
    uint16_t gyro_head;
    uint16_t gyro_count;
    uint16_t accel_head;
    uint16_t accel_count;
    uint32_t pending_gyro_flags;
    uint32_t pending_accel_flags;

    imu_preintegrator_diagnostics_t diagnostics;
    bool initialized;
} imu_preintegrator_t;

void imu_preintegrator_default_config(imu_preintegrator_config_t *config);

/* Use the same epoch_us for both lanes to obtain directly comparable windows. */
bool imu_preintegrator_init(imu_preintegrator_t *preintegrator,
                            const imu_preintegrator_config_t *config,
                            imu_source_t source,
                            uint64_t epoch_us);

void imu_preintegrator_reset(imu_preintegrator_t *preintegrator,
                             uint64_t epoch_us);

bool imu_preintegrator_push_gyro(imu_preintegrator_t *preintegrator,
                                 const imu_gyro_sample_t *sample);
bool imu_preintegrator_push_accel(imu_preintegrator_t *preintegrator,
                                  const imu_accel_sample_t *sample);

/* Non-mutating readiness query for coordinating multiple common-epoch lanes. */
imu_preintegrator_result_t imu_preintegrator_next_window_ready(
    const imu_preintegrator_t *preintegrator,
    uint64_t complete_through_us);

/*
 * complete_through_us is an event-time watermark: before calling, the caller
 * guarantees that every sample with timestamp <= this value has already been
 * pushed. The manager pushes each lane's samples straight from FIFO ingest,
 * then advances this watermark after its chosen FIFO/IRQ latency.
 * A stream normally also needs a sample at/after the window end for boundary
 * interpolation. If it stops, max_*_gap_us acts as the grace period before an
 * invalid window is emitted instead of deadlocking (window_us if the check is
 * disabled).
 */
imu_preintegrator_result_t imu_preintegrator_next_window(
    imu_preintegrator_t *preintegrator,
    uint64_t complete_through_us,
    imu_preintegrated_window_t *window);

const imu_preintegrator_diagnostics_t *imu_preintegrator_get_diagnostics(
    const imu_preintegrator_t *preintegrator);

#ifdef __cplusplus
}
#endif

#endif
