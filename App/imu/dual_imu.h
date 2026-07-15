#ifndef DUAL_IMU_H
#define DUAL_IMU_H

#include "imu_types.h"
#include "imu_calibration.h"
#include "imu_motion_guard.h"
#include "imu_selector.h"
#include "fast_attitude_predictor.h"

#include <stdbool.h>
#include <stdint.h>

#define DUAL_IMU_ATTITUDE_OUTPUT_RATE_HZ          (1600U)
#define DUAL_IMU_ATTITUDE_OUTPUT_PERIOD_US         (625U)
#define DUAL_IMU_ATTITUDE_PUBLISH_DEADLINE_US      (250U)
#define DUAL_IMU_ATTITUDE_MAX_PREDICTION_HORIZON_US (3000U)
#define DUAL_IMU_BMI088_ACCEL_REGISTER_SNAPSHOT_SIZE (16U)

typedef enum
{
    DUAL_IMU_MODE_NONE = 0,
    DUAL_IMU_MODE_BMI088,
    DUAL_IMU_MODE_ICM45686,
    DUAL_IMU_MODE_FUSED
} dual_imu_mode_t;

typedef enum
{
    DUAL_IMU_FAULT_NONE = 0U,
    DUAL_IMU_FAULT_INVALID = (1U << 0),
    DUAL_IMU_FAULT_STALE = (1U << 1),
    DUAL_IMU_FAULT_HEALTH = (1U << 2),
    DUAL_IMU_FAULT_ACCEL_SATURATED = (1U << 3),
    DUAL_IMU_FAULT_GYRO_SATURATED = (1U << 4),
    DUAL_IMU_FAULT_ACCEL_FROZEN = (1U << 5),
    DUAL_IMU_FAULT_GYRO_FROZEN = (1U << 6),
    DUAL_IMU_FAULT_FILTER = (1U << 7),
    DUAL_IMU_FAULT_ACCEL_TIMING = (1U << 8),
    DUAL_IMU_FAULT_GYRO_TIMING = (1U << 9),
    DUAL_IMU_FAULT_TIMING = DUAL_IMU_FAULT_ACCEL_TIMING |
                            DUAL_IMU_FAULT_GYRO_TIMING,
    DUAL_IMU_FAULT_SATURATED = DUAL_IMU_FAULT_ACCEL_SATURATED |
                               DUAL_IMU_FAULT_GYRO_SATURATED,
    DUAL_IMU_FAULT_FROZEN = DUAL_IMU_FAULT_ACCEL_FROZEN |
                            DUAL_IMU_FAULT_GYRO_FROZEN
} dual_imu_fault_t;

typedef enum
{
    DUAL_IMU_HINT_NONE = 0,
    DUAL_IMU_HINT_BMI088_BAD,
    DUAL_IMU_HINT_ICM45686_BAD
} dual_imu_isolation_hint_t;

typedef fast_attitude_output_t dual_imu_attitude_output_t;

typedef struct
{
    bool bmi088_initialized;
    bool icm45686_initialized;
    bool bmi088_calibrated;
    bool icm45686_calibrated;
    uint8_t bmi088_accel_id;
    uint8_t bmi088_gyro_id;
    uint8_t icm45686_id;
    dual_imu_mode_t mode;
    imu_accel_sample_t bmi088_accel_sample;
    imu_gyro_sample_t bmi088_gyro_sample;
    imu_sample_t bmi088_sample;
    imu_sample_t icm45686_sample;
    imu_sample_t fused_sample;
    imu_health_t bmi088_health;
    imu_health_t icm45686_health;
    imu_source_t selected_source;
    imu_source_t selected_accel_source;
    imu_selector_state_t selector_state;
    uint32_t selector_reason_flags;
    float selector_residual_nis;
    float lane_quaternion[IMU_SOURCE_COUNT][4];
    float lane_gyro_bias_rad_s[IMU_SOURCE_COUNT][3];
    float mekf_quaternion[4];
    float mekf_euler_rad[3];
    float quaternion[4];
    float euler_rad[3];
    float control_gyro_rad_s[3];
    uint64_t control_gyro_timestamp_us;
    float accel_residual_mps2;
    float gyro_residual_rad_s;
    uint32_t bmi088_fault_flags;
    uint32_t icm45686_fault_flags;
    uint32_t pair_skew_us;
    uint32_t fusion_count;
    uint32_t fast_attitude_output_count;
    uint32_t fast_attitude_invalid_count;
    uint32_t fast_attitude_missed_tick_count;
    uint32_t fast_attitude_last_integrity_flags;
    uint32_t fast_attitude_last_publish_latency_us;
    uint32_t fast_attitude_max_publish_latency_us;
    uint32_t fast_attitude_last_prediction_horizon_us;
    uint32_t fast_attitude_compare_event_count;
    uint32_t fast_attitude_compare_missed_count;
    uint32_t fast_attitude_tick_drop_count;
    uint32_t fast_attitude_queue_drop_count;
    bool fast_attitude_scheduler_running;
    uint32_t fifo_service_last_us;
    uint32_t fifo_service_max_us;
    uint32_t estimator_process_last_us;
    uint32_t estimator_process_max_us;
    uint32_t selection_count;
    uint32_t mismatch_count;
    uint32_t unpaired_count;
    uint32_t filter_reject_count;
    uint32_t bmi088_watchdog_failure_count;
    uint32_t bmi088_accel_watchdog_failure_count;
    uint32_t bmi088_gyro_watchdog_failure_count;
    uint32_t icm45686_watchdog_failure_count;
    uint32_t icm45686_accel_watchdog_failure_count;
    uint32_t icm45686_gyro_watchdog_failure_count;
    bool bmi088_accel_configuration_fault;
    bool bmi088_gyro_configuration_fault;
    bool icm45686_configuration_fault;
    bool icm45686_accel_configuration_fault;
    bool icm45686_gyro_configuration_fault;
    uint32_t lane_window_flags[IMU_SOURCE_COUNT];
    uint32_t lane_accel_reject_count[IMU_SOURCE_COUNT];
    uint32_t lane_zaru_accept_count[IMU_SOURCE_COUNT];
    bool stationary_candidate;
    bool stationary_confirmed;
    bool stationary_single_lane;
    uint8_t stationary_lane_mask;
    uint8_t stationary_last_reject_reason;
    uint16_t stationary_streak;
    uint16_t stationary_max_streak;
    float stationary_temporal_gyro_variance_rad2_s2[IMU_SOURCE_COUNT];
    float stationary_temporal_accel_variance_m2_s4[IMU_SOURCE_COUNT];
    bool stationary_hint_active;
    bool accel_update_inhibited;
    bool rotation_unobserved;
    bool heading_continuity_lost;
    uint64_t heading_continuity_lost_timestamp_us;
    uint64_t estimator_assessed_through_us;
    bool attitude_aiding_stale;
    bool attitude_converged;
    bool post_impact_reacquire_active;
    uint32_t post_impact_episode_count;
    uint32_t post_impact_reacquire_count;
    uint32_t motion_guard_common_impact_count;
    uint32_t motion_guard_accel_saturation_count[IMU_SOURCE_COUNT];
    uint32_t motion_guard_gyro_saturation_count[IMU_SOURCE_COUNT];
    uint32_t motion_guard_accel_subrange_candidate_count[IMU_SOURCE_COUNT];
    uint32_t motion_guard_accel_subrange_severe_count[IMU_SOURCE_COUNT];
    uint32_t motion_guard_accel_subrange_common_count;
    uint32_t motion_guard_accel_disturbance_episode_count;
    uint32_t motion_guard_accel_disturbance_extension_count;
    uint64_t motion_guard_last_accel_saturation_us;
    uint64_t motion_guard_last_gyro_saturation_us;
    uint64_t motion_guard_last_accel_disturbance_us;
    imu_motion_guard_saturation_window_t
        motion_guard_accel_saturation_history
            [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT];
    imu_motion_guard_saturation_window_t
        motion_guard_gyro_saturation_history
            [IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT];
    uint64_t motion_guard_last_impact_us;
    uint8_t motion_guard_gyro_hard_fault_mask;
    bool motion_guard_accel_saturation_seen;
    bool motion_guard_gyro_saturation_seen;
    bool motion_guard_accel_disturbance_valid;
    bool motion_guard_accel_disturbance_active;
    bool motion_guard_gyro_latch_suppressed;
    bool fused_accel_valid;
    bool selection_changed;
    bool alignment_blend_active;
    uint32_t bmi088_accel_irq_count;
    uint32_t bmi088_gyro_irq_count;
    uint32_t icm45686_irq_count;
    uint32_t stream_read_count[IMU_SOURCE_COUNT][2];
    uint32_t stream_coalesced_count[IMU_SOURCE_COUNT][2];
    uint32_t stream_max_irq_to_read_us[IMU_SOURCE_COUNT][2];
    uint32_t bmi088_accel_capture_overrun_count;
    uint32_t bmi088_gyro_capture_overrun_count;
    bool bmi088_hardware_timestamp_enabled;
    bool custom_calibration_loaded[IMU_SOURCE_COUNT];
    imu_calibration_diagnostics_t calibration_diagnostics[IMU_SOURCE_COUNT];
    bool temperature_stale[IMU_SOURCE_COUNT];
    uint32_t bmi088_accel_event_drop_count;
    uint32_t bmi088_gyro_event_drop_count;
    uint32_t icm45686_event_drop_count;
    uint32_t bmi088_accel_buffer_overwrite_count;
    uint32_t bmi088_gyro_buffer_overwrite_count;
    uint32_t icm45686_accel_buffer_overwrite_count;
    uint32_t icm45686_gyro_buffer_overwrite_count;
    uint32_t bmi088_accel_fifo_batch_count;
    uint32_t bmi088_gyro_fifo_batch_count;
    uint32_t bmi088_fifo_dma_error_count;
    uint32_t bmi088_temperature_read_count;
    uint32_t bmi088_temperature_read_error_count;
    uint32_t bmi088_accel_fifo_length_read_count;
    uint32_t bmi088_accel_fifo_empty_length_count;
    uint32_t bmi088_accel_timeline_reset_count;
    uint32_t bmi088_gyro_capture_mismatch_count;
    uint32_t bmi088_gyro_capture_queue_overflow_count;
    uint32_t bmi088_gyro_capture_mismatch_reason;
    uint32_t bmi088_gyro_warmup_discard_count;
    uint32_t bmi088_gyro_recovery_attempt_count;
    uint32_t bmi088_gyro_recovery_success_count;
    uint32_t bmi088_gyro_recovery_failure_count;
    uint32_t bmi088_gyro_last_recovery_reason;
    uint8_t bmi088_gyro_last_recovery_result;
    bool bmi088_gyro_recovery_pending;
    bool bmi088_gyro_recovery_warmup;
    bool bmi088_gyro_capture_sync_fault;
    uint16_t bmi088_accel_fifo_last_bytes;
    uint16_t bmi088_accel_fifo_peak_bytes;
    uint8_t bmi088_accel_fifo_length_response[4];
    uint8_t bmi088_accel_register_snapshot[
        DUAL_IMU_BMI088_ACCEL_REGISTER_SNAPSHOT_SIZE];
    uint32_t bmi088_accel_sensor_time_before;
    uint32_t bmi088_accel_sensor_time_after;
    int16_t bmi088_accel_direct_raw[3];
    uint16_t bmi088_accel_initial_fifo_bytes;
    bool bmi088_accel_snapshot_valid;
    uint32_t icm45686_fifo_batch_count;
    uint32_t icm45686_fifo_dma_error_count;
    uint32_t icm45686_temperature_invalid_frame_count;
    bool bmi088_fifo_clock_sync_valid;
    bool icm45686_fifo_clock_sync_valid;
    uint32_t fifo_clock_anchor_accepted_count[IMU_SOURCE_COUNT];
    uint32_t fifo_clock_anchor_rejected_count[IMU_SOURCE_COUNT];
    uint64_t fifo_clock_reference_mcu_us[IMU_SOURCE_COUNT];
    float fifo_clock_scale[IMU_SOURCE_COUNT];
    float fifo_clock_residual_sigma_us[IMU_SOURCE_COUNT];
    uint32_t fifo_clock_reject_reason_count[IMU_SOURCE_COUNT][4];
    uint32_t fifo_clock_stale_reset_count[IMU_SOURCE_COUNT];
    uint32_t fifo_clock_causal_reset_count[IMU_SOURCE_COUNT];
    uint32_t icm45686_timestamp_discontinuity_count;
    float fifo_clock_last_observed_scale[IMU_SOURCE_COUNT];
    float fifo_clock_last_residual_us[IMU_SOURCE_COUNT];
    uint16_t icm45686_fifo_last_frames;
    uint16_t icm45686_fifo_peak_frames;
} dual_imu_state_t;

bool dual_imu_init(void);
/* Call immediately before enabling the ICM EXTI line. */
bool dual_imu_start_streaming(void);
void dual_imu_process(void);
void dual_imu_handle_exti(uint16_t gpio_pin);
void dual_imu_notify_impact(uint32_t inhibit_duration_us);
void dual_imu_set_stationary_hint(bool stationary);
/* Non-NONE external isolation evidence must be refreshed at least every 100 ms. */
void dual_imu_set_isolation_hint(dual_imu_isolation_hint_t hint);
bool dual_imu_get_control_gyro(imu_gyro_sample_t *sample);
bool dual_imu_get_attitude(dual_imu_attitude_output_t *output);
/* Pops every TIM5-published frame in order; intended for non-ISR transports. */
bool dual_imu_pop_attitude(dual_imu_attitude_output_t *output);
bool dual_imu_set_calibration(imu_source_t source,
                              const imu_calibration_t *calibration);
bool dual_imu_get_calibration(imu_source_t source,
                              imu_calibration_t *calibration);
/* Enabling is pre-stream only; disabling is always allowed as a safe fallback. */
bool dual_imu_set_temperature_compensation_enabled(imu_source_t source,
                                                   bool enabled);
/* Enabling is qualification-only and must happen before the first stream read. */
bool dual_imu_set_gyro_g_sensitivity_enabled(imu_source_t source,
                                            bool enabled);
bool dual_imu_pop_accel(imu_source_t source, imu_accel_sample_t *sample);
bool dual_imu_pop_gyro(imu_source_t source, imu_gyro_sample_t *sample);
const dual_imu_state_t *dual_imu_get_state(void);

#endif
