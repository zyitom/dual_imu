#ifndef DUAL_IMU_ESTIMATOR_H
#define DUAL_IMU_ESTIMATOR_H

#include "attitude_mekf.h"
#include "imu_geometry.h"
#include "imu_preintegrator.h"
#include "imu_selector.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DUAL_IMU_ESTIMATOR_LANE_COUNT (2U)
#define DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT (8U)

typedef enum
{
    DUAL_IMU_ESTIMATOR_LANE_BMI088 = 0,
    DUAL_IMU_ESTIMATOR_LANE_ICM45686 = 1,
    DUAL_IMU_ESTIMATOR_LANE_NONE = 0xFF
} dual_imu_estimator_lane_t;

typedef enum
{
    DUAL_IMU_STATIONARY_REJECT_NONE = 0,
    DUAL_IMU_STATIONARY_REJECT_INVALID_OR_FAULT,
    DUAL_IMU_STATIONARY_REJECT_INSTANT_RATE,
    DUAL_IMU_STATIONARY_REJECT_ACCEL_NORM,
    DUAL_IMU_STATIONARY_REJECT_WINDOW_GYRO_VARIANCE,
    DUAL_IMU_STATIONARY_REJECT_ACCEL_PAIR,
    DUAL_IMU_STATIONARY_REJECT_TEMPORAL_VARIANCE,
    DUAL_IMU_STATIONARY_REJECT_MEAN_RATE,
    DUAL_IMU_STATIONARY_REJECT_GRAVITY_DIRECTION,
    DUAL_IMU_STATIONARY_REJECT_INHIBITED,
    DUAL_IMU_STATIONARY_REJECT_COUNT
} dual_imu_stationary_reject_reason_t;

typedef struct
{
    /* Half-open event-time interval [start_us, end_us). */
    uint64_t start_us;
    uint64_t end_us;
} dual_imu_estimator_accel_inhibit_interval_t;

typedef struct
{
    imu_preintegrator_config_t preintegrator[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    attitude_mekf_config_t mekf[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    imu_selector_config_t selector;

    /* Signed vectors from the common virtual origin to each MEMS center. */
    float reference_to_sensor_m[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];

    /* Conservative bring-up uncertainties; replace from latency/Allan tests. */
    float timestamp_jitter_std_s;
    float pipeline_delay_std_s;
    float alignment_std_rad;
    float rate_floor_std_rad_s;
    float output_alignment_slew_rad_s;
    /* One-sigma angular-rate uncertainty while neither gyro is observable. */
    float unobserved_rotation_rate_std_rad_s;
    float zaru_rate_std_rad_s[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float zaru_target_residual_limit_rad_s;
    float zaru_bias_slew_limit_rad_s2;
    float zaru_calibration_tolerance_rad_s;
    float zaru_calibration_revoke_tolerance_rad_s;

    float stationary_gyro_limit_rad_s;
    /* Per-axis variance limit inside one preintegration window. */
    float stationary_gyro_variance_limit_rad2_s2;
    /* Per-lane total 3-D sample variance of window means over the dwell. */
    float stationary_gyro_temporal_variance_limit_rad2_s2
        [DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float stationary_accel_norm_tolerance_mps2;
    float stationary_accel_pair_limit_mps2;
    /* Total 3-D sample variance of accel vectors over the full dwell. */
    float stationary_accel_temporal_variance_limit_m2_s4;
    /* Chord distance between unit specific-force directions. */
    float stationary_accel_direction_limit;
    uint16_t stationary_dwell_windows;
    uint16_t stationary_hint_dwell_windows;
    uint16_t stationary_single_lane_dwell_windows;
    uint16_t stationary_single_lane_hint_dwell_windows;
    /* Confirmed-stationary exit hysteresis. ZARU is paused while pending. */
    uint16_t stationary_soft_exit_windows;
    uint16_t stationary_rate_exit_windows;
    uint16_t attitude_convergence_windows;
    uint16_t attitude_aiding_timeout_windows;
    uint16_t post_impact_reacquire_dwell_windows;
    uint16_t post_impact_reacquire_single_lane_dwell_windows;
    uint16_t impact_gyro_disagreement_confirm_windows;
    uint16_t calibration_accept_windows;
    uint16_t calibration_revoke_windows;
    uint16_t accel_fault_enter_windows;
    uint16_t accel_fault_recovery_windows;

    float accel_update_max_rate_rad_s;
    float accel_update_max_angular_accel_rad_s2;
    float accel_rate_variance_scale_rad_s;
    float accel_angular_accel_variance_scale_rad_s2;
} dual_imu_estimator_config_t;

typedef struct
{
    uint64_t start_us;
    uint64_t end_us;
    imu_preintegrated_window_t lane_window[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float lane_specific_force_mps2[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    float lane_rate_rad_s[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    float lane_bias_rad_s[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    float lane_quaternion[DUAL_IMU_ESTIMATOR_LANE_COUNT][4];
    attitude_mekf_accel_result_t accel_result[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    attitude_mekf_zaru_result_t zaru_result[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    imu_selector_result_t selector;
    imu_selector_lane_t selected_accel_lane;
    float quaternion[4];
    float euler_rad[3];
    float specific_force_mps2[3];
    float angular_rate_rad_s[3];
    float angular_accel_rad_s2[3];
    float accel_pair_residual_mps2;
    float stationary_temporal_gyro_variance_rad2_s2[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float stationary_temporal_accel_variance_m2_s4[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    dual_imu_stationary_reject_reason_t stationary_last_reject_reason;
    uint16_t stationary_streak;
    uint16_t stationary_max_streak;
    uint8_t stationary_lane_mask;
    bool lane_seeded[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_calibrated[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_aided_propagation[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_accel_aided[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool output_alignment_active;
    bool stationary_candidate;
    bool stationary_confirmed;
    bool stationary_single_lane;
    bool accel_inhibited;
    bool rotation_unobserved;
    bool heading_continuity_lost;
    bool attitude_aiding_stale;
    bool attitude_converged;
    bool post_impact_reacquire_active;
    bool attitude_reacquired;
    bool specific_force_valid;
    bool output_valid;
} dual_imu_estimator_output_t;

typedef struct
{
    imu_preintegrator_t preintegrator[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    attitude_mekf_t mekf[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    attitude_mekf_t impact_gyro_mekf_checkpoint
        [DUAL_IMU_ESTIMATOR_LANE_COUNT];
    imu_selector_t selector;
    imu_angular_accel_estimator_t angular_accel_estimator;
    imu_angular_accel_estimator_t impact_gyro_angular_accel_checkpoint;
    dual_imu_estimator_config_t config;
    uint32_t hard_fault_flags[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t zaru_accept_count[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t zaru_divergence_count[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t accel_bad_streak[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t accel_good_streak[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t stationary_streak;
    uint16_t stationary_max_streak;
    uint16_t stationary_soft_exit_streak;
    uint16_t stationary_rate_exit_streak;
    uint16_t attitude_convergence_streak;
    uint16_t attitude_aiding_miss_streak;
    dual_imu_stationary_reject_reason_t stationary_last_reject_reason;
    uint32_t stationary_reject_count[DUAL_IMU_STATIONARY_REJECT_COUNT];
    float stationary_temporal_gyro_variance_rad2_s2[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float stationary_temporal_accel_variance_m2_s4[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint32_t stationary_statistics_count;
    uint32_t post_impact_episode_count;
    uint32_t post_impact_reacquire_count;
    /* Frozen normalized accel mean after the stationary statistics warmup. */
    float stationary_gravity_reference[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    float stationary_gyro_mean_rad_s[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    float stationary_gyro_m2_rad2_s2[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    float stationary_accel_mean_mps2[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    float stationary_accel_m2_m2_s4[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    dual_imu_estimator_accel_inhibit_interval_t accel_inhibit_intervals
        [DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT];
    dual_imu_estimator_accel_inhibit_interval_t impact_intervals
        [DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT];
    uint64_t filter_fault_window_end_us[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint64_t impact_gyro_suspect_start_us;
    uint64_t impact_gyro_suspect_last_end_us;
    uint64_t heading_continuity_lost_timestamp_us;
    uint64_t windows_processed;
    float output_quaternion[4];
    float output_alignment[4];
    float impact_gyro_output_quaternion_checkpoint[4];
    float impact_gyro_output_alignment_checkpoint[4];
    imu_selector_hint_t isolation_hint;
    imu_selector_lane_t previous_selected_lane;
    imu_selector_lane_t impact_gyro_previous_selected_lane_checkpoint;
    uint8_t stationary_lane_mask;
    uint8_t stationary_confirmed_lane_mask;
    uint8_t accel_inhibit_interval_count;
    uint8_t impact_interval_count;
    uint8_t impact_gyro_checkpoint_lane_seeded_mask;
    uint8_t impact_gyro_checkpoint_filter_fault_mask;
    uint16_t impact_gyro_disagreement_streak;
    bool lane_seeded[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_calibrated[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_accel_fault[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool stationary_gravity_reference_valid[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool stationary_hint;
    bool stationary_confirmed_latched;
    bool impact_interval_was_active;
    bool impact_gyro_checkpoint_valid;
    bool impact_gyro_disagreement_confirmed;
    bool impact_gyro_output_initialized_checkpoint;
    bool heading_continuity_lost;
    bool attitude_converged;
    bool post_impact_reacquire_active;
    bool output_initialized;
    bool initialized;
} dual_imu_estimator_t;

void dual_imu_estimator_default_config(dual_imu_estimator_config_t *config);

bool dual_imu_estimator_init(dual_imu_estimator_t *estimator,
                             const dual_imu_estimator_config_t *config,
                             uint64_t common_epoch_us);

bool dual_imu_estimator_push_accel(dual_imu_estimator_t *estimator,
                                   const imu_accel_sample_t *sample);
bool dual_imu_estimator_push_gyro(dual_imu_estimator_t *estimator,
                                  const imu_gyro_sample_t *sample);

void dual_imu_estimator_set_hard_faults(dual_imu_estimator_t *estimator,
                                        imu_source_t source,
                                        uint32_t hard_fault_flags);
void dual_imu_estimator_set_stationary_hint(dual_imu_estimator_t *estimator,
                                            bool stationary);
void dual_imu_estimator_set_isolation_hint(dual_imu_estimator_t *estimator,
                                           imu_selector_hint_t hint);
/* Pauses gravity aiding only for the half-open event-time interval [start, end). */
bool dual_imu_estimator_inhibit_accel_interval(
    dual_imu_estimator_t *estimator,
    uint64_t start_us,
    uint64_t end_us);
/* Also starts conservative post-impact tilt reacquisition and heading warning. */
bool dual_imu_estimator_notify_impact_interval(
    dual_imu_estimator_t *estimator,
    uint64_t start_us,
    uint64_t end_us);

/*
 * complete_through_us is the common event-time watermark. Both lanes always
 * advance together; a stopped lane therefore produces an invalid window and
 * cannot deadlock the healthy lane.
 */
imu_preintegrator_result_t dual_imu_estimator_process_next(
    dual_imu_estimator_t *estimator,
    uint64_t complete_through_us,
    dual_imu_estimator_output_t *output);

#ifdef __cplusplus
}
#endif

#endif
