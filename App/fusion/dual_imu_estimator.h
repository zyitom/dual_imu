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

typedef enum
{
    DUAL_IMU_ESTIMATOR_LANE_BMI088 = 0,
    DUAL_IMU_ESTIMATOR_LANE_ICM45686 = 1,
    DUAL_IMU_ESTIMATOR_LANE_NONE = 0xFF
} dual_imu_estimator_lane_t;

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
    float zaru_rate_std_rad_s[DUAL_IMU_ESTIMATOR_LANE_COUNT];

    float stationary_gyro_limit_rad_s;
    float stationary_accel_norm_tolerance_mps2;
    float stationary_accel_pair_limit_mps2;
    uint16_t stationary_dwell_windows;
    uint16_t calibration_accept_windows;
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
    bool lane_seeded[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_calibrated[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_aided_propagation[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_accel_aided[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool output_alignment_active;
    bool stationary_candidate;
    bool stationary_confirmed;
    bool accel_inhibited;
    bool specific_force_valid;
    bool output_valid;
} dual_imu_estimator_output_t;

typedef struct
{
    imu_preintegrator_t preintegrator[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    attitude_mekf_t mekf[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    imu_selector_t selector;
    imu_angular_accel_estimator_t angular_accel_estimator;
    dual_imu_estimator_config_t config;
    uint32_t hard_fault_flags[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t zaru_accept_count[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t accel_bad_streak[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t accel_good_streak[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t stationary_streak;
    uint64_t accel_inhibit_until_us;
    uint64_t windows_processed;
    float output_quaternion[4];
    float output_alignment[4];
    imu_selector_hint_t isolation_hint;
    imu_selector_lane_t previous_selected_lane;
    bool lane_seeded[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_calibrated[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_accel_fault[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool stationary_hint;
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
void dual_imu_estimator_inhibit_accel_until(dual_imu_estimator_t *estimator,
                                            uint64_t timestamp_us);

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
