#ifndef DUAL_IMU_ESTIMATOR_H
#define DUAL_IMU_ESTIMATOR_H

#include "attitude_mekf.h"
#include "cross_lane_calibrator.h"
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

/* Why the estimator last rewrote a lane's MEKF attitude outside of normal
 * propagate/update. Every rewrite is a potential output discontinuity; the
 * output path absorbs it into output_alignment and these codes let the host
 * attribute each absorbed jump to its mechanism. */
typedef enum
{
    DUAL_IMU_ATTITUDE_REWRITE_NONE = 0,
    /* Unseeded/faulted lane reseeded from the current accel window. */
    DUAL_IMU_ATTITUDE_REWRITE_SEED = 1,
    /* Stuck gravity-NIS escalation reseeded tilt from accel. */
    DUAL_IMU_ATTITUDE_REWRITE_ACCEL_RECOVERY = 2,
    /* Post-impact stationary reacquire reseeded tilt from the dwell mean. */
    DUAL_IMU_ATTITUDE_REWRITE_REACQUIRE = 3,
    /* Impact gyro-disagreement rollback restored the checkpoint state. */
    DUAL_IMU_ATTITUDE_REWRITE_ROLLBACK = 4,
} dual_imu_attitude_rewrite_reason_t;

/* attitude_rewrite_last_lane value when a rewrite touched both lanes. */
#define DUAL_IMU_ATTITUDE_REWRITE_LANE_BOTH (2U)

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
    cross_lane_calibrator_config_t cross_lane_calibrator;

    /* Signed vectors from the common virtual origin to each MEMS center. */
    float reference_to_sensor_m[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];

    /* Conservative bring-up uncertainties; replace from latency/Allan tests. */
    float timestamp_jitter_std_s;
    float pipeline_delay_std_s;
    float alignment_std_rad;
    /* Fraction of the rotation angle added to the cross-lane disagreement
     * covariance, base + (k*|omega|*dt)^2 per axis. Absorbs the rate-proportional
     * gyro cross-axis sensitivity (BMI088 1.4%, ICM45686 0.2% per datasheet) plus
     * scale-nonlinearity margin so a fast slew does not read as a lane fault.
     * At |omega|=0 the term vanishes and the gate is bit-identical to before, so
     * static 1v1 soft-fault sensitivity is preserved. Default 0.016; tighten
     * toward ~0.003 once per-lane cross-axis calibration is loaded (FIX_PLAN
     * §9.1-3). */
    float gyro_disagreement_rate_fraction;
    /* Values substituted for gyro_disagreement_rate_fraction and
     * alignment_std_rad in the selector covariance while the cross-lane
     * calibrator is converged (DUAL_FUSION_DESIGN.md §3.1): once the BMI
     * delta-angle is mapped through the estimated misalignment/scale/delay,
     * the remaining deterministic disagreement budget shrinks by roughly a
     * factor of five, so a real lane fault of the same size is detected that
     * much earlier during dynamics. */
    float gyro_disagreement_rate_fraction_calibrated;
    float alignment_std_calibrated_rad;
    float rate_floor_std_rad_s;
    float output_alignment_slew_rad_s;
    /* One-sigma angular-rate uncertainty while neither gyro is observable. */
    float unobserved_rotation_rate_std_rad_s;
    float zaru_rate_std_rad_s[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float zaru_target_residual_limit_rad_s;
    float zaru_bias_slew_limit_rad_s2;
    float zaru_calibration_tolerance_rad_s;
    float zaru_calibration_revoke_tolerance_rad_s;

    /* In-run bias recovery (FIX_PLAN §12.3). The bias-residual gate that admits
     * ZARU requires the learned bias to already be close to correct, so a bias
     * that diverged during violent motion deadlocks the only mechanism that can
     * repair it. When every bias-INDEPENDENT stationary criterion keeps passing
     * yet the residual stays large for this many consecutive windows, reseed the
     * diverged lanes' biases from their raw dwell means. Recovery is admitted
     * only while each lane's raw dwell mean, decomposed against that lane's
     * dwell-mean gravity direction, stays inside two rest tolerances:
     * (a) the gravity-PARALLEL (yaw) component must be near zero, because a
     * steady rotation about gravity keeps every other stationary criterion
     * passing and is genuinely indistinguishable from a yaw-axis bias
     * (FIX_PLAN §12.3-3) -- a slow turntable pan must never be learned as bias;
     * (b) the gravity-PERPENDICULAR (tilt) component gets a wider tolerance
     * sized for post-shock ZRO shift, because a real tilt-axis rotation at that
     * rate would visibly move the gravity direction across the dwell and be
     * rejected by the gravity-stability gate, so a steady perpendicular reading
     * under stable gravity can only be bias. Additionally both heterogeneous
     * lanes' raw dwell means must agree within the dual-lane tolerance so a
     * grossly lying lane cannot donate its fault to the reseed (a unilateral
     * post-shock shift is a genuine bias the reseed should absorb, so this
     * budget is deliberately loose). A single available lane has no cross-check
     * and must hold twice the dwell. */
    float zaru_recovery_rest_yaw_rate_tolerance_rad_s;
    float zaru_recovery_rest_tilt_rate_tolerance_rad_s;
    float zaru_recovery_dual_lane_rate_tolerance_rad_s;
    uint16_t zaru_recovery_reject_windows;

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
    /* After an impact/rollback, ordinary gravity updates stay closed until
     * this many consecutive low-rate, near-1g, dual-lane-consistent windows
     * have re-established that accelerometer direction is usable as gravity. */
    uint16_t post_impact_gravity_trust_windows;
    uint16_t impact_gyro_disagreement_confirm_windows;
    uint16_t calibration_accept_windows;
    uint16_t calibration_revoke_windows;
    uint16_t accel_fault_enter_windows;
    uint16_t accel_fault_recovery_windows;

    /* A shock can leave a large tilt error while covariance stays small, so
     * the gravity NIS gate rejects forever. After this many consecutive NIS
     * rejections on otherwise clean windows, inflate attitude covariance each
     * window until the gate reopens; after the larger count, reseed tilt from
     * gravity while preserving heading. A rejection only counts as stuck
     * evidence on a gravity-quality window (FIX_PLAN §1.2 第 1 层准入条件):
     * the window-mean specific-force magnitude must be within the norm
     * tolerance of gravity AND the rotation rate below the recovery rate
     * limit. During sustained rotation the measurement carries centripetal/
     * tangential acceleration, the gate is rejecting correctly, and inflating
     * or reseeding from it would drive tilt toward the contaminated
     * direction. The two admissions are complementary: perpendicular
     * contamination up to ~3 m/s2 barely changes the magnitude
     * (sqrt(g^2+a^2)-g ~= a^2/2g) yet tilts the direction ~17 deg, while at
     * rates below the limit the centripetal term is physically small. */
    uint16_t accel_recovery_stuck_windows;
    uint16_t accel_recovery_reseed_windows;
    /* One-shot rebuild of tilt straight from the accelerometer once a lane's
     * gravity updates have been NIS-rejected for reseed_windows. It exists as
     * a backstop from when gravity aiding could switch off entirely, and it
     * repairs by rewriting the state and slewing the step out -- the visible
     * "attitude freezes for a second, then jumps back". With aiding now
     * continuous and softly weighted the backstop mostly fires on top of a
     * filter that was already converging, so it is off by default. Covariance
     * inflation (the streak's other branch) still runs either way. */
    bool accel_recovery_reseed_enabled;
    float accel_recovery_inflation_std_rad;
    float accel_recovery_norm_tolerance_mps2;
    float accel_recovery_max_rate_rad_s;

    float accel_update_max_rate_rad_s;
    float accel_update_max_angular_accel_rad_s2;
    float accel_rate_variance_scale_rad_s;
    float accel_angular_accel_variance_scale_rad_s2;
    /* Variance multiplier applied while the motion guard holds its accel
     * inhibit, in place of dropping the measurement. The inhibit runs 100 ms
     * past each disturbance, and measured over real captures it covered ~50%
     * of all frames in which the accelerometer was in fact within 1 g of
     * gravity and the body below the old rate gate -- good data, discarded.
     * A genuinely destroyed reading (an impact at 15 g) is still refused by
     * the MEKF's own norm gate, so this only governs how little the tail of
     * an inhibit is trusted, not whether garbage gets in. */
    float accel_inhibit_variance_scale;

    /* Accelerometer-pair rotation witness (DUAL_FUSION_DESIGN.md §3.3).
     * Projecting the difference of the two accelerometers onto the sensor
     * baseline isolates the centripetal term d*mean(|w_perp|^2), a measured
     * bound on the rotation rate about the axes perpendicular to the
     * baseline. Two consumers: (a) gyro-blind windows use it instead of the
     * blind unobserved_rotation_rate_std_rad_s, so a translational shock
     * with little real rotation no longer costs a full-scale covariance
     * dump; (b) with valid gyros, a residual above the trigger for the
     * configured streak means rotation/vibration the gyros did not capture
     * (the FIX_PLAN §1 "shock that never trips the motion guard" hole) and
     * inflates attitude covariance about the witnessed axes only. Rotation
     * about the baseline itself is invisible to the witness and keeps the
     * blind uncertainty. leakage_fraction absorbs accel-pair misalignment
     * leaking common-mode acceleration into the difference. */
    float rotation_witness_leakage_fraction;
    float rotation_witness_noise_floor_mps2;
    float rotation_witness_safety_factor;
    float rotation_witness_min_rate_std_rad_s;
    float rotation_witness_trigger_residual_mps2;
    uint16_t rotation_witness_trigger_windows;
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
    /* Cumulative MEKF attitude rewrites (all lanes, all reasons) plus the
     * attribution of the most recent one. The residual tilt angle of
     * output_alignment shows how much absorbed discontinuity is still being
     * slewed out of the published attitude. */
    uint32_t attitude_rewrite_count;
    float output_alignment_tilt_rad;
    uint8_t attitude_rewrite_last_reason;
    uint8_t attitude_rewrite_last_lane;
    uint16_t stationary_streak;
    uint16_t stationary_max_streak;
    uint16_t post_impact_gravity_trust_streak;
    uint8_t stationary_lane_mask;
    bool impact_gyro_rollback_pending;
    bool lane_seeded[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_calibrated[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_aided_propagation[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_accel_aided[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_accel_recovery_inflating[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_accel_recovery_reseeded[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool lane_bias_recovery_reseeded[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    /* Cross-lane gyro calibration converged; the selector residual is being
     * computed on the calibrated BMI stream with the tightened budgets. */
    bool cross_lane_calibrated;
    /* Persistent accel-pair residual witnessed rotation the gyros missed;
     * attitude covariance was inflated about the witnessed axes. */
    bool accel_pair_rotation_witness_active;
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
    bool post_impact_gravity_trusted;
    bool gravity_aiding_inhibited;
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
    cross_lane_calibrator_t cross_lane_calibrator;
    imu_angular_accel_estimator_t angular_accel_estimator;
    imu_angular_accel_estimator_t impact_gyro_angular_accel_checkpoint;
    dual_imu_estimator_config_t config;
    uint32_t hard_fault_flags[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t zaru_accept_count[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t zaru_divergence_count[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t zaru_recovery_reject_streak;
    uint32_t zaru_bias_recovery_count;
    uint16_t accel_bad_streak[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t accel_good_streak[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t accel_nis_stuck_streak[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint16_t stationary_streak;
    uint16_t stationary_max_streak;
    uint16_t stationary_soft_exit_streak;
    uint16_t stationary_rate_exit_streak;
    uint16_t attitude_convergence_streak;
    uint16_t attitude_aiding_miss_streak;
    uint16_t post_impact_gravity_trust_streak;
    dual_imu_stationary_reject_reason_t stationary_last_reject_reason;
    uint32_t stationary_reject_count[DUAL_IMU_STATIONARY_REJECT_COUNT];
    float stationary_temporal_gyro_variance_rad2_s2[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    float stationary_temporal_accel_variance_m2_s4[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint32_t stationary_statistics_count;
    uint32_t post_impact_episode_count;
    uint32_t post_impact_reacquire_count;
    uint32_t accel_recovery_inflation_count;
    uint32_t accel_recovery_reseed_count;
    uint32_t accel_pair_witness_count;
    uint16_t accel_pair_witness_streak;
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
    uint32_t attitude_rewrite_count;
    uint8_t attitude_rewrite_last_reason;
    uint8_t attitude_rewrite_last_lane;
    /* Set when a lane's MEKF attitude was rewritten; consumed by the output
     * path, which bridges the jump through output_alignment like a lane
     * switch so the published quaternion never steps discontinuously. */
    bool lane_attitude_discontinuity[DUAL_IMU_ESTIMATOR_LANE_COUNT];
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
    bool post_impact_gravity_trusted;
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
