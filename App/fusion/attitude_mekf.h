#ifndef ATTITUDE_MEKF_H
#define ATTITUDE_MEKF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATTITUDE_MEKF_VECTOR_DIM (3U)
#define ATTITUDE_MEKF_STATE_DIM  (6U)

typedef enum
{
    ATTITUDE_MEKF_ACCEL_ACCEPTED = 0,
    ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT,
    ATTITUDE_MEKF_ACCEL_REJECTED_NORM,
    ATTITUDE_MEKF_ACCEL_REJECTED_NIS,
    ATTITUDE_MEKF_ACCEL_REJECTED_CORRECTION,
    ATTITUDE_MEKF_ACCEL_NUMERIC_FAILURE
} attitude_mekf_accel_result_t;

typedef enum
{
    ATTITUDE_MEKF_ZARU_ACCEPTED = 0,
    ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT,
    ATTITUDE_MEKF_ZARU_REJECTED_NIS,
    ATTITUDE_MEKF_ZARU_REJECTED_CORRECTION,
    ATTITUDE_MEKF_ZARU_NUMERIC_FAILURE
} attitude_mekf_zaru_result_t;

typedef struct
{
    /* Squares of rad/s/sqrt(Hz) and rad/s/sqrt(s), in the calibrated body frame. */
    float gyro_noise_psd[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];
    float gyro_bias_rw_psd[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];

    /* Covariance of normalized specific-force direction, in rad^2. */
    float accel_direction_covariance[ATTITUDE_MEKF_VECTOR_DIM][ATTITUDE_MEKF_VECTOR_DIM];

    float initial_attitude_std_rad;
    float initial_bias_std_rad_s;
    float standard_gravity_mps2;
    float accel_norm_soft_deviation_mps2;
    /* Admission gate for paths that WRITE tilt straight from the measured
     * specific force (seed/reset). A contaminated vector becomes the state
     * there, with no covariance to discount it, so this stays tight. */
    float accel_norm_hard_deviation_mps2;
    /* Admission gate for the measurement update, which only nudges the state
     * through a gain that accel_variance_scale_max already bounds. It is
     * deliberately looser: rejecting every off-norm sample during sustained
     * rotation leaves tilt running open-loop, which drifts far worse than a
     * heavily down-weighted update pulls it wrong. Must be >= the soft
     * deviation; there is no requirement that it match the seed gate. */
    float accel_update_norm_hard_deviation_mps2;
    float accel_variance_scale_max;
    float accel_nis_gate;
    float zaru_nis_gate;
    /* Reject corrections beyond max_attitude_correction_rad as implausible,
     * but apply an accepted accel correction by at most this much per update.
     * The gain is scaled with the state correction so the Joseph covariance
     * update remains consistent with the under-relaxed measurement update. */
    float max_accel_correction_step_rad;
    float max_attitude_correction_rad;
    float max_abs_bias_rad_s;
    float min_dt_s;
    float max_dt_s;
    float covariance_floor;
    float covariance_ceiling;
} attitude_mekf_config_t;

typedef struct
{
    uint32_t propagation_count;
    uint32_t propagation_reject_count;
    uint32_t unobserved_rotation_count;
    uint32_t accel_update_count;
    uint32_t accel_accept_count;
    uint32_t accel_invalid_reject_count;
    uint32_t accel_norm_reject_count;
    uint32_t accel_nis_reject_count;
    uint32_t accel_correction_reject_count;
    uint32_t zaru_update_count;
    uint32_t zaru_accept_count;
    uint32_t zaru_invalid_reject_count;
    uint32_t zaru_nis_reject_count;
    uint32_t zaru_correction_reject_count;
    uint32_t numeric_fault_count;
    float last_accel_norm_mps2;
    float last_accel_nis;
    float last_accel_variance_scale;
    attitude_mekf_accel_result_t last_accel_result;
    float last_zaru_nis;
    attitude_mekf_zaru_result_t last_zaru_result;
} attitude_mekf_diagnostics_t;

typedef struct
{
    /* q is [w, x, y, z] and rotates calibrated body vectors into NED world axes. */
    float q[4];
    float gyro_bias_rad_s[ATTITUDE_MEKF_VECTOR_DIM];
    float covariance[ATTITUDE_MEKF_STATE_DIM][ATTITUDE_MEKF_STATE_DIM];
    attitude_mekf_config_t config;
    attitude_mekf_diagnostics_t diagnostics;
    bool initialized;
} attitude_mekf_t;

/* Defaults are conservative bring-up values; replace all noise terms with lane data. */
void attitude_mekf_default_config(attitude_mekf_config_t *config);

/* Passing NULL for config selects attitude_mekf_default_config(). */
bool attitude_mekf_init(attitude_mekf_t *filter,
                        const attitude_mekf_config_t *config);

bool attitude_mekf_reset(attitude_mekf_t *filter,
                         const float quaternion_body_to_world[4],
                         const float gyro_bias_rad_s[ATTITUDE_MEKF_VECTOR_DIM]);

/* specific_force is accelerometer output: a - g, so level FRD is approximately -Z. */
bool attitude_mekf_seed_from_accel(attitude_mekf_t *filter,
                                   const float specific_force_mps2[ATTITUDE_MEKF_VECTOR_DIM],
                                   float yaw_rad,
                                   const float gyro_bias_rad_s[ATTITUDE_MEKF_VECTOR_DIM]);

/*
 * delta_angle must include every sub-sample and its coning compensation. Fixed
 * and temperature-table calibration may already be applied, but do not subtract
 * this filter instance's current bg: the filter performs that bg*dt correction.
 */
bool attitude_mekf_propagate_delta(attitude_mekf_t *filter,
                                   const float delta_angle_rad[ATTITUDE_MEKF_VECTOR_DIM],
                                   float dt_s);

/*
 * Records a time interval whose angular displacement was not observed. The
 * nominal quaternion and bias stay unchanged while the attitude covariance is
 * enlarged isotropically by rotation_std_rad^2. This is intentionally separate
 * from propagate_delta(): integrating a clipped gyro value would create a
 * precise but false attitude.
 */
bool attitude_mekf_mark_rotation_unobserved(attitude_mekf_t *filter,
                                            float rotation_std_rad);

/*
 * Anisotropic variant: the unobserved rotation has axis_std_rad uncertainty
 * about unit_axis and perpendicular_std_rad about the two orthogonal axes.
 * Used when an accelerometer-pair witness bounds the rotation about the axes
 * perpendicular to the sensor baseline while rotation about the baseline
 * itself stays unbounded (DUAL_FUSION_DESIGN.md §3.3). Either std may be
 * zero (no evidence-free inflation on that subspace), but not both.
 */
bool attitude_mekf_mark_rotation_unobserved_directional(
    attitude_mekf_t *filter,
    float axis_std_rad,
    float perpendicular_std_rad,
    const float unit_axis[ATTITUDE_MEKF_VECTOR_DIM]);

/*
 * The update is rank-two in the gravity tangent plane and cannot observe yaw.
 * variance_scale must be finite and >= 1; use it for caller-detected
 * vibration/dynamics, and skip this call entirely during a hard accel gate.
 */
attitude_mekf_accel_result_t attitude_mekf_update_accel(
    attitude_mekf_t *filter,
    const float specific_force_mps2[ATTITUDE_MEKF_VECTOR_DIM],
    float variance_scale);

/*
 * Call only after a stationary detector has passed. gyro_rad_s is the
 * calibrated window-average measurement before this filter's bg subtraction;
 * measurement_covariance is its SPD covariance in (rad/s)^2. The z=bg+n update
 * observes all three bias axes, but it still does not observe absolute yaw.
 */
attitude_mekf_zaru_result_t attitude_mekf_update_zero_rate(
    attitude_mekf_t *filter,
    const float gyro_rad_s[ATTITUDE_MEKF_VECTOR_DIM],
    const float measurement_covariance[ATTITUDE_MEKF_VECTOR_DIM]
                                      [ATTITUDE_MEKF_VECTOR_DIM]);

/*
 * Bounded form for an internally inferred stationary interval.  The input is
 * already fixed/temperature calibrated. Its residual relative to the current
 * bias estimate is clipped before the z=bg+n update, so a previously learned
 * bias outside that residual band is not pulled back toward zero. If the
 * unconstrained Kalman update would move the 3-D bias vector by more than
 * max_bias_correction_rad_s, the complete gain is scaled uniformly and the
 * same gain is used by the Joseph covariance update. bounded_target_rad_s may
 * be NULL.
 */
attitude_mekf_zaru_result_t attitude_mekf_update_zero_rate_bounded(
    attitude_mekf_t *filter,
    const float gyro_rad_s[ATTITUDE_MEKF_VECTOR_DIM],
    const float measurement_covariance[ATTITUDE_MEKF_VECTOR_DIM]
                                      [ATTITUDE_MEKF_VECTOR_DIM],
    float target_residual_limit_rad_s,
    float max_bias_correction_rad_s,
    float bounded_target_rad_s[ATTITUDE_MEKF_VECTOR_DIM]);

bool attitude_mekf_is_valid(const attitude_mekf_t *filter);
bool attitude_mekf_get_quaternion(const attitude_mekf_t *filter, float quaternion[4]);
bool attitude_mekf_get_euler(const attitude_mekf_t *filter,
                             float euler_rad[ATTITUDE_MEKF_VECTOR_DIM]);
bool attitude_mekf_get_bias(const attitude_mekf_t *filter,
                            float gyro_bias_rad_s[ATTITUDE_MEKF_VECTOR_DIM]);

#ifdef __cplusplus
}
#endif

#endif
