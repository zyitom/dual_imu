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
    float accel_norm_hard_deviation_mps2;
    float accel_variance_scale_max;
    float accel_nis_gate;
    float zaru_nis_gate;
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
 * The update is rank-two in the gravity tangent plane and cannot observe yaw.
 * variance_scale must be finite and >= 1; use it for externally detected
 * vibration/dynamics, and skip this call entirely during a hard accel gate.
 */
attitude_mekf_accel_result_t attitude_mekf_update_accel(
    attitude_mekf_t *filter,
    const float specific_force_mps2[ATTITUDE_MEKF_VECTOR_DIM],
    float variance_scale);

/*
 * Call only after an external stationary detector has passed. gyro_rad_s is the
 * calibrated window-average measurement before this filter's bg subtraction;
 * measurement_covariance is its SPD covariance in (rad/s)^2. The z=bg+n update
 * observes all three bias axes, but it still does not observe absolute yaw.
 */
attitude_mekf_zaru_result_t attitude_mekf_update_zero_rate(
    attitude_mekf_t *filter,
    const float gyro_rad_s[ATTITUDE_MEKF_VECTOR_DIM],
    const float measurement_covariance[ATTITUDE_MEKF_VECTOR_DIM]
                                      [ATTITUDE_MEKF_VECTOR_DIM]);

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
