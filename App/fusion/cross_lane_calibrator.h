#ifndef CROSS_LANE_CALIBRATOR_H
#define CROSS_LANE_CALIBRATOR_H

#include "imu_preintegrator.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Online estimator for the deterministic differences between the two gyro
 * lanes (DUAL_FUSION_DESIGN.md §3.1). The information lives in the
 * difference channel: with the common preintegration window
 *
 *   z = delta_theta_bmi - delta_theta_icm
 *     = db*dt + [m]x*delta_theta + diag(s)*delta_theta - tau*delta_omega + n
 *
 * where db is the raw bias difference, m the small-angle misalignment of the
 * BMI gyro frame relative to the ICM gyro frame, s the diagonal scale-factor
 * difference and tau the residual relative group delay (tau > 0 means the
 * BMI stream lags the ICM stream). The three parameter groups are separated
 * by excitation, encoded directly in the regressor: at rest only the db
 * column is nonzero, steady rotation excites m and s, and rate change
 * excites tau, so no explicit regime gating is required for identifiability.
 *
 * The bias difference db is estimated only so that it cannot leak into m/s;
 * consumers must NOT apply it: each lane's MEKF tracks its own additive
 * bias, and subtracting db as well would remove the bias twice.
 */

#define CROSS_LANE_CALIBRATOR_STATE_DIM (10U)

typedef enum
{
    CROSS_LANE_CALIBRATOR_ACCEPTED = 0,
    CROSS_LANE_CALIBRATOR_REJECTED_INVALID_INPUT,
    CROSS_LANE_CALIBRATOR_REJECTED_RATE,
    CROSS_LANE_CALIBRATOR_REJECTED_NIS,
    CROSS_LANE_CALIBRATOR_FAULT_LIMIT,
    CROSS_LANE_CALIBRATOR_NUMERIC_FAILURE
} cross_lane_calibrator_result_t;

typedef struct
{
    /* Sum of the two lanes' gyro noise PSDs, (rad/s)^2/Hz, isotropic. */
    float gyro_noise_psd_sum_rad2_s;
    /* Additive per-axis rate noise floor on the differenced window. */
    float rate_noise_floor_rad_s;
    /* Residual differential gyro g-sensitivity; inflates the measurement
     * noise with the window's specific-force deviation from gravity so
     * high-g windows cannot bias db. */
    float g_sensitivity_rad_s_per_mps2;

    /* Random-walk PSDs for slow drift tracking (temperature). Variances grow
     * by psd*dt per admitted window. */
    float bias_rw_psd_rad2_s;
    float misalignment_rw_psd_rad2_s;
    float scale_rw_psd_s;
    float delay_rw_psd_s2_s;

    float initial_bias_std_rad_s;
    float initial_misalignment_std_rad;
    float initial_scale_std;
    float initial_delay_std_s;

    /* Admission: both lanes' window-mean rates must stay below this so the
     * BMI scale nonlinearity near full range cannot poison the linear model. */
    float max_rate_rad_s;
    float min_dt_s;
    float max_dt_s;

    /* Chi-square gate on the 3-DOF innovation. */
    float nis_gate;
    /* Consecutive NIS rejections that revoke convergence and reopen the
     * covariance so the calibrator can re-learn after a real change. */
    uint16_t nis_reject_revoke_windows;

    /* Convergence: 1-sigma parameter uncertainties that must all be met
     * before consumers may rely on the calibration. db has no criterion: it
     * is a nuisance parameter here. */
    float converged_misalignment_std_rad;
    float converged_scale_std;
    float converged_delay_std_s;
    uint32_t converged_min_accepts;

    /* Hard physical limits; a violating update is rejected and latches the
     * fault flag (a real unit cannot be this wrong -- something else is). */
    float max_abs_bias_rad_s;
    float max_abs_misalignment_rad;
    float max_abs_scale;
    float max_abs_delay_s;
} cross_lane_calibrator_config_t;

typedef struct
{
    uint32_t update_count;
    uint32_t accept_count;
    uint32_t invalid_reject_count;
    uint32_t rate_reject_count;
    uint32_t nis_reject_count;
    uint32_t limit_fault_count;
    uint32_t numeric_fault_count;
    uint32_t convergence_revoke_count;
    float last_nis;
    cross_lane_calibrator_result_t last_result;
} cross_lane_calibrator_diagnostics_t;

typedef struct
{
    /* [0..2] db rad/s, [3..5] m rad, [6..8] s unitless, [9] tau s. */
    float state[CROSS_LANE_CALIBRATOR_STATE_DIM];
    float covariance[CROSS_LANE_CALIBRATOR_STATE_DIM]
                    [CROSS_LANE_CALIBRATOR_STATE_DIM];
    cross_lane_calibrator_config_t config;
    cross_lane_calibrator_diagnostics_t diagnostics;
    uint16_t nis_reject_streak;
    bool converged;
    bool faulted;
    bool initialized;
} cross_lane_calibrator_t;

void cross_lane_calibrator_default_config(
    cross_lane_calibrator_config_t *config);

/* Passing NULL for config selects the default configuration. */
bool cross_lane_calibrator_init(cross_lane_calibrator_t *calibrator,
                                const cross_lane_calibrator_config_t *config);

/*
 * Feed one common-window pair. The caller is responsible for the estimator
 * level admission (no hard faults, no impact/inhibit interval, no pending
 * gyro-disagreement checkpoint); this function checks window validity, dt
 * bounds and the rate admission itself. accel means are used only for the
 * g-sensitivity noise inflation and both windows must carry valid accel.
 */
cross_lane_calibrator_result_t cross_lane_calibrator_update(
    cross_lane_calibrator_t *calibrator,
    const imu_preintegrated_window_t *bmi_window,
    const imu_preintegrated_window_t *icm_window,
    float dt_s);

/*
 * Maps a BMI window delta-angle into the ICM gyro frame using the current
 * misalignment/scale/delay estimates. The bias difference is intentionally
 * NOT applied (see header comment). rate_change_rad_s is the BMI window's
 * gyro_end - gyro_start. Returns false (output untouched) when the
 * calibration is not converged or the calibrator is faulted.
 */
bool cross_lane_calibrator_correct_bmi_delta_angle(
    const cross_lane_calibrator_t *calibrator,
    const float bmi_delta_angle_rad[3],
    const float bmi_rate_change_rad_s[3],
    float corrected_delta_angle_rad[3]);

bool cross_lane_calibrator_is_converged(
    const cross_lane_calibrator_t *calibrator);

/* 1-sigma uncertainties of the current estimates; any pointer may be NULL. */
bool cross_lane_calibrator_get_std(
    const cross_lane_calibrator_t *calibrator,
    float bias_std_rad_s[3],
    float misalignment_std_rad[3],
    float scale_std[3],
    float *delay_std_s);

#ifdef __cplusplus
}
#endif

#endif
