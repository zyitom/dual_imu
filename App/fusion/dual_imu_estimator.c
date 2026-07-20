#include "dual_imu_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DUAL_IMU_ESTIMATOR_GRAVITY_MPS2 (9.80665f)
#define DUAL_IMU_ESTIMATOR_PI_F          (3.14159265358979323846f)
#define DUAL_IMU_ESTIMATOR_US_TO_S       (1.0e-6f)
/* Ceiling on the accel measurement-variance inflation. Now that a high slew
 * rate down-weights gravity aiding instead of switching it off, this ceiling
 * is what bounds how far the weight can fall; clipping at 100 would have made
 * a 1800 deg/s sweep look only as noisy as a 900 deg/s one. The quadratic in
 * estimator_accel_variance_scale() stays unclipped out past 17000 deg/s. */
#define DUAL_IMU_ESTIMATOR_MAX_SCALE     (10000.0f)
#define DUAL_IMU_ESTIMATOR_FILTER_FAULT  (1UL << 31)
#define DUAL_IMU_ESTIMATOR_STATIONARY_WARMUP_WINDOWS (40U)

static void estimator_reset_stationary_tracking(
    dual_imu_estimator_t *estimator);
static void estimator_clear_stationary_confirmation(
    dual_imu_estimator_t *estimator);
static void estimator_enter_post_impact_reacquire(
    dual_imu_estimator_t *estimator);
static void estimator_note_attitude_rewrite(
    dual_imu_estimator_t *estimator,
    uint32_t lane,
    dual_imu_attitude_rewrite_reason_t reason);

static bool estimator_source_to_lane(imu_source_t source, uint32_t *lane)
{
    if ((lane == NULL) || (source >= IMU_SOURCE_COUNT))
        return false;

    *lane = (source == IMU_SOURCE_BMI088)
                ? (uint32_t)DUAL_IMU_ESTIMATOR_LANE_BMI088
                : (uint32_t)DUAL_IMU_ESTIMATOR_LANE_ICM45686;
    return true;
}

static bool estimator_vector_is_finite(const float vector[3])
{
    return (vector != NULL) && isfinite(vector[0]) && isfinite(vector[1]) &&
           isfinite(vector[2]);
}

static float estimator_vector_norm(const float vector[3])
{
    return sqrtf((vector[0] * vector[0]) + (vector[1] * vector[1]) +
                 (vector[2] * vector[2]));
}

static float estimator_vector_distance(const float lhs[3], const float rhs[3])
{
    const float difference[3] = {
        lhs[0] - rhs[0],
        lhs[1] - rhs[1],
        lhs[2] - rhs[2],
    };
    return estimator_vector_norm(difference);
}

static uint32_t estimator_sensor_fault_flags(
    const dual_imu_estimator_t *estimator,
    size_t lane)
{
    return estimator->hard_fault_flags[lane] &
           ~DUAL_IMU_ESTIMATOR_FILTER_FAULT;
}

static bool estimator_raw_lane_usable_for_geometry(
    const dual_imu_estimator_t *estimator,
    size_t lane)
{
    const uint32_t fault_flags = estimator->hard_fault_flags[lane];
    return (fault_flags == 0U) ||
           (((fault_flags & DUAL_IMU_ESTIMATOR_FILTER_FAULT) != 0U) &&
            (estimator_sensor_fault_flags(estimator, lane) == 0U));
}

static void estimator_quaternion_identity(float quaternion[4])
{
    quaternion[0] = 1.0f;
    quaternion[1] = 0.0f;
    quaternion[2] = 0.0f;
    quaternion[3] = 0.0f;
}

static bool estimator_quaternion_normalize(float quaternion[4])
{
    const float norm_sq = (quaternion[0] * quaternion[0]) +
                          (quaternion[1] * quaternion[1]) +
                          (quaternion[2] * quaternion[2]) +
                          (quaternion[3] * quaternion[3]);
    if (!isfinite(norm_sq) || (norm_sq < 1.0e-12f))
        return false;

    const float inverse_norm = 1.0f / sqrtf(norm_sq);
    for (size_t index = 0U; index < 4U; ++index)
        quaternion[index] *= inverse_norm;
    return true;
}

static void estimator_quaternion_multiply(const float lhs[4],
                                          const float rhs[4],
                                          float result[4])
{
    result[0] = (lhs[0] * rhs[0]) - (lhs[1] * rhs[1]) -
                (lhs[2] * rhs[2]) - (lhs[3] * rhs[3]);
    result[1] = (lhs[0] * rhs[1]) + (lhs[1] * rhs[0]) +
                (lhs[2] * rhs[3]) - (lhs[3] * rhs[2]);
    result[2] = (lhs[0] * rhs[2]) - (lhs[1] * rhs[3]) +
                (lhs[2] * rhs[0]) + (lhs[3] * rhs[1]);
    result[3] = (lhs[0] * rhs[3]) + (lhs[1] * rhs[2]) -
                (lhs[2] * rhs[1]) + (lhs[3] * rhs[0]);
}

static void estimator_quaternion_inverse(const float quaternion[4], float inverse[4])
{
    inverse[0] = quaternion[0];
    inverse[1] = -quaternion[1];
    inverse[2] = -quaternion[2];
    inverse[3] = -quaternion[3];
}

static bool estimator_rotation_vector_to_quaternion(const float rotation[3],
                                                     float quaternion[4])
{
    const float angle_sq = (rotation[0] * rotation[0]) +
                           (rotation[1] * rotation[1]) +
                           (rotation[2] * rotation[2]);
    if (!isfinite(angle_sq))
        return false;

    float scale;
    if (angle_sq < 1.0e-8f) {
        quaternion[0] = 1.0f - (0.125f * angle_sq);
        scale = 0.5f - (angle_sq / 48.0f);
    } else {
        const float angle = sqrtf(angle_sq);
        quaternion[0] = cosf(0.5f * angle);
        scale = sinf(0.5f * angle) / angle;
    }
    quaternion[1] = scale * rotation[0];
    quaternion[2] = scale * rotation[1];
    quaternion[3] = scale * rotation[2];
    return estimator_quaternion_normalize(quaternion);
}

static void estimator_quaternion_to_euler(const float quaternion[4], float euler[3])
{
    const float w = quaternion[0];
    const float x = quaternion[1];
    const float y = quaternion[2];
    const float z = quaternion[3];
    const float sin_pitch = 2.0f * ((w * y) - (z * x));

    euler[0] = atan2f(2.0f * ((w * x) + (y * z)),
                      1.0f - (2.0f * ((x * x) + (y * y))));
    euler[1] = asinf(fmaxf(-1.0f, fminf(1.0f, sin_pitch)));
    euler[2] = atan2f(2.0f * ((w * z) + (x * y)),
                      1.0f - (2.0f * ((y * y) + (z * z))));
}

static bool estimator_reseed_tilt_preserving_heading(
    attitude_mekf_t *filter,
    const float specific_force_mps2[3],
    const float reference_quaternion[4],
    const float bias_rad_s[3])
{
    if ((filter == NULL) || (specific_force_mps2 == NULL) ||
        (reference_quaternion == NULL) || (bias_rad_s == NULL)) {
        return false;
    }

    float reference[4] = {
        reference_quaternion[0], reference_quaternion[1],
        reference_quaternion[2], reference_quaternion[3],
    };
    if (!estimator_quaternion_normalize(reference))
        return false;

    attitude_mekf_t candidate = *filter;
    const attitude_mekf_diagnostics_t saved_diagnostics = filter->diagnostics;
    if (!attitude_mekf_seed_from_accel(&candidate, specific_force_mps2,
                                       0.0f, bias_rad_s)) {
        return false;
    }

    float tilt_quaternion[4];
    float inverse_tilt[4];
    float relative[4];
    float heading_twist[4];
    float reacquired[4];
    if (!attitude_mekf_get_quaternion(&candidate, tilt_quaternion))
        return false;
    estimator_quaternion_inverse(tilt_quaternion, inverse_tilt);
    estimator_quaternion_multiply(reference, inverse_tilt, relative);
    const float twist_norm = sqrtf((relative[0] * relative[0]) +
                                   (relative[3] * relative[3]));
    if (isfinite(twist_norm) && (twist_norm > 1.0e-7f)) {
        heading_twist[0] = relative[0] / twist_norm;
        heading_twist[1] = 0.0f;
        heading_twist[2] = 0.0f;
        heading_twist[3] = relative[3] / twist_norm;
        estimator_quaternion_multiply(heading_twist, tilt_quaternion,
                                      reacquired);
        if (!estimator_quaternion_normalize(reacquired) ||
            !attitude_mekf_reset(&candidate, reacquired, bias_rad_s)) {
            return false;
        }
    } else {
        /* Antipodal gravity makes the closest twist non-unique. Retain the
         * prior ZYX yaw as a deterministic gauge instead of failing forever. */
        float reference_euler[3];
        estimator_quaternion_to_euler(reference, reference_euler);
        if (!attitude_mekf_seed_from_accel(&candidate, specific_force_mps2,
                                           reference_euler[2], bias_rad_s)) {
            return false;
        }
    }
    candidate.diagnostics = saved_diagnostics;
    *filter = candidate;
    return true;
}

#if defined(DUAL_IMU_ESTIMATOR_TESTING) && DUAL_IMU_ESTIMATOR_TESTING
bool dual_imu_estimator_test_reseed_tilt_preserving_heading(
    attitude_mekf_t *filter,
    const float specific_force_mps2[3],
    const float reference_quaternion[4],
    const float bias_rad_s[3])
{
    return estimator_reseed_tilt_preserving_heading(
        filter, specific_force_mps2, reference_quaternion, bias_rad_s);
}
#endif

static bool estimator_get_recoverable_bias(const attitude_mekf_t *filter,
                                           float bias_rad_s[3])
{
    if ((filter == NULL) || (bias_rad_s == NULL))
        return false;
    if (attitude_mekf_get_bias(filter, bias_rad_s))
        return true;

    bool valid = true;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        const float stored_bias = filter->gyro_bias_rad_s[axis];
        if (!isfinite(stored_bias) ||
            (fabsf(stored_bias) > filter->config.max_abs_bias_rad_s)) {
            bias_rad_s[axis] = 0.0f;
            valid = false;
        } else {
            bias_rad_s[axis] = stored_bias;
        }
    }
    return valid;
}

static bool estimator_config_is_valid(const dual_imu_estimator_config_t *config)
{
    if ((config == NULL) || (config->preintegrator[0].window_us == 0U) ||
        ((config->preintegrator[0].window_us & 1U) != 0U) ||
        (config->preintegrator[1].window_us !=
         config->preintegrator[0].window_us) ||
        !isfinite(config->timestamp_jitter_std_s) ||
        (config->timestamp_jitter_std_s < 0.0f) ||
        !isfinite(config->pipeline_delay_std_s) ||
        (config->pipeline_delay_std_s < 0.0f) ||
        !isfinite(config->alignment_std_rad) ||
        (config->alignment_std_rad < 0.0f) ||
        !isfinite(config->gyro_disagreement_rate_fraction) ||
        (config->gyro_disagreement_rate_fraction < 0.0f) ||
        (config->gyro_disagreement_rate_fraction >= 0.2f) ||
        !isfinite(config->gyro_disagreement_rate_fraction_calibrated) ||
        (config->gyro_disagreement_rate_fraction_calibrated < 0.0f) ||
        (config->gyro_disagreement_rate_fraction_calibrated >
         config->gyro_disagreement_rate_fraction) ||
        !isfinite(config->alignment_std_calibrated_rad) ||
        (config->alignment_std_calibrated_rad < 0.0f) ||
        (config->alignment_std_calibrated_rad >
         config->alignment_std_rad) ||
        !isfinite(config->rate_floor_std_rad_s) ||
        (config->rate_floor_std_rad_s <= 0.0f) ||
        !isfinite(config->output_alignment_slew_rad_s) ||
        (config->output_alignment_slew_rad_s <= 0.0f) ||
        !isfinite(config->unobserved_rotation_rate_std_rad_s) ||
        (config->unobserved_rotation_rate_std_rad_s <= 0.0f) ||
        !isfinite(config->stationary_gyro_limit_rad_s) ||
        (config->stationary_gyro_limit_rad_s <= 0.0f) ||
        !isfinite(config->zaru_target_residual_limit_rad_s) ||
        (config->zaru_target_residual_limit_rad_s <= 0.0f) ||
        (config->zaru_target_residual_limit_rad_s >
         config->stationary_gyro_limit_rad_s) ||
        !isfinite(config->zaru_bias_slew_limit_rad_s2) ||
        (config->zaru_bias_slew_limit_rad_s2 <= 0.0f) ||
        !isfinite(config->zaru_calibration_tolerance_rad_s) ||
        (config->zaru_calibration_tolerance_rad_s <= 0.0f) ||
        (config->zaru_calibration_tolerance_rad_s >=
         config->zaru_target_residual_limit_rad_s) ||
        !isfinite(config->zaru_calibration_revoke_tolerance_rad_s) ||
        (config->zaru_calibration_revoke_tolerance_rad_s <=
         config->zaru_calibration_tolerance_rad_s) ||
        (config->zaru_calibration_revoke_tolerance_rad_s >=
         config->zaru_target_residual_limit_rad_s) ||
        !isfinite(config->zaru_recovery_rest_yaw_rate_tolerance_rad_s) ||
        (config->zaru_recovery_rest_yaw_rate_tolerance_rad_s <= 0.0f) ||
        (config->zaru_recovery_rest_yaw_rate_tolerance_rad_s >
         config->zaru_recovery_rest_tilt_rate_tolerance_rad_s) ||
        !isfinite(config->zaru_recovery_rest_tilt_rate_tolerance_rad_s) ||
        (config->zaru_recovery_rest_tilt_rate_tolerance_rad_s >=
         config->stationary_gyro_limit_rad_s) ||
        !isfinite(config->zaru_recovery_dual_lane_rate_tolerance_rad_s) ||
        (config->zaru_recovery_dual_lane_rate_tolerance_rad_s <= 0.0f) ||
        (config->zaru_recovery_dual_lane_rate_tolerance_rad_s >=
         config->stationary_gyro_limit_rad_s) ||
        (config->zaru_recovery_reject_windows < 2U) ||
        !isfinite(config->stationary_gyro_variance_limit_rad2_s2) ||
        (config->stationary_gyro_variance_limit_rad2_s2 <= 0.0f) ||
        !isfinite(config->stationary_accel_norm_tolerance_mps2) ||
        (config->stationary_accel_norm_tolerance_mps2 <= 0.0f) ||
        !isfinite(config->stationary_accel_pair_limit_mps2) ||
        (config->stationary_accel_pair_limit_mps2 <= 0.0f) ||
        !isfinite(config->stationary_accel_temporal_variance_limit_m2_s4) ||
        (config->stationary_accel_temporal_variance_limit_m2_s4 <= 0.0f) ||
        !isfinite(config->stationary_accel_direction_limit) ||
        (config->stationary_accel_direction_limit <= 0.0f) ||
        (config->stationary_accel_direction_limit >= 2.0f) ||
        (config->stationary_dwell_windows < 2U) ||
        (config->stationary_hint_dwell_windows < 2U) ||
        (config->stationary_hint_dwell_windows >
         config->stationary_dwell_windows) ||
        (config->stationary_single_lane_dwell_windows <
         config->stationary_dwell_windows) ||
        (config->stationary_single_lane_hint_dwell_windows <
         config->stationary_hint_dwell_windows) ||
        (config->stationary_single_lane_hint_dwell_windows >
         config->stationary_single_lane_dwell_windows) ||
        (config->stationary_soft_exit_windows == 0U) ||
        (config->stationary_rate_exit_windows == 0U) ||
        (config->attitude_convergence_windows == 0U) ||
        (config->attitude_aiding_timeout_windows <
         config->attitude_convergence_windows) ||
        (config->post_impact_reacquire_dwell_windows < 2U) ||
        (config->post_impact_reacquire_single_lane_dwell_windows <
         config->post_impact_reacquire_dwell_windows) ||
        (config->post_impact_gravity_trust_windows < 2U) ||
        (config->impact_gyro_disagreement_confirm_windows < 2U) ||
        (config->calibration_accept_windows == 0U) ||
        (config->calibration_revoke_windows == 0U) ||
        (config->accel_fault_enter_windows == 0U) ||
        (config->accel_fault_recovery_windows == 0U) ||
        (config->accel_recovery_stuck_windows < 2U) ||
        (config->accel_recovery_reseed_windows <=
         config->accel_recovery_stuck_windows) ||
        !isfinite(config->accel_recovery_inflation_std_rad) ||
        (config->accel_recovery_inflation_std_rad <= 0.0f) ||
        !isfinite(config->accel_recovery_norm_tolerance_mps2) ||
        (config->accel_recovery_norm_tolerance_mps2 <= 0.0f) ||
        !isfinite(config->accel_recovery_max_rate_rad_s) ||
        (config->accel_recovery_max_rate_rad_s <= 0.0f) ||
        (config->accel_recovery_max_rate_rad_s >
         config->accel_update_max_rate_rad_s) ||
        !isfinite(config->accel_update_max_rate_rad_s) ||
        (config->accel_update_max_rate_rad_s <= 0.0f) ||
        !isfinite(config->accel_update_max_angular_accel_rad_s2) ||
        (config->accel_update_max_angular_accel_rad_s2 <= 0.0f) ||
        !isfinite(config->accel_rate_variance_scale_rad_s) ||
        (config->accel_rate_variance_scale_rad_s <= 0.0f) ||
        !isfinite(config->accel_angular_accel_variance_scale_rad_s2) ||
        (config->accel_angular_accel_variance_scale_rad_s2 <= 0.0f) ||
        !isfinite(config->rotation_witness_leakage_fraction) ||
        (config->rotation_witness_leakage_fraction < 0.0f) ||
        (config->rotation_witness_leakage_fraction >= 1.0f) ||
        !isfinite(config->rotation_witness_noise_floor_mps2) ||
        (config->rotation_witness_noise_floor_mps2 < 0.0f) ||
        !isfinite(config->rotation_witness_safety_factor) ||
        (config->rotation_witness_safety_factor < 1.0f) ||
        !isfinite(config->rotation_witness_min_rate_std_rad_s) ||
        (config->rotation_witness_min_rate_std_rad_s <= 0.0f) ||
        (config->rotation_witness_min_rate_std_rad_s >
         config->unobserved_rotation_rate_std_rad_s) ||
        !isfinite(config->rotation_witness_trigger_residual_mps2) ||
        (config->rotation_witness_trigger_residual_mps2 <=
         config->stationary_accel_pair_limit_mps2) ||
        (config->rotation_witness_trigger_windows == 0U))
        return false;

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (!estimator_vector_is_finite(config->reference_to_sensor_m[lane]) ||
            !isfinite(config->zaru_rate_std_rad_s[lane]) ||
            (config->zaru_rate_std_rad_s[lane] <= 0.0f) ||
            !isfinite(config->stationary_gyro_temporal_variance_limit_rad2_s2[lane]) ||
            (config->stationary_gyro_temporal_variance_limit_rad2_s2[lane] <= 0.0f))
            return false;
    }
    return true;
}

void dual_imu_estimator_default_config(dual_imu_estimator_config_t *config)
{
    if (config == NULL)
        return;

    memset(config, 0, sizeof(*config));
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane)
        imu_preintegrator_default_config(&config->preintegrator[lane]);
    imu_selector_default_config(&config->selector);
    cross_lane_calibrator_default_config(&config->cross_lane_calibrator);
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane)
        attitude_mekf_default_config(&config->mekf[lane]);

    memcpy(config->reference_to_sensor_m[DUAL_IMU_ESTIMATOR_LANE_BMI088],
           imu_dm_fc01_reference_to_bmi_m,
           sizeof(imu_dm_fc01_reference_to_bmi_m));
    memcpy(config->reference_to_sensor_m[DUAL_IMU_ESTIMATOR_LANE_ICM45686],
           imu_dm_fc01_reference_to_icm_m,
           sizeof(imu_dm_fc01_reference_to_icm_m));

    const float degree = DUAL_IMU_ESTIMATOR_PI_F / 180.0f;
    const float bmi_noise_density = 0.014f * degree;
    const float icm_noise_density = 0.0038f * degree;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        config->mekf[DUAL_IMU_ESTIMATOR_LANE_BMI088]
            .gyro_noise_psd[axis][axis] = bmi_noise_density * bmi_noise_density;
        config->mekf[DUAL_IMU_ESTIMATOR_LANE_ICM45686]
            .gyro_noise_psd[axis][axis] = icm_noise_density * icm_noise_density;
    }

    config->timestamp_jitter_std_s = 20.0e-6f;
    config->pipeline_delay_std_s = 100.0e-6f;
    config->alignment_std_rad = 0.25f * degree;
    /* 1.6% = BMI088 gyro cross-axis 1.4% (datasheet) + scale-nonlinearity margin
     * (FIX_PLAN §9.1-3). Tighten toward ~0.3% after per-lane cross-axis
     * calibration is loaded. */
    config->gyro_disagreement_rate_fraction = 0.016f;
    /* Residual budgets once the cross-lane calibrator has absorbed the
     * deterministic misalignment/scale/delay difference: 0.3% of the
     * rotation angle plus a 0.1 deg residual alignment. */
    config->gyro_disagreement_rate_fraction_calibrated = 0.003f;
    config->alignment_std_calibrated_rad = 0.1f * degree;
    config->rate_floor_std_rad_s = 0.03f * degree;
    config->output_alignment_slew_rad_s = 1.0f;
    config->unobserved_rotation_rate_std_rad_s = 4000.0f * degree;
    config->zaru_rate_std_rad_s[DUAL_IMU_ESTIMATOR_LANE_BMI088] = 0.30f * degree;
    config->zaru_rate_std_rad_s[DUAL_IMU_ESTIMATOR_LANE_ICM45686] = 0.10f * degree;
    config->stationary_gyro_limit_rad_s = 3.0f * degree;
    config->zaru_target_residual_limit_rad_s = 0.50f * degree;
    config->zaru_bias_slew_limit_rad_s2 = 0.10f * degree;
    config->zaru_calibration_tolerance_rad_s = 0.05f * degree;
    config->zaru_calibration_revoke_tolerance_rad_s = 0.20f * degree;
    /* Yaw (gravity-parallel) rest tolerance stays at the ZARU residual limit:
     * a steady sub-limit pan about gravity is indistinguishable from yaw bias,
     * so it must stay below the 1 dps turntable guarantee
     * (test_unobservable_static_yaw_rate_is_not_learned_as_bias). The tilt
     * (gravity-perpendicular) tolerance is sized for what a healthy-but-shifted
     * gyro really reads at rest -- fixed-calibration ZRO residual (~0.2 dps
     * measured) plus post-shock ZRO shift (FIX_PLAN §5-①, the very failure §12
     * recovers from) -- and is safe to widen because a real 2 dps tilt rotation
     * moves gravity ~10° over the 5 s dwell and trips the gravity-stability
     * gate. The dual-lane budget matches: a unilateral post-shock shift is a
     * genuine bias the reseed should absorb, so the cross-check only blocks a
     * grossly lying lane, not spec-level offset spread. 2000 windows = 5 s at
     * the 2.5 ms window rate (FIX_PLAN §12.3). */
    config->zaru_recovery_rest_yaw_rate_tolerance_rad_s = 0.50f * degree;
    config->zaru_recovery_rest_tilt_rate_tolerance_rad_s = 2.0f * degree;
    config->zaru_recovery_dual_lane_rate_tolerance_rad_s = 2.0f * degree;
    config->zaru_recovery_reject_windows = 2000U;
    const float stationary_gyro_rms_limit = 1.0f * degree;
    config->stationary_gyro_variance_limit_rad2_s2 =
        stationary_gyro_rms_limit * stationary_gyro_rms_limit;
    const float bmi_temporal_gyro_rms_limit = 0.80f * degree;
    const float icm_temporal_gyro_rms_limit = 0.25f * degree;
    config->stationary_gyro_temporal_variance_limit_rad2_s2
        [DUAL_IMU_ESTIMATOR_LANE_BMI088] =
        bmi_temporal_gyro_rms_limit * bmi_temporal_gyro_rms_limit;
    config->stationary_gyro_temporal_variance_limit_rad2_s2
        [DUAL_IMU_ESTIMATOR_LANE_ICM45686] =
        icm_temporal_gyro_rms_limit * icm_temporal_gyro_rms_limit;
    config->stationary_accel_norm_tolerance_mps2 = 0.30f;
    config->stationary_accel_pair_limit_mps2 = 0.35f;
    const float stationary_temporal_accel_rms_limit = 0.10f;
    config->stationary_accel_temporal_variance_limit_m2_s4 =
        stationary_temporal_accel_rms_limit *
        stationary_temporal_accel_rms_limit;
    config->stationary_accel_direction_limit = 0.02f;
    /* Windows are 2500 us, so these are 300 ms / 600 ms of continuous quiet.
     * The previous 800/1600 meant 2 s / 4 s, which no hand-held rest between
     * motions ever reached: across three captures the longest qualifying
     * still segment was 2116 ms, and the streak resets on any single failing
     * window, so stationary was confirmed exactly zero times. That starved
     * ZUPT/ZARU and left the gyro bias unobserved. Only the dwell shortens
     * here; every instantaneous admission test (rate, accel norm, window
     * variance, cross-lane accel agreement) is unchanged. */
    config->stationary_dwell_windows = 120U;
    config->stationary_hint_dwell_windows = 80U;
    config->stationary_single_lane_dwell_windows = 240U;
    config->stationary_single_lane_hint_dwell_windows = 160U;
    config->stationary_soft_exit_windows = 4U;
    config->stationary_rate_exit_windows = 3U;
    config->attitude_convergence_windows = 40U;
    config->attitude_aiding_timeout_windows = 800U;
    config->post_impact_reacquire_dwell_windows = 200U;
    config->post_impact_reacquire_single_lane_dwell_windows = 400U;
    /* 200 ms at 400 Hz. This is shorter than the stationary reseed dwell but
     * long enough that a fast-yaw inhibit cannot reopen gravity aiding on the
     * first still-contaminated window after its 100 ms hold expires. */
    config->post_impact_gravity_trust_windows = 8U;
    config->impact_gyro_disagreement_confirm_windows = 3U;
    config->calibration_accept_windows = 40U;
    config->calibration_revoke_windows = 40U;
    config->accel_fault_enter_windows = 200U;
    config->accel_fault_recovery_windows = 400U;
    config->accel_recovery_stuck_windows = 100U;
    config->accel_recovery_reseed_windows = 400U;
    config->accel_recovery_inflation_std_rad = 0.6f * degree;
    /* ~5% g: wide enough for post-shock rest with residual vibration, far
     * below the >1 m/s2 centripetal contamination of a hand-rate rotation. */
    config->accel_recovery_reseed_enabled = false;
    config->accel_recovery_norm_tolerance_mps2 = 0.5f;
    /* ~57 dps: below this the centripetal term at a 0.2 m lever is
     * <= 0.25 m/s2 (<= 1.5 deg direction error), so a rejected update is
     * attitude error, not dynamics. Catches the perpendicular-contamination
     * band the norm tolerance cannot see. */
    config->accel_recovery_max_rate_rad_s = 1.0f;
    config->accel_update_max_rate_rad_s = 15.0f;
    config->accel_update_max_angular_accel_rad_s2 = 1000.0f;
    config->accel_rate_variance_scale_rad_s = 3.0f;
    config->accel_angular_accel_variance_scale_rad_s2 = 200.0f;
    config->accel_inhibit_variance_scale = 400.0f;
    /* 0.5% of the common-mode magnitude covers a ~0.3 deg accel-pair
     * misalignment leaking translation into the difference channel. */
    config->rotation_witness_leakage_fraction = 0.005f;
    config->rotation_witness_noise_floor_mps2 = 0.05f;
    config->rotation_witness_safety_factor = 2.0f;
    /* ~115 dps floor: rotation modes the 7.1 mm baseline cannot resolve
     * (intermittent within the window, partial cancellation) must still
     * inflate a meaningful covariance. */
    config->rotation_witness_min_rate_std_rad_s = 2.0f;
    /* Far above both the stationary pair limit (0.35) and worst-case
     * misalignment leakage at moderate g, so only genuinely unmodeled
     * rotation/vibration can arm the trigger. */
    config->rotation_witness_trigger_residual_mps2 = 3.0f;
    config->rotation_witness_trigger_windows = 2U;
}

bool dual_imu_estimator_init(dual_imu_estimator_t *estimator,
                             const dual_imu_estimator_config_t *config,
                             uint64_t common_epoch_us)
{
    if ((estimator == NULL) || !estimator_config_is_valid(config))
        return false;

    memset(estimator, 0, sizeof(*estimator));
    estimator->config = *config;
    const imu_source_t sources[DUAL_IMU_ESTIMATOR_LANE_COUNT] = {
        IMU_SOURCE_BMI088,
        IMU_SOURCE_ICM45686,
    };
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (!imu_preintegrator_init(&estimator->preintegrator[lane],
                                    &config->preintegrator[lane],
                                    sources[lane],
                                    common_epoch_us) ||
            !attitude_mekf_init(&estimator->mekf[lane], &config->mekf[lane]))
            return false;
    }

    if (!imu_selector_init(&estimator->selector,
                           &config->selector,
                           IMU_SELECTOR_LANE_1))
        return false;

    if (!cross_lane_calibrator_init(&estimator->cross_lane_calibrator,
                                    &config->cross_lane_calibrator))
        return false;

    imu_angular_accel_estimator_init(&estimator->angular_accel_estimator,
                                     20.0f,
                                     5.0f *
                                         config->accel_update_max_angular_accel_rad_s2);
    estimator_quaternion_identity(estimator->output_quaternion);
    estimator_quaternion_identity(estimator->output_alignment);
    estimator->previous_selected_lane = IMU_SELECTOR_LANE_NONE;
    estimator->initialized = true;
    return true;
}

bool dual_imu_estimator_push_accel(dual_imu_estimator_t *estimator,
                                   const imu_accel_sample_t *sample)
{
    uint32_t lane;
    return (estimator != NULL) && estimator->initialized &&
           (sample != NULL) && estimator_source_to_lane(sample->source, &lane) &&
           imu_preintegrator_push_accel(&estimator->preintegrator[lane], sample);
}

bool dual_imu_estimator_push_gyro(dual_imu_estimator_t *estimator,
                                  const imu_gyro_sample_t *sample)
{
    uint32_t lane;
    return (estimator != NULL) && estimator->initialized &&
           (sample != NULL) && estimator_source_to_lane(sample->source, &lane) &&
           imu_preintegrator_push_gyro(&estimator->preintegrator[lane], sample);
}

void dual_imu_estimator_set_hard_faults(dual_imu_estimator_t *estimator,
                                        imu_source_t source,
                                        uint32_t hard_fault_flags)
{
    uint32_t lane;
    if ((estimator == NULL) || !estimator->initialized ||
        !estimator_source_to_lane(source, &lane)) {
        return;
    }

    const uint32_t previous_sensor_faults =
        estimator_sensor_fault_flags(estimator, lane);
    const uint32_t sensor_faults =
        hard_fault_flags & ~DUAL_IMU_ESTIMATOR_FILTER_FAULT;
    estimator->hard_fault_flags[lane] =
        (estimator->hard_fault_flags[lane] &
         DUAL_IMU_ESTIMATOR_FILTER_FAULT) |
        sensor_faults;
    if (sensor_faults != previous_sensor_faults) {
        estimator_reset_stationary_tracking(estimator);
        estimator_clear_stationary_confirmation(estimator);
    }
}

static void estimator_update_accel_health(
    dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    bool accel_updates_enabled,
    const bool evidence_valid[DUAL_IMU_ESTIMATOR_LANE_COUNT])
{
    if (!accel_updates_enabled) {
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            estimator->accel_bad_streak[lane] = 0U;
            estimator->accel_good_streak[lane] = 0U;
        }
        return;
    }

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        const size_t other = 1U - lane;
        const bool lane_evidence_valid = evidence_valid[lane];
        const bool other_evidence_valid = evidence_valid[other];
        const bool lane_accepted =
            output->accel_result[lane] == ATTITUDE_MEKF_ACCEL_ACCEPTED;
        const bool other_accepted =
            output->accel_result[other] == ATTITUDE_MEKF_ACCEL_ACCEPTED;

        if (!lane_evidence_valid) {
            estimator->accel_bad_streak[lane] = 0U;
            estimator->accel_good_streak[lane] = 0U;
        } else if (!lane_accepted && other_evidence_valid && other_accepted) {
            estimator->accel_good_streak[lane] = 0U;
            if (estimator->accel_bad_streak[lane] < UINT16_MAX)
                estimator->accel_bad_streak[lane]++;
            if (estimator->accel_bad_streak[lane] >=
                estimator->config.accel_fault_enter_windows) {
                estimator->lane_accel_fault[lane] = true;
            }
        } else if (lane_accepted) {
            estimator->accel_bad_streak[lane] = 0U;
            if (estimator->lane_accel_fault[lane]) {
                if (estimator->accel_good_streak[lane] < UINT16_MAX)
                    estimator->accel_good_streak[lane]++;
                if (estimator->accel_good_streak[lane] >=
                    estimator->config.accel_fault_recovery_windows) {
                    estimator->lane_accel_fault[lane] = false;
                    estimator->accel_good_streak[lane] = 0U;
                }
            }
        } else {
            estimator->accel_bad_streak[lane] = 0U;
            estimator->accel_good_streak[lane] = 0U;
        }
    }
}

void dual_imu_estimator_set_stationary_hint(dual_imu_estimator_t *estimator,
                                            bool stationary)
{
    if ((estimator != NULL) && estimator->initialized)
        estimator->stationary_hint = stationary;
}

void dual_imu_estimator_set_isolation_hint(dual_imu_estimator_t *estimator,
                                           imu_selector_hint_t hint)
{
    if ((estimator != NULL) && estimator->initialized)
        estimator->isolation_hint = hint;
}

static void estimator_sort_accel_inhibit_intervals(
    dual_imu_estimator_accel_inhibit_interval_t *intervals,
    size_t count)
{
    for (size_t index = 1U; index < count; ++index) {
        const dual_imu_estimator_accel_inhibit_interval_t current =
            intervals[index];
        size_t insertion = index;
        while ((insertion > 0U) &&
               ((intervals[insertion - 1U].start_us > current.start_us) ||
                ((intervals[insertion - 1U].start_us == current.start_us) &&
                 (intervals[insertion - 1U].end_us > current.end_us)))) {
            intervals[insertion] = intervals[insertion - 1U];
            insertion--;
        }
        intervals[insertion] = current;
    }
}

static bool estimator_add_interval(
    dual_imu_estimator_accel_inhibit_interval_t
        intervals[DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT],
    uint8_t *interval_count,
    uint64_t start_us,
    uint64_t end_us)
{
    if ((intervals == NULL) || (interval_count == NULL) ||
        (*interval_count > DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT) ||
        (start_us >= end_us)) {
        return false;
    }

    dual_imu_estimator_accel_inhibit_interval_t pending
        [DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT + 1U];
    size_t pending_count = *interval_count;
    memcpy(pending, intervals,
           pending_count * sizeof(pending[0]));
    pending[pending_count++] =
        (dual_imu_estimator_accel_inhibit_interval_t){start_us, end_us};
    estimator_sort_accel_inhibit_intervals(pending, pending_count);

    size_t normalized_count = 0U;
    for (size_t index = 0U; index < pending_count; ++index) {
        if ((normalized_count > 0U) &&
            (pending[index].start_us <=
             pending[normalized_count - 1U].end_us)) {
            if (pending[index].end_us >
                pending[normalized_count - 1U].end_us) {
                pending[normalized_count - 1U].end_us = pending[index].end_us;
            }
        } else {
            pending[normalized_count++] = pending[index];
        }
    }

    /* If a pathological backlog contains more disjoint impacts than the fixed
     * history can hold, bridge the smallest time gap. This remains causal and
     * fails conservatively by disabling gravity aiding only between events. */
    while (normalized_count >
           DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT) {
        size_t merge_index = 0U;
        uint64_t smallest_gap = UINT64_MAX;
        for (size_t index = 0U; index + 1U < normalized_count; ++index) {
            const uint64_t gap = pending[index + 1U].start_us -
                                 pending[index].end_us;
            if (gap < smallest_gap) {
                smallest_gap = gap;
                merge_index = index;
            }
        }
        pending[merge_index].end_us = pending[merge_index + 1U].end_us;
        for (size_t index = merge_index + 1U;
             index + 1U < normalized_count; ++index) {
            pending[index] = pending[index + 1U];
        }
        normalized_count--;
    }

    memset(intervals, 0,
           sizeof(*intervals) *
               DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT);
    memcpy(intervals, pending,
           normalized_count * sizeof(pending[0]));
    *interval_count = (uint8_t)normalized_count;
    return true;
}

bool dual_imu_estimator_inhibit_accel_interval(
    dual_imu_estimator_t *estimator,
    uint64_t start_us,
    uint64_t end_us)
{
    return (estimator != NULL) && estimator->initialized &&
           estimator_add_interval(estimator->accel_inhibit_intervals,
                                  &estimator->accel_inhibit_interval_count,
                                  start_us, end_us);
}

bool dual_imu_estimator_notify_impact_interval(
    dual_imu_estimator_t *estimator,
    uint64_t start_us,
    uint64_t end_us)
{
    if ((estimator == NULL) || !estimator->initialized ||
        (start_us >= end_us)) {
        return false;
    }

    const bool accel_added = estimator_add_interval(
        estimator->accel_inhibit_intervals,
        &estimator->accel_inhibit_interval_count,
        start_us, end_us);
    const bool impact_added = estimator_add_interval(
        estimator->impact_intervals,
        &estimator->impact_interval_count,
        start_us, end_us);
    return accel_added && impact_added;
}

static bool estimator_interval_active_for_window(
    dual_imu_estimator_accel_inhibit_interval_t
        intervals[DUAL_IMU_ESTIMATOR_ACCEL_INHIBIT_INTERVAL_COUNT],
    uint8_t *interval_count,
    uint64_t start_us,
    uint64_t end_us,
    uint64_t *first_interval_start_us)
{
    bool active = false;
    size_t retained_count = 0U;
    const size_t count = *interval_count;
    for (size_t index = 0U; index < count; ++index) {
        const dual_imu_estimator_accel_inhibit_interval_t interval =
            intervals[index];
        if (interval.end_us <= start_us)
            continue;

        if ((interval.start_us < end_us) && (start_us < interval.end_us)) {
            if (!active && (first_interval_start_us != NULL))
                *first_interval_start_us = interval.start_us;
            active = true;
        }
        if (interval.end_us > end_us)
            intervals[retained_count++] = interval;
    }
    for (size_t index = retained_count; index < count; ++index) {
        intervals[index] =
            (dual_imu_estimator_accel_inhibit_interval_t){0U, 0U};
    }
    *interval_count = (uint8_t)retained_count;
    return active;
}

static void estimator_window_rate(const imu_preintegrated_window_t *window,
                                  float rate[3])
{
    for (size_t axis = 0U; axis < 3U; ++axis)
        rate[axis] = window->gyro_mean_rad_s[axis];
}

/*
 * DUAL_FUSION_DESIGN.md §3.3: projecting the difference of the two
 * accelerometers' specific force onto the sensor-baseline direction isolates
 * the mean centripetal term:
 *
 *   (f_bmi - f_icm) . unit(r_bmi - r_icm) = -|r_bmi - r_icm| * mean(|w_perp|^2)
 *
 * because the tangential wdot x dr term is exactly perpendicular to dr. The
 * result is a measured one-sigma bound on the window's rotation rate about
 * the axes perpendicular to the baseline; rotation about the baseline is
 * invisible. Works on raw sensor-position means (gyro-blind windows) and on
 * lever-arm-translated residuals (hidden-rotation trigger) alike: in the
 * latter case the gyro-predicted part is already subtracted, so the
 * projection measures the rotation the gyros missed.
 */
static bool estimator_baseline_rotation_witness(
    const dual_imu_estimator_t *estimator,
    const float bmi_specific_force_mps2[3],
    const float icm_specific_force_mps2[3],
    float *perpendicular_rate_std_rad_s,
    float baseline_axis[3])
{
    const dual_imu_estimator_config_t *config = &estimator->config;
    float baseline[3];
    float difference[3];
    for (size_t axis = 0U; axis < 3U; ++axis) {
        baseline[axis] =
            config->reference_to_sensor_m[DUAL_IMU_ESTIMATOR_LANE_BMI088]
                                         [axis] -
            config->reference_to_sensor_m[DUAL_IMU_ESTIMATOR_LANE_ICM45686]
                                         [axis];
        difference[axis] = bmi_specific_force_mps2[axis] -
                           icm_specific_force_mps2[axis];
    }
    if (!estimator_vector_is_finite(baseline) ||
        !estimator_vector_is_finite(difference) ||
        !estimator_vector_is_finite(bmi_specific_force_mps2) ||
        !estimator_vector_is_finite(icm_specific_force_mps2))
        return false;
    const float distance = estimator_vector_norm(baseline);
    if (!isfinite(distance) || (distance < 1.0e-4f))
        return false;

    float projection = 0.0f;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        baseline_axis[axis] = baseline[axis] / distance;
        projection += difference[axis] * baseline_axis[axis];
    }
    const float common_mode = fmaxf(
        estimator_vector_norm(bmi_specific_force_mps2),
        estimator_vector_norm(icm_specific_force_mps2));
    const float leakage_margin =
        (config->rotation_witness_leakage_fraction * common_mode) +
        config->rotation_witness_noise_floor_mps2;
    const float centripetal = fmaxf(0.0f, -projection - leakage_margin);
    const float witnessed_rate = sqrtf(centripetal / distance);
    const float rate_std = fminf(
        config->unobserved_rotation_rate_std_rad_s,
        (config->rotation_witness_safety_factor * witnessed_rate) +
            config->rotation_witness_min_rate_std_rad_s);
    if (!isfinite(rate_std) || (rate_std <= 0.0f))
        return false;
    *perpendicular_rate_std_rad_s = rate_std;
    return true;
}

static bool estimator_propagate_half_window(
    dual_imu_estimator_t *estimator,
    uint32_t lane,
    const imu_preintegrated_window_t windows[DUAL_IMU_ESTIMATOR_LANE_COUNT],
    uint32_t half_index,
    float half_dt_s,
    bool witness_valid,
    float witness_rate_std_rad_s,
    const float witness_axis[3],
    bool *aided,
    bool *rotation_unobserved)
{
    if (!estimator->lane_seeded[lane] ||
        !attitude_mekf_is_valid(&estimator->mekf[lane]) ||
        (half_index > 1U))
        return false;

    float delta_angle[3];
    const bool own_gyro_usable = windows[lane].gyro_propagation_valid &&
                                 (estimator->hard_fault_flags[lane] == 0U);
    if (own_gyro_usable) {
        const float *const half_delta =
            (half_index == 0U)
                ? windows[lane].first_half_delta_angle_rad
                : windows[lane].second_half_delta_angle_rad;
        for (size_t axis = 0U; axis < 3U; ++axis)
            delta_angle[axis] = half_delta[axis];
        *aided = false;
    } else {
        const uint32_t other = 1U - lane;
        const bool other_gyro_usable = windows[other].gyro_propagation_valid &&
                                       (estimator->hard_fault_flags[other] == 0U);
        if (!other_gyro_usable) {
            *aided = false;
            if (rotation_unobserved != NULL)
                *rotation_unobserved = true;
            if (witness_valid) {
                /* The accel-pair witness bounds the rotation about the axes
                 * perpendicular to the baseline; the baseline axis itself
                 * keeps the blind full-range uncertainty. */
                return attitude_mekf_mark_rotation_unobserved_directional(
                    &estimator->mekf[lane],
                    estimator->config.unobserved_rotation_rate_std_rad_s *
                        half_dt_s,
                    witness_rate_std_rad_s * half_dt_s,
                    witness_axis);
            }
            return attitude_mekf_mark_rotation_unobserved(
                &estimator->mekf[lane],
                estimator->config.unobserved_rotation_rate_std_rad_s *
                    half_dt_s);
        }

        float lane_bias[3];
        float other_bias[3];
        if (!attitude_mekf_get_bias(&estimator->mekf[lane], lane_bias) ||
            !attitude_mekf_get_bias(&estimator->mekf[other], other_bias))
            return false;
        const float *const other_half_delta =
            (half_index == 0U)
                ? windows[other].first_half_delta_angle_rad
                : windows[other].second_half_delta_angle_rad;
        for (size_t axis = 0U; axis < 3U; ++axis) {
            const float other_corrected_half =
                other_half_delta[axis] -
                (other_bias[axis] * half_dt_s);
            delta_angle[axis] = other_corrected_half +
                                (lane_bias[axis] * half_dt_s);
        }
        *aided = true;
    }

    const bool degraded = !windows[lane].gyro_valid;
    float saved_gyro_psd[3][3];
    if (*aided || degraded) {
        memcpy(saved_gyro_psd,
               estimator->mekf[lane].config.gyro_noise_psd,
               sizeof(saved_gyro_psd));
        const uint32_t other = 1U - lane;
        for (size_t row = 0U; row < 3U; ++row) {
            for (size_t column = 0U; column < 3U; ++column) {
                float psd = saved_gyro_psd[row][column];
                if (*aided)
                    psd += estimator->mekf[other]
                               .config.gyro_noise_psd[row][column];
                estimator->mekf[lane].config.gyro_noise_psd[row][column] =
                    4.0f * psd;
            }
        }
    }

    const bool propagated = attitude_mekf_propagate_delta(
        &estimator->mekf[lane], delta_angle, half_dt_s);
    if (*aided || degraded) {
        memcpy(estimator->mekf[lane].config.gyro_noise_psd,
               saved_gyro_psd,
               sizeof(saved_gyro_psd));
    }
    return propagated;
}

static bool estimator_bias_correct_second_moment(
    const imu_preintegrated_window_t *window,
    const float bias_rad_s[3],
    float corrected_second_moment[3][3])
{
    if ((window == NULL) || (bias_rad_s == NULL) ||
        (corrected_second_moment == NULL))
        return false;

    for (size_t row = 0U; row < 3U; ++row) {
        if (!isfinite(window->gyro_mean_rad_s[row]) ||
            !isfinite(bias_rad_s[row]))
            return false;
        for (size_t column = 0U; column < 3U; ++column) {
            const float corrected =
                window->gyro_second_moment_rad2_s2[row][column] -
                (window->gyro_mean_rad_s[row] * bias_rad_s[column]) -
                (bias_rad_s[row] * window->gyro_mean_rad_s[column]) +
                (bias_rad_s[row] * bias_rad_s[column]);
            if (!isfinite(corrected))
                return false;
            corrected_second_moment[row][column] = corrected;
        }
    }
    return true;
}

static void estimator_selector_covariance(const dual_imu_estimator_t *estimator,
                                          uint32_t lane,
                                          const float angular_accel[3],
                                          const float delta_angle[3],
                                          float dt_s,
                                          float alignment_std_rad,
                                          float disagreement_fraction,
                                          float covariance[3][3])
{
    const float dt_sq = dt_s * dt_s;
    const float delta_norm = estimator_vector_norm(delta_angle);
    const float delta_norm_sq = delta_norm * delta_norm;
    const float alignment_variance_scale =
        alignment_std_rad * alignment_std_rad;
    const float floor_delta = estimator->config.rate_floor_std_rad_s * dt_s;
    const float floor_variance = floor_delta * floor_delta;
    /* Rate-proportional cross-lane disagreement budget: gyro cross-axis
     * sensitivity and scale nonlinearity produce an apparent disagreement that
     * grows with the rotation angle |omega|*dt = delta_norm (FIX_PLAN §9.1-3,
     * §12.1). Add (k*delta_norm)^2 isotropically so a fast slew widens the NIS
     * gate instead of reading as a lane fault. delta_norm is this lane's own
     * rotation angle; both lanes measure the same motion and their covariances
     * are summed in the selector NIS, so each contributing its own term yields a
     * correctly scaled combined gate. The term is exactly zero at delta_norm=0,
     * keeping the static gate bit-identical to before. The caller passes the
     * tightened calibrated budgets while the cross-lane calibrator is
     * converged (DUAL_FUSION_DESIGN.md §3.1). */
    const float disagreement_variance =
        (disagreement_fraction * disagreement_fraction) * delta_norm_sq;

    for (size_t row = 0U; row < 3U; ++row) {
        for (size_t column = 0U; column < 3U; ++column) {
            covariance[row][column] =
                (estimator->mekf[lane].config.gyro_noise_psd[row][column] * dt_s) +
                (estimator->mekf[lane].covariance[row + 3U][column + 3U] * dt_sq);
            const float alignment_jacobian_covariance =
                ((row == column) ? delta_norm_sq : 0.0f) -
                (delta_angle[row] * delta_angle[column]);
            covariance[row][column] += alignment_variance_scale *
                                       alignment_jacobian_covariance;
        }
        /* A common-window phase error changes delta-angle by alpha*T*dt. */
        const float jitter_delta = angular_accel[row] * dt_s *
                                   estimator->config.timestamp_jitter_std_s;
        const float delay_delta = angular_accel[row] * dt_s *
                                  estimator->config.pipeline_delay_std_s;
        covariance[row][row] += (jitter_delta * jitter_delta) +
                                (delay_delta * delay_delta) +
                                floor_variance + disagreement_variance;
    }
}

static void estimator_reset_stationary_tracking(
    dual_imu_estimator_t *estimator)
{
    memset(estimator->stationary_gravity_reference_valid, 0,
           sizeof(estimator->stationary_gravity_reference_valid));
    estimator->stationary_statistics_count = 0U;
    memset(estimator->stationary_gyro_mean_rad_s, 0,
           sizeof(estimator->stationary_gyro_mean_rad_s));
    memset(estimator->stationary_gyro_m2_rad2_s2, 0,
           sizeof(estimator->stationary_gyro_m2_rad2_s2));
    memset(estimator->stationary_accel_mean_mps2, 0,
           sizeof(estimator->stationary_accel_mean_mps2));
    memset(estimator->stationary_accel_m2_m2_s4, 0,
           sizeof(estimator->stationary_accel_m2_m2_s4));
    memset(estimator->stationary_temporal_gyro_variance_rad2_s2, 0,
           sizeof(estimator->stationary_temporal_gyro_variance_rad2_s2));
    memset(estimator->stationary_temporal_accel_variance_m2_s4, 0,
           sizeof(estimator->stationary_temporal_accel_variance_m2_s4));
    estimator->stationary_lane_mask = 0U;
    estimator->stationary_streak = 0U;
}

static void estimator_clear_stationary_confirmation(
    dual_imu_estimator_t *estimator)
{
    estimator->stationary_confirmed_latched = false;
    estimator->stationary_confirmed_lane_mask = 0U;
    estimator->stationary_soft_exit_streak = 0U;
    estimator->stationary_rate_exit_streak = 0U;
}

static void estimator_latch_filter_fault(dual_imu_estimator_t *estimator,
                                         size_t lane,
                                         uint64_t window_end_us)
{
    const bool newly_latched =
        (estimator->hard_fault_flags[lane] &
         DUAL_IMU_ESTIMATOR_FILTER_FAULT) == 0U;
    estimator->hard_fault_flags[lane] |= DUAL_IMU_ESTIMATOR_FILTER_FAULT;
    estimator->lane_seeded[lane] = false;
    if (newly_latched ||
        (window_end_us > estimator->filter_fault_window_end_us[lane])) {
        estimator->filter_fault_window_end_us[lane] = window_end_us;
        estimator_reset_stationary_tracking(estimator);
        estimator_clear_stationary_confirmation(estimator);
    }
}

static bool estimator_filter_recovery_window_is_new(
    const dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    size_t lane)
{
    return output->start_us >= estimator->filter_fault_window_end_us[lane];
}

static void estimator_enter_post_impact_reacquire(
    dual_imu_estimator_t *estimator)
{
    if (!estimator->post_impact_reacquire_active &&
        (estimator->post_impact_episode_count < UINT32_MAX)) {
        estimator->post_impact_episode_count++;
    }
    estimator->post_impact_reacquire_active = true;
    estimator->post_impact_gravity_trusted = false;
    estimator->post_impact_gravity_trust_streak = 0U;
    estimator->attitude_converged = false;
    estimator->attitude_convergence_streak = 0U;
    estimator->attitude_aiding_miss_streak = 0U;
    estimator->stationary_streak = 0U;
    estimator_reset_stationary_tracking(estimator);
    estimator_clear_stationary_confirmation(estimator);
}

static void estimator_complete_post_impact_reacquire(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output)
{
    if (!estimator->post_impact_reacquire_active)
        return;

    estimator->post_impact_reacquire_active = false;
    estimator->post_impact_gravity_trusted = false;
    estimator->post_impact_gravity_trust_streak = 0U;
    estimator->attitude_converged = true;
    estimator->attitude_convergence_streak =
        estimator->config.attitude_convergence_windows;
    estimator->attitude_aiding_miss_streak = 0U;
    if (estimator->post_impact_reacquire_count < UINT32_MAX)
        estimator->post_impact_reacquire_count++;
    output->attitude_reacquired = true;
}

static void estimator_mark_heading_continuity_lost(
    dual_imu_estimator_t *estimator,
    uint64_t event_timestamp_us)
{
    if (!estimator->heading_continuity_lost)
        estimator->heading_continuity_lost_timestamp_us = event_timestamp_us;
    estimator->heading_continuity_lost = true;
}

static void estimator_record_unobserved_rotation(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output)
{
    if (!output->rotation_unobserved)
        return;

    estimator_mark_heading_continuity_lost(estimator, output->start_us);
    estimator_enter_post_impact_reacquire(estimator);
}

/* A shock that never trips the impact detectors can leave a large tilt error
 * while covariance stays small; the gravity NIS gate then rejects every
 * update and the lane never reconverges. Count consecutive NIS rejections on
 * otherwise clean windows, reopen the gate by inflating attitude covariance,
 * and fall back to a heading-preserving tilt reseed if inflation alone is
 * not enough. Rotation was observed throughout, so heading continuity is
 * kept; only the gravity-observable tilt is corrected. */
static void estimator_handle_accel_recovery(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output,
    const imu_preintegrated_window_t windows[DUAL_IMU_ESTIMATOR_LANE_COUNT],
    bool accel_updates_enabled)
{
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        /* Windows without a clean own-lane gravity attempt are evidence in
         * neither direction: hold the streak so dynamics or an inhibit
         * interval cannot silently clear a stuck gate. */
        if (!accel_updates_enabled || !estimator->lane_seeded[lane] ||
            !windows[lane].accel_valid ||
            (estimator->hard_fault_flags[lane] != 0U) ||
            estimator->lane_accel_fault[lane])
            continue;

        if (output->accel_result[lane] == ATTITUDE_MEKF_ACCEL_ACCEPTED) {
            estimator->accel_nis_stuck_streak[lane] = 0U;
            continue;
        }
        /* REJECTED_CORRECTION is equally conclusive stuck evidence: inflation
         * reopened the NIS gate but the needed correction exceeds the trust
         * region, so only the reseed escalation can recover. */
        if ((output->accel_result[lane] != ATTITUDE_MEKF_ACCEL_REJECTED_NIS) &&
            (output->accel_result[lane] !=
             ATTITUDE_MEKF_ACCEL_REJECTED_CORRECTION))
            continue;
        /* Gravity-quality admission (FIX_PLAN §1.2 第 1 层): a rejection is
         * stuck evidence only when the measured magnitude is close to gravity
         * AND the rotation rate is low. During sustained rotation the window
         * mean carries centripetal and tangential acceleration, the direction
         * is genuinely not gravity and the NIS gate is rejecting CORRECTLY;
         * inflating the covariance or reseeding tilt from such windows would
         * drive the attitude toward the contaminated direction. Perpendicular
         * contamination below ~3 m/s2 barely moves the magnitude, so the rate
         * limit is the admission that actually excludes it. Hold the streak
         * instead of advancing it. */
        const float specific_force_norm = estimator_vector_norm(
            output->lane_specific_force_mps2[lane]);
        const float rotation_rate = estimator_vector_norm(
            output->angular_rate_rad_s);
        if (!isfinite(specific_force_norm) || !isfinite(rotation_rate) ||
            (fabsf(specific_force_norm - DUAL_IMU_ESTIMATOR_GRAVITY_MPS2) >
             estimator->config.accel_recovery_norm_tolerance_mps2) ||
            (rotation_rate > estimator->config.accel_recovery_max_rate_rad_s))
            continue;

        if (estimator->accel_nis_stuck_streak[lane] < UINT16_MAX)
            estimator->accel_nis_stuck_streak[lane]++;
        const uint16_t streak = estimator->accel_nis_stuck_streak[lane];
        if (streak < estimator->config.accel_recovery_stuck_windows)
            continue;

        if (estimator->config.accel_recovery_reseed_enabled &&
            (streak >= estimator->config.accel_recovery_reseed_windows)) {
            float recoverable_bias[3];
            if (!estimator_get_recoverable_bias(&estimator->mekf[lane],
                                                recoverable_bias)) {
                estimator->lane_calibrated[lane] = false;
                estimator->zaru_accept_count[lane] = 0U;
                estimator->zaru_divergence_count[lane] = 0U;
            }
            float reference_quaternion[4];
            if (!attitude_mekf_get_quaternion(&estimator->mekf[lane],
                                              reference_quaternion) ||
                !estimator_reseed_tilt_preserving_heading(
                    &estimator->mekf[lane],
                    output->lane_specific_force_mps2[lane],
                    reference_quaternion,
                    recoverable_bias)) {
                estimator_latch_filter_fault(estimator, lane, output->end_us);
                estimator->accel_nis_stuck_streak[lane] = 0U;
                continue;
            }
            estimator->accel_nis_stuck_streak[lane] = 0U;
            output->lane_accel_recovery_reseeded[lane] = true;
            if (estimator->accel_recovery_reseed_count < UINT32_MAX)
                estimator->accel_recovery_reseed_count++;
            estimator_note_attitude_rewrite(
                estimator, (uint32_t)lane,
                DUAL_IMU_ATTITUDE_REWRITE_ACCEL_RECOVERY);
            estimator_enter_post_impact_reacquire(estimator);
            continue;
        }

        if (!attitude_mekf_mark_rotation_unobserved(
                &estimator->mekf[lane],
                estimator->config.accel_recovery_inflation_std_rad)) {
            estimator_latch_filter_fault(estimator, lane, output->end_us);
            estimator->accel_nis_stuck_streak[lane] = 0U;
            continue;
        }
        output->lane_accel_recovery_inflating[lane] = true;
        if (estimator->accel_recovery_inflation_count < UINT32_MAX)
            estimator->accel_recovery_inflation_count++;
    }
}

static void estimator_note_attitude_rewrite(
    dual_imu_estimator_t *estimator,
    uint32_t lane,
    dual_imu_attitude_rewrite_reason_t reason)
{
    if (estimator->attitude_rewrite_count < UINT32_MAX)
        estimator->attitude_rewrite_count++;
    estimator->attitude_rewrite_last_reason = (uint8_t)reason;
    estimator->attitude_rewrite_last_lane = (uint8_t)lane;
    if (lane == DUAL_IMU_ATTITUDE_REWRITE_LANE_BOTH) {
        estimator->lane_attitude_discontinuity[0] = true;
        estimator->lane_attitude_discontinuity[1] = true;
    } else if (lane < DUAL_IMU_ESTIMATOR_LANE_COUNT) {
        estimator->lane_attitude_discontinuity[lane] = true;
    }
}

static void estimator_reset_impact_gyro_disagreement(
    dual_imu_estimator_t *estimator)
{
    estimator->impact_gyro_suspect_start_us = 0U;
    estimator->impact_gyro_suspect_last_end_us = 0U;
    estimator->impact_gyro_disagreement_streak = 0U;
    estimator->impact_gyro_checkpoint_lane_seeded_mask = 0U;
    estimator->impact_gyro_checkpoint_filter_fault_mask = 0U;
    estimator->impact_gyro_checkpoint_valid = false;
    estimator->impact_gyro_disagreement_confirmed = false;
    estimator->impact_gyro_output_initialized_checkpoint = false;
}

static void estimator_capture_impact_gyro_checkpoint(
    dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    const attitude_mekf_t mekf_before[DUAL_IMU_ESTIMATOR_LANE_COUNT],
    const imu_angular_accel_estimator_t *angular_accel_before,
    uint8_t lane_seeded_mask,
    uint8_t filter_fault_mask)
{
    memcpy(estimator->impact_gyro_mekf_checkpoint, mekf_before,
           sizeof(estimator->impact_gyro_mekf_checkpoint));
    estimator->impact_gyro_angular_accel_checkpoint = *angular_accel_before;
    estimator->impact_gyro_suspect_start_us = output->start_us;
    estimator->impact_gyro_suspect_last_end_us = output->end_us;
    estimator->impact_gyro_disagreement_streak = 1U;
    estimator->impact_gyro_checkpoint_lane_seeded_mask = lane_seeded_mask;
    estimator->impact_gyro_checkpoint_filter_fault_mask = filter_fault_mask;
    memcpy(estimator->impact_gyro_output_quaternion_checkpoint,
           estimator->output_quaternion,
           sizeof(estimator->impact_gyro_output_quaternion_checkpoint));
    memcpy(estimator->impact_gyro_output_alignment_checkpoint,
           estimator->output_alignment,
           sizeof(estimator->impact_gyro_output_alignment_checkpoint));
    estimator->impact_gyro_previous_selected_lane_checkpoint =
        estimator->previous_selected_lane;
    estimator->impact_gyro_output_initialized_checkpoint =
        estimator->output_initialized;
    estimator->impact_gyro_checkpoint_valid = true;
    estimator->impact_gyro_disagreement_confirmed = false;
}

static void estimator_restore_impact_gyro_checkpoint(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output)
{
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        const attitude_mekf_diagnostics_t live_diagnostics =
            estimator->mekf[lane].diagnostics;
        const attitude_mekf_config_t live_config =
            estimator->mekf[lane].config;
        estimator->mekf[lane] =
            estimator->impact_gyro_mekf_checkpoint[lane];
        estimator->mekf[lane].diagnostics = live_diagnostics;
        estimator->mekf[lane].config = live_config;
    }
    estimator->angular_accel_estimator =
        estimator->impact_gyro_angular_accel_checkpoint;
    memcpy(estimator->output_quaternion,
           estimator->impact_gyro_output_quaternion_checkpoint,
           sizeof(estimator->output_quaternion));
    memcpy(estimator->output_alignment,
           estimator->impact_gyro_output_alignment_checkpoint,
           sizeof(estimator->output_alignment));
    estimator->previous_selected_lane =
        estimator->impact_gyro_previous_selected_lane_checkpoint;
    estimator->output_initialized =
        estimator->impact_gyro_output_initialized_checkpoint;

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        const uint8_t lane_mask = (uint8_t)(1U << lane);
        const bool live_filter_fault =
            (estimator->hard_fault_flags[lane] &
             DUAL_IMU_ESTIMATOR_FILTER_FAULT) != 0U;
        estimator->lane_seeded[lane] =
            (estimator->impact_gyro_checkpoint_lane_seeded_mask & lane_mask) !=
                0U &&
            !live_filter_fault;
        estimator->hard_fault_flags[lane] =
            estimator_sensor_fault_flags(estimator, lane) |
            ((live_filter_fault ||
              ((estimator->impact_gyro_checkpoint_filter_fault_mask &
                lane_mask) != 0U))
                 ? DUAL_IMU_ESTIMATOR_FILTER_FAULT
                 : 0U);
        output->lane_aided_propagation[lane] = false;
    }
    estimator_note_attitude_rewrite(estimator,
                                    DUAL_IMU_ATTITUDE_REWRITE_LANE_BOTH,
                                    DUAL_IMU_ATTITUDE_REWRITE_ROLLBACK);
}

static bool estimator_handle_impact_gyro_disagreement(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output,
    bool disagreement_evidence,
    const attitude_mekf_t mekf_before[DUAL_IMU_ESTIMATOR_LANE_COUNT],
    const imu_angular_accel_estimator_t *angular_accel_before,
    uint8_t lane_seeded_mask,
    uint8_t filter_fault_mask)
{
    if (!output->accel_inhibited || !disagreement_evidence) {
        estimator_reset_impact_gyro_disagreement(estimator);
        return false;
    }

    if (!estimator->impact_gyro_checkpoint_valid ||
        (output->start_us != estimator->impact_gyro_suspect_last_end_us)) {
        estimator_capture_impact_gyro_checkpoint(
            estimator, output, mekf_before, angular_accel_before,
            lane_seeded_mask, filter_fault_mask);
    } else {
        estimator->impact_gyro_suspect_last_end_us = output->end_us;
        if (estimator->impact_gyro_disagreement_streak < UINT16_MAX)
            estimator->impact_gyro_disagreement_streak++;
    }

    /* Do not commit a gyro interval that the two lanes cannot attribute. The
     * selector remains live so consecutive evidence can still confirm the
     * event, while every other estimator state stays at the first suspect
     * boundary. A later clear window therefore resumes from the last trusted
     * attitude without publishing a transient lane spike. */
    estimator_restore_impact_gyro_checkpoint(estimator, output);

    if (estimator->impact_gyro_disagreement_streak <
        estimator->config.impact_gyro_disagreement_confirm_windows) {
        return false;
    }
    estimator->impact_gyro_disagreement_confirmed = true;

    const float suspect_duration_s =
        (float)(output->end_us - estimator->impact_gyro_suspect_start_us) *
        DUAL_IMU_ESTIMATOR_US_TO_S;
    const float rotation_std_rad =
        estimator->config.unobserved_rotation_rate_std_rad_s *
        suspect_duration_s;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (estimator->lane_seeded[lane] &&
            !attitude_mekf_mark_rotation_unobserved(
                &estimator->mekf[lane], rotation_std_rad)) {
            estimator_latch_filter_fault(estimator, lane, output->end_us);
        }
    }
    output->rotation_unobserved = true;
    estimator_mark_heading_continuity_lost(
        estimator, estimator->impact_gyro_suspect_start_us);
    estimator_enter_post_impact_reacquire(estimator);
    return true;
}

static bool estimator_reject_stationary(
    dual_imu_estimator_t *estimator,
    dual_imu_stationary_reject_reason_t reason)
{
    estimator->stationary_last_reject_reason = reason;
    if ((reason > DUAL_IMU_STATIONARY_REJECT_NONE) &&
        (reason < DUAL_IMU_STATIONARY_REJECT_COUNT) &&
        (estimator->stationary_reject_count[reason] < UINT32_MAX)) {
        estimator->stationary_reject_count[reason]++;
    }
    /* Any genuine reject (motion, variance, gravity drift, invalid) is evidence
     * against a stuck-bias deadlock; only the in-run bias-recovery hold path
     * (which preserves the dwell statistics) advances the streak. */
    estimator->zaru_recovery_reject_streak = 0U;
    estimator_reset_stationary_tracking(estimator);
    return false;
}

static uint16_t estimator_stationary_required_windows(
    const dual_imu_estimator_t *estimator,
    uint8_t lane_mask)
{
    const bool single_lane = lane_mask != 0x03U;
    if (single_lane) {
        return estimator->stationary_hint
            ? estimator->config.stationary_single_lane_hint_dwell_windows
            : estimator->config.stationary_single_lane_dwell_windows;
    }
    return estimator->stationary_hint
        ? estimator->config.stationary_hint_dwell_windows
        : estimator->config.stationary_dwell_windows;
}

static uint32_t estimator_stationary_warmup_windows(
    const dual_imu_estimator_t *estimator,
    uint8_t lane_mask)
{
    const uint32_t required_windows =
        estimator_stationary_required_windows(estimator, lane_mask);
    return (required_windows < DUAL_IMU_ESTIMATOR_STATIONARY_WARMUP_WINDOWS)
        ? required_windows
        : DUAL_IMU_ESTIMATOR_STATIONARY_WARMUP_WINDOWS;
}

static uint8_t estimator_stationary_available_lane_mask(
    const dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output)
{
    uint8_t lane_mask = 0U;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        const bool filter_fault =
            (estimator->hard_fault_flags[lane] &
             DUAL_IMU_ESTIMATOR_FILTER_FAULT) != 0U;
        if (estimator_raw_lane_usable_for_geometry(estimator, lane) &&
            (!filter_fault || estimator_filter_recovery_window_is_new(
                                  estimator, output, lane)) &&
            !estimator->lane_accel_fault[lane] &&
            output->lane_window[lane].gyro_valid &&
            output->lane_window[lane].accel_valid) {
            lane_mask |= (uint8_t)(1U << lane);
        }
    }
    return lane_mask;
}

static bool estimator_update_stationary_statistics(
    dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    uint8_t lane_mask)
{
    const uint32_t previous_count = estimator->stationary_statistics_count;
    const uint32_t memory_windows =
        estimator_stationary_required_windows(estimator, lane_mask);
    const uint32_t count = (previous_count < memory_windows)
                               ? previous_count + 1U
                               : memory_windows;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((lane_mask & (uint8_t)(1U << lane)) == 0U)
            continue;
        for (size_t axis = 0U; axis < 3U; ++axis) {
            const float gyro = output->lane_rate_rad_s[lane][axis];
            const float gyro_delta =
                gyro - estimator->stationary_gyro_mean_rad_s[lane][axis];
            const float accel = output->lane_specific_force_mps2[lane][axis];
            const float accel_delta =
                accel - estimator->stationary_accel_mean_mps2[lane][axis];
            if (previous_count < memory_windows) {
                const float inverse_count = 1.0f / (float)count;
                estimator->stationary_gyro_mean_rad_s[lane][axis] +=
                    gyro_delta * inverse_count;
                estimator->stationary_gyro_m2_rad2_s2[lane][axis] +=
                    gyro_delta *
                    (gyro - estimator->stationary_gyro_mean_rad_s[lane][axis]);
                estimator->stationary_accel_mean_mps2[lane][axis] +=
                    accel_delta * inverse_count;
                estimator->stationary_accel_m2_m2_s4[lane][axis] +=
                    accel_delta *
                    (accel - estimator->stationary_accel_mean_mps2[lane][axis]);
            } else {
                const float alpha = 1.0f / (float)memory_windows;
                const float decay = 1.0f - alpha;
                const float degrees_of_freedom =
                    (float)(memory_windows - 1U);
                const float gyro_variance =
                    estimator->stationary_gyro_m2_rad2_s2[lane][axis] /
                    degrees_of_freedom;
                const float accel_variance =
                    estimator->stationary_accel_m2_m2_s4[lane][axis] /
                    degrees_of_freedom;
                estimator->stationary_gyro_mean_rad_s[lane][axis] +=
                    alpha * gyro_delta;
                estimator->stationary_accel_mean_mps2[lane][axis] +=
                    alpha * accel_delta;
                estimator->stationary_gyro_m2_rad2_s2[lane][axis] =
                    decay * (gyro_variance +
                             (alpha * gyro_delta * gyro_delta)) *
                    degrees_of_freedom;
                estimator->stationary_accel_m2_m2_s4[lane][axis] =
                    decay * (accel_variance +
                             (alpha * accel_delta * accel_delta)) *
                    degrees_of_freedom;
            }
        }
    }
    estimator->stationary_statistics_count = count;
    const uint32_t warmup_windows =
        estimator_stationary_warmup_windows(estimator, lane_mask);
    if (count < warmup_windows)
        return true;

    const float inverse_degrees_of_freedom = 1.0f / (float)(count - 1U);
    bool within_limits = true;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((lane_mask & (uint8_t)(1U << lane)) == 0U)
            continue;
        float gyro_variance = 0.0f;
        float accel_variance = 0.0f;
        for (size_t axis = 0U; axis < 3U; ++axis) {
            const float gyro_axis_variance =
                estimator->stationary_gyro_m2_rad2_s2[lane][axis] *
                inverse_degrees_of_freedom;
            const float accel_axis_variance =
                estimator->stationary_accel_m2_m2_s4[lane][axis] *
                inverse_degrees_of_freedom;
            if (!isfinite(gyro_axis_variance) ||
                !isfinite(accel_axis_variance)) {
                within_limits = false;
                continue;
            }
            gyro_variance += fmaxf(0.0f, gyro_axis_variance);
            accel_variance += fmaxf(0.0f, accel_axis_variance);
        }
        estimator->stationary_temporal_gyro_variance_rad2_s2[lane] =
            gyro_variance;
        estimator->stationary_temporal_accel_variance_m2_s4[lane] =
            accel_variance;
        if ((gyro_variance >
             estimator->config
                 .stationary_gyro_temporal_variance_limit_rad2_s2[lane]) ||
            (accel_variance >
             estimator->config
                 .stationary_accel_temporal_variance_limit_m2_s4))
            within_limits = false;
    }
    return within_limits;
}

/* Bias-independent gravity-direction stability over the dwell. Establishes the
 * per-lane reference on first use and rejects once the dwell-mean specific-force
 * direction drifts past the limit (a slow tilting rotation). Shared by the
 * normal candidate path and the in-run bias-recovery admissibility test. */
static bool estimator_stationary_gravity_direction_stable(
    dual_imu_estimator_t *estimator,
    uint8_t lane_mask)
{
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((lane_mask & (uint8_t)(1U << lane)) == 0U)
            continue;
        const float norm = estimator_vector_norm(
            estimator->stationary_accel_mean_mps2[lane]);
        if (!isfinite(norm) || (norm <= 0.0f))
            return false;
        float gravity_direction[3];
        for (size_t axis = 0U; axis < 3U; ++axis)
            gravity_direction[axis] =
                estimator->stationary_accel_mean_mps2[lane][axis] / norm;

        if (!estimator->stationary_gravity_reference_valid[lane]) {
            memcpy(estimator->stationary_gravity_reference[lane],
                   gravity_direction, sizeof(gravity_direction));
            estimator->stationary_gravity_reference_valid[lane] = true;
            continue;
        }
        if (estimator_vector_distance(
                gravity_direction,
                estimator->stationary_gravity_reference[lane]) >
            estimator->config.stationary_accel_direction_limit)
            return false;
    }
    return true;
}

/* Admissibility for in-run gyro-bias recovery (FIX_PLAN §12.3). The bias
 * residual is large, but that alone cannot tell a stuck-bias deadlock from
 * genuine slow motion, so require the bias-INDEPENDENT evidence that only real
 * rest produces. Each lane's raw dwell mean is decomposed against that lane's
 * dwell-mean gravity direction: the gravity-parallel (yaw) component must be
 * near zero because a steady pan about gravity is indistinguishable from
 * yaw-axis bias (FIX_PLAN §12.3-3, accepted residual risk below the yaw
 * tolerance), while the perpendicular (tilt) component only needs to stay
 * inside the post-shock ZRO-shift budget because a real tilt-axis rotation at
 * that rate would move the gravity direction across the dwell and fail the
 * gravity-stability gate that guards this path. When both lanes are available
 * the two heterogeneous sensors must also agree so a grossly lying lane cannot
 * self-absolve; a single available lane has no cross-check and the caller
 * compensates with a longer required dwell. */
static bool estimator_bias_recovery_admissible(
    dual_imu_estimator_t *estimator,
    uint8_t lane_mask)
{
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((lane_mask & (uint8_t)(1U << lane)) == 0U)
            continue;
        const float accel_norm = estimator_vector_norm(
            estimator->stationary_accel_mean_mps2[lane]);
        if (!isfinite(accel_norm) || (accel_norm <= 0.0f))
            return false;
        float parallel_rate = 0.0f;
        for (size_t axis = 0U; axis < 3U; ++axis) {
            parallel_rate +=
                estimator->stationary_gyro_mean_rad_s[lane][axis] *
                (estimator->stationary_accel_mean_mps2[lane][axis] /
                 accel_norm);
        }
        const float total_rate = estimator_vector_norm(
            estimator->stationary_gyro_mean_rad_s[lane]);
        const float tilt_rate_squared =
            (total_rate * total_rate) - (parallel_rate * parallel_rate);
        const float tilt_rate = sqrtf(fmaxf(0.0f, tilt_rate_squared));
        if (!isfinite(parallel_rate) || !isfinite(tilt_rate) ||
            (fabsf(parallel_rate) >
             estimator->config.zaru_recovery_rest_yaw_rate_tolerance_rad_s) ||
            (tilt_rate >
             estimator->config.zaru_recovery_rest_tilt_rate_tolerance_rad_s))
            return false;
    }
    if (lane_mask == 0x03U) {
        const float difference = estimator_vector_distance(
            estimator->stationary_gyro_mean_rad_s[0],
            estimator->stationary_gyro_mean_rad_s[1]);
        if (!isfinite(difference) ||
            (difference >
             estimator->config.zaru_recovery_dual_lane_rate_tolerance_rad_s))
            return false;
    }
    return estimator_stationary_gravity_direction_stable(estimator, lane_mask);
}

/* Reseeds each diverged lane's gyro bias from its raw stationary dwell mean and
 * inflates covariance to reflect the discontinuous reset (FIX_PLAN §12.3).
 * attitude_mekf_reset preserves the current attitude estimate (its mean is
 * unchanged) while restoring the bias and attitude covariance blocks to their
 * initial one-sigma; gravity aiding then re-tightens tilt within the
 * confirmation dwell. Calibration is revoked so the normal ZARU path must
 * re-confirm the lane. */
static void estimator_recover_diverged_bias(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output,
    uint8_t reseed_mask)
{
    float target_bias_rad_s[DUAL_IMU_ESTIMATOR_LANE_COUNT][3];
    memcpy(target_bias_rad_s, estimator->stationary_gyro_mean_rad_s,
           sizeof(target_bias_rad_s));
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((reseed_mask & (uint8_t)(1U << lane)) == 0U)
            continue;
        float current_quaternion[4];
        if (!attitude_mekf_get_quaternion(&estimator->mekf[lane],
                                          current_quaternion) ||
            !attitude_mekf_reset(&estimator->mekf[lane], current_quaternion,
                                 target_bias_rad_s[lane])) {
            estimator_latch_filter_fault(estimator, lane, output->end_us);
            continue;
        }
        estimator->lane_calibrated[lane] = false;
        estimator->zaru_accept_count[lane] = 0U;
        estimator->zaru_divergence_count[lane] = 0U;
        output->lane_bias_recovery_reseeded[lane] = true;
        if (estimator->zaru_bias_recovery_count < UINT32_MAX)
            estimator->zaru_bias_recovery_count++;
    }
}

/* Residual-gate hold path (FIX_PLAN §12.3). Every bias-independent stationary
 * criterion passed this window, only the bias residual failed -- exactly the
 * signature of a stuck-bias deadlock. Record the reject for attribution but
 * KEEP the dwell statistics (estimator_reject_stationary would reset them), and
 * count consecutive admissible windows toward the recovery reseed. Any other
 * reject reason resets the streak in estimator_reject_stationary. */
static bool estimator_hold_stationary_for_bias_recovery(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output,
    uint8_t lane_mask,
    uint8_t diverged_mask)
{
    estimator->stationary_last_reject_reason =
        DUAL_IMU_STATIONARY_REJECT_MEAN_RATE;
    if (estimator->stationary_reject_count[DUAL_IMU_STATIONARY_REJECT_MEAN_RATE] <
        UINT32_MAX) {
        estimator->stationary_reject_count[DUAL_IMU_STATIONARY_REJECT_MEAN_RATE]++;
    }

    if (!estimator_bias_recovery_admissible(estimator, lane_mask)) {
        estimator->zaru_recovery_reject_streak = 0U;
        return false;
    }
    if (estimator->zaru_recovery_reject_streak < UINT16_MAX)
        estimator->zaru_recovery_reject_streak++;
    /* A single lane has no heterogeneous cross-check; demand a longer dwell. */
    const uint32_t required_windows = (lane_mask == 0x03U)
        ? estimator->config.zaru_recovery_reject_windows
        : 2U * (uint32_t)estimator->config.zaru_recovery_reject_windows;
    if (estimator->zaru_recovery_reject_streak >= required_windows) {
        estimator_recover_diverged_bias(estimator, output, diverged_mask);
        estimator->zaru_recovery_reject_streak = 0U;
    }
    return false;
}

static bool estimator_stationary_candidate(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output,
    uint8_t lane_mask)
{
    if (lane_mask == 0U) {
        return estimator_reject_stationary(
            estimator, DUAL_IMU_STATIONARY_REJECT_INVALID_OR_FAULT);
    }
    if (lane_mask != estimator->stationary_lane_mask) {
        estimator_reset_stationary_tracking(estimator);
        estimator->stationary_lane_mask = lane_mask;
    }

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((lane_mask & (uint8_t)(1U << lane)) == 0U)
            continue;
        if (estimator_vector_norm(output->lane_rate_rad_s[lane]) >
            estimator->config.stationary_gyro_limit_rad_s) {
            return estimator_reject_stationary(
                estimator, DUAL_IMU_STATIONARY_REJECT_INSTANT_RATE);
        }
        if (fabsf(estimator_vector_norm(
                       output->lane_specific_force_mps2[lane]) -
                   DUAL_IMU_ESTIMATOR_GRAVITY_MPS2) >
            estimator->config.stationary_accel_norm_tolerance_mps2) {
            return estimator_reject_stationary(
                estimator, DUAL_IMU_STATIONARY_REJECT_ACCEL_NORM);
        }

        float gyro_variance = 0.0f;
        for (size_t axis = 0U; axis < 3U; ++axis) {
            const float mean = output->lane_window[lane].gyro_mean_rad_s[axis];
            const float variance =
                output->lane_window[lane].gyro_second_moment_rad2_s2[axis][axis] -
                (mean * mean);
            gyro_variance += fmaxf(0.0f, variance);
        }
        if (!isfinite(gyro_variance) ||
            (gyro_variance >
             (3.0f * estimator->config.stationary_gyro_variance_limit_rad2_s2))) {
            return estimator_reject_stationary(
                estimator, DUAL_IMU_STATIONARY_REJECT_WINDOW_GYRO_VARIANCE);
        }
    }

    if ((lane_mask == 0x03U) &&
        (output->accel_pair_residual_mps2 >
         estimator->config.stationary_accel_pair_limit_mps2)) {
        return estimator_reject_stationary(
            estimator, DUAL_IMU_STATIONARY_REJECT_ACCEL_PAIR);
    }

    if (!estimator_update_stationary_statistics(estimator, output, lane_mask)) {
        return estimator_reject_stationary(
            estimator, DUAL_IMU_STATIONARY_REJECT_TEMPORAL_VARIANCE);
    }

    const uint32_t warmup_windows =
        estimator_stationary_warmup_windows(estimator, lane_mask);
    if (estimator->stationary_statistics_count < warmup_windows)
        return true;

    uint8_t diverged_mask = 0U;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((lane_mask & (uint8_t)(1U << lane)) == 0U)
            continue;
        const bool filter_only_recovery =
            estimator->post_impact_reacquire_active &&
            ((estimator->hard_fault_flags[lane] &
              DUAL_IMU_ESTIMATOR_FILTER_FAULT) != 0U) &&
            (estimator_sensor_fault_flags(estimator, lane) == 0U);
        const bool filter_valid = estimator->lane_seeded[lane] &&
                                  attitude_mekf_is_valid(
                                      &estimator->mekf[lane]);
        if (!filter_valid && !filter_only_recovery) {
            return estimator_reject_stationary(
                estimator, DUAL_IMU_STATIONARY_REJECT_INVALID_OR_FAULT);
        }

        float learned_bias_rad_s[3];
        if (filter_valid) {
            if (!attitude_mekf_get_bias(&estimator->mekf[lane],
                                        learned_bias_rad_s)) {
                return estimator_reject_stationary(
                    estimator, DUAL_IMU_STATIONARY_REJECT_INVALID_OR_FAULT);
            }
        } else {
            (void)estimator_get_recoverable_bias(&estimator->mekf[lane],
                                                 learned_bias_rad_s);
        }
        if (!estimator_vector_is_finite(learned_bias_rad_s)) {
            return estimator_reject_stationary(
                estimator, DUAL_IMU_STATIONARY_REJECT_INVALID_OR_FAULT);
        }
        float residual_rad_s[3];
        for (size_t axis = 0U; axis < 3U; ++axis) {
            residual_rad_s[axis] =
                estimator->stationary_gyro_mean_rad_s[lane][axis] -
                learned_bias_rad_s[axis];
        }
        if (estimator_vector_norm(residual_rad_s) >
            estimator->config.zaru_target_residual_limit_rad_s) {
            diverged_mask |= (uint8_t)(1U << lane);
        }
    }

    /* Bias-independent, so it outranks the bias-residual verdict: unstable
     * gravity is positive evidence of real motion and must fully reset the
     * dwell rather than enter the stuck-bias hold path. */
    if (!estimator_stationary_gravity_direction_stable(estimator, lane_mask)) {
        return estimator_reject_stationary(
            estimator, DUAL_IMU_STATIONARY_REJECT_GRAVITY_DIRECTION);
    }

    if (diverged_mask != 0U) {
        return estimator_hold_stationary_for_bias_recovery(
            estimator, output, lane_mask, diverged_mask);
    }
    estimator->zaru_recovery_reject_streak = 0U;
    return true;
}

static bool estimator_stationary_reject_requires_immediate_exit(
    dual_imu_stationary_reject_reason_t reason)
{
    return (reason == DUAL_IMU_STATIONARY_REJECT_INVALID_OR_FAULT) ||
           (reason == DUAL_IMU_STATIONARY_REJECT_INSTANT_RATE) ||
           (reason == DUAL_IMU_STATIONARY_REJECT_INHIBITED);
}

static bool estimator_confirmed_stationary_integrity_valid(
    const dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    uint8_t available_lane_mask)
{
    const uint8_t confirmed_mask = estimator->stationary_confirmed_lane_mask;
    if (output->accel_inhibited || (confirmed_mask == 0U) ||
        (available_lane_mask != confirmed_mask)) {
        return false;
    }

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((confirmed_mask & (uint8_t)(1U << lane)) == 0U)
            continue;
        if ((estimator->hard_fault_flags[lane] != 0U) ||
            estimator->lane_accel_fault[lane] ||
            !output->lane_window[lane].gyro_valid ||
            !output->lane_window[lane].accel_valid ||
            !estimator->lane_seeded[lane] ||
            !attitude_mekf_is_valid(&estimator->mekf[lane])) {
            return false;
        }
    }
    return true;
}

static bool estimator_stationary_rate_exit_evidence(
    const dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    uint8_t lane_mask,
    bool *any_lane_exceeds,
    bool *all_lanes_exceed)
{
    float minimum_residual_norm = INFINITY;
    float maximum_residual_norm = 0.0f;
    *any_lane_exceeds = false;
    *all_lanes_exceed = false;
    if (lane_mask == 0U)
        return false;

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((lane_mask & (uint8_t)(1U << lane)) == 0U)
            continue;

        float learned_bias_rad_s[3];
        if (!attitude_mekf_get_bias(&estimator->mekf[lane],
                                    learned_bias_rad_s) ||
            !estimator_vector_is_finite(output->lane_rate_rad_s[lane])) {
            return false;
        }
        float residual_rad_s[3];
        for (size_t axis = 0U; axis < 3U; ++axis) {
            residual_rad_s[axis] = output->lane_rate_rad_s[lane][axis] -
                                   learned_bias_rad_s[axis];
        }
        const float residual_norm = estimator_vector_norm(residual_rad_s);
        if (!isfinite(residual_norm))
            return false;
        minimum_residual_norm = fminf(minimum_residual_norm, residual_norm);
        maximum_residual_norm = fmaxf(maximum_residual_norm, residual_norm);
    }

    *any_lane_exceeds = maximum_residual_norm >
        estimator->config.zaru_target_residual_limit_rad_s;
    *all_lanes_exceed = minimum_residual_norm >
        estimator->config.zaru_target_residual_limit_rad_s;
    return true;
}

static bool estimator_update_stationary_confirmation(
    dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    uint8_t available_lane_mask)
{
    if (!estimator->stationary_confirmed_latched) {
        estimator->stationary_soft_exit_streak = 0U;
        estimator->stationary_rate_exit_streak = 0U;
        const uint16_t required_windows =
            estimator_stationary_required_windows(estimator,
                                                  available_lane_mask);
        if (output->stationary_candidate && (available_lane_mask != 0U) &&
            (estimator->stationary_streak >= required_windows)) {
            estimator->stationary_confirmed_latched = true;
            estimator->stationary_confirmed_lane_mask = available_lane_mask;
        }
    }

    if (!estimator->stationary_confirmed_latched)
        return false;

    const bool immediate_reject =
        !output->stationary_candidate &&
        estimator_stationary_reject_requires_immediate_exit(
            estimator->stationary_last_reject_reason);
    if (immediate_reject ||
        !estimator_confirmed_stationary_integrity_valid(
            estimator, output, available_lane_mask)) {
        estimator_reset_stationary_tracking(estimator);
        estimator_clear_stationary_confirmation(estimator);
        return false;
    }

    bool any_lane_rate_exit_evidence = false;
    bool all_lanes_rate_exit_evidence = false;
    const bool rate_evidence_valid = estimator_stationary_rate_exit_evidence(
        estimator, output, estimator->stationary_confirmed_lane_mask,
        &any_lane_rate_exit_evidence, &all_lanes_rate_exit_evidence);
    if (!rate_evidence_valid) {
        estimator_reset_stationary_tracking(estimator);
        estimator_clear_stationary_confirmation(estimator);
        return false;
    }

    if (all_lanes_rate_exit_evidence) {
        estimator->stationary_soft_exit_streak = 0U;
        if (estimator->stationary_rate_exit_streak < UINT16_MAX)
            estimator->stationary_rate_exit_streak++;
        if (estimator->stationary_rate_exit_streak >=
            estimator->config.stationary_rate_exit_windows) {
            estimator_reset_stationary_tracking(estimator);
            estimator_clear_stationary_confirmation(estimator);
        }
        return false;
    }
    estimator->stationary_rate_exit_streak = 0U;

    if (any_lane_rate_exit_evidence || !output->stationary_candidate) {
        if (estimator->stationary_soft_exit_streak < UINT16_MAX)
            estimator->stationary_soft_exit_streak++;
        if (estimator->stationary_soft_exit_streak >=
            estimator->config.stationary_soft_exit_windows) {
            estimator_reset_stationary_tracking(estimator);
            estimator_clear_stationary_confirmation(estimator);
        }
        return false;
    }
    estimator->stationary_soft_exit_streak = 0U;

    const uint32_t warmup_windows = estimator_stationary_warmup_windows(
        estimator, estimator->stationary_confirmed_lane_mask);
    return estimator->stationary_statistics_count >= warmup_windows;
}

static bool estimator_reacquire_attitude(
    dual_imu_estimator_t *estimator,
    dual_imu_estimator_output_t *output)
{
    const uint8_t lane_mask = estimator->stationary_lane_mask;
    if (!estimator->post_impact_reacquire_active ||
        output->accel_inhibited || (lane_mask == 0U))
        return false;

    const bool single_lane = lane_mask != 0x03U;
    uint16_t required_windows = single_lane
        ? estimator->config.post_impact_reacquire_single_lane_dwell_windows
        : estimator->config.post_impact_reacquire_dwell_windows;
    const uint32_t warmup_windows =
        estimator_stationary_warmup_windows(estimator, lane_mask);
    if (required_windows < warmup_windows)
        required_windows = (uint16_t)warmup_windows;
    if (estimator->stationary_streak < required_windows)
        return false;

    uint32_t accel_lane = 0U;
    while ((accel_lane < DUAL_IMU_ESTIMATOR_LANE_COUNT) &&
           ((lane_mask & (uint8_t)(1U << accel_lane)) == 0U)) {
        accel_lane++;
    }
    if (accel_lane >= DUAL_IMU_ESTIMATOR_LANE_COUNT)
        return false;

    float reference_quaternion[4];
    if (estimator->output_initialized)
        memcpy(reference_quaternion, estimator->output_quaternion,
               sizeof(reference_quaternion));
    else
        estimator_quaternion_identity(reference_quaternion);

    attitude_mekf_t staged[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    bool staged_seeded[DUAL_IMU_ESTIMATOR_LANE_COUNT] = {false, false};
    bool staged_bias_preserved[DUAL_IMU_ESTIMATOR_LANE_COUNT] = {false, false};
    memcpy(staged, estimator->mekf, sizeof(staged));
    bool any_seeded = false;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        const uint32_t sensor_faults =
            estimator_sensor_fault_flags(estimator, lane);
        if ((sensor_faults != 0U) || !output->lane_window[lane].gyro_valid)
            continue;

        const bool filter_fault =
            (estimator->hard_fault_flags[lane] &
             DUAL_IMU_ESTIMATOR_FILTER_FAULT) != 0U;
        const bool lane_has_complete_recovery_window =
            (lane_mask & (uint8_t)(1U << lane)) != 0U;
        if (filter_fault &&
            (!lane_has_complete_recovery_window ||
             !output->lane_window[lane].accel_valid ||
             estimator->lane_accel_fault[lane] ||
             !estimator_filter_recovery_window_is_new(
                 estimator, output, lane))) {
            continue;
        }

        const uint32_t source_accel_lane =
            lane_has_complete_recovery_window
                ? (uint32_t)lane
                : accel_lane;
        float bias_rad_s[3];
        staged_bias_preserved[lane] =
            estimator_get_recoverable_bias(&staged[lane], bias_rad_s);
        if (!estimator_reseed_tilt_preserving_heading(
                &staged[lane],
                estimator->stationary_accel_mean_mps2[source_accel_lane],
                reference_quaternion,
                bias_rad_s)) {
            continue;
        }
        staged_seeded[lane] = true;
        any_seeded = true;
    }
    if (!any_seeded)
        return false;

    memcpy(estimator->mekf, staged, sizeof(staged));
    memcpy(estimator->lane_seeded, staged_seeded, sizeof(staged_seeded));
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (staged_seeded[lane]) {
            estimator->hard_fault_flags[lane] &=
                ~DUAL_IMU_ESTIMATOR_FILTER_FAULT;
            if (!staged_bias_preserved[lane]) {
                estimator->lane_calibrated[lane] = false;
                estimator->zaru_accept_count[lane] = 0U;
                estimator->zaru_divergence_count[lane] = 0U;
            }
        }
    }
    /* Do NOT reset output_initialized here: dropping the output history would
     * republish the reseeded lane attitude as a step. Marking the rewrite
     * lets the output path bridge the jump through output_alignment and slew
     * it out instead. */
    estimator_note_attitude_rewrite(estimator,
                                    DUAL_IMU_ATTITUDE_REWRITE_LANE_BOTH,
                                    DUAL_IMU_ATTITUDE_REWRITE_REACQUIRE);
    /* Gravity can reconverge roll/pitch even when the separate heading
     * continuity warning remains sticky. */
    estimator_complete_post_impact_reacquire(estimator, output);
    return true;
}

static float estimator_accel_variance_scale(
    const dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output)
{
    const float rate_norm = estimator_vector_norm(output->angular_rate_rad_s);
    const float accel_norm = estimator_vector_norm(output->angular_accel_rad_s2);
    const float normalized_rate =
        rate_norm / estimator->config.accel_rate_variance_scale_rad_s;
    const float normalized_accel =
        accel_norm /
        estimator->config.accel_angular_accel_variance_scale_rad_s2;
    float scale = 1.0f + (normalized_rate * normalized_rate) +
                  (normalized_accel * normalized_accel);
    if ((output->lane_window[0].accel_valid) &&
        (output->lane_window[1].accel_valid)) {
        const float normalized_pair =
            output->accel_pair_residual_mps2 /
            estimator->config.stationary_accel_pair_limit_mps2;
        scale += normalized_pair * normalized_pair;
    }
    if (output->accel_inhibited)
        scale *= estimator->config.accel_inhibit_variance_scale;
    return fminf(DUAL_IMU_ESTIMATOR_MAX_SCALE, fmaxf(1.0f, scale));
}

static bool estimator_update_post_impact_gravity_trust(
    dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    const imu_preintegrated_window_t windows[DUAL_IMU_ESTIMATOR_LANE_COUNT],
    bool dynamic_accel_gate)
{
    if (!estimator->post_impact_reacquire_active) {
        estimator->post_impact_gravity_trusted = false;
        estimator->post_impact_gravity_trust_streak = 0U;
        return true;
    }

    bool candidate = !output->accel_inhibited && !dynamic_accel_gate;
    const float rate_norm = estimator_vector_norm(output->angular_rate_rad_s);
    candidate = candidate && isfinite(rate_norm) &&
        (rate_norm <= estimator->config.accel_recovery_max_rate_rad_s);

    /* Evidence is collected per lane, not from both lanes unconditionally.
     * BMI088's gyro clips at 2000 dps against ICM45686's 4000 dps, so a hand
     * impact in the 1960-3920 dps band latches a BMI-only saturation fault
     * while the ICM samples stay valid. Requiring every lane here let that
     * one-sided fault block the healthy lane's gravity recovery for seconds
     * (observed: 41 deg of tilt error held for ~7 s while stationary with
     * accel aiding nominally enabled). A faulted lane is excluded from the
     * evidence set instead; recovery proceeds on whatever lane is sound, and
     * an empty set is still no evidence. */
    uint8_t usable_mask = 0U;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (!estimator->lane_seeded[lane] ||
            (estimator->hard_fault_flags[lane] != 0U) ||
            estimator->lane_accel_fault[lane] || !windows[lane].accel_valid)
            continue;
        const float specific_force_norm = estimator_vector_norm(
            output->lane_specific_force_mps2[lane]);
        if (!isfinite(specific_force_norm) ||
            (fabsf(specific_force_norm - DUAL_IMU_ESTIMATOR_GRAVITY_MPS2) >
             estimator->config.accel_recovery_norm_tolerance_mps2)) {
            /* A lane that is healthy but reads a non-gravity field is live
             * evidence of motion, not an absent witness: reject outright. */
            candidate = false;
            break;
        }
        usable_mask |= (uint8_t)(1U << lane);
    }
    candidate = candidate && (usable_mask != 0U);

    /* The pair residual only means anything when both lanes are in the
     * evidence set; with one lane out it is stale or undefined. */
    if (usable_mask == 0x03U) {
        candidate = candidate && isfinite(output->accel_pair_residual_mps2) &&
            (output->accel_pair_residual_mps2 <=
             estimator->config.stationary_accel_pair_limit_mps2);
    }

    if (!candidate) {
        estimator->post_impact_gravity_trusted = false;
        estimator->post_impact_gravity_trust_streak = 0U;
        return false;
    }

    if (!estimator->post_impact_gravity_trusted) {
        if (estimator->post_impact_gravity_trust_streak < UINT16_MAX)
            estimator->post_impact_gravity_trust_streak++;
        if (estimator->post_impact_gravity_trust_streak >=
            estimator->config.post_impact_gravity_trust_windows) {
            estimator->post_impact_gravity_trusted = true;
        }
    }
    return estimator->post_impact_gravity_trusted;
}

static bool estimator_split_output_alignment(const float alignment[4],
                                             float yaw_alignment[4],
                                             float tilt_alignment[4])
{
    float canonical[4] = {
        alignment[0], alignment[1], alignment[2], alignment[3],
    };
    if (!estimator_quaternion_normalize(canonical))
        return false;
    if (canonical[0] < 0.0f) {
        for (size_t index = 0U; index < 4U; ++index)
            canonical[index] = -canonical[index];
    }

    /* output_alignment left-multiplies a body-to-world attitude.  Its
     * unobservable component is therefore the twist about world gravity. */
    const float yaw_norm = sqrtf((canonical[0] * canonical[0]) +
                                 (canonical[3] * canonical[3]));
    if (!isfinite(yaw_norm) || (yaw_norm <= 1.0e-7f))
        return false;

    yaw_alignment[0] = canonical[0] / yaw_norm;
    yaw_alignment[1] = 0.0f;
    yaw_alignment[2] = 0.0f;
    yaw_alignment[3] = canonical[3] / yaw_norm;

    float inverse_yaw[4];
    estimator_quaternion_inverse(yaw_alignment, inverse_yaw);
    estimator_quaternion_multiply(inverse_yaw, canonical, tilt_alignment);
    /* The exact twist-swing decomposition has no residual Z twist. */
    tilt_alignment[3] = 0.0f;
    return estimator_quaternion_normalize(tilt_alignment);
}

static bool estimator_output_alignment_has_tilt(const float alignment[4])
{
    float yaw_alignment[4];
    float tilt_alignment[4];
    if (!estimator_split_output_alignment(alignment,
                                          yaw_alignment,
                                          tilt_alignment))
        return true;

    return (fabsf(tilt_alignment[1]) > 1.0e-7f) ||
           (fabsf(tilt_alignment[2]) > 1.0e-7f);
}

static bool estimator_decay_output_alignment(dual_imu_estimator_t *estimator,
                                             float dt_s)
{
    float yaw_alignment[4];
    float tilt_alignment[4];
    if (!estimator_split_output_alignment(estimator->output_alignment,
                                          yaw_alignment,
                                          tilt_alignment))
        return true;

    const float vector_norm = sqrtf((tilt_alignment[1] * tilt_alignment[1]) +
                                    (tilt_alignment[2] * tilt_alignment[2]));
    const float angle = 2.0f * atan2f(vector_norm,
                                     fmaxf(0.0f, tilt_alignment[0]));
    const float maximum_step =
        estimator->config.output_alignment_slew_rad_s * dt_s;
    if (!isfinite(angle) || !isfinite(maximum_step) ||
        (maximum_step <= 0.0f))
        return vector_norm > 1.0e-7f;
    if ((angle <= maximum_step) || (vector_norm <= 1.0e-7f)) {
        memcpy(estimator->output_alignment, yaw_alignment,
               sizeof(estimator->output_alignment));
        return false;
    }

    const float remaining_angle = angle - maximum_step;
    const float vector_scale = sinf(0.5f * remaining_angle) / vector_norm;
    float remaining_tilt[4] = {
        cosf(0.5f * remaining_angle),
        tilt_alignment[1] * vector_scale,
        tilt_alignment[2] * vector_scale,
        0.0f,
    };
    estimator_quaternion_multiply(yaw_alignment,
                                  remaining_tilt,
                                  estimator->output_alignment);
    if (!estimator_quaternion_normalize(estimator->output_alignment)) {
        memcpy(estimator->output_alignment, yaw_alignment,
               sizeof(estimator->output_alignment));
        return false;
    }
    return true;
}

static float estimator_output_alignment_tilt_angle(
    const dual_imu_estimator_t *estimator)
{
    float yaw_alignment[4];
    float tilt_alignment[4];
    if (!estimator->output_initialized ||
        !estimator_split_output_alignment(estimator->output_alignment,
                                          yaw_alignment,
                                          tilt_alignment))
        return 0.0f;

    const float vector_norm = sqrtf((tilt_alignment[1] * tilt_alignment[1]) +
                                    (tilt_alignment[2] * tilt_alignment[2]));
    const float angle = 2.0f * atan2f(vector_norm,
                                      fmaxf(0.0f, tilt_alignment[0]));
    return isfinite(angle) ? angle : 0.0f;
}

static void estimator_update_output_alignment(dual_imu_estimator_t *estimator,
                                              imu_selector_lane_t selected_lane,
                                              const float lane_quaternion[4],
                                              const float corrected_delta_angle[3],
                                              float dt_s,
                                              bool attitude_discontinuity,
                                              bool *alignment_active)
{
    bool switched = false;
    bool tilt_alignment_active = false;
    if (!estimator->output_initialized) {
        memcpy(estimator->output_quaternion, lane_quaternion,
               sizeof(estimator->output_quaternion));
        estimator_quaternion_identity(estimator->output_alignment);
        estimator->output_initialized = true;
    } else if ((selected_lane != estimator->previous_selected_lane) ||
               attitude_discontinuity) {
        switched = true;
        float delta_quaternion[4];
        float propagated_output[4];
        if (estimator_rotation_vector_to_quaternion(corrected_delta_angle,
                                                    delta_quaternion)) {
            estimator_quaternion_multiply(estimator->output_quaternion,
                                          delta_quaternion,
                                          propagated_output);
            if (estimator_quaternion_normalize(propagated_output))
                memcpy(estimator->output_quaternion, propagated_output,
                       sizeof(propagated_output));
        }
        float inverse_lane[4];
        estimator_quaternion_inverse(lane_quaternion, inverse_lane);
        estimator_quaternion_multiply(estimator->output_quaternion,
                                      inverse_lane,
                                      estimator->output_alignment);
        if (!estimator_quaternion_normalize(estimator->output_alignment))
            estimator_quaternion_identity(estimator->output_alignment);
        tilt_alignment_active = estimator_output_alignment_has_tilt(
            estimator->output_alignment);
    }

    if (!switched && estimator->output_initialized)
        tilt_alignment_active =
            estimator_decay_output_alignment(estimator, dt_s);

    float aligned[4];
    estimator_quaternion_multiply(estimator->output_alignment,
                                  lane_quaternion,
                                  aligned);
    if (estimator_quaternion_normalize(aligned))
        memcpy(estimator->output_quaternion, aligned, sizeof(aligned));

    estimator->previous_selected_lane = selected_lane;
    if (alignment_active != NULL)
        *alignment_active = tilt_alignment_active;
}

imu_preintegrator_result_t dual_imu_estimator_process_next(
    dual_imu_estimator_t *estimator,
    uint64_t complete_through_us,
    dual_imu_estimator_output_t *output)
{
    if ((estimator == NULL) || !estimator->initialized || (output == NULL))
        return IMU_PREINTEGRATOR_ERROR;

    imu_preintegrated_window_t windows[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    imu_preintegrator_result_t results[DUAL_IMU_ESTIMATOR_LANE_COUNT];
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        results[lane] = imu_preintegrator_next_window_ready(
            &estimator->preintegrator[lane], complete_through_us);
    }
    if ((results[0] == IMU_PREINTEGRATOR_ERROR) ||
        (results[1] == IMU_PREINTEGRATOR_ERROR))
        return IMU_PREINTEGRATOR_ERROR;
    if ((results[0] == IMU_PREINTEGRATOR_NOT_READY) ||
        (results[1] == IMU_PREINTEGRATOR_NOT_READY))
        return IMU_PREINTEGRATOR_NOT_READY;

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        results[lane] = imu_preintegrator_next_window(
            &estimator->preintegrator[lane], complete_through_us, &windows[lane]);
        if (results[lane] != IMU_PREINTEGRATOR_WINDOW_READY)
            return IMU_PREINTEGRATOR_ERROR;
    }
    if ((windows[0].start_us != windows[1].start_us) ||
        (windows[0].end_us != windows[1].end_us))
        return IMU_PREINTEGRATOR_ERROR;

    memset(output, 0, sizeof(*output));
    output->start_us = windows[0].start_us;
    output->end_us = windows[0].end_us;
    memcpy(output->lane_window, windows, sizeof(windows));
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        output->accel_result[lane] = ATTITUDE_MEKF_ACCEL_REJECTED_INVALID_INPUT;
        output->zaru_result[lane] = ATTITUDE_MEKF_ZARU_REJECTED_INVALID_INPUT;
    }

    const float dt_s = (float)(output->end_us - output->start_us) *
                       DUAL_IMU_ESTIMATOR_US_TO_S;
    const float half_dt_s = 0.5f * dt_s;
    float bias_before[DUAL_IMU_ESTIMATOR_LANE_COUNT][3] = {{0.0f}};
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        estimator_window_rate(&windows[lane], output->lane_rate_rad_s[lane]);
        (void)attitude_mekf_get_bias(&estimator->mekf[lane], bias_before[lane]);
        if (!windows[lane].gyro_propagation_valid)
            memset(output->lane_rate_rad_s[lane], 0,
                   sizeof(output->lane_rate_rad_s[lane]));
    }

    uint32_t geometry_lane = DUAL_IMU_ESTIMATOR_LANE_COUNT;
    const uint32_t preferred_lane = estimator->selector.selected_lane;
    if ((preferred_lane < DUAL_IMU_ESTIMATOR_LANE_COUNT) &&
        windows[preferred_lane].gyro_valid &&
        estimator_raw_lane_usable_for_geometry(estimator, preferred_lane))
        geometry_lane = preferred_lane;
    const uint32_t lane_priority[DUAL_IMU_ESTIMATOR_LANE_COUNT] = {
        DUAL_IMU_ESTIMATOR_LANE_ICM45686,
        DUAL_IMU_ESTIMATOR_LANE_BMI088,
    };
    for (size_t index = 0U;
         (geometry_lane >= DUAL_IMU_ESTIMATOR_LANE_COUNT) &&
         (index < DUAL_IMU_ESTIMATOR_LANE_COUNT);
         ++index) {
        const uint32_t lane = lane_priority[index];
        if (windows[lane].gyro_valid &&
            estimator_raw_lane_usable_for_geometry(estimator, lane))
            geometry_lane = lane;
    }
    for (size_t index = 0U;
         (geometry_lane >= DUAL_IMU_ESTIMATOR_LANE_COUNT) &&
         (index < DUAL_IMU_ESTIMATOR_LANE_COUNT);
         ++index) {
        const uint32_t lane = lane_priority[index];
        if (windows[lane].gyro_propagation_valid &&
            estimator_raw_lane_usable_for_geometry(estimator, lane))
            geometry_lane = lane;
    }

    const bool geometry_valid =
        geometry_lane < DUAL_IMU_ESTIMATOR_LANE_COUNT;
    if (geometry_valid) {
        memcpy(output->angular_rate_rad_s,
               output->lane_rate_rad_s[geometry_lane],
               sizeof(output->angular_rate_rad_s));
    }

    output->accel_inhibited = estimator_interval_active_for_window(
        estimator->accel_inhibit_intervals,
        &estimator->accel_inhibit_interval_count,
        output->start_us, output->end_us, NULL);
    uint64_t impact_start_us = output->start_us;
    const bool impact_interval_active = estimator_interval_active_for_window(
        estimator->impact_intervals,
        &estimator->impact_interval_count,
        output->start_us, output->end_us, &impact_start_us);
    const bool impact_episode_started =
        impact_interval_active && !estimator->impact_interval_was_active;
    estimator->impact_interval_was_active = impact_interval_active;
    if (impact_episode_started) {
        /* A saturated/high-g episode makes common-mode gyro g-sensitivity
         * indistinguishable from real rotation with only two six-axis IMUs.
         * Keep propagating matching gyros for responsiveness, but force a
         * later gravity-based tilt reacquisition and report heading integrity
         * as unknown from the causal event timestamp. */
        estimator_mark_heading_continuity_lost(estimator, impact_start_us);
        estimator_enter_post_impact_reacquire(estimator);
    }

    const imu_angular_accel_estimator_t angular_accel_estimator_before =
        estimator->angular_accel_estimator;
    imu_angular_accel_estimator_t angular_accel_estimator_staged =
        angular_accel_estimator_before;
    if (!geometry_valid ||
        !imu_angular_accel_estimator_update(&angular_accel_estimator_staged,
                                            output->angular_rate_rad_s,
                                            dt_s,
                                            output->angular_accel_rad_s2))
        memset(output->angular_accel_rad_s2, 0,
               sizeof(output->angular_accel_rad_s2));
    if (!output->accel_inhibited)
        estimator->angular_accel_estimator = angular_accel_estimator_staged;

    float mean_angular_accel_rad_s2[3] = {0.0f};
    float corrected_second_moment[3][3] = {{0.0f}};
    bool geometry_statistics_valid = geometry_valid;
    if (geometry_statistics_valid) {
        for (size_t axis = 0U; axis < 3U; ++axis) {
            mean_angular_accel_rad_s2[axis] =
                (windows[geometry_lane].gyro_end_rad_s[axis] -
                 windows[geometry_lane].gyro_start_rad_s[axis]) /
                dt_s;
            if (!isfinite(mean_angular_accel_rad_s2[axis]))
                geometry_statistics_valid = false;
        }
        geometry_statistics_valid = geometry_statistics_valid &&
            estimator_bias_correct_second_moment(
                &windows[geometry_lane], bias_before[geometry_lane],
                corrected_second_moment);
    }

    /* Raw-mean rotation witness for gyro-blind windows. Computed before the
     * translation loop below because that loop clears accel_valid whenever
     * the geometry lane is missing -- exactly the both-gyros-unusable case
     * this witness serves (DUAL_FUSION_DESIGN.md §3.3(b)). */
    float unobserved_witness_rate_std_rad_s = 0.0f;
    float unobserved_witness_axis[3] = {0.0f, 0.0f, 0.0f};
    const bool unobserved_witness_valid =
        windows[0].accel_valid && windows[1].accel_valid &&
        estimator_baseline_rotation_witness(
            estimator,
            windows[DUAL_IMU_ESTIMATOR_LANE_BMI088].accel_mean_mps2,
            windows[DUAL_IMU_ESTIMATOR_LANE_ICM45686].accel_mean_mps2,
            &unobserved_witness_rate_std_rad_s,
            unobserved_witness_axis);

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (windows[lane].accel_valid && geometry_statistics_valid) {
            if (!imu_translate_mean_specific_force(
                    windows[lane].accel_mean_mps2,
                    &corrected_second_moment[0][0],
                    mean_angular_accel_rad_s2,
                    estimator->config.reference_to_sensor_m[lane],
                    output->lane_specific_force_mps2[lane]))
                windows[lane].accel_valid = false;
        } else if (windows[lane].accel_valid) {
            windows[lane].accel_valid = false;
        }
    }
    output->lane_window[0].accel_valid = windows[0].accel_valid;
    output->lane_window[1].accel_valid = windows[1].accel_valid;
    if (windows[0].accel_valid && windows[1].accel_valid)
        output->accel_pair_residual_mps2 =
            estimator_vector_distance(output->lane_specific_force_mps2[0],
                                      output->lane_specific_force_mps2[1]);

    const float rate_norm = estimator_vector_norm(output->angular_rate_rad_s);
    const float angular_accel_norm =
        estimator_vector_norm(output->angular_accel_rad_s2);
    const bool dynamic_accel_gate =
        (rate_norm > estimator->config.accel_update_max_rate_rad_s) ||
        (angular_accel_norm >
         estimator->config.accel_update_max_angular_accel_rad_s2);

    attitude_mekf_t mekf_before_gyro_propagation
        [DUAL_IMU_ESTIMATOR_LANE_COUNT];
    uint8_t lane_seeded_before_gyro_propagation_mask = 0U;
    uint8_t filter_fault_before_gyro_propagation_mask = 0U;
    if (output->accel_inhibited) {
        memcpy(mekf_before_gyro_propagation, estimator->mekf,
               sizeof(mekf_before_gyro_propagation));
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            const uint8_t lane_mask = (uint8_t)(1U << lane);
            if (estimator->lane_seeded[lane])
                lane_seeded_before_gyro_propagation_mask |= lane_mask;
            if ((estimator->hard_fault_flags[lane] &
                 DUAL_IMU_ESTIMATOR_FILTER_FAULT) != 0U) {
                filter_fault_before_gyro_propagation_mask |= lane_mask;
            }
        }
    }
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (estimator->lane_seeded[lane]) {
            bool aided = false;
            bool rotation_unobserved = false;
            if (!estimator_propagate_half_window(estimator,
                                                 (uint32_t)lane,
                                                 windows,
                                                 0U,
                                                 half_dt_s,
                                                 unobserved_witness_valid,
                                                 unobserved_witness_rate_std_rad_s,
                                                 unobserved_witness_axis,
                                                 &aided,
                                                 &rotation_unobserved) &&
                windows[lane].gyro_propagation_valid &&
                (estimator->hard_fault_flags[lane] == 0U)) {
                estimator_latch_filter_fault(estimator, lane,
                                             output->end_us);
            }
            output->lane_aided_propagation[lane] = aided;
            output->rotation_unobserved =
                output->rotation_unobserved || rotation_unobserved;
        }
    }
    estimator_record_unobserved_rotation(estimator, output);

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        const size_t other = 1U - lane;
        size_t seed_accel_lane = lane;
        if (!windows[seed_accel_lane].accel_valid ||
            estimator->lane_accel_fault[seed_accel_lane])
            seed_accel_lane = other;
        if (!estimator->lane_seeded[lane] &&
            windows[seed_accel_lane].accel_valid &&
            !estimator->lane_accel_fault[seed_accel_lane] &&
            (estimator_sensor_fault_flags(estimator, lane) == 0U) &&
            (((estimator->hard_fault_flags[lane] &
               DUAL_IMU_ESTIMATOR_FILTER_FAULT) == 0U) ||
             (windows[lane].gyro_valid && windows[lane].accel_valid &&
              !estimator->lane_accel_fault[lane] &&
              estimator_filter_recovery_window_is_new(
                  estimator, output, lane))) &&
            !output->accel_inhibited && !dynamic_accel_gate &&
            !estimator->post_impact_reacquire_active) {
            float recoverable_bias[3];
            const bool bias_preserved = estimator_get_recoverable_bias(
                &estimator->mekf[lane], recoverable_bias);
            if (!bias_preserved) {
                estimator->lane_calibrated[lane] = false;
                estimator->zaru_accept_count[lane] = 0U;
                estimator->zaru_divergence_count[lane] = 0U;
            }
            float reference_quaternion[4];
            estimator_quaternion_identity(reference_quaternion);
            if (estimator->lane_seeded[other] &&
                attitude_mekf_is_valid(&estimator->mekf[other])) {
                if (attitude_mekf_get_quaternion(&estimator->mekf[other],
                                                 reference_quaternion)) {
                    (void)estimator_quaternion_normalize(reference_quaternion);
                }
            }
            estimator->lane_seeded[lane] =
                estimator_reseed_tilt_preserving_heading(
                &estimator->mekf[lane],
                output->lane_specific_force_mps2[seed_accel_lane],
                reference_quaternion,
                recoverable_bias);
            if (estimator->lane_seeded[lane]) {
                estimator->hard_fault_flags[lane] &=
                    ~DUAL_IMU_ESTIMATOR_FILTER_FAULT;
                estimator_note_attitude_rewrite(
                    estimator, (uint32_t)lane,
                    DUAL_IMU_ATTITUDE_REWRITE_SEED);
            }
            output->lane_accel_aided[lane] =
                estimator->lane_seeded[lane] && (seed_accel_lane != lane);
        }
    }

    /* Cross-lane calibrator (DUAL_FUSION_DESIGN.md §3.1): learn the
     * deterministic BMI-vs-ICM gyro differences from the raw difference
     * channel. Windows under any suspicion machinery (inhibit/impact
     * interval, pending gyro-disagreement checkpoint, external sensor
     * faults) never enter: a shock's differential g-sensitivity or a lying
     * lane must not be learned as calibration. Window validity, the rate
     * admission and outlier gating are enforced inside the module. */
    if (!output->accel_inhibited && !impact_interval_active &&
        !estimator->impact_gyro_checkpoint_valid &&
        (estimator_sensor_fault_flags(estimator, 0U) == 0U) &&
        (estimator_sensor_fault_flags(estimator, 1U) == 0U)) {
        (void)cross_lane_calibrator_update(
            &estimator->cross_lane_calibrator,
            &windows[DUAL_IMU_ESTIMATOR_LANE_BMI088],
            &windows[DUAL_IMU_ESTIMATOR_LANE_ICM45686],
            dt_s);
    }
    const bool cross_lane_calibrated = cross_lane_calibrator_is_converged(
        &estimator->cross_lane_calibrator);
    output->cross_lane_calibrated = cross_lane_calibrated;
    const float effective_alignment_std_rad =
        cross_lane_calibrated ? estimator->config.alignment_std_calibrated_rad
                              : estimator->config.alignment_std_rad;
    const float effective_disagreement_fraction =
        cross_lane_calibrated
            ? estimator->config.gyro_disagreement_rate_fraction_calibrated
            : estimator->config.gyro_disagreement_rate_fraction;

    /* Select before any ZARU update so bias learning cannot hide this residual. */
    imu_selector_input_t selector_input;
    memset(&selector_input, 0, sizeof(selector_input));
    selector_input.residual_enabled = windows[0].gyro_valid &&
                                      windows[1].gyro_valid;
    selector_input.isolation_hint = estimator->isolation_hint;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        selector_input.lane[lane].window_valid =
            windows[lane].gyro_valid && estimator->lane_seeded[lane] &&
            attitude_mekf_is_valid(&estimator->mekf[lane]);
        selector_input.lane[lane].hard_fault_flags =
            estimator_sensor_fault_flags(estimator, lane);
        float lane_delta[3];
        memcpy(lane_delta, windows[lane].delta_angle_rad,
               sizeof(lane_delta));
        if (cross_lane_calibrated &&
            (lane == DUAL_IMU_ESTIMATOR_LANE_BMI088)) {
            /* Map the BMI window into the ICM gyro frame. The bias
             * difference stays in: this lane's own MEKF bias is subtracted
             * below, exactly as before. */
            const float rate_change[3] = {
                windows[lane].gyro_end_rad_s[0] -
                    windows[lane].gyro_start_rad_s[0],
                windows[lane].gyro_end_rad_s[1] -
                    windows[lane].gyro_start_rad_s[1],
                windows[lane].gyro_end_rad_s[2] -
                    windows[lane].gyro_start_rad_s[2],
            };
            float corrected_delta[3];
            if (cross_lane_calibrator_correct_bmi_delta_angle(
                    &estimator->cross_lane_calibrator,
                    lane_delta,
                    rate_change,
                    corrected_delta))
                memcpy(lane_delta, corrected_delta, sizeof(lane_delta));
        }
        for (size_t axis = 0U; axis < 3U; ++axis) {
            selector_input.lane[lane].delta_angle_rad[axis] =
                lane_delta[axis] - (bias_before[lane][axis] * dt_s);
        }
        estimator_selector_covariance(
            estimator,
            (uint32_t)lane,
            mean_angular_accel_rad_s2,
            selector_input.lane[lane].delta_angle_rad,
            dt_s,
            effective_alignment_std_rad,
            effective_disagreement_fraction,
            selector_input.lane[lane].covariance_rad2);
    }
    if (!imu_selector_update(&estimator->selector,
                             &selector_input,
                             &output->selector))
        return IMU_PREINTEGRATOR_ERROR;

    /* Accel inhibition removes the independent arbiter. Require consecutive
     * cross-lane NIS evidence before failing closed, then roll the full suspect
     * segment back to the state before its first window. */
    const bool impact_gyro_disagreement_evidence =
        output->accel_inhibited && output->selector.residual_valid &&
        (output->selector.hard_fault_mask == 0U) &&
        (output->selector.isolated_mask == 0U) &&
        (output->selector.usable_mask == 0x03U) &&
        isfinite(output->selector.residual_nis) &&
        (output->selector.residual_nis >
         estimator->selector.config.nis_enter_threshold);
    (void)estimator_handle_impact_gyro_disagreement(
        estimator, output, impact_gyro_disagreement_evidence,
        mekf_before_gyro_propagation,
        &angular_accel_estimator_before,
        lane_seeded_before_gyro_propagation_mask,
        filter_fault_before_gyro_propagation_mask);
    const bool impact_gyro_disagreement_pending =
        estimator->impact_gyro_checkpoint_valid;

    bool accel_health_evidence[DUAL_IMU_ESTIMATOR_LANE_COUNT] = {false, false};
    const bool post_impact_quality_pending =
        estimator->post_impact_reacquire_active;
    const bool post_impact_gravity_trusted =
        estimator_update_post_impact_gravity_trust(estimator, output, windows,
                                                   dynamic_accel_gate);
    const float accel_variance_scale = estimator_accel_variance_scale(estimator,
                                                                      output);
    /* dynamic_accel_gate is deliberately NOT a hard cutoff here. Above
     * accel_update_max_rate_rad_s the old code dropped gravity aiding
     * entirely, so a fast yaw sweep ran open-loop on gyro alone for as long
     * as it lasted; measured, that let tilt error grow to ~16 deg over 12 s
     * and it was only repaired afterwards by a one-shot alignment patch.
     * estimator_accel_variance_scale() already inflates R quadratically in
     * rate and angular acceleration, so a contaminated measurement is
     * down-weighted continuously rather than discarded, and the MEKF's NIS
     * gate still rejects one that is outright inconsistent. The gate is kept
     * hard for reseeding and for the post-impact trust evidence above, where
     * a contaminated gravity vector would be written into the state instead
     * of merely nudging it.
     *
     * accel_inhibited is soft here for the same reason and it matters more:
     * the motion guard holds it 100 ms past every disturbance, which across
     * real captures covered ~50% of all frames while the accelerometer was
     * still within 1 g of gravity. Those frames now arrive scaled by
     * accel_inhibit_variance_scale rather than being dropped. What actually
     * keeps a destroyed reading out is the MEKF norm gate, which an impact at
     * 15 g fails outright. Every path that WRITES attitude from the accel
     * (seed, reseed, accel recovery, post-impact trust) still tests
     * accel_inhibited directly and stays hard.
     *
     * post_impact_gravity_trusted, by contrast, STAYS hard, and the reason is
     * worth recording because softening it looks tempting: measured, 94.7% of
     * all frames still carrying more than 3 deg of error were sitting in this
     * state waiting for the streak. But right after an impact the covariance
     * is deliberately huge, so an inflated R does not hold the filter back --
     * it correctly trusts the measurement, and a sustained non-gravity
     * specific force is a measurement that lies. Down-weighting a lie the
     * filter is primed to believe does not work; refusing it does. What is
     * safe to shorten is how long the evidence must persist, which is
     * post_impact_gravity_trust_windows, not whether it is required. */
    const bool accel_updates_enabled = post_impact_gravity_trusted;
    output->post_impact_gravity_trusted =
        estimator->post_impact_gravity_trusted;
    output->post_impact_gravity_trust_streak =
        estimator->post_impact_gravity_trust_streak;
    if (accel_updates_enabled) {
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            if (estimator->lane_seeded[lane] && windows[lane].accel_valid) {
                accel_health_evidence[lane] =
                    estimator->hard_fault_flags[lane] == 0U;
                output->accel_result[lane] = attitude_mekf_update_accel(
                    &estimator->mekf[lane],
                    output->lane_specific_force_mps2[lane],
                    accel_variance_scale);
            }
        }
    }
    /* Reported as an outcome, not an intention. Aiding no longer switches
     * off, so "we decided not to update" is never true and would make the
     * flag a constant; what a reader needs to know is whether gravity
     * actually corrected anything this window, which is false when every
     * lane was unseeded, invalid, or had its measurement rejected. */
    output->gravity_aiding_inhibited = true;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (output->accel_result[lane] == ATTITUDE_MEKF_ACCEL_ACCEPTED)
            output->gravity_aiding_inhibited = false;
    }
    estimator_update_accel_health(estimator,
                                  output,
                                  accel_updates_enabled &&
                                      !post_impact_quality_pending,
                                  accel_health_evidence);
    if (accel_updates_enabled) {
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            const size_t other = 1U - lane;
            if (!estimator->lane_seeded[lane] ||
                (output->accel_result[lane] == ATTITUDE_MEKF_ACCEL_ACCEPTED) ||
                estimator->lane_accel_fault[other] ||
                !windows[other].accel_valid ||
                (output->accel_result[other] != ATTITUDE_MEKF_ACCEL_ACCEPTED) ||
                (windows[lane].accel_valid &&
                 !estimator->lane_accel_fault[lane]))
                continue;

            output->lane_accel_aided[lane] =
                attitude_mekf_update_accel(
                    &estimator->mekf[lane],
                    output->lane_specific_force_mps2[other],
                    accel_variance_scale) == ATTITUDE_MEKF_ACCEL_ACCEPTED;
        }
    }
    estimator_handle_accel_recovery(estimator, output, windows,
                                    accel_updates_enabled);

    /* Hidden-rotation trigger (DUAL_FUSION_DESIGN.md §3.3(b), FIX_PLAN §1's
     * "shock that never trips the motion guard" hole): with both gyros
     * valid, the lever-arm translation already removed the gyro-predicted
     * centripetal field from lane_specific_force_mps2, so a persistent pair
     * residual above the trigger witnesses rotation/vibration the gyros did
     * not capture. Inflate attitude covariance about the witnessed axes only
     * (axis component zero: no evidence there) and declare nothing about
     * heading -- this is honest-uncertainty widening, not impact machinery.
     * Runs after this window's gravity updates, whose pair-residual variance
     * scaling already down-weighted the contaminated measurement. */
    output->accel_pair_rotation_witness_active = false;
    if (!output->accel_inhibited && !impact_interval_active &&
        !impact_gyro_disagreement_pending &&
        !estimator->stationary_confirmed_latched &&
        windows[0].gyro_valid && windows[1].gyro_valid &&
        windows[0].accel_valid && windows[1].accel_valid &&
        (estimator->hard_fault_flags[0] == 0U) &&
        (estimator->hard_fault_flags[1] == 0U) &&
        !estimator->lane_accel_fault[0] &&
        !estimator->lane_accel_fault[1] &&
        isfinite(output->accel_pair_residual_mps2) &&
        (output->accel_pair_residual_mps2 >
         estimator->config.rotation_witness_trigger_residual_mps2)) {
        if (estimator->accel_pair_witness_streak < UINT16_MAX)
            estimator->accel_pair_witness_streak++;
    } else {
        estimator->accel_pair_witness_streak = 0U;
    }
    if (estimator->accel_pair_witness_streak >=
        estimator->config.rotation_witness_trigger_windows) {
        float hidden_rate_std_rad_s = 0.0f;
        float hidden_axis[3] = {0.0f, 0.0f, 0.0f};
        if (estimator_baseline_rotation_witness(
                estimator,
                output->lane_specific_force_mps2
                    [DUAL_IMU_ESTIMATOR_LANE_BMI088],
                output->lane_specific_force_mps2
                    [DUAL_IMU_ESTIMATOR_LANE_ICM45686],
                &hidden_rate_std_rad_s,
                hidden_axis)) {
            bool inflated = false;
            for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT;
                 ++lane) {
                if (!estimator->lane_seeded[lane])
                    continue;
                inflated = attitude_mekf_mark_rotation_unobserved_directional(
                               &estimator->mekf[lane],
                               0.0f,
                               hidden_rate_std_rad_s * dt_s,
                               hidden_axis) ||
                           inflated;
            }
            if (inflated) {
                output->accel_pair_rotation_witness_active = true;
                if (estimator->accel_pair_witness_count < UINT32_MAX)
                    estimator->accel_pair_witness_count++;
            }
        }
    }

    bool selected_lane_aiding_evidence = false;
    if (accel_updates_enabled) {
        const imu_selector_lane_t selected_lane =
            output->selector.selected_lane;
        if (selected_lane < IMU_SELECTOR_LANE_COUNT) {
            selected_lane_aiding_evidence =
                estimator->lane_seeded[selected_lane] &&
                ((output->accel_result[selected_lane] ==
                  ATTITUDE_MEKF_ACCEL_ACCEPTED) ||
                 output->lane_accel_aided[selected_lane]);
        }
    }
    /* Quality evidence for leaving reacquire is gathered per lane, for the
     * same reason as the gravity trust above: a lane held out by a one-sided
     * fault (BMI088 clipping at 2000 dps where ICM45686 clips at 4000) must
     * not be able to veto the healthy lane's recovery forever. The lane whose
     * attitude is actually published has to carry the evidence; the
     * cross-lane checks only apply when both lanes are in the evidence set,
     * since with one lane out they compare against nothing. */
    uint8_t quality_evidence_mask = 0U;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (estimator->lane_seeded[lane] &&
            (estimator->hard_fault_flags[lane] == 0U) &&
            !estimator->lane_accel_fault[lane] &&
            windows[lane].accel_valid &&
            (output->accel_result[lane] == ATTITUDE_MEKF_ACCEL_ACCEPTED))
            quality_evidence_mask |= (uint8_t)(1U << lane);
    }
    const imu_selector_lane_t quality_lane = output->selector.selected_lane;
    const bool quality_lane_has_evidence =
        (quality_lane < IMU_SELECTOR_LANE_COUNT)
            ? ((quality_evidence_mask & (uint8_t)(1U << quality_lane)) != 0U)
            : (quality_evidence_mask != 0U);
    const bool quality_cross_lane_evidence =
        (quality_evidence_mask != 0x03U) ||
        (isfinite(output->accel_pair_residual_mps2) &&
         (output->accel_pair_residual_mps2 <=
          estimator->config.stationary_accel_pair_limit_mps2) &&
         output->selector.residual_valid &&
         isfinite(output->selector.residual_nis) &&
         (output->selector.residual_nis <
          estimator->selector.config.nis_clear_threshold));
    const bool post_impact_quality_evidence =
        post_impact_quality_pending && accel_updates_enabled &&
        quality_lane_has_evidence && quality_cross_lane_evidence;
    const bool attitude_convergence_evidence =
        post_impact_quality_pending ? post_impact_quality_evidence
                                    : selected_lane_aiding_evidence;
    if (attitude_convergence_evidence) {
        estimator->attitude_aiding_miss_streak = 0U;
        if (!estimator->attitude_converged ||
            estimator->post_impact_reacquire_active) {
            if (estimator->attitude_convergence_streak < UINT16_MAX)
                estimator->attitude_convergence_streak++;
            if (estimator->attitude_convergence_streak >=
                estimator->config.attitude_convergence_windows) {
                if (estimator->post_impact_reacquire_active) {
                    estimator_complete_post_impact_reacquire(estimator,
                                                              output);
                } else {
                    estimator->attitude_converged = true;
                }
            }
        }
    } else if (estimator->post_impact_reacquire_active) {
        estimator->attitude_convergence_streak = 0U;
    } else {
        estimator->attitude_convergence_streak = 0U;
        if (estimator->attitude_aiding_miss_streak < UINT16_MAX)
            estimator->attitude_aiding_miss_streak++;
        if (estimator->attitude_aiding_miss_streak >=
            estimator->config.attitude_aiding_timeout_windows) {
            estimator->attitude_converged = false;
        }
    }

    const uint8_t available_stationary_lane_mask =
        estimator_stationary_available_lane_mask(estimator, output);
    if (output->accel_inhibited) {
        output->stationary_candidate = estimator_reject_stationary(
            estimator, DUAL_IMU_STATIONARY_REJECT_INHIBITED);
    } else {
        output->stationary_candidate =
            estimator_stationary_candidate(
                estimator, output, available_stationary_lane_mask);
    }
    if (output->stationary_candidate) {
        if (estimator->stationary_streak < UINT16_MAX)
            estimator->stationary_streak++;
        if (estimator->stationary_streak > estimator->stationary_max_streak)
            estimator->stationary_max_streak = estimator->stationary_streak;
    } else {
        estimator->stationary_streak = 0U;
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            if (!estimator->lane_calibrated[lane])
                estimator->zaru_accept_count[lane] = 0U;
            estimator->zaru_divergence_count[lane] = 0U;
        }
    }
    (void)estimator_reacquire_attitude(estimator, output);
    const bool zaru_allowed = estimator_update_stationary_confirmation(
        estimator, output, available_stationary_lane_mask);
    output->stationary_last_reject_reason =
        estimator->stationary_last_reject_reason;
    memcpy(output->stationary_temporal_gyro_variance_rad2_s2,
           estimator->stationary_temporal_gyro_variance_rad2_s2,
           sizeof(output->stationary_temporal_gyro_variance_rad2_s2));
    memcpy(output->stationary_temporal_accel_variance_m2_s4,
           estimator->stationary_temporal_accel_variance_m2_s4,
           sizeof(output->stationary_temporal_accel_variance_m2_s4));
    output->stationary_streak = estimator->stationary_streak;
    output->stationary_max_streak = estimator->stationary_max_streak;
    output->stationary_lane_mask = estimator->stationary_confirmed_latched
        ? estimator->stationary_confirmed_lane_mask
        : estimator->stationary_lane_mask;
    output->stationary_single_lane =
        (output->stationary_lane_mask != 0U) &&
        (output->stationary_lane_mask != 0x03U);
    output->stationary_confirmed =
        estimator->stationary_confirmed_latched;
    if (zaru_allowed) {
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            if ((estimator->stationary_confirmed_lane_mask &
                 (uint8_t)(1U << lane)) == 0U ||
                !estimator->lane_seeded[lane] ||
                !windows[lane].gyro_valid ||
                (estimator->hard_fault_flags[lane] != 0U)) {
                if (!estimator->lane_calibrated[lane])
                    estimator->zaru_accept_count[lane] = 0U;
                continue;
            }

            const float variance =
                estimator->config.zaru_rate_std_rad_s[lane] *
                estimator->config.zaru_rate_std_rad_s[lane];
            const float covariance[3][3] = {
                {variance, 0.0f, 0.0f},
                {0.0f, variance, 0.0f},
                {0.0f, 0.0f, variance},
            };
            float bounded_target_rad_s[3] = {0.0f};
            const float maximum_bias_correction_rad_s =
                estimator->config.zaru_bias_slew_limit_rad_s2 * dt_s;
            /* The dwell mean is the stationary pseudo-measurement; using the
             * noisy 2.5 ms window here would make the 0.05 dps convergence
             * criterion unattainable on the BMI lane. */
            output->zaru_result[lane] =
                attitude_mekf_update_zero_rate_bounded(
                    &estimator->mekf[lane],
                    estimator->stationary_gyro_mean_rad_s[lane],
                    covariance,
                    estimator->config.zaru_target_residual_limit_rad_s,
                    maximum_bias_correction_rad_s,
                    bounded_target_rad_s);
            float learned_bias_rad_s[3] = {0.0f};
            const bool learned_bias_valid = attitude_mekf_get_bias(
                &estimator->mekf[lane], learned_bias_rad_s);
            const float target_distance = learned_bias_valid
                ? estimator_vector_distance(bounded_target_rad_s,
                                            learned_bias_rad_s)
                : INFINITY;
            if (estimator->lane_calibrated[lane]) {
                const bool diverged =
                    (output->zaru_result[lane] !=
                     ATTITUDE_MEKF_ZARU_ACCEPTED) ||
                    !isfinite(target_distance) ||
                    (target_distance > estimator->config
                         .zaru_calibration_revoke_tolerance_rad_s);
                if (diverged) {
                    if (estimator->zaru_divergence_count[lane] < UINT16_MAX)
                        estimator->zaru_divergence_count[lane]++;
                    if (estimator->zaru_divergence_count[lane] >=
                        estimator->config.calibration_revoke_windows) {
                        estimator->lane_calibrated[lane] = false;
                        estimator->zaru_accept_count[lane] = 0U;
                        estimator->zaru_divergence_count[lane] = 0U;
                    }
                } else {
                    estimator->zaru_divergence_count[lane] = 0U;
                }
            } else if ((output->zaru_result[lane] ==
                        ATTITUDE_MEKF_ZARU_ACCEPTED) &&
                       isfinite(target_distance) &&
                       (target_distance < estimator->config
                            .zaru_calibration_tolerance_rad_s)) {
                    if (estimator->zaru_accept_count[lane] < UINT16_MAX)
                        estimator->zaru_accept_count[lane]++;
                    if (estimator->zaru_accept_count[lane] >=
                        estimator->config.calibration_accept_windows)
                        estimator->lane_calibrated[lane] = true;
            } else if (!estimator->lane_calibrated[lane]) {
                estimator->zaru_accept_count[lane] = 0U;
            }
        }
    }

    /* The window-mean accel/ZARU measurements are centered in time. */
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (!estimator->lane_seeded[lane] ||
            impact_gyro_disagreement_pending)
            continue;
        bool aided = false;
        bool rotation_unobserved = false;
        if (!estimator_propagate_half_window(estimator,
                                             (uint32_t)lane,
                                             windows,
                                             1U,
                                             half_dt_s,
                                             unobserved_witness_valid,
                                             unobserved_witness_rate_std_rad_s,
                                             unobserved_witness_axis,
                                             &aided,
                                             &rotation_unobserved) &&
            windows[lane].gyro_propagation_valid &&
            (estimator->hard_fault_flags[lane] == 0U)) {
            estimator_latch_filter_fault(estimator, lane,
                                         output->end_us);
        }
        output->lane_aided_propagation[lane] =
            output->lane_aided_propagation[lane] || aided;
        output->rotation_unobserved =
            output->rotation_unobserved || rotation_unobserved;
    }
    estimator_record_unobserved_rotation(estimator, output);
    if (!estimator->stationary_confirmed_latched) {
        output->stationary_confirmed = false;
        output->stationary_lane_mask = estimator->stationary_lane_mask;
        output->stationary_single_lane =
            (output->stationary_lane_mask != 0U) &&
            (output->stationary_lane_mask != 0x03U);
    }

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        output->lane_seeded[lane] = estimator->lane_seeded[lane];
        output->lane_calibrated[lane] = estimator->lane_calibrated[lane];
        (void)attitude_mekf_get_bias(&estimator->mekf[lane],
                                     output->lane_bias_rad_s[lane]);
        (void)attitude_mekf_get_quaternion(&estimator->mekf[lane],
                                           output->lane_quaternion[lane]);
    }
    output->attitude_converged = estimator->attitude_converged;
    output->heading_continuity_lost =
        estimator->heading_continuity_lost;
    output->attitude_aiding_stale =
        estimator->attitude_aiding_miss_streak >=
        estimator->config.attitude_aiding_timeout_windows;
    output->post_impact_reacquire_active =
        estimator->post_impact_reacquire_active;

    const imu_selector_lane_t selected = output->selector.selected_lane;
    if (!output->rotation_unobserved &&
        !impact_gyro_disagreement_pending &&
        (selected < IMU_SELECTOR_LANE_COUNT) &&
        estimator->lane_seeded[selected] &&
        attitude_mekf_is_valid(&estimator->mekf[selected])) {
        estimator_update_output_alignment(estimator,
                                          selected,
                                          output->lane_quaternion[selected],
                                          selector_input.lane[selected]
                                              .delta_angle_rad,
                                          dt_s,
                                          estimator->lane_attitude_discontinuity
                                              [selected],
                                          &output->output_alignment_active);
        /* Consumed: the published attitude is bridged; a later lane switch
         * re-bridges against the other lane's live quaternion anyway. */
        estimator->lane_attitude_discontinuity[0] = false;
        estimator->lane_attitude_discontinuity[1] = false;
        memcpy(output->quaternion, estimator->output_quaternion,
               sizeof(output->quaternion));
        estimator_quaternion_to_euler(output->quaternion, output->euler_rad);
        uint32_t accel_lane = selected;
        if (!windows[accel_lane].accel_valid ||
            estimator->lane_accel_fault[accel_lane])
            accel_lane = 1U - accel_lane;
        if (windows[accel_lane].accel_valid &&
            !estimator->lane_accel_fault[accel_lane]) {
            memcpy(output->specific_force_mps2,
                   output->lane_specific_force_mps2[accel_lane],
                   sizeof(output->specific_force_mps2));
            output->selected_accel_lane = (imu_selector_lane_t)accel_lane;
            output->specific_force_valid = true;
        } else {
            output->selected_accel_lane = IMU_SELECTOR_LANE_NONE;
        }
        for (size_t axis = 0U; axis < 3U; ++axis) {
            output->angular_rate_rad_s[axis] =
                output->lane_rate_rad_s[selected][axis] -
                output->lane_bias_rad_s[selected][axis];
        }
        output->output_valid = true;
    } else {
        estimator_quaternion_identity(output->quaternion);
    }

    output->attitude_rewrite_count = estimator->attitude_rewrite_count;
    output->attitude_rewrite_last_reason =
        estimator->attitude_rewrite_last_reason;
    output->attitude_rewrite_last_lane =
        estimator->attitude_rewrite_last_lane;
    output->impact_gyro_rollback_pending =
        estimator->impact_gyro_checkpoint_valid;
    output->output_alignment_tilt_rad =
        estimator_output_alignment_tilt_angle(estimator);

    estimator->windows_processed++;
    return IMU_PREINTEGRATOR_WINDOW_READY;
}
