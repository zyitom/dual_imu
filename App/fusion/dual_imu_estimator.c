#include "dual_imu_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DUAL_IMU_ESTIMATOR_GRAVITY_MPS2 (9.80665f)
#define DUAL_IMU_ESTIMATOR_PI_F          (3.14159265358979323846f)
#define DUAL_IMU_ESTIMATOR_US_TO_S       (1.0e-6f)
#define DUAL_IMU_ESTIMATOR_MAX_SCALE     (100.0f)
#define DUAL_IMU_ESTIMATOR_FILTER_FAULT  (1UL << 31)

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
        !isfinite(config->rate_floor_std_rad_s) ||
        (config->rate_floor_std_rad_s <= 0.0f) ||
        !isfinite(config->output_alignment_slew_rad_s) ||
        (config->output_alignment_slew_rad_s <= 0.0f) ||
        !isfinite(config->stationary_gyro_limit_rad_s) ||
        (config->stationary_gyro_limit_rad_s <= 0.0f) ||
        !isfinite(config->stationary_accel_norm_tolerance_mps2) ||
        (config->stationary_accel_norm_tolerance_mps2 <= 0.0f) ||
        !isfinite(config->stationary_accel_pair_limit_mps2) ||
        (config->stationary_accel_pair_limit_mps2 <= 0.0f) ||
        (config->stationary_dwell_windows == 0U) ||
        (config->calibration_accept_windows == 0U) ||
        (config->accel_fault_enter_windows == 0U) ||
        (config->accel_fault_recovery_windows == 0U) ||
        !isfinite(config->accel_update_max_rate_rad_s) ||
        (config->accel_update_max_rate_rad_s <= 0.0f) ||
        !isfinite(config->accel_update_max_angular_accel_rad_s2) ||
        (config->accel_update_max_angular_accel_rad_s2 <= 0.0f) ||
        !isfinite(config->accel_rate_variance_scale_rad_s) ||
        (config->accel_rate_variance_scale_rad_s <= 0.0f) ||
        !isfinite(config->accel_angular_accel_variance_scale_rad_s2) ||
        (config->accel_angular_accel_variance_scale_rad_s2 <= 0.0f))
        return false;

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (!estimator_vector_is_finite(config->reference_to_sensor_m[lane]) ||
            !isfinite(config->zaru_rate_std_rad_s[lane]) ||
            (config->zaru_rate_std_rad_s[lane] <= 0.0f))
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
    config->rate_floor_std_rad_s = 0.03f * degree;
    config->output_alignment_slew_rad_s = 1.0f;
    config->zaru_rate_std_rad_s[DUAL_IMU_ESTIMATOR_LANE_BMI088] = 0.30f * degree;
    config->zaru_rate_std_rad_s[DUAL_IMU_ESTIMATOR_LANE_ICM45686] = 0.10f * degree;
    config->stationary_gyro_limit_rad_s = 0.03f;
    config->stationary_accel_norm_tolerance_mps2 = 0.30f;
    config->stationary_accel_pair_limit_mps2 = 0.35f;
    config->stationary_dwell_windows = 400U;
    config->calibration_accept_windows = 40U;
    config->accel_fault_enter_windows = 200U;
    config->accel_fault_recovery_windows = 400U;
    config->accel_update_max_rate_rad_s = 15.0f;
    config->accel_update_max_angular_accel_rad_s2 = 1000.0f;
    config->accel_rate_variance_scale_rad_s = 3.0f;
    config->accel_angular_accel_variance_scale_rad_s2 = 200.0f;
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
    if ((estimator != NULL) && estimator->initialized &&
        estimator_source_to_lane(source, &lane))
        estimator->hard_fault_flags[lane] =
            (estimator->hard_fault_flags[lane] &
             DUAL_IMU_ESTIMATOR_FILTER_FAULT) |
            hard_fault_flags;
}

static void estimator_update_accel_health(
    dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output,
    bool accel_updates_enabled)
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
        const bool lane_accepted =
            output->accel_result[lane] == ATTITUDE_MEKF_ACCEL_ACCEPTED;
        const bool other_accepted =
            output->accel_result[other] == ATTITUDE_MEKF_ACCEL_ACCEPTED;

        if (!lane_accepted && other_accepted) {
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

void dual_imu_estimator_inhibit_accel_until(dual_imu_estimator_t *estimator,
                                            uint64_t timestamp_us)
{
    if ((estimator != NULL) && estimator->initialized &&
        (timestamp_us > estimator->accel_inhibit_until_us))
        estimator->accel_inhibit_until_us = timestamp_us;
}

static void estimator_window_rate(const imu_preintegrated_window_t *window,
                                  float rate[3])
{
    for (size_t axis = 0U; axis < 3U; ++axis)
        rate[axis] = window->gyro_mean_rad_s[axis];
}

static bool estimator_propagate_half_window(
    dual_imu_estimator_t *estimator,
    uint32_t lane,
    const imu_preintegrated_window_t windows[DUAL_IMU_ESTIMATOR_LANE_COUNT],
    uint32_t half_index,
    float half_dt_s,
    bool *aided)
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
        if (!other_gyro_usable)
            return false;

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
                                          float covariance[3][3])
{
    const float dt_sq = dt_s * dt_s;
    const float delta_norm = estimator_vector_norm(delta_angle);
    const float delta_norm_sq = delta_norm * delta_norm;
    const float alignment_variance_scale =
        estimator->config.alignment_std_rad *
        estimator->config.alignment_std_rad;
    const float floor_delta = estimator->config.rate_floor_std_rad_s * dt_s;
    const float floor_variance = floor_delta * floor_delta;

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
                                floor_variance;
    }
}

static bool estimator_stationary_candidate(
    const dual_imu_estimator_t *estimator,
    const dual_imu_estimator_output_t *output)
{
    uint32_t usable_count = 0U;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if ((estimator->hard_fault_flags[lane] != 0U) ||
            !output->lane_window[lane].gyro_valid ||
            !output->lane_window[lane].accel_valid)
            continue;

        usable_count++;
        if ((estimator_vector_norm(output->lane_rate_rad_s[lane]) >
             estimator->config.stationary_gyro_limit_rad_s) ||
            (fabsf(estimator_vector_norm(output->lane_specific_force_mps2[lane]) -
                   DUAL_IMU_ESTIMATOR_GRAVITY_MPS2) >
             estimator->config.stationary_accel_norm_tolerance_mps2))
            return false;
    }

    if (usable_count == 0U)
        return false;
    if ((usable_count == DUAL_IMU_ESTIMATOR_LANE_COUNT) &&
        (output->accel_pair_residual_mps2 >
         estimator->config.stationary_accel_pair_limit_mps2))
        return false;
    if ((usable_count == DUAL_IMU_ESTIMATOR_LANE_COUNT) &&
        (!output->selector.residual_valid ||
         (output->selector.residual_nis >=
          estimator->config.selector.nis_clear_threshold)))
        return false;
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
    return fminf(DUAL_IMU_ESTIMATOR_MAX_SCALE, fmaxf(1.0f, scale));
}

static bool estimator_decay_output_alignment(dual_imu_estimator_t *estimator,
                                             float dt_s)
{
    float *const alignment = estimator->output_alignment;
    if (alignment[0] < 0.0f) {
        for (size_t index = 0U; index < 4U; ++index)
            alignment[index] = -alignment[index];
    }

    const float vector_norm = sqrtf((alignment[1] * alignment[1]) +
                                    (alignment[2] * alignment[2]) +
                                    (alignment[3] * alignment[3]));
    const float angle = 2.0f * atan2f(vector_norm,
                                     fmaxf(0.0f, alignment[0]));
    const float maximum_step =
        estimator->config.output_alignment_slew_rad_s * dt_s;
    if (!isfinite(angle) || !isfinite(maximum_step) ||
        (maximum_step <= 0.0f))
        return vector_norm > 1.0e-7f;
    if ((angle <= maximum_step) || (vector_norm <= 1.0e-7f)) {
        estimator_quaternion_identity(alignment);
        return false;
    }

    const float remaining_angle = angle - maximum_step;
    const float vector_scale = sinf(0.5f * remaining_angle) / vector_norm;
    alignment[0] = cosf(0.5f * remaining_angle);
    alignment[1] *= vector_scale;
    alignment[2] *= vector_scale;
    alignment[3] *= vector_scale;
    if (!estimator_quaternion_normalize(alignment)) {
        estimator_quaternion_identity(alignment);
        return false;
    }
    return true;
}

static void estimator_update_output_alignment(dual_imu_estimator_t *estimator,
                                              imu_selector_lane_t selected_lane,
                                              const float lane_quaternion[4],
                                              const float corrected_delta_angle[3],
                                              float dt_s,
                                              bool *alignment_active)
{
    bool switched = false;
    if (!estimator->output_initialized) {
        memcpy(estimator->output_quaternion, lane_quaternion,
               sizeof(estimator->output_quaternion));
        estimator_quaternion_identity(estimator->output_alignment);
        estimator->output_initialized = true;
    } else if (selected_lane != estimator->previous_selected_lane) {
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
    }

    if (!switched && estimator->output_initialized)
        (void)estimator_decay_output_alignment(estimator, dt_s);

    float aligned[4];
    estimator_quaternion_multiply(estimator->output_alignment,
                                  lane_quaternion,
                                  aligned);
    if (estimator_quaternion_normalize(aligned))
        memcpy(estimator->output_quaternion, aligned, sizeof(aligned));

    estimator->previous_selected_lane = selected_lane;
    if (alignment_active != NULL) {
        *alignment_active =
            (fabsf(estimator->output_alignment[1]) > 1.0e-7f) ||
            (fabsf(estimator->output_alignment[2]) > 1.0e-7f) ||
            (fabsf(estimator->output_alignment[3]) > 1.0e-7f);
    }
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
        (estimator->hard_fault_flags[preferred_lane] == 0U))
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
            (estimator->hard_fault_flags[lane] == 0U))
            geometry_lane = lane;
    }
    for (size_t index = 0U;
         (geometry_lane >= DUAL_IMU_ESTIMATOR_LANE_COUNT) &&
         (index < DUAL_IMU_ESTIMATOR_LANE_COUNT);
         ++index) {
        const uint32_t lane = lane_priority[index];
        if (windows[lane].gyro_propagation_valid &&
            (estimator->hard_fault_flags[lane] == 0U))
            geometry_lane = lane;
    }

    const bool geometry_valid =
        geometry_lane < DUAL_IMU_ESTIMATOR_LANE_COUNT;
    if (geometry_valid) {
        memcpy(output->angular_rate_rad_s,
               output->lane_rate_rad_s[geometry_lane],
               sizeof(output->angular_rate_rad_s));
    }
    if (!geometry_valid ||
        !imu_angular_accel_estimator_update(&estimator->angular_accel_estimator,
                                            output->angular_rate_rad_s,
                                            dt_s,
                                            output->angular_accel_rad_s2))
        memset(output->angular_accel_rad_s2, 0,
               sizeof(output->angular_accel_rad_s2));

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

    output->accel_inhibited = output->start_us < estimator->accel_inhibit_until_us;
    const float rate_norm = estimator_vector_norm(output->angular_rate_rad_s);
    const float angular_accel_norm =
        estimator_vector_norm(output->angular_accel_rad_s2);
    const bool dynamic_accel_gate =
        (rate_norm > estimator->config.accel_update_max_rate_rad_s) ||
        (angular_accel_norm >
         estimator->config.accel_update_max_angular_accel_rad_s2);

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        if (estimator->lane_seeded[lane]) {
            bool aided = false;
            if (!estimator_propagate_half_window(estimator,
                                                 (uint32_t)lane,
                                                 windows,
                                                 0U,
                                                 half_dt_s,
                                                 &aided) &&
                windows[lane].gyro_propagation_valid &&
                (estimator->hard_fault_flags[lane] == 0U))
                estimator->hard_fault_flags[lane] |=
                    DUAL_IMU_ESTIMATOR_FILTER_FAULT;
            output->lane_aided_propagation[lane] = aided;
        }
    }

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        const size_t other = 1U - lane;
        size_t seed_accel_lane = lane;
        if (!windows[seed_accel_lane].accel_valid ||
            estimator->lane_accel_fault[seed_accel_lane])
            seed_accel_lane = other;
        if (!estimator->lane_seeded[lane] &&
            windows[seed_accel_lane].accel_valid &&
            !estimator->lane_accel_fault[seed_accel_lane] &&
            !output->accel_inhibited && !dynamic_accel_gate) {
            const float zero_bias[3] = {0.0f, 0.0f, 0.0f};
            float seed_yaw_rad = 0.0f;
            if (estimator->lane_seeded[other] &&
                attitude_mekf_is_valid(&estimator->mekf[other])) {
                float other_quaternion[4];
                float other_euler[3];
                if (attitude_mekf_get_quaternion(&estimator->mekf[other],
                                                 other_quaternion)) {
                    estimator_quaternion_to_euler(other_quaternion, other_euler);
                    seed_yaw_rad = other_euler[2];
                }
            }
            estimator->lane_seeded[lane] = attitude_mekf_seed_from_accel(
                &estimator->mekf[lane],
                output->lane_specific_force_mps2[seed_accel_lane],
                seed_yaw_rad,
                zero_bias);
            output->lane_accel_aided[lane] =
                estimator->lane_seeded[lane] && (seed_accel_lane != lane);
        }
    }

    /* Select before any ZARU update so bias learning cannot hide this residual. */
    imu_selector_input_t selector_input;
    memset(&selector_input, 0, sizeof(selector_input));
    selector_input.residual_enabled = !output->accel_inhibited &&
                                      windows[0].gyro_valid &&
                                      windows[1].gyro_valid;
    selector_input.isolation_hint = estimator->isolation_hint;
    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        selector_input.lane[lane].window_valid =
            windows[lane].gyro_valid && estimator->lane_seeded[lane] &&
            attitude_mekf_is_valid(&estimator->mekf[lane]);
        selector_input.lane[lane].hard_fault_flags =
            estimator->hard_fault_flags[lane];
        for (size_t axis = 0U; axis < 3U; ++axis) {
            selector_input.lane[lane].delta_angle_rad[axis] =
                windows[lane].delta_angle_rad[axis] -
                (bias_before[lane][axis] * dt_s);
        }
        estimator_selector_covariance(
            estimator,
            (uint32_t)lane,
            mean_angular_accel_rad_s2,
            selector_input.lane[lane].delta_angle_rad,
            dt_s,
            selector_input.lane[lane].covariance_rad2);
    }
    if (!imu_selector_update(&estimator->selector,
                             &selector_input,
                             &output->selector))
        return IMU_PREINTEGRATOR_ERROR;

    const float accel_variance_scale = estimator_accel_variance_scale(estimator,
                                                                      output);
    if (!output->accel_inhibited && !dynamic_accel_gate) {
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            if (estimator->lane_seeded[lane] && windows[lane].accel_valid)
                output->accel_result[lane] = attitude_mekf_update_accel(
                    &estimator->mekf[lane],
                    output->lane_specific_force_mps2[lane],
                    accel_variance_scale);
        }
    }
    estimator_update_accel_health(estimator,
                                  output,
                                  !output->accel_inhibited &&
                                      !dynamic_accel_gate);
    if (!output->accel_inhibited && !dynamic_accel_gate) {
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

    output->stationary_candidate = !output->accel_inhibited &&
                                   estimator_stationary_candidate(estimator, output);
    const bool zaru_allowed = estimator->stationary_hint;
    if (output->stationary_candidate && zaru_allowed) {
        if (estimator->stationary_streak < UINT16_MAX)
            estimator->stationary_streak++;
    } else {
        estimator->stationary_streak = 0U;
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            if (!estimator->lane_calibrated[lane])
                estimator->zaru_accept_count[lane] = 0U;
        }
    }
    output->stationary_confirmed =
        estimator->stationary_streak >= estimator->config.stationary_dwell_windows;
    if (output->stationary_confirmed && zaru_allowed) {
        for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
            if (!estimator->lane_seeded[lane] ||
                !windows[lane].gyro_valid ||
                (estimator->hard_fault_flags[lane] != 0U))
                continue;

            const float variance =
                estimator->config.zaru_rate_std_rad_s[lane] *
                estimator->config.zaru_rate_std_rad_s[lane];
            const float covariance[3][3] = {
                {variance, 0.0f, 0.0f},
                {0.0f, variance, 0.0f},
                {0.0f, 0.0f, variance},
            };
            output->zaru_result[lane] = attitude_mekf_update_zero_rate(
                &estimator->mekf[lane], output->lane_rate_rad_s[lane], covariance);
            if (output->zaru_result[lane] == ATTITUDE_MEKF_ZARU_ACCEPTED) {
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
        if (!estimator->lane_seeded[lane])
            continue;
        bool aided = false;
        if (!estimator_propagate_half_window(estimator,
                                             (uint32_t)lane,
                                             windows,
                                             1U,
                                             half_dt_s,
                                             &aided) &&
            windows[lane].gyro_propagation_valid &&
            (estimator->hard_fault_flags[lane] == 0U))
            estimator->hard_fault_flags[lane] |=
                DUAL_IMU_ESTIMATOR_FILTER_FAULT;
        output->lane_aided_propagation[lane] =
            output->lane_aided_propagation[lane] || aided;
    }

    for (size_t lane = 0U; lane < DUAL_IMU_ESTIMATOR_LANE_COUNT; ++lane) {
        output->lane_seeded[lane] = estimator->lane_seeded[lane];
        output->lane_calibrated[lane] = estimator->lane_calibrated[lane];
        (void)attitude_mekf_get_bias(&estimator->mekf[lane],
                                     output->lane_bias_rad_s[lane]);
        (void)attitude_mekf_get_quaternion(&estimator->mekf[lane],
                                           output->lane_quaternion[lane]);
    }

    const imu_selector_lane_t selected = output->selector.selected_lane;
    if ((selected < IMU_SELECTOR_LANE_COUNT) &&
        estimator->lane_seeded[selected] &&
        attitude_mekf_is_valid(&estimator->mekf[selected])) {
        estimator_update_output_alignment(estimator,
                                          selected,
                                          output->lane_quaternion[selected],
                                          selector_input.lane[selected]
                                              .delta_angle_rad,
                                          dt_s,
                                          &output->output_alignment_active);
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

    estimator->windows_processed++;
    return IMU_PREINTEGRATOR_WINDOW_READY;
}
