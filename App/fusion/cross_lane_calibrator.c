#include "cross_lane_calibrator.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define CAL_DIM        CROSS_LANE_CALIBRATOR_STATE_DIM
#define CAL_MEAS_DIM   (3U)
#define CAL_PI_F       (3.14159265358979323846f)

/* State layout offsets. */
#define CAL_BIAS       (0U)
#define CAL_MISALIGN   (3U)
#define CAL_SCALE      (6U)
#define CAL_DELAY      (9U)

static bool cal_vector_is_finite(const float vector[3])
{
    return isfinite(vector[0]) && isfinite(vector[1]) && isfinite(vector[2]);
}

static float cal_vector_norm(const float vector[3])
{
    return sqrtf((vector[0] * vector[0]) + (vector[1] * vector[1]) +
                 (vector[2] * vector[2]));
}

/* Non-const matrix parameter: ISO C before C23 rejects passing float[3][3]
 * where const float[3][3] is expected. The function does not modify it. */
static bool cal_invert_3x3(float matrix[3][3], float inverse[3][3])
{
    const float a = matrix[0][0];
    const float b = matrix[0][1];
    const float c = matrix[0][2];
    const float d = matrix[1][0];
    const float e = matrix[1][1];
    const float f = matrix[1][2];
    const float g = matrix[2][0];
    const float h = matrix[2][1];
    const float i = matrix[2][2];

    const float co00 = (e * i) - (f * h);
    const float co01 = (f * g) - (d * i);
    const float co02 = (d * h) - (e * g);
    const float det = (a * co00) + (b * co01) + (c * co02);
    if (!isfinite(det) || (fabsf(det) < FLT_MIN))
        return false;

    const float inv_det = 1.0f / det;
    inverse[0][0] = co00 * inv_det;
    inverse[0][1] = ((c * h) - (b * i)) * inv_det;
    inverse[0][2] = ((b * f) - (c * e)) * inv_det;
    inverse[1][0] = co01 * inv_det;
    inverse[1][1] = ((a * i) - (c * g)) * inv_det;
    inverse[1][2] = ((c * d) - (a * f)) * inv_det;
    inverse[2][0] = co02 * inv_det;
    inverse[2][1] = ((b * g) - (a * h)) * inv_det;
    inverse[2][2] = ((a * e) - (b * d)) * inv_det;
    for (size_t row = 0U; row < 3U; ++row) {
        if (!cal_vector_is_finite(inverse[row]))
            return false;
    }
    return true;
}

void cross_lane_calibrator_default_config(
    cross_lane_calibrator_config_t *config)
{
    if (config == NULL)
        return;
    memset(config, 0, sizeof(*config));

    const float degree = CAL_PI_F / 180.0f;
    /* BMI088 0.014 + ICM45686 0.0038 dps/sqrt(Hz), summed in variance. */
    const float bmi_noise = 0.014f * degree;
    const float icm_noise = 0.0038f * degree;
    config->gyro_noise_psd_sum_rad2_s =
        (bmi_noise * bmi_noise) + (icm_noise * icm_noise);
    config->rate_noise_floor_rad_s = 0.03f * degree;
    /* BMI088 datasheet g-sensitivity bound 0.1 dps/g dominates the pair. */
    config->g_sensitivity_rad_s_per_mps2 = (0.1f * degree) / 9.80665f;

    config->bias_rw_psd_rad2_s = 2.0e-9f;
    config->misalignment_rw_psd_rad2_s = 1.0e-12f;
    config->scale_rw_psd_s = 1.0e-10f;
    config->delay_rw_psd_s2_s = 1.0e-14f;

    config->initial_bias_std_rad_s = 0.02f;
    config->initial_misalignment_std_rad = 0.02f;
    config->initial_scale_std = 0.02f;
    config->initial_delay_std_s = 5.0e-4f;

    /* Well below the BMI088 2000 dps full scale so its near-range scale
     * nonlinearity never enters the linear model. */
    config->max_rate_rad_s = 20.0f;
    config->min_dt_s = 1.0e-4f;
    config->max_dt_s = 0.05f;

    /* 99% chi-square gate, 3 DOF. */
    config->nis_gate = 11.34f;
    config->nis_reject_revoke_windows = 50U;

    config->converged_misalignment_std_rad = 0.05f * degree;
    config->converged_scale_std = 1.0e-3f;
    config->converged_delay_std_s = 20.0e-6f;
    config->converged_min_accepts = 400U;

    config->max_abs_bias_rad_s = 0.10f;
    config->max_abs_misalignment_rad = 3.0f * degree;
    config->max_abs_scale = 0.05f;
    config->max_abs_delay_s = 1.0e-3f;
}

static bool cal_config_is_valid(const cross_lane_calibrator_config_t *config)
{
    return (config != NULL) &&
           isfinite(config->gyro_noise_psd_sum_rad2_s) &&
           (config->gyro_noise_psd_sum_rad2_s > 0.0f) &&
           isfinite(config->rate_noise_floor_rad_s) &&
           (config->rate_noise_floor_rad_s >= 0.0f) &&
           isfinite(config->g_sensitivity_rad_s_per_mps2) &&
           (config->g_sensitivity_rad_s_per_mps2 >= 0.0f) &&
           isfinite(config->bias_rw_psd_rad2_s) &&
           (config->bias_rw_psd_rad2_s >= 0.0f) &&
           isfinite(config->misalignment_rw_psd_rad2_s) &&
           (config->misalignment_rw_psd_rad2_s >= 0.0f) &&
           isfinite(config->scale_rw_psd_s) &&
           (config->scale_rw_psd_s >= 0.0f) &&
           isfinite(config->delay_rw_psd_s2_s) &&
           (config->delay_rw_psd_s2_s >= 0.0f) &&
           isfinite(config->initial_bias_std_rad_s) &&
           (config->initial_bias_std_rad_s > 0.0f) &&
           isfinite(config->initial_misalignment_std_rad) &&
           (config->initial_misalignment_std_rad > 0.0f) &&
           isfinite(config->initial_scale_std) &&
           (config->initial_scale_std > 0.0f) &&
           isfinite(config->initial_delay_std_s) &&
           (config->initial_delay_std_s > 0.0f) &&
           isfinite(config->max_rate_rad_s) &&
           (config->max_rate_rad_s > 0.0f) &&
           isfinite(config->min_dt_s) && (config->min_dt_s > 0.0f) &&
           isfinite(config->max_dt_s) &&
           (config->max_dt_s > config->min_dt_s) &&
           isfinite(config->nis_gate) && (config->nis_gate > 0.0f) &&
           (config->nis_reject_revoke_windows > 0U) &&
           isfinite(config->converged_misalignment_std_rad) &&
           (config->converged_misalignment_std_rad > 0.0f) &&
           isfinite(config->converged_scale_std) &&
           (config->converged_scale_std > 0.0f) &&
           isfinite(config->converged_delay_std_s) &&
           (config->converged_delay_std_s > 0.0f) &&
           (config->converged_min_accepts > 0U) &&
           isfinite(config->max_abs_bias_rad_s) &&
           (config->max_abs_bias_rad_s > 0.0f) &&
           isfinite(config->max_abs_misalignment_rad) &&
           (config->max_abs_misalignment_rad > 0.0f) &&
           isfinite(config->max_abs_scale) &&
           (config->max_abs_scale > 0.0f) &&
           isfinite(config->max_abs_delay_s) &&
           (config->max_abs_delay_s > 0.0f);
}

bool cross_lane_calibrator_init(cross_lane_calibrator_t *calibrator,
                                const cross_lane_calibrator_config_t *config)
{
    if (calibrator == NULL)
        return false;

    cross_lane_calibrator_config_t resolved;
    if (config == NULL) {
        cross_lane_calibrator_default_config(&resolved);
        config = &resolved;
    }
    if (!cal_config_is_valid(config))
        return false;

    memset(calibrator, 0, sizeof(*calibrator));
    calibrator->config = *config;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        calibrator->covariance[CAL_BIAS + axis][CAL_BIAS + axis] =
            config->initial_bias_std_rad_s * config->initial_bias_std_rad_s;
        calibrator->covariance[CAL_MISALIGN + axis][CAL_MISALIGN + axis] =
            config->initial_misalignment_std_rad *
            config->initial_misalignment_std_rad;
        calibrator->covariance[CAL_SCALE + axis][CAL_SCALE + axis] =
            config->initial_scale_std * config->initial_scale_std;
    }
    calibrator->covariance[CAL_DELAY][CAL_DELAY] =
        config->initial_delay_std_s * config->initial_delay_std_s;
    calibrator->initialized = true;
    return true;
}

static void cal_add_process_noise(cross_lane_calibrator_t *calibrator,
                                  float dt_s)
{
    for (size_t axis = 0U; axis < 3U; ++axis) {
        calibrator->covariance[CAL_BIAS + axis][CAL_BIAS + axis] +=
            calibrator->config.bias_rw_psd_rad2_s * dt_s;
        calibrator->covariance[CAL_MISALIGN + axis][CAL_MISALIGN + axis] +=
            calibrator->config.misalignment_rw_psd_rad2_s * dt_s;
        calibrator->covariance[CAL_SCALE + axis][CAL_SCALE + axis] +=
            calibrator->config.scale_rw_psd_s * dt_s;
    }
    calibrator->covariance[CAL_DELAY][CAL_DELAY] +=
        calibrator->config.delay_rw_psd_s2_s * dt_s;
}

static void cal_refresh_convergence(cross_lane_calibrator_t *calibrator)
{
    if (calibrator->faulted ||
        (calibrator->diagnostics.accept_count <
         calibrator->config.converged_min_accepts)) {
        calibrator->converged = false;
        return;
    }
    const float misalign_limit =
        calibrator->config.converged_misalignment_std_rad *
        calibrator->config.converged_misalignment_std_rad;
    const float scale_limit = calibrator->config.converged_scale_std *
                              calibrator->config.converged_scale_std;
    const float delay_limit = calibrator->config.converged_delay_std_s *
                              calibrator->config.converged_delay_std_s;
    bool converged =
        calibrator->covariance[CAL_DELAY][CAL_DELAY] <= delay_limit;
    for (size_t axis = 0U; converged && (axis < 3U); ++axis) {
        converged =
            (calibrator->covariance[CAL_MISALIGN + axis]
                                   [CAL_MISALIGN + axis] <= misalign_limit) &&
            (calibrator->covariance[CAL_SCALE + axis][CAL_SCALE + axis] <=
             scale_limit);
    }
    calibrator->converged = converged;
}

static void cal_revoke_and_reopen(cross_lane_calibrator_t *calibrator)
{
    /* A sustained run of outliers means the model stopped matching reality
     * (temperature step, unnoticed mechanical event). Reopen a fraction of
     * the initial uncertainty so the filter can re-learn quickly instead of
     * averaging the change in over the full history. */
    const cross_lane_calibrator_config_t *config = &calibrator->config;
    const float reopen = 0.25f;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        calibrator->covariance[CAL_BIAS + axis][CAL_BIAS + axis] +=
            reopen * config->initial_bias_std_rad_s *
            config->initial_bias_std_rad_s;
        calibrator->covariance[CAL_MISALIGN + axis][CAL_MISALIGN + axis] +=
            reopen * config->initial_misalignment_std_rad *
            config->initial_misalignment_std_rad;
        calibrator->covariance[CAL_SCALE + axis][CAL_SCALE + axis] +=
            reopen * config->initial_scale_std * config->initial_scale_std;
    }
    calibrator->covariance[CAL_DELAY][CAL_DELAY] +=
        reopen * config->initial_delay_std_s * config->initial_delay_std_s;
    calibrator->converged = false;
    calibrator->nis_reject_streak = 0U;
    if (calibrator->diagnostics.convergence_revoke_count < UINT32_MAX)
        calibrator->diagnostics.convergence_revoke_count++;
}

static bool cal_state_within_limits(const cross_lane_calibrator_t *calibrator)
{
    const cross_lane_calibrator_config_t *config = &calibrator->config;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        if ((fabsf(calibrator->state[CAL_BIAS + axis]) >
             config->max_abs_bias_rad_s) ||
            (fabsf(calibrator->state[CAL_MISALIGN + axis]) >
             config->max_abs_misalignment_rad) ||
            (fabsf(calibrator->state[CAL_SCALE + axis]) >
             config->max_abs_scale))
            return false;
    }
    return fabsf(calibrator->state[CAL_DELAY]) <= config->max_abs_delay_s;
}

cross_lane_calibrator_result_t cross_lane_calibrator_update(
    cross_lane_calibrator_t *calibrator,
    const imu_preintegrated_window_t *bmi_window,
    const imu_preintegrated_window_t *icm_window,
    float dt_s)
{
    if ((calibrator == NULL) || !calibrator->initialized)
        return CROSS_LANE_CALIBRATOR_REJECTED_INVALID_INPUT;

    cross_lane_calibrator_diagnostics_t *diag = &calibrator->diagnostics;
    if (diag->update_count < UINT32_MAX)
        diag->update_count++;

    if (calibrator->faulted) {
        diag->last_result = CROSS_LANE_CALIBRATOR_FAULT_LIMIT;
        return CROSS_LANE_CALIBRATOR_FAULT_LIMIT;
    }

    if ((bmi_window == NULL) || (icm_window == NULL) ||
        !isfinite(dt_s) || (dt_s < calibrator->config.min_dt_s) ||
        (dt_s > calibrator->config.max_dt_s) ||
        !bmi_window->gyro_valid || !icm_window->gyro_valid ||
        !bmi_window->accel_valid || !icm_window->accel_valid ||
        (bmi_window->start_us != icm_window->start_us) ||
        (bmi_window->end_us != icm_window->end_us) ||
        !cal_vector_is_finite(bmi_window->delta_angle_rad) ||
        !cal_vector_is_finite(icm_window->delta_angle_rad) ||
        !cal_vector_is_finite(bmi_window->gyro_mean_rad_s) ||
        !cal_vector_is_finite(icm_window->gyro_mean_rad_s) ||
        !cal_vector_is_finite(icm_window->gyro_start_rad_s) ||
        !cal_vector_is_finite(icm_window->gyro_end_rad_s) ||
        !cal_vector_is_finite(bmi_window->accel_mean_mps2) ||
        !cal_vector_is_finite(icm_window->accel_mean_mps2)) {
        if (diag->invalid_reject_count < UINT32_MAX)
            diag->invalid_reject_count++;
        diag->last_result = CROSS_LANE_CALIBRATOR_REJECTED_INVALID_INPUT;
        return CROSS_LANE_CALIBRATOR_REJECTED_INVALID_INPUT;
    }

    /* Time passes for the drifting parameters on every well-formed window,
     * admitted or not. */
    cal_add_process_noise(calibrator, dt_s);

    if ((cal_vector_norm(bmi_window->gyro_mean_rad_s) >
         calibrator->config.max_rate_rad_s) ||
        (cal_vector_norm(icm_window->gyro_mean_rad_s) >
         calibrator->config.max_rate_rad_s)) {
        if (diag->rate_reject_count < UINT32_MAX)
            diag->rate_reject_count++;
        diag->last_result = CROSS_LANE_CALIBRATOR_REJECTED_RATE;
        cal_refresh_convergence(calibrator);
        return CROSS_LANE_CALIBRATOR_REJECTED_RATE;
    }

    /* Rate-domain measurement: z = (dtheta_bmi - dtheta_icm) / dt. */
    float z[CAL_MEAS_DIM];
    float reference_rate[CAL_MEAS_DIM];
    float rate_change[CAL_MEAS_DIM];
    for (size_t axis = 0U; axis < 3U; ++axis) {
        z[axis] = (bmi_window->delta_angle_rad[axis] -
                   icm_window->delta_angle_rad[axis]) / dt_s;
        reference_rate[axis] = icm_window->delta_angle_rad[axis] / dt_s;
        rate_change[axis] = (icm_window->gyro_end_rad_s[axis] -
                             icm_window->gyro_start_rad_s[axis]) / dt_s;
    }

    /*
     * H rows (rate domain):
     *   db      -> I
     *   m       -> -[reference_rate]x        (H*m = m x reference_rate)
     *   s       -> diag(reference_rate)
     *   tau     -> -rate_change              (BMI lag shifts its window back)
     */
    float H[CAL_MEAS_DIM][CAL_DIM];
    memset(H, 0, sizeof(H));
    for (size_t axis = 0U; axis < 3U; ++axis) {
        H[axis][CAL_BIAS + axis] = 1.0f;
        H[axis][CAL_SCALE + axis] = reference_rate[axis];
        H[axis][CAL_DELAY] = -rate_change[axis];
    }
    /* m x omega = -[omega]x m. */
    H[0][CAL_MISALIGN + 1] = reference_rate[2];
    H[0][CAL_MISALIGN + 2] = -reference_rate[1];
    H[1][CAL_MISALIGN + 0] = -reference_rate[2];
    H[1][CAL_MISALIGN + 2] = reference_rate[0];
    H[2][CAL_MISALIGN + 0] = reference_rate[1];
    H[2][CAL_MISALIGN + 1] = -reference_rate[0];

    /* Measurement noise, inflated with the specific-force deviation so
     * differential g-sensitivity in high-g windows reads as noise, not bias. */
    const float gravity = 9.80665f;
    const float bmi_dev =
        fabsf(cal_vector_norm(bmi_window->accel_mean_mps2) - gravity);
    const float icm_dev =
        fabsf(cal_vector_norm(icm_window->accel_mean_mps2) - gravity);
    const float g_dev = fmaxf(bmi_dev, icm_dev);
    const float g_noise =
        calibrator->config.g_sensitivity_rad_s_per_mps2 * g_dev;
    const float floor_noise = calibrator->config.rate_noise_floor_rad_s;
    const float base_variance =
        (calibrator->config.gyro_noise_psd_sum_rad2_s / dt_s) +
        (floor_noise * floor_noise) + (g_noise * g_noise);

    /* S = H P H' + R. */
    float PHt[CAL_DIM][CAL_MEAS_DIM];
    for (size_t row = 0U; row < CAL_DIM; ++row) {
        for (size_t column = 0U; column < CAL_MEAS_DIM; ++column) {
            float sum = 0.0f;
            for (size_t k = 0U; k < CAL_DIM; ++k)
                sum += calibrator->covariance[row][k] * H[column][k];
            PHt[row][column] = sum;
        }
    }
    float S[CAL_MEAS_DIM][CAL_MEAS_DIM];
    for (size_t row = 0U; row < CAL_MEAS_DIM; ++row) {
        for (size_t column = 0U; column < CAL_MEAS_DIM; ++column) {
            float sum = 0.0f;
            for (size_t k = 0U; k < CAL_DIM; ++k)
                sum += H[row][k] * PHt[k][column];
            S[row][column] = sum;
        }
        S[row][row] += base_variance;
    }
    float S_inverse[CAL_MEAS_DIM][CAL_MEAS_DIM];
    if (!cal_invert_3x3(S, S_inverse)) {
        if (diag->numeric_fault_count < UINT32_MAX)
            diag->numeric_fault_count++;
        diag->last_result = CROSS_LANE_CALIBRATOR_NUMERIC_FAILURE;
        return CROSS_LANE_CALIBRATOR_NUMERIC_FAILURE;
    }

    /* Innovation and NIS gate. */
    float innovation[CAL_MEAS_DIM];
    for (size_t row = 0U; row < CAL_MEAS_DIM; ++row) {
        float predicted = 0.0f;
        for (size_t k = 0U; k < CAL_DIM; ++k)
            predicted += H[row][k] * calibrator->state[k];
        innovation[row] = z[row] - predicted;
    }
    float nis = 0.0f;
    for (size_t row = 0U; row < CAL_MEAS_DIM; ++row) {
        for (size_t column = 0U; column < CAL_MEAS_DIM; ++column)
            nis += innovation[row] * S_inverse[row][column] *
                   innovation[column];
    }
    diag->last_nis = nis;
    if (!isfinite(nis)) {
        if (diag->numeric_fault_count < UINT32_MAX)
            diag->numeric_fault_count++;
        diag->last_result = CROSS_LANE_CALIBRATOR_NUMERIC_FAILURE;
        return CROSS_LANE_CALIBRATOR_NUMERIC_FAILURE;
    }
    if (nis > calibrator->config.nis_gate) {
        if (diag->nis_reject_count < UINT32_MAX)
            diag->nis_reject_count++;
        if (calibrator->nis_reject_streak < UINT16_MAX)
            calibrator->nis_reject_streak++;
        if (calibrator->nis_reject_streak >=
            calibrator->config.nis_reject_revoke_windows)
            cal_revoke_and_reopen(calibrator);
        diag->last_result = CROSS_LANE_CALIBRATOR_REJECTED_NIS;
        cal_refresh_convergence(calibrator);
        return CROSS_LANE_CALIBRATOR_REJECTED_NIS;
    }
    calibrator->nis_reject_streak = 0U;

    /* Transactional Joseph-form update. */
    float state_backup[CAL_DIM];
    float covariance_backup[CAL_DIM][CAL_DIM];
    memcpy(state_backup, calibrator->state, sizeof(state_backup));
    memcpy(covariance_backup, calibrator->covariance,
           sizeof(covariance_backup));

    float K[CAL_DIM][CAL_MEAS_DIM];
    for (size_t row = 0U; row < CAL_DIM; ++row) {
        for (size_t column = 0U; column < CAL_MEAS_DIM; ++column) {
            float sum = 0.0f;
            for (size_t k = 0U; k < CAL_MEAS_DIM; ++k)
                sum += PHt[row][k] * S_inverse[k][column];
            K[row][column] = sum;
        }
    }
    for (size_t row = 0U; row < CAL_DIM; ++row) {
        float delta = 0.0f;
        for (size_t k = 0U; k < CAL_MEAS_DIM; ++k)
            delta += K[row][k] * innovation[k];
        calibrator->state[row] += delta;
    }

    float IKH[CAL_DIM][CAL_DIM];
    for (size_t row = 0U; row < CAL_DIM; ++row) {
        for (size_t column = 0U; column < CAL_DIM; ++column) {
            float sum = (row == column) ? 1.0f : 0.0f;
            for (size_t k = 0U; k < CAL_MEAS_DIM; ++k)
                sum -= K[row][k] * H[k][column];
            IKH[row][column] = sum;
        }
    }
    float temporary[CAL_DIM][CAL_DIM];
    for (size_t row = 0U; row < CAL_DIM; ++row) {
        for (size_t column = 0U; column < CAL_DIM; ++column) {
            float sum = 0.0f;
            for (size_t k = 0U; k < CAL_DIM; ++k)
                sum += IKH[row][k] * covariance_backup[k][column];
            temporary[row][column] = sum;
        }
    }
    for (size_t row = 0U; row < CAL_DIM; ++row) {
        for (size_t column = 0U; column < CAL_DIM; ++column) {
            float sum = 0.0f;
            for (size_t k = 0U; k < CAL_DIM; ++k)
                sum += temporary[row][k] * IKH[column][k];
            for (size_t k = 0U; k < CAL_MEAS_DIM; ++k)
                sum += K[row][k] * base_variance * K[column][k];
            calibrator->covariance[row][column] = sum;
        }
    }
    /* Symmetrize and validate. */
    bool numeric_ok = true;
    for (size_t row = 0U; numeric_ok && (row < CAL_DIM); ++row) {
        if (!isfinite(calibrator->state[row]))
            numeric_ok = false;
        for (size_t column = row; numeric_ok && (column < CAL_DIM);
             ++column) {
            const float symmetric = 0.5f *
                (calibrator->covariance[row][column] +
                 calibrator->covariance[column][row]);
            if (!isfinite(symmetric))
                numeric_ok = false;
            calibrator->covariance[row][column] = symmetric;
            calibrator->covariance[column][row] = symmetric;
        }
        if (numeric_ok && (calibrator->covariance[row][row] < 0.0f))
            numeric_ok = false;
    }
    if (!numeric_ok) {
        memcpy(calibrator->state, state_backup, sizeof(state_backup));
        memcpy(calibrator->covariance, covariance_backup,
               sizeof(covariance_backup));
        if (diag->numeric_fault_count < UINT32_MAX)
            diag->numeric_fault_count++;
        diag->last_result = CROSS_LANE_CALIBRATOR_NUMERIC_FAILURE;
        return CROSS_LANE_CALIBRATOR_NUMERIC_FAILURE;
    }

    if (!cal_state_within_limits(calibrator)) {
        memcpy(calibrator->state, state_backup, sizeof(state_backup));
        memcpy(calibrator->covariance, covariance_backup,
               sizeof(covariance_backup));
        calibrator->faulted = true;
        calibrator->converged = false;
        if (diag->limit_fault_count < UINT32_MAX)
            diag->limit_fault_count++;
        diag->last_result = CROSS_LANE_CALIBRATOR_FAULT_LIMIT;
        return CROSS_LANE_CALIBRATOR_FAULT_LIMIT;
    }

    if (diag->accept_count < UINT32_MAX)
        diag->accept_count++;
    cal_refresh_convergence(calibrator);
    diag->last_result = CROSS_LANE_CALIBRATOR_ACCEPTED;
    return CROSS_LANE_CALIBRATOR_ACCEPTED;
}

bool cross_lane_calibrator_correct_bmi_delta_angle(
    const cross_lane_calibrator_t *calibrator,
    const float bmi_delta_angle_rad[3],
    const float bmi_rate_change_rad_s[3],
    float corrected_delta_angle_rad[3])
{
    if ((calibrator == NULL) || !calibrator->initialized ||
        calibrator->faulted || !calibrator->converged ||
        (bmi_delta_angle_rad == NULL) || (bmi_rate_change_rad_s == NULL) ||
        (corrected_delta_angle_rad == NULL) ||
        !cal_vector_is_finite(bmi_delta_angle_rad) ||
        !cal_vector_is_finite(bmi_rate_change_rad_s))
        return false;

    const float *m = &calibrator->state[CAL_MISALIGN];
    const float *s = &calibrator->state[CAL_SCALE];
    const float tau = calibrator->state[CAL_DELAY];
    const float *d = bmi_delta_angle_rad;

    /* Invert the forward model to first order:
     * dtheta_true = dtheta_bmi - m x dtheta_bmi - diag(s)*dtheta_bmi
     *               + tau*delta_omega. The db*dt term is intentionally left
     * in: the consumer's own MEKF bias subtraction removes it. */
    const float cross[3] = {
        (m[1] * d[2]) - (m[2] * d[1]),
        (m[2] * d[0]) - (m[0] * d[2]),
        (m[0] * d[1]) - (m[1] * d[0]),
    };
    for (size_t axis = 0U; axis < 3U; ++axis) {
        corrected_delta_angle_rad[axis] =
            d[axis] - cross[axis] - (s[axis] * d[axis]) +
            (tau * bmi_rate_change_rad_s[axis]);
        if (!isfinite(corrected_delta_angle_rad[axis]))
            return false;
    }
    return true;
}

bool cross_lane_calibrator_is_converged(
    const cross_lane_calibrator_t *calibrator)
{
    return (calibrator != NULL) && calibrator->initialized &&
           !calibrator->faulted && calibrator->converged;
}

bool cross_lane_calibrator_get_std(
    const cross_lane_calibrator_t *calibrator,
    float bias_std_rad_s[3],
    float misalignment_std_rad[3],
    float scale_std[3],
    float *delay_std_s)
{
    if ((calibrator == NULL) || !calibrator->initialized)
        return false;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        if (bias_std_rad_s != NULL)
            bias_std_rad_s[axis] = sqrtf(fmaxf(
                calibrator->covariance[CAL_BIAS + axis][CAL_BIAS + axis],
                0.0f));
        if (misalignment_std_rad != NULL)
            misalignment_std_rad[axis] = sqrtf(fmaxf(
                calibrator->covariance[CAL_MISALIGN + axis]
                                      [CAL_MISALIGN + axis],
                0.0f));
        if (scale_std != NULL)
            scale_std[axis] = sqrtf(fmaxf(
                calibrator->covariance[CAL_SCALE + axis][CAL_SCALE + axis],
                0.0f));
    }
    if (delay_std_s != NULL)
        *delay_std_s = sqrtf(
            fmaxf(calibrator->covariance[CAL_DELAY][CAL_DELAY], 0.0f));
    return true;
}
