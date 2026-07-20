#include "cross_lane_calibrator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_PI_F      (3.14159265358979323846f)
#define TEST_WINDOW_S  (0.0025f)
#define TEST_GRAVITY   (9.80665f)

static unsigned test_failures;

#define TEST_EXPECT(condition)                                              \
    do {                                                                    \
        if (!(condition)) {                                                 \
            ++test_failures;                                                \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,         \
                    #condition);                                            \
        }                                                                   \
    } while (0)

static float degrees(float value)
{
    return value * TEST_PI_F / 180.0f;
}

typedef struct
{
    float bias_rad_s[3];
    float misalignment_rad[3];
    float scale[3];
    float delay_s;
} lane_truth_t;

/* Truth angular rate: multi-axis sinusoid so misalignment, scale and delay
 * all receive excitation. amplitude=0 with offset=0 yields rest windows. */
static void truth_rate(float time_s, float amplitude_rad_s,
                       float rate_rad_s[3])
{
    const float w0 = 2.0f * TEST_PI_F * 1.3f;
    const float w1 = 2.0f * TEST_PI_F * 2.1f;
    const float w2 = 2.0f * TEST_PI_F * 0.7f;
    rate_rad_s[0] = amplitude_rad_s * sinf(w0 * time_s);
    rate_rad_s[1] = 0.8f * amplitude_rad_s * sinf((w1 * time_s) + 0.5f);
    rate_rad_s[2] = 0.6f * amplitude_rad_s * sinf((w2 * time_s) + 1.1f);
}

static void make_window_pair(const lane_truth_t *truth,
                             uint32_t index,
                             float amplitude_rad_s,
                             imu_preintegrated_window_t *bmi,
                             imu_preintegrated_window_t *icm)
{
    const float start_s = (float)index * TEST_WINDOW_S;
    const float end_s = start_s + TEST_WINDOW_S;
    float rate_start[3];
    float rate_end[3];
    float rate_mid[3];
    truth_rate(start_s, amplitude_rad_s, rate_start);
    truth_rate(end_s, amplitude_rad_s, rate_end);
    truth_rate(start_s + (0.5f * TEST_WINDOW_S), amplitude_rad_s, rate_mid);

    memset(bmi, 0, sizeof(*bmi));
    memset(icm, 0, sizeof(*icm));
    const uint64_t start_us = (uint64_t)index * 2500U;
    bmi->start_us = start_us;
    icm->start_us = start_us;
    bmi->end_us = start_us + 2500U;
    icm->end_us = start_us + 2500U;

    float delta_true[3];
    float delta_omega[3];
    for (size_t axis = 0U; axis < 3U; ++axis) {
        /* Midpoint rule is exact enough; both lanes share the same truth so
         * only the cross-lane relation matters to the calibrator. */
        delta_true[axis] = rate_mid[axis] * TEST_WINDOW_S;
        delta_omega[axis] = rate_end[axis] - rate_start[axis];
        icm->delta_angle_rad[axis] = delta_true[axis];
        icm->gyro_mean_rad_s[axis] = rate_mid[axis];
        icm->gyro_start_rad_s[axis] = rate_start[axis];
        icm->gyro_end_rad_s[axis] = rate_end[axis];
    }
    const float *m = truth->misalignment_rad;
    const float cross[3] = {
        (m[1] * delta_true[2]) - (m[2] * delta_true[1]),
        (m[2] * delta_true[0]) - (m[0] * delta_true[2]),
        (m[0] * delta_true[1]) - (m[1] * delta_true[0]),
    };
    for (size_t axis = 0U; axis < 3U; ++axis) {
        bmi->delta_angle_rad[axis] =
            delta_true[axis] + cross[axis] +
            (truth->scale[axis] * delta_true[axis]) +
            (truth->bias_rad_s[axis] * TEST_WINDOW_S) -
            (truth->delay_s * delta_omega[axis]);
        bmi->gyro_mean_rad_s[axis] =
            bmi->delta_angle_rad[axis] / TEST_WINDOW_S;
        bmi->gyro_start_rad_s[axis] =
            rate_start[axis] + truth->bias_rad_s[axis];
        bmi->gyro_end_rad_s[axis] =
            rate_end[axis] + truth->bias_rad_s[axis];
    }
    bmi->accel_mean_mps2[2] = -TEST_GRAVITY;
    icm->accel_mean_mps2[2] = -TEST_GRAVITY;
    bmi->gyro_valid = true;
    icm->gyro_valid = true;
    bmi->gyro_propagation_valid = true;
    icm->gyro_propagation_valid = true;
    bmi->accel_valid = true;
    icm->accel_valid = true;
}

static uint32_t drive(cross_lane_calibrator_t *calibrator,
                      const lane_truth_t *truth,
                      uint32_t window_count,
                      float amplitude_rad_s,
                      uint32_t first_index)
{
    uint32_t accepted = 0U;
    for (uint32_t index = 0U; index < window_count; ++index) {
        imu_preintegrated_window_t bmi;
        imu_preintegrated_window_t icm;
        make_window_pair(truth, first_index + index, amplitude_rad_s,
                         &bmi, &icm);
        if (cross_lane_calibrator_update(calibrator, &bmi, &icm,
                                         TEST_WINDOW_S) ==
            CROSS_LANE_CALIBRATOR_ACCEPTED)
            ++accepted;
    }
    return accepted;
}

static void test_init_rejects_bad_config(void)
{
    cross_lane_calibrator_t calibrator;
    cross_lane_calibrator_config_t config;
    cross_lane_calibrator_default_config(&config);
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, &config));
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, NULL));

    config.nis_gate = -1.0f;
    TEST_EXPECT(!cross_lane_calibrator_init(&calibrator, &config));
    cross_lane_calibrator_default_config(&config);
    config.max_dt_s = config.min_dt_s;
    TEST_EXPECT(!cross_lane_calibrator_init(&calibrator, &config));
    TEST_EXPECT(!cross_lane_calibrator_init(NULL, NULL));
}

static void test_converges_to_known_parameters(void)
{
    lane_truth_t truth = {
        .bias_rad_s = {degrees(0.4f), degrees(-0.2f), degrees(0.3f)},
        .misalignment_rad = {degrees(0.5f), degrees(-0.3f), degrees(0.8f)},
        .scale = {0.008f, -0.005f, 0.011f},
        .delay_s = 150.0e-6f,
    };
    cross_lane_calibrator_t calibrator;
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, NULL));

    /* 6 s of moderate multi-axis motion at 400 Hz. */
    const uint32_t windows = 2400U;
    const uint32_t accepted = drive(&calibrator, &truth, windows, 1.5f, 0U);
    TEST_EXPECT(accepted >= (windows - (windows / 10U)));
    TEST_EXPECT(cross_lane_calibrator_is_converged(&calibrator));

    for (size_t axis = 0U; axis < 3U; ++axis) {
        TEST_EXPECT(fabsf(calibrator.state[axis] -
                          truth.bias_rad_s[axis]) < degrees(0.05f));
        TEST_EXPECT(fabsf(calibrator.state[3U + axis] -
                          truth.misalignment_rad[axis]) < degrees(0.05f));
        TEST_EXPECT(fabsf(calibrator.state[6U + axis] -
                          truth.scale[axis]) < 1.0e-3f);
    }
    TEST_EXPECT(fabsf(calibrator.state[9] - truth.delay_s) < 20.0e-6f);

    /* Correction maps the BMI window into the ICM frame up to the bias
     * term, which is deliberately preserved. */
    imu_preintegrated_window_t bmi;
    imu_preintegrated_window_t icm;
    make_window_pair(&truth, 12345U, 1.5f, &bmi, &icm);
    float rate_change[3];
    for (size_t axis = 0U; axis < 3U; ++axis)
        rate_change[axis] =
            bmi.gyro_end_rad_s[axis] - bmi.gyro_start_rad_s[axis];
    float corrected[3];
    TEST_EXPECT(cross_lane_calibrator_correct_bmi_delta_angle(
        &calibrator, bmi.delta_angle_rad, rate_change, corrected));
    for (size_t axis = 0U; axis < 3U; ++axis) {
        const float residual = corrected[axis] -
                               icm.delta_angle_rad[axis] -
                               (truth.bias_rad_s[axis] * TEST_WINDOW_S);
        const float raw_residual = bmi.delta_angle_rad[axis] -
                                   icm.delta_angle_rad[axis] -
                                   (truth.bias_rad_s[axis] * TEST_WINDOW_S);
        TEST_EXPECT(fabsf(residual) < degrees(0.002f));
        TEST_EXPECT(fabsf(residual) <= fabsf(raw_residual) + 1.0e-9f);
    }
}

static void test_rest_only_estimates_bias_but_never_converges(void)
{
    lane_truth_t truth = {
        .bias_rad_s = {degrees(0.6f), degrees(-0.4f), degrees(0.2f)},
        .misalignment_rad = {degrees(1.0f), 0.0f, 0.0f},
        .scale = {0.01f, 0.0f, 0.0f},
        .delay_s = 200.0e-6f,
    };
    cross_lane_calibrator_t calibrator;
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, NULL));

    (void)drive(&calibrator, &truth, 2400U, 0.0f, 0U);
    /* Bias difference is observable at rest... */
    for (size_t axis = 0U; axis < 3U; ++axis)
        TEST_EXPECT(fabsf(calibrator.state[axis] -
                          truth.bias_rad_s[axis]) < degrees(0.05f));
    /* ...but misalignment/scale/delay received no excitation, so the
     * calibration must not report convergence and the correction API must
     * refuse to run. */
    TEST_EXPECT(!cross_lane_calibrator_is_converged(&calibrator));
    float misalignment_std[3];
    TEST_EXPECT(cross_lane_calibrator_get_std(&calibrator, NULL,
                                              misalignment_std, NULL, NULL));
    TEST_EXPECT(misalignment_std[0] > degrees(0.5f));
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    float corrected[3];
    TEST_EXPECT(!cross_lane_calibrator_correct_bmi_delta_angle(
        &calibrator, zero, zero, corrected));
}

static void test_rate_admission_rejects_fast_windows(void)
{
    lane_truth_t truth;
    memset(&truth, 0, sizeof(truth));
    cross_lane_calibrator_t calibrator;
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, NULL));

    imu_preintegrated_window_t bmi;
    imu_preintegrated_window_t icm;
    make_window_pair(&truth, 0U, 60.0f, &bmi, &icm);
    TEST_EXPECT(cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                             TEST_WINDOW_S) ==
                CROSS_LANE_CALIBRATOR_REJECTED_RATE);
    TEST_EXPECT(calibrator.diagnostics.rate_reject_count == 1U);
    TEST_EXPECT(calibrator.diagnostics.accept_count == 0U);
}

static void test_invalid_windows_are_rejected(void)
{
    lane_truth_t truth;
    memset(&truth, 0, sizeof(truth));
    cross_lane_calibrator_t calibrator;
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, NULL));

    imu_preintegrated_window_t bmi;
    imu_preintegrated_window_t icm;
    make_window_pair(&truth, 0U, 1.0f, &bmi, &icm);
    bmi.gyro_valid = false;
    TEST_EXPECT(cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                             TEST_WINDOW_S) ==
                CROSS_LANE_CALIBRATOR_REJECTED_INVALID_INPUT);
    make_window_pair(&truth, 0U, 1.0f, &bmi, &icm);
    icm.accel_valid = false;
    TEST_EXPECT(cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                             TEST_WINDOW_S) ==
                CROSS_LANE_CALIBRATOR_REJECTED_INVALID_INPUT);
    make_window_pair(&truth, 0U, 1.0f, &bmi, &icm);
    icm.start_us += 1U;
    TEST_EXPECT(cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                             TEST_WINDOW_S) ==
                CROSS_LANE_CALIBRATOR_REJECTED_INVALID_INPUT);
    make_window_pair(&truth, 0U, 1.0f, &bmi, &icm);
    TEST_EXPECT(cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                             1.0f) ==
                CROSS_LANE_CALIBRATOR_REJECTED_INVALID_INPUT);
}

static void test_outlier_is_gated_and_streak_revokes(void)
{
    lane_truth_t truth = {
        .bias_rad_s = {0.0f, 0.0f, 0.0f},
        .misalignment_rad = {degrees(0.5f), 0.0f, 0.0f},
        .scale = {0.005f, 0.0f, 0.0f},
        .delay_s = 100.0e-6f,
    };
    cross_lane_calibrator_t calibrator;
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, NULL));
    (void)drive(&calibrator, &truth, 2400U, 1.5f, 0U);
    TEST_EXPECT(cross_lane_calibrator_is_converged(&calibrator));

    /* One wild window: rejected, state untouched. */
    imu_preintegrated_window_t bmi;
    imu_preintegrated_window_t icm;
    make_window_pair(&truth, 2400U, 1.5f, &bmi, &icm);
    float state_before[CROSS_LANE_CALIBRATOR_STATE_DIM];
    memcpy(state_before, calibrator.state, sizeof(state_before));
    bmi.delta_angle_rad[0] += degrees(5.0f);
    TEST_EXPECT(cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                             TEST_WINDOW_S) ==
                CROSS_LANE_CALIBRATOR_REJECTED_NIS);
    TEST_EXPECT(memcmp(state_before, calibrator.state,
                       sizeof(state_before)) == 0);
    TEST_EXPECT(cross_lane_calibrator_is_converged(&calibrator));

    /* A sustained outlier run must revoke convergence and reopen the
     * covariance so re-learning is possible. */
    const uint16_t revoke_windows =
        calibrator.config.nis_reject_revoke_windows;
    for (uint16_t index = 0U; index < revoke_windows; ++index) {
        make_window_pair(&truth, 2401U + index, 1.5f, &bmi, &icm);
        bmi.delta_angle_rad[0] += degrees(5.0f);
        (void)cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                           TEST_WINDOW_S);
    }
    TEST_EXPECT(!cross_lane_calibrator_is_converged(&calibrator));
    TEST_EXPECT(calibrator.diagnostics.convergence_revoke_count >= 1U);
}

static void test_hard_limit_latches_fault(void)
{
    /* A physically impossible 5 degree misalignment must latch the fault
     * before the state ever reports it as a usable calibration. */
    lane_truth_t truth = {
        .bias_rad_s = {0.0f, 0.0f, 0.0f},
        .misalignment_rad = {degrees(5.0f), 0.0f, 0.0f},
        .scale = {0.0f, 0.0f, 0.0f},
        .delay_s = 0.0f,
    };
    cross_lane_calibrator_t calibrator;
    cross_lane_calibrator_config_t config;
    cross_lane_calibrator_default_config(&config);
    /* Open the NIS gate so the huge residual reaches the state update. */
    config.nis_gate = 1.0e9f;
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, &config));

    (void)drive(&calibrator, &truth, 2400U, 1.5f, 0U);
    TEST_EXPECT(calibrator.faulted);
    TEST_EXPECT(calibrator.diagnostics.limit_fault_count >= 1U);
    TEST_EXPECT(!cross_lane_calibrator_is_converged(&calibrator));
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    float corrected[3];
    TEST_EXPECT(!cross_lane_calibrator_correct_bmi_delta_angle(
        &calibrator, zero, zero, corrected));
    /* Faulted calibrator refuses further updates. */
    imu_preintegrated_window_t bmi;
    imu_preintegrated_window_t icm;
    make_window_pair(&truth, 0U, 1.0f, &bmi, &icm);
    TEST_EXPECT(cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                             TEST_WINDOW_S) ==
                CROSS_LANE_CALIBRATOR_FAULT_LIMIT);
}

static void test_high_g_windows_do_not_poison_bias(void)
{
    /* Sustained 3 g linear acceleration with differential g-sensitivity
     * pollution: the inflated measurement noise must keep the polluted
     * windows from dragging the bias-difference estimate away. */
    lane_truth_t truth;
    memset(&truth, 0, sizeof(truth));
    truth.bias_rad_s[0] = degrees(0.3f);
    cross_lane_calibrator_t calibrator;
    TEST_EXPECT(cross_lane_calibrator_init(&calibrator, NULL));
    (void)drive(&calibrator, &truth, 1200U, 0.0f, 0U);
    const float settled = calibrator.state[0];
    TEST_EXPECT(fabsf(settled - truth.bias_rad_s[0]) < degrees(0.05f));

    for (uint32_t index = 0U; index < 400U; ++index) {
        imu_preintegrated_window_t bmi;
        imu_preintegrated_window_t icm;
        make_window_pair(&truth, 1200U + index, 0.0f, &bmi, &icm);
        bmi.accel_mean_mps2[0] = 3.0f * TEST_GRAVITY;
        icm.accel_mean_mps2[0] = 3.0f * TEST_GRAVITY;
        /* Fake g-sensitivity pollution on the difference channel. */
        bmi.delta_angle_rad[0] += degrees(0.2f) * TEST_WINDOW_S * 3.0f;
        (void)cross_lane_calibrator_update(&calibrator, &bmi, &icm,
                                           TEST_WINDOW_S);
    }
    TEST_EXPECT(fabsf(calibrator.state[0] - truth.bias_rad_s[0]) <
                degrees(0.15f));
}

int main(void)
{
    test_init_rejects_bad_config();
    test_converges_to_known_parameters();
    test_rest_only_estimates_bias_but_never_converges();
    test_rate_admission_rejects_fast_windows();
    test_invalid_windows_are_rejected();
    test_outlier_is_gated_and_streak_revokes();
    test_hard_limit_latches_fault();
    test_high_g_windows_do_not_poison_bias();

    if (test_failures != 0U) {
        fprintf(stderr, "%u cross_lane_calibrator test(s) failed\n",
                test_failures);
        return EXIT_FAILURE;
    }
    printf("all cross_lane_calibrator tests passed\n");
    return EXIT_SUCCESS;
}
