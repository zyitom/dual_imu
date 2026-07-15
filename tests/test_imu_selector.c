#include "imu_selector.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_ASSERT(expression)                                                   \
    do {                                                                          \
        if (!(expression)) {                                                       \
            fprintf(stderr, "%s:%d: assertion failed: %s\n",                    \
                    __FILE__, __LINE__, #expression);                             \
            exit(EXIT_FAILURE);                                                    \
        }                                                                          \
    } while (false)

static imu_selector_config_t test_config(void)
{
    imu_selector_config_t config;
    imu_selector_default_config(&config);
    config.suspect_enter_windows = 2U;
    config.ambiguous_enter_windows = 4U;
    config.soft_fault_confirm_windows = 3U;
    config.clear_windows = 3U;
    config.isolated_recovery_windows = 3U;
    config.preferred_recovery_windows = 3U;
    return config;
}

static imu_selector_input_t healthy_input(void)
{
    imu_selector_input_t input;
    memset(&input, 0, sizeof(input));
    input.residual_enabled = true;

    for (uint32_t lane = 0U; lane < IMU_SELECTOR_LANE_COUNT; ++lane) {
        input.lane[lane].window_valid = true;
        for (uint32_t axis = 0U; axis < 3U; ++axis)
            input.lane[lane].covariance_rad2[axis][axis] = 0.5f;
    }
    return input;
}

static void update_or_fail(imu_selector_t *selector,
                           const imu_selector_input_t *input,
                           imu_selector_result_t *result)
{
    TEST_ASSERT(imu_selector_update(selector, input, result));
}

static void run_clear_windows(imu_selector_t *selector,
                              imu_selector_input_t *input,
                              imu_selector_result_t *result,
                              uint16_t windows)
{
    for (uint16_t window = 0U; window < windows; ++window)
        update_or_fail(selector, input, result);
}

static void test_configuration_validation(void)
{
    imu_selector_t selector;
    imu_selector_config_t config = test_config();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    config.nis_clear_threshold = config.nis_enter_threshold;
    TEST_ASSERT(!imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    config = test_config();
    config.preferred_recovery_windows = 0U;
    TEST_ASSERT(!imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    config = test_config();
    TEST_ASSERT(!imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_NONE));
}

static void test_healthy_startup_and_full_covariance_nis(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    input.lane[0].delta_angle_rad[0] = 1.0f;
    input.lane[0].delta_angle_rad[1] = 2.0f;
    input.lane[0].delta_angle_rad[2] = 2.0f;
    for (uint32_t lane = 0U; lane < IMU_SELECTOR_LANE_COUNT; ++lane) {
        input.lane[lane].covariance_rad2[0][0] = 1.0f;
        input.lane[lane].covariance_rad2[1][1] = 1.0f;
        input.lane[lane].covariance_rad2[2][2] = 0.5f;
        input.lane[lane].covariance_rad2[0][1] = 0.5f;
        input.lane[lane].covariance_rad2[1][0] = 0.5f;
    }
    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_1));

    run_clear_windows(&selector, &input, &result, config.clear_windows);
    TEST_ASSERT(result.state == IMU_SELECTOR_HEALTHY);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(result.residual_valid);
    TEST_ASSERT(fabsf(result.residual_nis - 6.0f) < 1.0e-4f);
}

static void test_hard_fault_switch_and_recovery_without_failback(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    run_clear_windows(&selector, &input, &result, config.clear_windows);
    TEST_ASSERT(result.state == IMU_SELECTOR_HEALTHY);

    input.lane[0].hard_fault_flags = 1U;
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.state == IMU_SELECTOR_FAULT);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(result.selection_changed);
    TEST_ASSERT((result.isolated_mask & IMU_SELECTOR_LANE_MASK(0U)) != 0U);

    input.lane[0].hard_fault_flags = 0U;
    run_clear_windows(&selector,
                      &input,
                      &result,
                      (uint16_t)(config.isolated_recovery_windows - 1U));
    TEST_ASSERT(result.state == IMU_SELECTOR_FAULT);
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.isolated_mask == 0U);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(!result.selection_changed);
}

static void test_unattributed_residual_becomes_ambiguous(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    run_clear_windows(&selector, &input, &result, config.clear_windows);
    input.lane[0].delta_angle_rad[0] = 10.0f;

    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.state == IMU_SELECTOR_HEALTHY);
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.state == IMU_SELECTOR_SUSPECT);
    run_clear_windows(&selector, &input, &result, 2U);
    TEST_ASSERT(result.state == IMU_SELECTOR_AMBIGUOUS);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_ASSERT(result.isolated_mask == 0U);

    input.lane[0].delta_angle_rad[0] = 0.0f;
    run_clear_windows(&selector,
                      &input,
                      &result,
                      (uint16_t)(config.clear_windows - 1U));
    TEST_ASSERT(result.state == IMU_SELECTOR_AMBIGUOUS);
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.state == IMU_SELECTOR_HEALTHY);
}

static void test_external_hint_can_isolate_one_lane(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    run_clear_windows(&selector, &input, &result, config.clear_windows);
    input.lane[0].delta_angle_rad[1] = 10.0f;
    input.isolation_hint = IMU_SELECTOR_HINT_LANE_0_BAD;
    run_clear_windows(&selector,
                      &input,
                      &result,
                      config.soft_fault_confirm_windows);

    TEST_ASSERT(result.state == IMU_SELECTOR_FAULT);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT((result.isolated_mask & IMU_SELECTOR_LANE_MASK(0U)) != 0U);
    TEST_ASSERT((result.reason_flags & IMU_SELECTOR_REASON_EXTERNAL_HINT) != 0U);
}

static void test_changing_hint_does_not_isolate(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    run_clear_windows(&selector, &input, &result, config.clear_windows);
    input.lane[0].delta_angle_rad[2] = 10.0f;
    for (uint16_t window = 0U; window < config.ambiguous_enter_windows; ++window) {
        input.isolation_hint = ((window & 1U) == 0U)
                                   ? IMU_SELECTOR_HINT_LANE_0_BAD
                                   : IMU_SELECTOR_HINT_LANE_1_BAD;
        update_or_fail(&selector, &input, &result);
    }

    TEST_ASSERT(result.state == IMU_SELECTOR_AMBIGUOUS);
    TEST_ASSERT(result.isolated_mask == 0U);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_0);
}

static void test_transient_invalid_input_uses_other_lane(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    run_clear_windows(&selector, &input, &result, config.clear_windows);
    input.lane[0].window_valid = false;
    update_or_fail(&selector, &input, &result);

    TEST_ASSERT(result.state == IMU_SELECTOR_SUSPECT);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(result.isolated_mask == 0U);
    TEST_ASSERT((result.reason_flags & IMU_SELECTOR_REASON_LANE_0_INPUT) != 0U);

    input.lane[0].window_valid = true;
    run_clear_windows(&selector,
                      &input,
                      &result,
                      (uint16_t)(config.preferred_recovery_windows - 1U));
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(!result.selection_changed);
    TEST_ASSERT(result.preferred_recovery_streak ==
                (uint16_t)(config.preferred_recovery_windows - 1U));

    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_ASSERT(result.selection_changed);
    TEST_ASSERT(result.preferred_recovery_streak == 0U);
}

static void test_preferred_recovery_interruptions_reset_dwell(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    run_clear_windows(&selector, &input, &result, config.clear_windows);

    input.lane[0].window_valid = false;
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(result.preferred_recovery_streak == 0U);

    input.lane[0].window_valid = true;
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.preferred_recovery_streak == 1U);

    /* NIS=9 lies between the clear and enter thresholds. It is valid
     * residual evidence, but not clear recovery evidence. */
    input.lane[0].delta_angle_rad[0] = 3.0f;
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.residual_valid);
    TEST_ASSERT(result.residual_nis > config.nis_clear_threshold);
    TEST_ASSERT(result.residual_nis < config.nis_enter_threshold);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(result.preferred_recovery_streak == 0U);

    input.lane[0].delta_angle_rad[0] = 0.0f;
    run_clear_windows(&selector,
                      &input,
                      &result,
                      (uint16_t)(config.preferred_recovery_windows - 1U));
    TEST_ASSERT(result.preferred_recovery_streak ==
                (uint16_t)(config.preferred_recovery_windows - 1U));

    input.lane[0].window_valid = false;
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(!result.residual_valid);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(result.preferred_recovery_streak == 0U);

    input.lane[0].window_valid = true;
    run_clear_windows(&selector,
                      &input,
                      &result,
                      (uint16_t)(config.preferred_recovery_windows - 1U));
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_1);
    TEST_ASSERT(result.preferred_recovery_streak ==
                (uint16_t)(config.preferred_recovery_windows - 1U));
    update_or_fail(&selector, &input, &result);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_0);
    TEST_ASSERT(result.selection_changed);
    TEST_ASSERT(result.preferred_recovery_streak == 0U);
}

static void test_both_hard_faults_remove_output(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    input.lane[0].hard_fault_flags = 1U;
    input.lane[1].hard_fault_flags = 2U;
    update_or_fail(&selector, &input, &result);

    TEST_ASSERT(result.state == IMU_SELECTOR_FAULT);
    TEST_ASSERT(result.selected_lane == IMU_SELECTOR_LANE_NONE);
    TEST_ASSERT(result.isolated_mask == 0x03U);
    TEST_ASSERT((result.reason_flags & IMU_SELECTOR_REASON_NO_OUTPUT) != 0U);
}

static void test_residual_inhibit_pauses_soft_scoring(void)
{
    imu_selector_t selector;
    imu_selector_result_t result;
    imu_selector_config_t config = test_config();
    imu_selector_input_t input = healthy_input();

    TEST_ASSERT(imu_selector_init(&selector, &config, IMU_SELECTOR_LANE_0));
    run_clear_windows(&selector, &input, &result, config.clear_windows);
    input.lane[0].delta_angle_rad[0] = 10.0f;
    input.isolation_hint = IMU_SELECTOR_HINT_LANE_0_BAD;
    input.residual_enabled = false;
    run_clear_windows(&selector, &input, &result, 20U);

    TEST_ASSERT(result.state == IMU_SELECTOR_HEALTHY);
    TEST_ASSERT(result.isolated_mask == 0U);
    TEST_ASSERT(result.mismatch_streak == 0U);
}

int main(void)
{
    test_configuration_validation();
    test_healthy_startup_and_full_covariance_nis();
    test_hard_fault_switch_and_recovery_without_failback();
    test_unattributed_residual_becomes_ambiguous();
    test_external_hint_can_isolate_one_lane();
    test_changing_hint_does_not_isolate();
    test_transient_invalid_input_uses_other_lane();
    test_preferred_recovery_interruptions_reset_dwell();
    test_both_hard_faults_remove_output();
    test_residual_inhibit_pauses_soft_scoring();
    puts("imu_selector_test: all tests passed");
    return EXIT_SUCCESS;
}
