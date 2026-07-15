#include "imu_motion_guard.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned int failures;

#define TEST_EXPECT(condition)                                                   \
    do {                                                                         \
        if (!(condition))                                                        \
        {                                                                        \
            (void)fprintf(stderr, "%s:%d: failed: %s\n",                      \
                          __FILE__, __LINE__, #condition);                       \
            failures++;                                                          \
        }                                                                        \
    } while (0)

static void test_saturated_or_untrusted_samples_reset_frozen_evidence(void)
{
    TEST_EXPECT(imu_motion_guard_sample_can_update_frozen_monitor(
        true, true, false));
    TEST_EXPECT(!imu_motion_guard_sample_can_update_frozen_monitor(
        true, true, true));
    TEST_EXPECT(!imu_motion_guard_sample_can_update_frozen_monitor(
        true, false, false));
    TEST_EXPECT(!imu_motion_guard_sample_can_update_frozen_monitor(
        false, true, false));
}

static imu_motion_guard_t initialized_guard(void)
{
    imu_motion_guard_t guard;
    TEST_EXPECT(imu_motion_guard_init(&guard, NULL));
    return guard;
}

static bool observe_accel_vector(imu_motion_guard_t *guard,
                                 uint8_t lane,
                                 uint64_t timestamp_us,
                                 bool trusted,
                                 const float accel_mps2[3])
{
    return imu_motion_guard_observe_accel_sample(
        guard, lane, timestamp_us, false, timestamp_us, trusted,
        accel_mps2);
}

static void gravity_vector(float gravity_scale, float output[3])
{
    output[0] = 0.0f;
    output[1] = 0.0f;
    output[2] = gravity_scale * IMU_MOTION_GUARD_GRAVITY_MPS2;
}

static void test_default_config_and_api_validation(void)
{
    imu_motion_guard_config_t config;
    imu_motion_guard_default_config(&config);
    TEST_EXPECT(config.gyro_debounce_us == 3000U);
    TEST_EXPECT(config.common_impact_window_us == 3000U);
    TEST_EXPECT(config.gyro_hard_latch_suppression_us == 20000U);
    TEST_EXPECT(config.accel_inhibit_us == 100000U);
    TEST_EXPECT(config.max_sample_gap_us == 2000U);

    imu_motion_guard_t guard;
    config.max_sample_gap_us = 0U;
    TEST_EXPECT(!imu_motion_guard_init(&guard, &config));
    TEST_EXPECT(!imu_motion_guard_observe_accel(&guard, 0U, 1U, false));
    TEST_EXPECT(!imu_motion_guard_init(NULL, NULL));

    guard = initialized_guard();
    TEST_EXPECT(!imu_motion_guard_observe_accel(&guard, 2U, 1U, false));
    TEST_EXPECT(!imu_motion_guard_observe_gyro(&guard, 2U, 1U, false));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 2U));
}

static void test_single_gyro_pulse_is_not_a_hard_fault(void)
{
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1000U, true));
    TEST_EXPECT(imu_motion_guard_gyro_saturated(&guard, 0U));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 0U));
    TEST_EXPECT(imu_motion_guard_gyro_hard_fault_mask(&guard) == 0U);
    TEST_EXPECT(!imu_motion_guard_gyro_hard_latch_suppressed(&guard, 1000U));
    TEST_EXPECT(!imu_motion_guard_accel_inhibited(&guard, 1000U));

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1500U, false));
    TEST_EXPECT(!imu_motion_guard_gyro_saturated(&guard, 0U));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 0U));
}

static void test_continuous_single_lane_saturation_is_debounced(void)
{
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 2000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 3999U, true));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 0U));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 4000U, true));
    TEST_EXPECT(imu_motion_guard_gyro_hard_fault_active(&guard, 0U));
    TEST_EXPECT(imu_motion_guard_gyro_hard_fault_mask(&guard) == 1U);
    TEST_EXPECT(!imu_motion_guard_gyro_hard_latch_suppressed(&guard, 4000U));

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 4500U, false));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 0U));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, 4500U, &diagnostics);
    TEST_EXPECT(diagnostics.counters.gyro_hard_fault_assertion_count[0] == 1U);
    TEST_EXPECT(diagnostics.counters.gyro_saturation_count[0] == 4U);
}

static void test_two_lane_common_impact_windows_and_extension(void)
{
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 10000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 12000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 1U, 13000U, true));
    TEST_EXPECT(imu_motion_guard_gyro_hard_latch_suppressed(&guard, 13000U));
    TEST_EXPECT(imu_motion_guard_accel_inhibited(&guard, 13000U));

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 14000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 14500U, true));
    TEST_EXPECT(imu_motion_guard_gyro_hard_latch_suppressed(&guard, 34499U));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_latch_suppressed(&guard, 34500U));
    TEST_EXPECT(imu_motion_guard_accel_inhibited(&guard, 114499U));
    TEST_EXPECT(!imu_motion_guard_accel_inhibited(&guard, 114500U));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, 14500U, &diagnostics);
    TEST_EXPECT(diagnostics.last_common_impact_us == 14500U);
    TEST_EXPECT(diagnostics.counters.gyro_common_impact_observation_count == 3U);
    TEST_EXPECT(diagnostics.counters.common_impact_episode_count == 1U);
    TEST_EXPECT(diagnostics.gyro_hard_latch_suppressed);
    TEST_EXPECT(diagnostics.accel_inhibited);
}

static void test_gyro_events_outside_common_window_are_independent(void)
{
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 1U, 4001U, true));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_latch_suppressed(&guard, 4001U));
    TEST_EXPECT(!imu_motion_guard_accel_inhibited(&guard, 4001U));
}

static void test_accel_saturation_is_immediate_common_impact(void)
{
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 50000U, true));
    TEST_EXPECT(imu_motion_guard_accel_saturated(&guard, 1U));
    TEST_EXPECT(imu_motion_guard_accel_inhibited(&guard, 50000U));
    TEST_EXPECT(imu_motion_guard_gyro_hard_latch_suppressed(&guard, 50000U));
    TEST_EXPECT(imu_motion_guard_accel_inhibit_until_us(&guard) == 150000U);
    uint64_t inhibit_start_us = 0U;
    uint64_t inhibit_end_us = 0U;
    TEST_EXPECT(imu_motion_guard_accel_inhibit_interval(
        &guard, &inhibit_start_us, &inhibit_end_us));
    TEST_EXPECT(inhibit_start_us == 50000U);
    TEST_EXPECT(inhibit_end_us == 150000U);

    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 50500U, true));
    TEST_EXPECT(imu_motion_guard_accel_inhibit_until_us(&guard) == 150500U);
    TEST_EXPECT(imu_motion_guard_accel_inhibit_interval(
        &guard, &inhibit_start_us, &inhibit_end_us));
    TEST_EXPECT(inhibit_start_us == 50000U);
    TEST_EXPECT(inhibit_end_us == 150500U);

    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 51000U, false));
    TEST_EXPECT(!imu_motion_guard_accel_saturated(&guard, 1U));
    TEST_EXPECT(imu_motion_guard_accel_inhibited(&guard, 51000U));
    TEST_EXPECT(imu_motion_guard_gyro_hard_latch_suppressed(&guard, 70000U));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_latch_suppressed(&guard, 70500U));
    TEST_EXPECT(imu_motion_guard_accel_inhibited(&guard, 150000U));
    TEST_EXPECT(!imu_motion_guard_accel_inhibited(&guard, 150500U));

    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 200000U, true));
    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, 200000U, &diagnostics);
    TEST_EXPECT(diagnostics.counters.accel_impact_observation_count == 2U);
    TEST_EXPECT(diagnostics.counters.common_impact_episode_count == 2U);

    guard.last_common_impact_us = UINT64_MAX - 1U;
    TEST_EXPECT(imu_motion_guard_accel_inhibit_until_us(&guard) == UINT64_MAX);
}

static void test_continuous_accel_clipping_extends_inhibit_until_after_exit(void)
{
    imu_motion_guard_t guard = initialized_guard();
    for (uint64_t timestamp_us = 1000U; timestamp_us <= 501000U;
         timestamp_us += 1000U) {
        TEST_EXPECT(imu_motion_guard_observe_accel(
            &guard, 0U, timestamp_us, true));
    }
    TEST_EXPECT(imu_motion_guard_accel_inhibit_until_us(&guard) == 601000U);
    uint64_t inhibit_start_us = 0U;
    uint64_t inhibit_end_us = 0U;
    TEST_EXPECT(imu_motion_guard_accel_inhibit_interval(
        &guard, &inhibit_start_us, &inhibit_end_us));
    TEST_EXPECT(inhibit_start_us == 1000U);
    TEST_EXPECT(inhibit_end_us == 601000U);
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 502000U, false));
    TEST_EXPECT(imu_motion_guard_accel_inhibited(&guard, 600999U));
    TEST_EXPECT(!imu_motion_guard_accel_inhibited(&guard, 601000U));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, 502000U, &diagnostics);
    TEST_EXPECT(diagnostics.common_impact_start_us == 1000U);
    TEST_EXPECT(diagnostics.last_common_impact_us == 501000U);
    TEST_EXPECT(diagnostics.counters.accel_impact_observation_count == 1U);
    TEST_EXPECT(diagnostics.counters.common_impact_episode_count == 1U);
}

static void test_common_impact_episode_boundaries_and_overflow(void)
{
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, 100000U, true));
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 1U, 101000U, true));
    /* This lane remains monotonic, but its event precedes the latest event
     * observed on the other lane. It still belongs to the same episode. */
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, 100500U, true));

    uint64_t inhibit_start_us = 0U;
    uint64_t inhibit_end_us = 0U;
    TEST_EXPECT(imu_motion_guard_accel_inhibit_interval(
        &guard, &inhibit_start_us, &inhibit_end_us));
    TEST_EXPECT(inhibit_start_us == 100000U);
    TEST_EXPECT(inhibit_end_us == 201000U);
    TEST_EXPECT(guard.counters.common_impact_episode_count == 1U);

    /* The half-open boundary starts a new episode. */
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 1U, 201000U, true));
    TEST_EXPECT(imu_motion_guard_accel_inhibit_interval(
        &guard, &inhibit_start_us, &inhibit_end_us));
    TEST_EXPECT(inhibit_start_us == 201000U);
    TEST_EXPECT(inhibit_end_us == 301000U);
    TEST_EXPECT(guard.counters.common_impact_episode_count == 2U);

    /* Late overlapping evidence cannot move the published episode start. */
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, 200500U, true));
    TEST_EXPECT(imu_motion_guard_accel_inhibit_interval(
        &guard, &inhibit_start_us, &inhibit_end_us));
    TEST_EXPECT(inhibit_start_us == 201000U);
    TEST_EXPECT(inhibit_end_us == 301000U);
    TEST_EXPECT(guard.counters.common_impact_episode_count == 2U);

    imu_motion_guard_config_t config;
    imu_motion_guard_default_config(&config);
    config.accel_inhibit_us = 100U;
    TEST_EXPECT(imu_motion_guard_init(&guard, &config));
    const uint64_t near_max_us = UINT64_MAX - 50U;
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, near_max_us, true));
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 1U, UINT64_MAX - 25U, true));
    TEST_EXPECT(imu_motion_guard_accel_inhibit_interval(
        &guard, &inhibit_start_us, &inhibit_end_us));
    TEST_EXPECT(inhibit_start_us == near_max_us);
    TEST_EXPECT(inhibit_end_us == UINT64_MAX);
    TEST_EXPECT(guard.last_common_impact_us == UINT64_MAX - 25U);
    TEST_EXPECT(guard.counters.common_impact_episode_count == 1U);
}

static void test_common_impact_does_not_hide_debounced_state(void)
{
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 2000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 3000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 4000U, true));
    TEST_EXPECT(imu_motion_guard_gyro_hard_fault_active(&guard, 0U));

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 1U, 4000U, true));
    TEST_EXPECT(imu_motion_guard_gyro_hard_fault_active(&guard, 0U));
    TEST_EXPECT(imu_motion_guard_gyro_hard_latch_suppressed(&guard, 4000U));
}

static void test_timestamp_discontinuities_reset_streaks(void)
{
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 2000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1500U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 3500U, true));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 0U));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 4500U, true));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 0U));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 6500U, true));
    TEST_EXPECT(imu_motion_guard_gyro_hard_fault_active(&guard, 0U));

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 9001U, true));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 0U));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 9001U, true));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_fault_active(&guard, 0U));

    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 1000U, false));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 4000U, false));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 3000U, false));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, 9001U, &diagnostics);
    TEST_EXPECT(diagnostics.counters.gyro_nonmonotonic_count[0] == 2U);
    TEST_EXPECT(diagnostics.counters.gyro_gap_count[0] == 1U);
    TEST_EXPECT(diagnostics.counters.accel_nonmonotonic_count[1] == 1U);
    TEST_EXPECT(diagnostics.counters.accel_gap_count[1] == 1U);
}

static void test_discontinuous_samples_are_not_cross_lane_impact_evidence(void)
{
    imu_motion_guard_t guard = initialized_guard();

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 10000U, false));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 9000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 1U, 9500U, true));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_latch_suppressed(&guard, 9500U));
    TEST_EXPECT(!imu_motion_guard_accel_inhibited(&guard, 9500U));

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 15000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 1U, 12000U, true));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_latch_suppressed(&guard, 15000U));

    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 20000U, false));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 19000U, true));
    TEST_EXPECT(!imu_motion_guard_accel_inhibited(&guard, 20000U));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 19500U, true));
    TEST_EXPECT(!imu_motion_guard_accel_inhibited(&guard, 20000U));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 20500U, true));
    TEST_EXPECT(imu_motion_guard_accel_inhibited(&guard, 20500U));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, 20500U, &diagnostics);
    TEST_EXPECT(diagnostics.counters.gyro_common_impact_observation_count == 0U);
    TEST_EXPECT(diagnostics.counters.accel_impact_observation_count == 1U);
    TEST_EXPECT(diagnostics.counters.common_impact_episode_count == 1U);
    TEST_EXPECT(diagnostics.counters.accel_nonmonotonic_count[0] == 2U);
}

static void test_recent_saturation_windows_are_aggregated_and_bounded(void)
{
    const uint64_t hold_us = UINT64_C(2000000);
    imu_motion_guard_t guard = initialized_guard();

    TEST_EXPECT(!imu_motion_guard_accel_saturation_recent(
        &guard, 1000U, hold_us));
    TEST_EXPECT(!imu_motion_guard_gyro_saturation_recent(
        &guard, 1000U, hold_us));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_recent(
        NULL, 1000U, hold_us));

    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 1000U, true));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_recent(
        &guard, 999U, hold_us));
    TEST_EXPECT(imu_motion_guard_accel_saturation_recent(
        &guard, 1000U, hold_us));
    TEST_EXPECT(imu_motion_guard_accel_saturation_recent(
        &guard, 2000999U, hold_us));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_recent(
        &guard, 2001000U, hold_us));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_recent(
        &guard, 1000U, 0U));

    /* A second lane and a repeated event both extend the aggregate window. */
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 1500U, true));
    TEST_EXPECT(imu_motion_guard_accel_saturation_recent(
        &guard, 2001499U, hold_us));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 3000U, true));
    TEST_EXPECT(imu_motion_guard_accel_saturation_recent(
        &guard, 2002999U, hold_us));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_recent(
        &guard, 2003000U, hold_us));

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 1U, 5000U, true));
    TEST_EXPECT(imu_motion_guard_gyro_saturation_recent(
        &guard, 2004999U, hold_us));
    TEST_EXPECT(!imu_motion_guard_gyro_saturation_recent(
        &guard, 2005000U, hold_us));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, 5000U, &diagnostics);
    TEST_EXPECT(diagnostics.accel_saturation_seen);
    TEST_EXPECT(diagnostics.gyro_saturation_seen);
    TEST_EXPECT(diagnostics.last_accel_saturation_us == 3000U);
    TEST_EXPECT(diagnostics.last_gyro_saturation_us == 5000U);
}

static void test_status_history_is_causal_across_separate_episodes(void)
{
    imu_motion_guard_config_t config;
    imu_motion_guard_default_config(&config);
    config.max_sample_gap_us = UINT64_C(3000000);
    imu_motion_guard_t guard;
    TEST_EXPECT(imu_motion_guard_init(&guard, &config));

    const uint64_t first_event_us = 1000U;
    const uint64_t second_event_us =
        first_event_us + IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US + 100U;
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, first_event_us, true));
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, first_event_us + 1000U, false));
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, second_event_us, true));

    TEST_EXPECT(imu_motion_guard_accel_saturation_status_at(
        &guard, first_event_us));
    TEST_EXPECT(imu_motion_guard_accel_saturation_status_at(
        &guard,
        first_event_us + IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US - 1U));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_status_at(
        &guard,
        first_event_us + IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US));
    TEST_EXPECT(imu_motion_guard_accel_saturation_status_at(
        &guard, second_event_us));
    TEST_EXPECT(!imu_motion_guard_gyro_saturation_status_at(
        &guard, second_event_us));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, second_event_us, &diagnostics);
    TEST_EXPECT(diagnostics.accel_saturation_history[0].valid);
    TEST_EXPECT(diagnostics.accel_saturation_history[0].start_us ==
                second_event_us);
    TEST_EXPECT(diagnostics.accel_saturation_history[1].valid);
    TEST_EXPECT(diagnostics.accel_saturation_history[1].start_us ==
                first_event_us);
}

static void test_status_history_normalizes_an_out_of_order_bridge(void)
{
    imu_motion_guard_config_t config;
    imu_motion_guard_default_config(&config);
    config.max_sample_gap_us = UINT64_C(5000000);
    imu_motion_guard_t guard;
    TEST_EXPECT(imu_motion_guard_init(&guard, &config));

    const uint64_t first_event_us = 1000U;
    const uint64_t bridge_event_us =
        first_event_us + IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US;
    const uint64_t last_event_us =
        bridge_event_us + IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US;

    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, first_event_us, true));
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 1U, last_event_us, true));
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, bridge_event_us, true));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, last_event_us, &diagnostics);
    TEST_EXPECT(diagnostics.accel_saturation_history[0].valid);
    TEST_EXPECT(diagnostics.accel_saturation_history[0].start_us ==
                first_event_us);
    TEST_EXPECT(diagnostics.accel_saturation_history[0].end_us ==
                last_event_us +
                    IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US - 1U);
    TEST_EXPECT(!diagnostics.accel_saturation_history[1].valid);
    TEST_EXPECT(!diagnostics.accel_saturation_history[2].valid);
    TEST_EXPECT(imu_motion_guard_accel_saturation_status_at(
        &guard, bridge_event_us - 1U));
    TEST_EXPECT(imu_motion_guard_accel_saturation_status_at(
        &guard, bridge_event_us));
}

static void test_three_episode_history_and_snapshot_are_causal(void)
{
    imu_motion_guard_config_t config;
    imu_motion_guard_default_config(&config);
    config.max_sample_gap_us = UINT64_C(10000000);
    imu_motion_guard_t guard;
    TEST_EXPECT(imu_motion_guard_init(&guard, &config));

    const uint64_t spacing_us =
        IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US + 100U;
    const uint64_t event_us[4] = {
        1000U,
        1000U + spacing_us,
        1000U + (2U * spacing_us),
        1000U + (3U * spacing_us),
    };
    for (size_t index = 0U; index < 3U; ++index) {
        TEST_EXPECT(imu_motion_guard_observe_gyro(
            &guard, 0U, event_us[index], true));
    }

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, event_us[2], &diagnostics);
    for (size_t index = 0U;
         index < IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT; ++index) {
        TEST_EXPECT(diagnostics.gyro_saturation_history[index].valid);
        TEST_EXPECT(diagnostics.gyro_saturation_history[index].start_us ==
                    event_us[2U - index]);
        TEST_EXPECT(imu_motion_guard_gyro_saturation_status_at(
            &guard, event_us[index]));
    }

    imu_motion_guard_saturation_snapshot_t snapshot;
    TEST_EXPECT(imu_motion_guard_export_saturation_snapshot(
        &guard, &snapshot));
    TEST_EXPECT(imu_motion_guard_snapshot_gyro_saturation_status_at(
        &snapshot, event_us[0]));
    TEST_EXPECT(!imu_motion_guard_snapshot_accel_saturation_status_at(
        &snapshot, event_us[0]));

    TEST_EXPECT(imu_motion_guard_observe_gyro(
        &guard, 0U, event_us[3], true));
    TEST_EXPECT(!imu_motion_guard_gyro_saturation_status_at(
        &guard, event_us[0]));
    TEST_EXPECT(imu_motion_guard_gyro_saturation_status_at(
        &guard, event_us[3]));
    TEST_EXPECT(imu_motion_guard_snapshot_gyro_saturation_status_at(
        &snapshot, event_us[0]));
    TEST_EXPECT(!imu_motion_guard_snapshot_gyro_saturation_status_at(
        &snapshot, event_us[3]));

    TEST_EXPECT(!imu_motion_guard_export_saturation_snapshot(NULL,
                                                              &snapshot));
    TEST_EXPECT(!snapshot.valid);
    TEST_EXPECT(!imu_motion_guard_export_saturation_snapshot(&guard, NULL));
    TEST_EXPECT(!imu_motion_guard_snapshot_gyro_saturation_status_at(
        NULL, event_us[0]));
}

static void test_forward_gap_records_saturation_but_nonmonotonic_does_not(void)
{
    const uint64_t hold_us = UINT64_C(2000);
    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 1000U, true));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 4001U, true));
    TEST_EXPECT(imu_motion_guard_accel_saturation_recent(
        &guard, 4001U, hold_us));
    TEST_EXPECT(imu_motion_guard_accel_saturation_recent(
        &guard, 6000U, hold_us));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_recent(
        &guard, 6001U, hold_us));
    TEST_EXPECT(imu_motion_guard_accel_saturation_status_at(&guard, 4001U));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 4001U, false));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 0U, 3500U, true));
    TEST_EXPECT(imu_motion_guard_accel_saturated(&guard, 0U));

    imu_motion_guard_diagnostics_t diagnostics;
    imu_motion_guard_get_diagnostics(&guard, 6001U, &diagnostics);
    TEST_EXPECT(diagnostics.last_accel_saturation_us == 4001U);
    TEST_EXPECT(diagnostics.counters.accel_gap_count[0] == 1U);
    TEST_EXPECT(diagnostics.counters.accel_nonmonotonic_count[0] == 2U);
    TEST_EXPECT(diagnostics.common_impact_start_us == 1000U);
    TEST_EXPECT(diagnostics.last_common_impact_us == 4001U);
    TEST_EXPECT(diagnostics.counters.common_impact_episode_count == 1U);

    guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1000U, true));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 4001U, true));
    TEST_EXPECT(imu_motion_guard_gyro_saturation_recent(
        &guard, 4001U, hold_us));
    TEST_EXPECT(imu_motion_guard_gyro_saturation_recent(
        &guard, 6000U, hold_us));
    TEST_EXPECT(!imu_motion_guard_gyro_saturation_recent(
        &guard, 6001U, hold_us));
    TEST_EXPECT(imu_motion_guard_gyro_saturation_status_at(&guard, 4001U));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 4001U, false));
    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 3500U, true));
    TEST_EXPECT(imu_motion_guard_gyro_saturated(&guard, 0U));
    imu_motion_guard_get_diagnostics(&guard, 6001U, &diagnostics);
    TEST_EXPECT(diagnostics.last_gyro_saturation_us == 4001U);
    TEST_EXPECT(diagnostics.counters.gyro_gap_count[0] == 1U);
    TEST_EXPECT(diagnostics.counters.gyro_nonmonotonic_count[0] == 2U);

    guard = initialized_guard();
    TEST_EXPECT(imu_motion_guard_observe_accel(
        &guard, 0U, UINT64_MAX - 1U, true));
    TEST_EXPECT(imu_motion_guard_accel_saturation_recent(
        &guard, UINT64_MAX - 1U, 1U));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_recent(
        &guard, UINT64_MAX, 1U));
}

static void test_subrange_dual_lane_shock_is_accel_only(void)
{
    imu_motion_guard_t guard = initialized_guard();
    float level[3];
    float shock[3];
    gravity_vector(1.0f, level);
    gravity_vector(2.0f, shock);

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 100000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 103000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 100625U, true, shock));
    TEST_EXPECT(!guard.accel_disturbance_valid);
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 103625U, true, shock));

    uint64_t start_us = 0U;
    uint64_t end_us = 0U;
    TEST_EXPECT(imu_motion_guard_accel_disturbance_interval(
        &guard, &start_us, &end_us));
    TEST_EXPECT(start_us == 100625U);
    TEST_EXPECT(end_us == 203625U);
    TEST_EXPECT(imu_motion_guard_accel_disturbance_inhibited(
        &guard, 103625U));
    TEST_EXPECT(!guard.common_impact_valid);
    TEST_EXPECT(!guard.accel_saturation_seen);
    TEST_EXPECT(!guard.gyro_saturation_seen);
    TEST_EXPECT(!imu_motion_guard_accel_saturation_status_at(
        &guard, 103625U));
    TEST_EXPECT(!imu_motion_guard_gyro_saturation_status_at(
        &guard, 103625U));
    TEST_EXPECT(!imu_motion_guard_accel_inhibit_interval(
        &guard, &start_us, &end_us));
    TEST_EXPECT(!imu_motion_guard_gyro_hard_latch_suppressed(
        &guard, 103625U));
    TEST_EXPECT(guard.counters.accel_subrange_common_observation_count == 1U);
    TEST_EXPECT(guard.counters.accel_disturbance_episode_count == 1U);
}

static void test_subrange_coincidence_boundaries_and_arrival_order(void)
{
    float level[3];
    float shock[3];
    gravity_vector(1.0f, level);
    gravity_vector(2.0f, shock);

    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 103000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 103625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 100000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 100625U, true, shock));
    TEST_EXPECT(guard.accel_disturbance_valid);
    TEST_EXPECT(guard.accel_disturbance_start_us == 100625U);
    TEST_EXPECT(guard.last_accel_disturbance_us == 103625U);

    guard = initialized_guard();
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 100000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 103001U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 100625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 103626U, true, shock));
    TEST_EXPECT(!guard.accel_disturbance_valid);
    TEST_EXPECT(guard.counters.accel_subrange_common_observation_count == 0U);
    TEST_EXPECT(guard.lane[0].accel_disturbance_candidate_armed);
    TEST_EXPECT(guard.lane[1].accel_disturbance_candidate_armed);
}

static void test_subrange_single_lane_moderate_does_not_disarm(void)
{
    imu_motion_guard_t guard = initialized_guard();
    float level[3];
    float shock[3];
    gravity_vector(1.0f, level);
    gravity_vector(2.0f, shock);

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1625U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1625U, true, shock));
    TEST_EXPECT(!guard.accel_disturbance_valid);
    TEST_EXPECT(guard.lane[0].accel_disturbance_candidate_valid);
    TEST_EXPECT(guard.lane[0].accel_disturbance_candidate_armed);
    TEST_EXPECT(guard.lane[1].accel_disturbance_candidate_armed);

    TEST_EXPECT(observe_accel_vector(&guard, 1U, 2250U, true, shock));
    TEST_EXPECT(guard.accel_disturbance_valid);
    TEST_EXPECT(guard.counters.accel_disturbance_episode_count == 1U);
}

static void test_subrange_first_sample_and_single_lane_severe(void)
{
    float level[3];
    float severe[3];
    float reverse_severe[3];
    gravity_vector(1.0f, level);
    gravity_vector(9.0f, severe);
    gravity_vector(-9.0f, reverse_severe);

    imu_motion_guard_t guard = initialized_guard();
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, severe));
    TEST_EXPECT(!guard.accel_disturbance_valid);
    TEST_EXPECT(!guard.common_impact_valid);
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[0] == 0U);

    guard = initialized_guard();
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1625U, true, severe));
    TEST_EXPECT(guard.accel_disturbance_valid);
    TEST_EXPECT(guard.accel_disturbance_start_us == 1625U);
    TEST_EXPECT(guard.last_accel_disturbance_us == 1625U);
    TEST_EXPECT(guard.counters.accel_subrange_severe_observation_count[0] ==
                1U);
    TEST_EXPECT(guard.common_impact_valid);
    TEST_EXPECT(guard.common_impact_start_us == 1625U);
    TEST_EXPECT(guard.last_common_impact_us == 1625U);
    TEST_EXPECT(guard.counters.common_impact_episode_count == 1U);
    TEST_EXPECT(!guard.accel_saturation_seen);
    TEST_EXPECT(!guard.gyro_saturation_seen);
    TEST_EXPECT(!imu_motion_guard_accel_saturation_status_at(&guard, 1625U));
    TEST_EXPECT(!imu_motion_guard_gyro_saturation_status_at(&guard, 1625U));

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2250U, true, severe));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2875U, true, severe));
    TEST_EXPECT(guard.last_accel_disturbance_us == 1625U);
    TEST_EXPECT(guard.last_common_impact_us == 1625U);
    TEST_EXPECT(guard.counters.accel_disturbance_extension_count == 0U);

    TEST_EXPECT(observe_accel_vector(
        &guard, 0U, 3500U, true, reverse_severe));
    TEST_EXPECT(guard.common_impact_start_us == 1625U);
    TEST_EXPECT(guard.last_common_impact_us == 3500U);
    TEST_EXPECT(guard.counters.common_impact_episode_count == 1U);
    TEST_EXPECT(guard.counters.accel_subrange_severe_observation_count[0] ==
                2U);

    uint64_t start_us = 0U;
    uint64_t end_us = 0U;
    TEST_EXPECT(imu_motion_guard_accel_inhibit_interval(
        &guard, &start_us, &end_us));
    TEST_EXPECT(start_us == 1625U);
    TEST_EXPECT(end_us == 103500U);
    TEST_EXPECT(imu_motion_guard_gyro_hard_latch_suppressed(&guard, 3500U));
    TEST_EXPECT(!imu_motion_guard_accel_saturation_status_at(&guard, 3500U));
    TEST_EXPECT(!imu_motion_guard_gyro_saturation_status_at(&guard, 3500U));
}

static void test_subrange_slow_maneuver_and_sample_gap(void)
{
    float vector[3];
    imu_motion_guard_t guard = initialized_guard();
    uint64_t timestamp_us = 1000U;
    gravity_vector(1.0f, vector);
    TEST_EXPECT(observe_accel_vector(&guard, 0U, timestamp_us, true, vector));
    for (uint32_t sample = 1U; sample <= 30U; ++sample) {
        timestamp_us += 625U;
        gravity_vector(1.0f + (0.02f * (float)sample), vector);
        TEST_EXPECT(observe_accel_vector(
            &guard, 0U, timestamp_us, true, vector));
    }
    TEST_EXPECT(!guard.accel_disturbance_valid);
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[0] == 0U);

    float level[3];
    float shock[3];
    gravity_vector(1.0f, level);
    gravity_vector(2.0f, shock);
    guard = initialized_guard();
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1625U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2875U, true, shock));
    TEST_EXPECT(guard.counters.accel_subrange_gap_count[0] == 1U);
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[0] == 0U);
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 3500U, true, shock));
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[0] == 0U);
    /* The first high-g sample after a gap is only a baseline. Its fast return
     * to 1 g must still be recognized from the trailing edge. */
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 4125U, true, level));
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[0] == 1U);
}

static void test_subrange_invalid_evidence_cannot_bridge(void)
{
    imu_motion_guard_t guard = initialized_guard();
    float level[3];
    float shock[3];
    float not_finite[3] = {NAN, 0.0f, 0.0f};
    gravity_vector(1.0f, level);
    gravity_vector(2.0f, shock);

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1625U, true, shock));
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[0] == 0U);

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2250U, false, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2875U, true, shock));
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[0] == 0U);

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 3500U, true, not_finite));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 4125U, true, level));
    not_finite[0] = INFINITY;
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 4750U, true, not_finite));
    TEST_EXPECT(!guard.accel_disturbance_valid);
    TEST_EXPECT(!guard.lane[0].previous_accel_valid);
}

static void test_subrange_episode_extension_and_constant_high_g(void)
{
    imu_motion_guard_t guard = initialized_guard();
    float level[3];
    float shock[3];
    float stronger_shock[3];
    float reverse_shock[3];
    gravity_vector(1.0f, level);
    gravity_vector(2.0f, shock);
    gravity_vector(3.0f, stronger_shock);
    gravity_vector(-6.0f, reverse_shock);

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1625U, true, shock));
    TEST_EXPECT(guard.last_accel_disturbance_us == 1625U);

    TEST_EXPECT(observe_accel_vector(
        &guard, 0U, 2250U, true, stronger_shock));
    TEST_EXPECT(observe_accel_vector(
        &guard, 1U, 2250U, true, stronger_shock));
    TEST_EXPECT(guard.last_accel_disturbance_us == 2250U);
    TEST_EXPECT(guard.counters.accel_disturbance_extension_count == 1U);
    TEST_EXPECT(guard.counters.accel_disturbance_episode_count == 1U);

    TEST_EXPECT(observe_accel_vector(
        &guard, 0U, 2875U, true, stronger_shock));
    TEST_EXPECT(observe_accel_vector(
        &guard, 1U, 2875U, true, stronger_shock));
    TEST_EXPECT(guard.last_accel_disturbance_us == 2250U);
    TEST_EXPECT(guard.counters.accel_disturbance_extension_count == 1U);

    TEST_EXPECT(observe_accel_vector(
        &guard, 0U, 3500U, true, reverse_shock));
    TEST_EXPECT(guard.last_accel_disturbance_us == 3500U);
    TEST_EXPECT(guard.counters.accel_disturbance_extension_count == 2U);
}

static void test_subrange_requires_fresh_dual_lane_quiet_to_rearm(void)
{
    imu_motion_guard_config_t config;
    imu_motion_guard_default_config(&config);
    config.accel_inhibit_us = 5000U;
    imu_motion_guard_t guard;
    TEST_EXPECT(imu_motion_guard_init(&guard, &config));

    float level[3];
    float shock[3];
    float quiet_interruption[3];
    gravity_vector(1.0f, level);
    gravity_vector(2.0f, shock);
    gravity_vector(1.1f, quiet_interruption);

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2250U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 2250U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2875U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 2875U, true, level));

    uint64_t timestamp_us = 2875U;
    for (uint32_t sample = 0U; sample < 12U; ++sample) {
        timestamp_us += 625U;
        TEST_EXPECT(observe_accel_vector(
            &guard, 0U, timestamp_us, true, level));
        TEST_EXPECT(observe_accel_vector(
            &guard, 1U, timestamp_us, true, level));
    }
    timestamp_us += 625U;
    TEST_EXPECT(observe_accel_vector(
        &guard, 0U, timestamp_us, true, quiet_interruption));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, timestamp_us, true, level));
    timestamp_us += 625U;
    TEST_EXPECT(observe_accel_vector(&guard, 0U, timestamp_us, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, timestamp_us, true, level));

    for (uint32_t sample = 0U; sample < 31U; ++sample) {
        timestamp_us += 625U;
        TEST_EXPECT(observe_accel_vector(
            &guard, 0U, timestamp_us, true, level));
        TEST_EXPECT(observe_accel_vector(
            &guard, 1U, timestamp_us, true, level));
    }
    TEST_EXPECT(!guard.lane[0].accel_disturbance_candidate_armed);
    timestamp_us += 625U;
    TEST_EXPECT(observe_accel_vector(&guard, 0U, timestamp_us, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, timestamp_us, true, level));
    timestamp_us += 625U;
    TEST_EXPECT(observe_accel_vector(&guard, 0U, timestamp_us, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, timestamp_us, true, level));
    TEST_EXPECT(guard.lane[0].accel_disturbance_candidate_armed);
    TEST_EXPECT(guard.lane[1].accel_disturbance_candidate_armed);
    TEST_EXPECT(!guard.lane[0].accel_disturbance_rearm_required);
    TEST_EXPECT(!guard.lane[1].accel_disturbance_rearm_required);

    timestamp_us += 625U;
    TEST_EXPECT(observe_accel_vector(&guard, 0U, timestamp_us, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, timestamp_us, true, shock));
    TEST_EXPECT(guard.counters.accel_disturbance_episode_count == 2U);

    guard = initialized_guard();
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2250U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 2250U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2875U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 2875U, true, level));
    for (uint64_t time_us = 3500U; time_us <= 23500U; time_us += 625U) {
        TEST_EXPECT(observe_accel_vector(&guard, 0U, time_us, true, level));
        if (time_us <= 4125U) {
            TEST_EXPECT(observe_accel_vector(
                &guard, 1U, time_us, true, level));
        }
    }
    TEST_EXPECT(!guard.lane[0].accel_disturbance_candidate_armed);
    TEST_EXPECT(!guard.lane[1].accel_disturbance_candidate_armed);
}

static void test_subrange_counters_saturate(void)
{
    imu_motion_guard_t guard = initialized_guard();
    guard.counters.accel_subrange_candidate_count[0] = UINT32_MAX;
    guard.counters.accel_subrange_candidate_count[1] = UINT32_MAX;
    guard.counters.accel_subrange_common_observation_count = UINT32_MAX;
    guard.counters.accel_subrange_severe_observation_count[0] = UINT32_MAX;
    guard.counters.accel_disturbance_episode_count = UINT32_MAX;
    guard.counters.accel_disturbance_extension_count = UINT32_MAX;
    float level[3];
    float shock[3];
    float severe[3];
    gravity_vector(1.0f, level);
    gravity_vector(2.0f, shock);
    gravity_vector(-9.0f, severe);

    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1000U, true, level));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 1625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 1U, 1625U, true, shock));
    TEST_EXPECT(observe_accel_vector(&guard, 0U, 2250U, true, severe));
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[0] == UINT32_MAX);
    TEST_EXPECT(guard.counters.accel_subrange_candidate_count[1] == UINT32_MAX);
    TEST_EXPECT(guard.counters.accel_subrange_common_observation_count ==
                UINT32_MAX);
    TEST_EXPECT(guard.counters.accel_subrange_severe_observation_count[0] ==
                UINT32_MAX);
    TEST_EXPECT(guard.counters.accel_disturbance_episode_count == UINT32_MAX);
    TEST_EXPECT(guard.counters.accel_disturbance_extension_count == UINT32_MAX);
}

static void test_counters_saturate(void)
{
    imu_motion_guard_t guard = initialized_guard();
    guard.counters.gyro_observation_count[0] = UINT32_MAX;
    guard.counters.gyro_saturation_count[0] = UINT32_MAX;
    guard.counters.accel_observation_count[1] = UINT32_MAX;
    guard.counters.accel_saturation_count[1] = UINT32_MAX;
    guard.counters.common_impact_episode_count = UINT32_MAX;

    TEST_EXPECT(imu_motion_guard_observe_gyro(&guard, 0U, 1U, true));
    TEST_EXPECT(imu_motion_guard_observe_accel(&guard, 1U, 1U, true));
    TEST_EXPECT(guard.counters.gyro_observation_count[0] == UINT32_MAX);
    TEST_EXPECT(guard.counters.gyro_saturation_count[0] == UINT32_MAX);
    TEST_EXPECT(guard.counters.accel_observation_count[1] == UINT32_MAX);
    TEST_EXPECT(guard.counters.accel_saturation_count[1] == UINT32_MAX);
    TEST_EXPECT(guard.counters.common_impact_episode_count == UINT32_MAX);
}

int main(void)
{
    test_saturated_or_untrusted_samples_reset_frozen_evidence();
    test_default_config_and_api_validation();
    test_single_gyro_pulse_is_not_a_hard_fault();
    test_continuous_single_lane_saturation_is_debounced();
    test_two_lane_common_impact_windows_and_extension();
    test_gyro_events_outside_common_window_are_independent();
    test_accel_saturation_is_immediate_common_impact();
    test_continuous_accel_clipping_extends_inhibit_until_after_exit();
    test_common_impact_episode_boundaries_and_overflow();
    test_common_impact_does_not_hide_debounced_state();
    test_timestamp_discontinuities_reset_streaks();
    test_discontinuous_samples_are_not_cross_lane_impact_evidence();
    test_recent_saturation_windows_are_aggregated_and_bounded();
    test_status_history_is_causal_across_separate_episodes();
    test_status_history_normalizes_an_out_of_order_bridge();
    test_three_episode_history_and_snapshot_are_causal();
    test_forward_gap_records_saturation_but_nonmonotonic_does_not();
    test_subrange_dual_lane_shock_is_accel_only();
    test_subrange_coincidence_boundaries_and_arrival_order();
    test_subrange_single_lane_moderate_does_not_disarm();
    test_subrange_first_sample_and_single_lane_severe();
    test_subrange_slow_maneuver_and_sample_gap();
    test_subrange_invalid_evidence_cannot_bridge();
    test_subrange_episode_extension_and_constant_high_g();
    test_subrange_requires_fresh_dual_lane_quiet_to_rearm();
    test_subrange_counters_saturate();
    test_counters_saturate();

    if (failures != 0U)
        return EXIT_FAILURE;
    (void)puts("imu_motion_guard: all tests passed");
    return EXIT_SUCCESS;
}
