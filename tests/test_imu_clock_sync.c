#include "imu_clock_sync.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static imu_clock_sync_config_t config(uint8_t bits, double tick_us)
{
    return (imu_clock_sync_config_t) {
        .counter_bits = bits,
        .nominal_tick_us = tick_us,
        .minimum_clock_scale = 0.95,
        .maximum_clock_scale = 1.05,
        .phase_time_constant_us = 20000.0,
        .rate_time_constant_us = 500000.0,
        .minimum_anchor_interval_us = 500.0,
        .initial_residual_limit_us = 1000.0,
    };
}

static void test_unwrap_across_16_bit_rollover(void)
{
    imu_clock_sync_t sync;
    const imu_clock_sync_config_t cfg = config(16U, 1.0);
    assert(imu_clock_sync_init(&sync, &cfg));

    uint64_t tick;
    assert(imu_clock_sync_unwrap(&sync, 65530U, &tick));
    assert(tick == 65530U);
    assert(imu_clock_sync_unwrap(&sync, 4U, &tick));
    assert(tick == 65540U);
    assert(!imu_clock_sync_unwrap(&sync, 3U, &tick));
}

static void test_tracks_clock_rate_and_rejects_outlier(void)
{
    imu_clock_sync_t sync;
    imu_clock_sync_config_t cfg = config(16U, 1.0);
    assert(imu_clock_sync_init(&sync, &cfg));

    uint64_t sensor_tick = 1000U;
    uint64_t mcu_us = 100000U;
    assert(imu_clock_sync_observe(&sync, sensor_tick, mcu_us));
    for (uint32_t index = 0U; index < 2000U; ++index)
    {
        sensor_tick += 1000U;
        mcu_us += 1020U;
        assert(imu_clock_sync_observe(&sync, sensor_tick, mcu_us));
    }

    uint64_t mapped_us;
    assert(imu_clock_sync_map(&sync, sensor_tick - 500U, &mapped_us));
    assert(llabs((long long)mapped_us - (long long)(mcu_us - 510U)) <= 25);

    const uint32_t rejected_before = sync.rejected_anchor_count;
    assert(!imu_clock_sync_observe(&sync, sensor_tick + 1000U,
                                   mcu_us + 5000U));
    assert(sync.rejected_anchor_count == rejected_before + 1U);
}

static void test_bmi_tick_mapping(void)
{
    imu_clock_sync_t sync;
    const imu_clock_sync_config_t cfg = config(24U, 39.0625);
    assert(imu_clock_sync_init(&sync, &cfg));

    assert(imu_clock_sync_observe(&sync, 16000U, 1000000U));
    assert(imu_clock_sync_observe(&sync, 16064U, 1002500U));

    uint64_t mapped_us;
    assert(imu_clock_sync_map(&sync, 16048U, &mapped_us));
    assert(llabs((long long)mapped_us - 1001875LL) <= 1);
}

static void test_reference_freshness_and_causal_bound(void)
{
    imu_clock_sync_t sync;
    const imu_clock_sync_config_t cfg = config(16U, 1.0);
    assert(imu_clock_sync_init(&sync, &cfg));
    assert(imu_clock_sync_observe(&sync, 1000U, 5000U));

    assert(imu_clock_sync_reference_is_fresh(&sync, 6000U, 1000U));
    assert(!imu_clock_sync_reference_is_fresh(&sync, 6001U, 1000U));
    assert(!imu_clock_sync_reference_is_fresh(&sync, 4999U, 1000U));

    /* The filtered phase can temporarily lead wall time while the slope is
     * converging. That does not make the last real hardware anchor stale. */
    sync.reference_mcu_us = 5100.0;
    assert(imu_clock_sync_reference_is_fresh(&sync, 5050U, 1000U));

    uint64_t mapped_us = 0U;
    assert(imu_clock_sync_map_bounded(&sync, 1500U, 5600U, &mapped_us));
    assert(mapped_us == 5600U);
    assert(!imu_clock_sync_map_bounded(&sync, 1501U, 5600U, &mapped_us));
}

int main(void)
{
    test_unwrap_across_16_bit_rollover();
    test_tracks_clock_rate_and_rejects_outlier();
    test_bmi_tick_mapping();
    test_reference_freshness_and_causal_bound();
    puts("imu_clock_sync tests passed");
    return 0;
}
