#ifndef IMU_CLOCK_SYNC_H
#define IMU_CLOCK_SYNC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t counter_bits;
    double nominal_tick_us;
    double minimum_clock_scale;
    double maximum_clock_scale;
    double phase_time_constant_us;
    double rate_time_constant_us;
    double minimum_anchor_interval_us;
    double initial_residual_limit_us;
} imu_clock_sync_config_t;

typedef struct
{
    imu_clock_sync_config_t config;
    uint64_t counter_modulus;
    uint64_t counter_mask;
    uint64_t last_unwrapped_tick;
    uint64_t reference_tick;
    uint64_t last_anchor_tick;
    uint64_t last_anchor_mcu_us;
    double reference_mcu_us;
    double slope_us_per_tick;
    double residual_variance_us2;
    uint32_t accepted_anchor_count;
    uint32_t rejected_anchor_count;
    uint32_t nonmonotonic_reject_count;
    uint32_t interval_reject_count;
    uint32_t slope_reject_count;
    uint32_t residual_reject_count;
    double last_observed_slope_us_per_tick;
    double last_residual_us;
    bool unwrap_initialized;
    bool reference_initialized;
} imu_clock_sync_t;

typedef struct
{
    double clock_scale;
    double last_reference_mcu_us;
    double residual_sigma_us;
    uint32_t accepted_anchor_count;
    uint32_t rejected_anchor_count;
    uint32_t nonmonotonic_reject_count;
    uint32_t interval_reject_count;
    uint32_t slope_reject_count;
    uint32_t residual_reject_count;
    double last_observed_clock_scale;
    double last_residual_us;
    bool valid;
} imu_clock_sync_diagnostics_t;

bool imu_clock_sync_init(imu_clock_sync_t *sync,
                         const imu_clock_sync_config_t *config);
void imu_clock_sync_reset(imu_clock_sync_t *sync);
bool imu_clock_sync_unwrap(imu_clock_sync_t *sync,
                           uint32_t raw_tick,
                           uint64_t *unwrapped_tick);
bool imu_clock_sync_observe(imu_clock_sync_t *sync,
                            uint64_t sensor_tick,
                            uint64_t mcu_anchor_us);
bool imu_clock_sync_map(const imu_clock_sync_t *sync,
                        uint64_t sensor_tick,
                        uint64_t *mcu_timestamp_us);
/* Freshness is based on the last observed hardware anchor, not the filtered
 * phase reference. Mapping causality is checked separately by map_bounded(). */
bool imu_clock_sync_reference_is_fresh(const imu_clock_sync_t *sync,
                                       uint64_t now_us,
                                       uint64_t maximum_age_us);
bool imu_clock_sync_map_bounded(const imu_clock_sync_t *sync,
                                uint64_t sensor_tick,
                                uint64_t latest_allowed_mcu_us,
                                uint64_t *mcu_timestamp_us);
void imu_clock_sync_get_diagnostics(const imu_clock_sync_t *sync,
                                    imu_clock_sync_diagnostics_t *diagnostics);

#endif
