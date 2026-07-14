#include "imu_clock_sync.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_CLOCK_SYNC_MIN_ANCHORS 4U

static bool config_is_valid(const imu_clock_sync_config_t *config)
{
    return (config != NULL) &&
           (config->counter_bits >= 2U) && (config->counter_bits <= 32U) &&
           isfinite(config->nominal_tick_us) &&
           (config->nominal_tick_us > 0.0) &&
           isfinite(config->minimum_clock_scale) &&
           isfinite(config->maximum_clock_scale) &&
           (config->minimum_clock_scale > 0.0) &&
           (config->maximum_clock_scale > config->minimum_clock_scale) &&
           isfinite(config->phase_time_constant_us) &&
           (config->phase_time_constant_us > 0.0) &&
           isfinite(config->rate_time_constant_us) &&
           (config->rate_time_constant_us > 0.0) &&
           isfinite(config->minimum_anchor_interval_us) &&
           (config->minimum_anchor_interval_us > 0.0) &&
           isfinite(config->initial_residual_limit_us) &&
           (config->initial_residual_limit_us > 0.0);
}

bool imu_clock_sync_init(imu_clock_sync_t *sync,
                         const imu_clock_sync_config_t *config)
{
    if ((sync == NULL) || !config_is_valid(config))
        return false;

    memset(sync, 0, sizeof(*sync));
    sync->config = *config;
    sync->counter_modulus = UINT64_C(1) << config->counter_bits;
    sync->counter_mask = sync->counter_modulus - 1U;
    sync->slope_us_per_tick = config->nominal_tick_us;
    return true;
}

void imu_clock_sync_reset(imu_clock_sync_t *sync)
{
    if (sync == NULL)
        return;

    const imu_clock_sync_config_t config = sync->config;
    (void)imu_clock_sync_init(sync, &config);
}

bool imu_clock_sync_unwrap(imu_clock_sync_t *sync,
                           uint32_t raw_tick,
                           uint64_t *unwrapped_tick)
{
    if ((sync == NULL) || (unwrapped_tick == NULL) ||
        (sync->counter_modulus == 0U))
        return false;

    const uint64_t raw = (uint64_t)raw_tick & sync->counter_mask;
    if (!sync->unwrap_initialized)
    {
        sync->last_unwrapped_tick = raw;
        sync->unwrap_initialized = true;
        *unwrapped_tick = raw;
        return true;
    }

    const uint64_t previous_raw =
        sync->last_unwrapped_tick & sync->counter_mask;
    const uint64_t modular_delta =
        (raw - previous_raw) & sync->counter_mask;
    if (modular_delta >= (sync->counter_modulus / 2U))
        return false;

    sync->last_unwrapped_tick += modular_delta;
    *unwrapped_tick = sync->last_unwrapped_tick;
    return true;
}

bool imu_clock_sync_observe(imu_clock_sync_t *sync,
                            uint64_t sensor_tick,
                            uint64_t mcu_anchor_us)
{
    if ((sync == NULL) || (sync->counter_modulus == 0U) ||
        (mcu_anchor_us == 0U))
        return false;

    if (!sync->reference_initialized)
    {
        sync->reference_tick = sensor_tick;
        sync->last_anchor_tick = sensor_tick;
        sync->last_anchor_mcu_us = mcu_anchor_us;
        sync->reference_mcu_us = (double)mcu_anchor_us;
        sync->slope_us_per_tick = sync->config.nominal_tick_us;
        sync->accepted_anchor_count = 1U;
        sync->reference_initialized = true;
        return true;
    }

    if ((sensor_tick <= sync->last_anchor_tick) ||
        (mcu_anchor_us <= sync->last_anchor_mcu_us))
    {
        sync->nonmonotonic_reject_count++;
        sync->rejected_anchor_count++;
        return false;
    }

    const double anchor_delta_tick =
        (double)(sensor_tick - sync->last_anchor_tick);
    const double sensor_interval_us =
        anchor_delta_tick * sync->config.nominal_tick_us;
    if (sensor_interval_us < sync->config.minimum_anchor_interval_us)
    {
        sync->interval_reject_count++;
        sync->rejected_anchor_count++;
        return false;
    }

    const double mcu_interval_us =
        (double)(mcu_anchor_us - sync->last_anchor_mcu_us);
    const double observed_slope = mcu_interval_us / anchor_delta_tick;
    sync->last_observed_slope_us_per_tick = observed_slope;
    const double minimum_slope = sync->config.nominal_tick_us *
                                 sync->config.minimum_clock_scale;
    const double maximum_slope = sync->config.nominal_tick_us *
                                 sync->config.maximum_clock_scale;
    if (!isfinite(observed_slope) || (observed_slope < minimum_slope) ||
        (observed_slope > maximum_slope))
    {
        sync->slope_reject_count++;
        sync->rejected_anchor_count++;
        return false;
    }

    const double reference_delta_tick =
        (double)(sensor_tick - sync->reference_tick);
    const double predicted_mcu_us = sync->reference_mcu_us +
        reference_delta_tick * sync->slope_us_per_tick;
    const double residual_us = (double)mcu_anchor_us - predicted_mcu_us;
    sync->last_residual_us = residual_us;
    double residual_limit_us = sync->config.initial_residual_limit_us;
    if (sync->accepted_anchor_count >= IMU_CLOCK_SYNC_MIN_ANCHORS)
    {
        const double learned_limit =
            3.0 * sqrt(sync->residual_variance_us2 + 1.0);
        if (learned_limit > residual_limit_us)
            residual_limit_us = learned_limit;
    }
    if (!isfinite(residual_us) || (fabs(residual_us) > residual_limit_us))
    {
        sync->residual_reject_count++;
        sync->rejected_anchor_count++;
        return false;
    }

    const double rate_gain = sensor_interval_us /
        (sync->config.rate_time_constant_us + sensor_interval_us);
    const double phase_gain = sensor_interval_us /
        (sync->config.phase_time_constant_us + sensor_interval_us);
    sync->slope_us_per_tick +=
        rate_gain * (observed_slope - sync->slope_us_per_tick);
    if (sync->slope_us_per_tick < minimum_slope)
        sync->slope_us_per_tick = minimum_slope;
    else if (sync->slope_us_per_tick > maximum_slope)
        sync->slope_us_per_tick = maximum_slope;

    sync->reference_tick = sensor_tick;
    sync->reference_mcu_us = predicted_mcu_us + phase_gain * residual_us;
    sync->last_anchor_tick = sensor_tick;
    sync->last_anchor_mcu_us = mcu_anchor_us;
    const double variance_gain = (phase_gain < 0.01) ? 0.01 : phase_gain;
    sync->residual_variance_us2 += variance_gain *
        ((residual_us * residual_us) - sync->residual_variance_us2);
    sync->accepted_anchor_count++;
    return true;
}

bool imu_clock_sync_map(const imu_clock_sync_t *sync,
                        uint64_t sensor_tick,
                        uint64_t *mcu_timestamp_us)
{
    if ((sync == NULL) || (mcu_timestamp_us == NULL) ||
        !sync->reference_initialized)
        return false;

    double mapped_us = sync->reference_mcu_us;
    if (sensor_tick >= sync->reference_tick)
    {
        mapped_us += (double)(sensor_tick - sync->reference_tick) *
                     sync->slope_us_per_tick;
    }
    else
    {
        mapped_us -= (double)(sync->reference_tick - sensor_tick) *
                     sync->slope_us_per_tick;
    }
    if (!isfinite(mapped_us) || (mapped_us < 0.5) ||
        (mapped_us > (double)UINT64_MAX))
        return false;

    *mcu_timestamp_us = (uint64_t)(mapped_us + 0.5);
    return true;
}

bool imu_clock_sync_reference_is_fresh(const imu_clock_sync_t *sync,
                                       uint64_t now_us,
                                       uint64_t maximum_age_us)
{
    if ((sync == NULL) || !sync->reference_initialized ||
        (maximum_age_us == 0U) || (sync->last_anchor_mcu_us == 0U) ||
        (sync->last_anchor_mcu_us > now_us))
        return false;

    return (now_us - sync->last_anchor_mcu_us) <= maximum_age_us;
}

bool imu_clock_sync_map_bounded(const imu_clock_sync_t *sync,
                                uint64_t sensor_tick,
                                uint64_t latest_allowed_mcu_us,
                                uint64_t *mcu_timestamp_us)
{
    uint64_t mapped_us;
    if (!imu_clock_sync_map(sync, sensor_tick, &mapped_us) ||
        (mapped_us > latest_allowed_mcu_us))
        return false;

    if (mcu_timestamp_us != NULL)
        *mcu_timestamp_us = mapped_us;
    return true;
}

void imu_clock_sync_get_diagnostics(const imu_clock_sync_t *sync,
                                    imu_clock_sync_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
        return;

    memset(diagnostics, 0, sizeof(*diagnostics));
    if ((sync == NULL) || (sync->config.nominal_tick_us <= 0.0))
        return;

    diagnostics->clock_scale = sync->slope_us_per_tick /
                               sync->config.nominal_tick_us;
    diagnostics->last_reference_mcu_us = sync->reference_mcu_us;
    diagnostics->residual_sigma_us = sqrt(sync->residual_variance_us2);
    diagnostics->accepted_anchor_count = sync->accepted_anchor_count;
    diagnostics->rejected_anchor_count = sync->rejected_anchor_count;
    diagnostics->nonmonotonic_reject_count =
        sync->nonmonotonic_reject_count;
    diagnostics->interval_reject_count = sync->interval_reject_count;
    diagnostics->slope_reject_count = sync->slope_reject_count;
    diagnostics->residual_reject_count = sync->residual_reject_count;
    diagnostics->last_observed_clock_scale =
        sync->last_observed_slope_us_per_tick /
        sync->config.nominal_tick_us;
    diagnostics->last_residual_us = sync->last_residual_us;
    diagnostics->valid = sync->reference_initialized &&
                         (sync->accepted_anchor_count >=
                          IMU_CLOCK_SYNC_MIN_ANCHORS);
}
