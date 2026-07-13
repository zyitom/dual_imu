#ifndef DUAL_IMU_H
#define DUAL_IMU_H

#include "imu_types.h"
#include "imu_calibration.h"
#include "imu_selector.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DUAL_IMU_MODE_NONE = 0,
    DUAL_IMU_MODE_BMI088,
    DUAL_IMU_MODE_ICM45686,
    DUAL_IMU_MODE_FUSED
} dual_imu_mode_t;

typedef enum
{
    DUAL_IMU_FAULT_NONE = 0U,
    DUAL_IMU_FAULT_INVALID = (1U << 0),
    DUAL_IMU_FAULT_STALE = (1U << 1),
    DUAL_IMU_FAULT_HEALTH = (1U << 2),
    DUAL_IMU_FAULT_ACCEL_SATURATED = (1U << 3),
    DUAL_IMU_FAULT_GYRO_SATURATED = (1U << 4),
    DUAL_IMU_FAULT_ACCEL_FROZEN = (1U << 5),
    DUAL_IMU_FAULT_GYRO_FROZEN = (1U << 6),
    DUAL_IMU_FAULT_FILTER = (1U << 7),
    DUAL_IMU_FAULT_ACCEL_TIMING = (1U << 8),
    DUAL_IMU_FAULT_GYRO_TIMING = (1U << 9),
    DUAL_IMU_FAULT_TIMING = DUAL_IMU_FAULT_ACCEL_TIMING |
                            DUAL_IMU_FAULT_GYRO_TIMING,
    DUAL_IMU_FAULT_SATURATED = DUAL_IMU_FAULT_ACCEL_SATURATED |
                               DUAL_IMU_FAULT_GYRO_SATURATED,
    DUAL_IMU_FAULT_FROZEN = DUAL_IMU_FAULT_ACCEL_FROZEN |
                            DUAL_IMU_FAULT_GYRO_FROZEN
} dual_imu_fault_t;

typedef enum
{
    DUAL_IMU_HINT_NONE = 0,
    DUAL_IMU_HINT_BMI088_BAD,
    DUAL_IMU_HINT_ICM45686_BAD
} dual_imu_isolation_hint_t;

typedef struct
{
    bool bmi088_initialized;
    bool icm45686_initialized;
    bool bmi088_calibrated;
    bool icm45686_calibrated;
    uint8_t bmi088_accel_id;
    uint8_t bmi088_gyro_id;
    uint8_t icm45686_id;
    dual_imu_mode_t mode;
    imu_accel_sample_t bmi088_accel_sample;
    imu_gyro_sample_t bmi088_gyro_sample;
    imu_sample_t bmi088_sample;
    imu_sample_t icm45686_sample;
    imu_sample_t fused_sample;
    imu_health_t bmi088_health;
    imu_health_t icm45686_health;
    imu_source_t selected_source;
    imu_source_t selected_accel_source;
    imu_selector_state_t selector_state;
    uint32_t selector_reason_flags;
    float selector_residual_nis;
    float lane_quaternion[IMU_SOURCE_COUNT][4];
    float lane_gyro_bias_rad_s[IMU_SOURCE_COUNT][3];
    float quaternion[4];
    float euler_rad[3];
    float control_gyro_rad_s[3];
    uint64_t control_gyro_timestamp_us;
    float accel_residual_mps2;
    float gyro_residual_rad_s;
    uint32_t bmi088_fault_flags;
    uint32_t icm45686_fault_flags;
    uint32_t pair_skew_us;
    uint32_t fusion_count;
    uint32_t selection_count;
    uint32_t mismatch_count;
    uint32_t unpaired_count;
    uint32_t filter_reject_count;
    uint32_t bmi088_watchdog_failure_count;
    uint32_t bmi088_accel_watchdog_failure_count;
    uint32_t bmi088_gyro_watchdog_failure_count;
    uint32_t icm45686_watchdog_failure_count;
    uint32_t icm45686_accel_watchdog_failure_count;
    uint32_t icm45686_gyro_watchdog_failure_count;
    bool bmi088_accel_configuration_fault;
    bool bmi088_gyro_configuration_fault;
    bool icm45686_configuration_fault;
    bool icm45686_accel_configuration_fault;
    bool icm45686_gyro_configuration_fault;
    uint32_t lane_window_flags[IMU_SOURCE_COUNT];
    uint32_t lane_accel_reject_count[IMU_SOURCE_COUNT];
    uint32_t lane_zaru_accept_count[IMU_SOURCE_COUNT];
    bool stationary_candidate;
    bool stationary_confirmed;
    bool stationary_hint_active;
    bool accel_update_inhibited;
    bool fused_accel_valid;
    bool selection_changed;
    bool alignment_blend_active;
    uint32_t bmi088_accel_irq_count;
    uint32_t bmi088_gyro_irq_count;
    uint32_t icm45686_irq_count;
    uint32_t stream_read_count[IMU_SOURCE_COUNT][2];
    uint32_t stream_coalesced_count[IMU_SOURCE_COUNT][2];
    uint32_t stream_max_irq_to_read_us[IMU_SOURCE_COUNT][2];
    uint32_t bmi088_accel_capture_overrun_count;
    uint32_t bmi088_gyro_capture_overrun_count;
    bool bmi088_hardware_timestamp_enabled;
    bool custom_calibration_loaded[IMU_SOURCE_COUNT];
    uint32_t bmi088_accel_event_drop_count;
    uint32_t bmi088_gyro_event_drop_count;
    uint32_t icm45686_event_drop_count;
    uint32_t bmi088_accel_buffer_overwrite_count;
    uint32_t bmi088_gyro_buffer_overwrite_count;
    uint32_t icm45686_accel_buffer_overwrite_count;
    uint32_t icm45686_gyro_buffer_overwrite_count;
} dual_imu_state_t;

bool dual_imu_init(void);
void dual_imu_process(void);
void dual_imu_handle_exti(uint16_t gpio_pin);
void dual_imu_notify_impact(uint32_t inhibit_duration_us);
void dual_imu_set_stationary_hint(bool stationary);
/* Non-NONE external isolation evidence must be refreshed at least every 100 ms. */
void dual_imu_set_isolation_hint(dual_imu_isolation_hint_t hint);
bool dual_imu_get_control_gyro(imu_gyro_sample_t *sample);
bool dual_imu_set_calibration(imu_source_t source,
                              const imu_calibration_t *calibration);
bool dual_imu_get_calibration(imu_source_t source,
                              imu_calibration_t *calibration);
bool dual_imu_pop_accel(imu_source_t source, imu_accel_sample_t *sample);
bool dual_imu_pop_gyro(imu_source_t source, imu_gyro_sample_t *sample);
const dual_imu_state_t *dual_imu_get_state(void);

#endif
