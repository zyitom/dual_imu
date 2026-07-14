#ifndef DUAL_IMU_USB_STREAM_H
#define DUAL_IMU_USB_STREAM_H

#include "dual_imu.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The manager may finish more than one 2.5 ms estimator window per call, so
 * the USB observer can legitimately skip an intermediate fused snapshot. */
#define DUAL_IMU_USB_ACCEL_MAX_AGE_US (7500U)

typedef enum
{
    DUAL_IMU_USB_STATUS_ATTITUDE_VALID = (1U << 0),
    DUAL_IMU_USB_STATUS_ACCEL_VALID = (1U << 1),
    DUAL_IMU_USB_STATUS_GYRO_VALID = (1U << 2),
    DUAL_IMU_USB_STATUS_DEGRADED = (1U << 3),
    DUAL_IMU_USB_STATUS_DEADLINE_MISS = (1U << 4),
    DUAL_IMU_USB_STATUS_EULER_SINGULAR = (1U << 5),
    DUAL_IMU_USB_STATUS_BOTH_INITIALIZED = (1U << 6),
    DUAL_IMU_USB_STATUS_SELECTOR_HEALTHY = (1U << 7),
    DUAL_IMU_USB_STATUS_BMI_BIAS_CONVERGED = (1U << 8),
    DUAL_IMU_USB_STATUS_ICM_BIAS_CONVERGED = (1U << 9),
    DUAL_IMU_USB_STATUS_SELECTED_ICM = (1U << 10),
    DUAL_IMU_USB_STATUS_STATIONARY = (1U << 11),
    DUAL_IMU_USB_STATUS_BMI_FAULT = (1U << 12),
    DUAL_IMU_USB_STATUS_ICM_FAULT = (1U << 13),
    DUAL_IMU_USB_STATUS_STREAM_DROP = (1U << 14),
    DUAL_IMU_USB_STATUS_YAW_RELATIVE = (1U << 15)
} dual_imu_usb_status_t;

typedef struct
{
    uint32_t attitude_frame_count;
    uint32_t encoded_frame_count;
    uint32_t submitted_frame_count;
    uint32_t transport_queue_drop_count;
    uint32_t usb_busy_count;
    uint32_t usb_fail_count;
    uint32_t encoder_error_count;
    uint32_t accel_no_causal_frame_count;
    uint32_t accel_invalid_frame_count;
    uint32_t accel_stale_frame_count;
    uint32_t last_attitude_sequence;
    uint16_t queued_frame_count;
    bool drop_sticky;
    bool port_open;
} dual_imu_usb_stream_diagnostics_t;

void dual_imu_usb_stream_init(void);
void dual_imu_usb_stream_process(const dual_imu_state_t *state);
void dual_imu_usb_stream_get_diagnostics(
    dual_imu_usb_stream_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif

#endif
