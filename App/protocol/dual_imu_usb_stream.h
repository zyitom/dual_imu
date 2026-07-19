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
/* Invalid frames may carry the latest trustworthy payload for display
 * continuity. The validity bits remain clear and are always authoritative. */
#define DUAL_IMU_USB_INVALID_HOLD_US (20000U)
/* Hold transport-only output until three complete 2.5 ms estimator windows
 * have had a chance to classify a causal gyro disagreement. */
#define DUAL_IMU_USB_INTEGRITY_HOLDBACK_US (7500U)
#define DUAL_IMU_USB_SATURATION_HOLD_US \
    IMU_MOTION_GUARD_STATUS_SATURATION_HOLD_US

/*
 * MAIN_STATUS bitfield follows the official HiPNUC HI91 status word so the
 * host decoder can be used unmodified. Bits the host reads as "converged /
 * healthy" are POSITIVE polarity (1 = good). This hardware has no odometer,
 * no UTC source and no magnetometer, so those official bits are wired to a
 * constant 0. HI91 Reserved positions (bit0-1, bit6-7, bit13-15) carry our
 * private diagnostics; a stock HiPNUC host ignores them.
 */
typedef enum
{
    /* --- Private diagnostics (HI91 Reserved bit0-1) --- */
    DUAL_IMU_USB_STATUS_PRIVATE_GYRO_VALID = (1U << 0),
    DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID = (1U << 1),

    /* --- Official HI91 bits, unsupported on this hardware (always 0) --- */
    DUAL_IMU_USB_STATUS_OD = (1U << 2),         /* odometer: none */
    DUAL_IMU_USB_STATUS_SOUT_PULSE = (1U << 3), /* sync-out pulse: none */
    DUAL_IMU_USB_STATUS_UTC_TIME = (1U << 4),   /* UTC synced: never */
    DUAL_IMU_USB_STATUS_MAG_AIDING = (1U << 5), /* mag in fusion: never */

    /* --- Private diagnostics (HI91 Reserved bit6-7) --- */
    DUAL_IMU_USB_STATUS_PRIVATE_STATIONARY = (1U << 6),
    DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST = (1U << 7),

    /* --- Official HI91 bits, live semantics --- */
    DUAL_IMU_USB_STATUS_ATT_CONV = (1U << 8), /* attitude converged (positive) */
    DUAL_IMU_USB_STATUS_GYR_SAT = (1U << 9),  /* gyro over-range */
    DUAL_IMU_USB_STATUS_ACC_SAT = (1U << 10), /* accel over-range */
    DUAL_IMU_USB_STATUS_MAG_DIST = (1U << 11), /* mag disturbance: none, always 0 */
    DUAL_IMU_USB_STATUS_WB_CONV = (1U << 12), /* gyro bias converged (positive) */

    /* --- Private diagnostics (HI91 Reserved bit13-15) --- */
    DUAL_IMU_USB_STATUS_PRIVATE_BMI_FAULT = (1U << 13),
    DUAL_IMU_USB_STATUS_PRIVATE_ICM_FAULT = (1U << 14),
    DUAL_IMU_USB_STATUS_PRIVATE_STREAM_DROP = (1U << 15)
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
    uint32_t heading_event_queue_patch_count;
    uint32_t maturity_wait_count;
    uint32_t accel_no_causal_frame_count;
    uint32_t accel_invalid_frame_count;
    uint32_t accel_stale_frame_count;
    uint32_t attitude_held_frame_count;
    uint32_t accel_held_frame_count;
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
