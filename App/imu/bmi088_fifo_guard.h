#ifndef BMI088_FIFO_GUARD_H
#define BMI088_FIFO_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#define BMI088_FIFO_GUARD_GYRO_RANGE_MASK UINT8_C(0x07)
#define BMI088_FIFO_GUARD_GYRO_BW_MASK UINT8_C(0x0F)
#define BMI088_FIFO_GUARD_GYRO_POWER_MASK UINT8_C(0xA0)
#define BMI088_FIFO_GUARD_GYRO_INT_CTRL_MASK UINT8_C(0xC0)
#define BMI088_FIFO_GUARD_GYRO_DRDY_ENABLE UINT8_C(0x80)
#define BMI088_FIFO_GUARD_GYRO_INT3_IO_MASK UINT8_C(0x03)
#define BMI088_FIFO_GUARD_GYRO_INT3_ACTIVE_HIGH_PUSH_PULL UINT8_C(0x01)
#define BMI088_FIFO_GUARD_GYRO_RELEVANT_MAP_MASK UINT8_C(0xA5)
#define BMI088_FIFO_GUARD_GYRO_DRDY_INT3_MAP UINT8_C(0x01)
#define BMI088_FIFO_GUARD_GYRO_STATUS_OVERRUN_MASK UINT8_C(0x80)
#define BMI088_FIFO_GUARD_GYRO_STATUS_FRAME_COUNT_MASK UINT8_C(0x7F)

static inline bool bmi088_fifo_capture_epoch_valid(uint32_t expected_epoch,
                                                   uint32_t current_epoch,
                                                   bool faulted)
{
    return !faulted && (expected_epoch == current_epoch);
}

static inline bool bmi088_fifo_gyro_failure_is_ambiguous(
    bool fifo_data_transaction_started)
{
    return fifo_data_transaction_started;
}

static inline bool bmi088_fifo_normal_service_allowed(
    bool gyro_recovery_quiesced,
    bool gyro_recovery_in_progress)
{
    return !gyro_recovery_quiesced && !gyro_recovery_in_progress;
}

static inline bool bmi088_fifo_gyro_boundary_status_valid(uint8_t fifo_status)
{
    return (fifo_status &
            (BMI088_FIFO_GUARD_GYRO_STATUS_OVERRUN_MASK |
             BMI088_FIFO_GUARD_GYRO_STATUS_FRAME_COUNT_MASK)) == 0U;
}

static inline uint32_t bmi088_fifo_gyro_recovery_gap_count(
    uint16_t fifo_frames,
    bool fifo_overrun)
{
    uint32_t discarded_frames = (uint32_t)fifo_frames;
    if (fifo_overrun)
        discarded_frames++;

    /* The sensor kept sampling while capture was faulted. Even if FIFO status
     * is already empty or an overrun made the exact count unknowable, the next
     * accepted sample must not look contiguous with the pre-fault stream. */
    return (discarded_frames != 0U) ? discarded_frames : 1U;
}

static inline bool bmi088_fifo_gyro_core_config_valid(
    uint8_t range_register,
    uint8_t bandwidth_register,
    uint8_t power_register,
    uint8_t expected_range,
    uint8_t expected_bandwidth,
    uint8_t expected_power)
{
    return ((range_register & BMI088_FIFO_GUARD_GYRO_RANGE_MASK) ==
            expected_range) &&
           ((bandwidth_register & BMI088_FIFO_GUARD_GYRO_BW_MASK) ==
            expected_bandwidth) &&
           ((power_register & BMI088_FIFO_GUARD_GYRO_POWER_MASK) ==
            expected_power);
}

static inline bool bmi088_fifo_gyro_interrupt_config_valid(
    uint8_t interrupt_control,
    uint8_t interrupt_io,
    uint8_t interrupt_map)
{
    return ((interrupt_control & BMI088_FIFO_GUARD_GYRO_INT_CTRL_MASK) ==
            BMI088_FIFO_GUARD_GYRO_DRDY_ENABLE) &&
           ((interrupt_io & BMI088_FIFO_GUARD_GYRO_INT3_IO_MASK) ==
            BMI088_FIFO_GUARD_GYRO_INT3_ACTIVE_HIGH_PUSH_PULL) &&
           ((interrupt_map & BMI088_FIFO_GUARD_GYRO_RELEVANT_MAP_MASK) ==
            BMI088_FIFO_GUARD_GYRO_DRDY_INT3_MAP);
}

static inline uint32_t bmi088_fifo_accel_missing_frame_count(
    uint8_t skipped_frames,
    bool sample_dropped)
{
    return (uint32_t)skipped_frames + (sample_dropped ? 1U : 0U);
}

static inline uint32_t bmi088_fifo_accel_invalid_frame_count(
    bool metadata_valid,
    uint16_t metadata_frame_count,
    uint16_t parsed_frame_count,
    uint8_t skipped_frames,
    bool sample_dropped)
{
    uint32_t consumed_frames = metadata_valid
        ? (uint32_t)metadata_frame_count
        : (uint32_t)parsed_frame_count;
    consumed_frames += bmi088_fifo_accel_missing_frame_count(skipped_frames,
                                                             sample_dropped);

    /* Even an unclassifiable transfer must make the next accepted sample
     * discontinuous. Otherwise one corrupt frame can be invisible upstream. */
    return (consumed_frames != 0U) ? consumed_frames : 1U;
}

#endif
