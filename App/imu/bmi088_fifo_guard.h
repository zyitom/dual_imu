#ifndef BMI088_FIFO_GUARD_H
#define BMI088_FIFO_GUARD_H

#include <stdbool.h>
#include <stdint.h>

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
