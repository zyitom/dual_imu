#ifndef ICM45686_H
#define ICM45686_H

#include "imu_types.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef ICM45686_USE_ROBUST_HIGH_RATE_PROFILE
#define ICM45686_USE_ROBUST_HIGH_RATE_PROFILE 1
#endif

#if (ICM45686_USE_ROBUST_HIGH_RATE_PROFILE != 0) && \
    (ICM45686_USE_ROBUST_HIGH_RATE_PROFILE != 1)
#error "ICM45686_USE_ROBUST_HIGH_RATE_PROFILE must be 0 or 1"
#endif

#if ICM45686_USE_ROBUST_HIGH_RATE_PROFILE
#define ICM45686_CONFIG_ACCEL_ODR_HZ          UINT16_C(1600)
#define ICM45686_CONFIG_ACCEL_BANDWIDTH_HZ    UINT16_C(100)
#define ICM45686_CONFIG_ACCEL_MAX_GAP_US      UINT32_C(1000)
#define ICM45686_CONFIG_GYRO_ODR_HZ           UINT16_C(3200)
#define ICM45686_CONFIG_GYRO_BANDWIDTH_HZ     UINT16_C(200)
#else
#define ICM45686_CONFIG_ACCEL_ODR_HZ          UINT16_C(400)
#define ICM45686_CONFIG_ACCEL_BANDWIDTH_HZ    UINT16_C(50)
#define ICM45686_CONFIG_ACCEL_MAX_GAP_US      UINT32_C(4000)
#define ICM45686_CONFIG_GYRO_ODR_HZ           UINT16_C(400)
#define ICM45686_CONFIG_GYRO_BANDWIDTH_HZ     UINT16_C(50)
#endif

#define ICM45686_CONFIG_ACCEL_RANGE_G         UINT16_C(32)
#define ICM45686_CONFIG_GYRO_RANGE_DPS        UINT16_C(4000)
#define ICM45686_CONFIG_FIFO_ENABLED          1

#define ICM45686_FIFO_FRAME_SIZE_BYTES        UINT16_C(20)
#define ICM45686_FIFO_WATERMARK_FRAMES        UINT16_C(2)
#define ICM45686_FIFO_TIMESTAMP_RESOLUTION_US UINT16_C(1)
#define ICM45686_FIFO_SPI_OVERHEAD_BYTES      UINT16_C(1)
#define ICM45686_FIFO_DMA_MAX_FRAMES          UINT16_C(25)
#define ICM45686_FIFO_DMA_MAX_TRANSFER_BYTES  \
    (ICM45686_FIFO_SPI_OVERHEAD_BYTES +       \
     (ICM45686_FIFO_DMA_MAX_FRAMES * ICM45686_FIFO_FRAME_SIZE_BYTES))

typedef struct
{
    uint16_t accel_odr_hz;
    /* UI LPF setting; FIR AAF and mechanics can further shape the response. */
    uint16_t accel_bandwidth_hz;
    uint16_t accel_range_g;
    uint16_t gyro_odr_hz;
    uint16_t gyro_bandwidth_hz;
    uint16_t gyro_range_dps;
    uint16_t fifo_frame_size_bytes;
    uint16_t fifo_watermark_frames;
    uint16_t fifo_timestamp_resolution_us;
    bool fifo_enabled;
    bool register_readback_verified;
} icm45686_configuration_t;

typedef struct
{
    int32_t accel_raw[3];
    int32_t gyro_raw[3];
    float accel_mps2[3];
    float gyro_rad_s[3];
    float temperature_c;
    uint64_t mcu_timestamp_us;
    uint64_t sensor_timestamp_unwrapped_us;
    uint64_t watermark_anchor_us;
    uint16_t sensor_timestamp_us;
    int16_t temperature_raw;
    uint8_t header;
    bool accel_valid;
    bool gyro_valid;
    bool temperature_valid;
    bool timestamp_valid;
    bool mcu_timestamp_valid;
    bool fsync_event;
} icm45686_fifo_frame_t;

/* A valid timestamp identifies a real FIFO sample slot even when the device
 * encodes the gyro axes with its invalid-data sentinel. Preserve that slot as
 * a finite invalid node so downstream integration cannot bridge across it. */
static inline bool icm45686_fifo_frame_to_gyro_sample(
    const icm45686_fifo_frame_t *frame,
    float temperature_c,
    imu_gyro_sample_t *sample)
{
    if ((frame == NULL) || (sample == NULL) || !frame->timestamp_valid ||
        !frame->mcu_timestamp_valid || (frame->mcu_timestamp_us == 0U)) {
        return false;
    }

    sample->timestamp_us = frame->mcu_timestamp_us;
    sample->sequence = 0U;
    for (size_t axis = 0U; axis < 3U; ++axis) {
        sample->gyro_rad_s[axis] = frame->gyro_valid
                                      ? frame->gyro_rad_s[axis]
                                      : 0.0f;
    }
    sample->temperature_c = temperature_c;
    sample->temperature_timestamp_us = frame->temperature_valid
                                           ? frame->mcu_timestamp_us
                                           : 0U;
    sample->source = IMU_SOURCE_ICM45686;
    sample->temperature_valid = frame->temperature_valid;
    sample->valid = frame->gyro_valid;
    return true;
}

typedef struct
{
    uint64_t watermark_anchor_us;
    uint16_t requested_frame_count;
    uint16_t parsed_frame_count;
    uint16_t accel_frame_count;
    uint16_t gyro_frame_count;
    uint16_t temperature_invalid_frame_count;
    uint16_t malformed_frame_count;
    uint16_t timestamp_discontinuity_count;
} icm45686_fifo_batch_report_t;

typedef struct
{
    uint32_t irq_request_count;
    uint32_t coalesced_irq_count;
    uint32_t dma_batch_count;
    uint32_t dma_error_count;
    uint32_t fifo_full_count;
    uint32_t fifo_flush_count;
    uint32_t malformed_frame_count;
    uint32_t timestamp_discontinuity_count;
    uint32_t output_overrun_count;
    uint32_t count_read_error_count;
    uint32_t temperature_invalid_frame_count;
    uint32_t clock_anchor_accepted_count;
    uint32_t clock_anchor_rejected_count;
    uint32_t clock_nonmonotonic_reject_count;
    uint32_t clock_interval_reject_count;
    uint32_t clock_slope_reject_count;
    uint32_t clock_residual_reject_count;
    uint32_t clock_stale_reset_count;
    uint32_t clock_causal_reset_count;
    uint64_t clock_reference_mcu_us;
    float clock_scale;
    float clock_last_observed_scale;
    float clock_last_residual_us;
    float clock_residual_sigma_us;
    uint16_t last_fifo_count;
    uint16_t peak_fifo_count;
    bool dma_active;
    bool request_pending;
    bool clock_sync_valid;
} icm45686_fifo_diagnostics_t;

#ifdef __cplusplus
extern "C" {
#endif

bool icm45686_init(void);
bool icm45686_check_configuration(void);
bool icm45686_check_accel_configuration(void);
bool icm45686_check_gyro_configuration(void);
bool icm45686_read(uint64_t timestamp_us, imu_sample_t *sample);

/*
 * PE4 watermark ISR entry point. The anchor is the TIM5 timestamp captured at
 * the interrupt edge. The function only starts the asynchronous DMA chain;
 * call icm45686_fifo_service() and drain icm45686_fifo_pop_frame() in thread
 * context.
 */
bool icm45686_fifo_request(uint64_t watermark_anchor_us);
void icm45686_fifo_service(void);
bool icm45686_fifo_pop_frame(icm45686_fifo_frame_t *frame);
bool icm45686_fifo_flush(void);
bool icm45686_fifo_read_frame_count(uint16_t *frame_count);
void icm45686_fifo_get_diagnostics(icm45686_fifo_diagnostics_t *diagnostics);

/* Stateless helpers for a caller that owns the DMA scheduling. */
size_t icm45686_fifo_dma_transfer_size(uint16_t frame_count);
bool icm45686_fifo_prepare_count_read(uint8_t *tx_data, size_t tx_capacity);
bool icm45686_fifo_parse_count_response(const uint8_t *rx_data,
                                        size_t rx_length,
                                        uint16_t *frame_count);
bool icm45686_fifo_prepare_data_read(uint16_t frame_count,
                                     uint8_t *tx_data,
                                     size_t tx_capacity);
bool icm45686_fifo_parse_dma_response(
    const uint8_t *rx_data,
    size_t rx_length,
    uint16_t frame_count,
    uint64_t watermark_anchor_us,
    icm45686_fifo_frame_t *frames,
    size_t frame_capacity,
    icm45686_fifo_batch_report_t *report);
void icm45686_fifo_reset_timestamp_unwrap(void);

void icm45686_get_health(imu_health_t *health);
void icm45686_get_configuration(icm45686_configuration_t *configuration);
uint8_t icm45686_who_am_i(void);

#ifdef __cplusplus
}
#endif

#endif
