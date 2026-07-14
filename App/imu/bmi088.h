#ifndef BMI088_H
#define BMI088_H

#include "imu_types.h"
#include "imu_clock_sync.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef BMI088_USE_ROBUST_HIGH_RATE_PROFILE
#define BMI088_USE_ROBUST_HIGH_RATE_PROFILE 1
#endif

#if (BMI088_USE_ROBUST_HIGH_RATE_PROFILE != 0) && \
    (BMI088_USE_ROBUST_HIGH_RATE_PROFILE != 1)
#error "BMI088_USE_ROBUST_HIGH_RATE_PROFILE must be 0 or 1"
#endif

#if BMI088_USE_ROBUST_HIGH_RATE_PROFILE
#define BMI088_CONFIG_ACCEL_ODR_HZ          UINT16_C(1600)
#define BMI088_CONFIG_ACCEL_BANDWIDTH_HZ    UINT16_C(145)
#define BMI088_CONFIG_GYRO_ODR_HZ           UINT16_C(2000)
#define BMI088_CONFIG_GYRO_BANDWIDTH_HZ     UINT16_C(230)
#else
#define BMI088_CONFIG_ACCEL_ODR_HZ          UINT16_C(400)
#define BMI088_CONFIG_ACCEL_BANDWIDTH_HZ    UINT16_C(40)
#define BMI088_CONFIG_GYRO_ODR_HZ           UINT16_C(400)
#define BMI088_CONFIG_GYRO_BANDWIDTH_HZ     UINT16_C(47)
#endif

#define BMI088_CONFIG_ACCEL_RANGE_G         UINT16_C(24)
#define BMI088_CONFIG_GYRO_RANGE_DPS        UINT16_C(2000)

#define BMI088_ACCEL_FIFO_WATERMARK_FRAMES  UINT16_C(4)
#define BMI088_ACCEL_FIFO_WATERMARK_BYTES   \
    (BMI088_ACCEL_FIFO_WATERMARK_FRAMES * UINT16_C(7))
#define BMI088_GYRO_FIFO_WATERMARK_FRAMES   UINT16_C(1)
#define BMI088_FIFO_MAX_BATCH_SAMPLES       UINT16_C(64)
#define BMI088_ACCEL_SENSOR_TIME_TICK_NS    UINT32_C(39063)
#define BMI088_ACCEL_REGISTER_SNAPSHOT_SIZE UINT8_C(16)

typedef enum
{
    BMI088_FIFO_BATCH_FLAG_NONE = 0,
    BMI088_FIFO_BATCH_FLAG_SENSOR_TIME_VALID = (1U << 0),
    BMI088_FIFO_BATCH_FLAG_SKIPPED = (1U << 1),
    BMI088_FIFO_BATCH_FLAG_OVERRUN = (1U << 2),
    BMI088_FIFO_BATCH_FLAG_TRUNCATED = (1U << 3),
    BMI088_FIFO_BATCH_FLAG_PARSE_ERROR = (1U << 4),
    BMI088_FIFO_BATCH_FLAG_TIMESTAMP_ESTIMATED = (1U << 5),
    BMI088_FIFO_BATCH_FLAG_CAPTURE_MISMATCH = (1U << 6)
} bmi088_fifo_batch_flag_t;

typedef struct
{
    imu_accel_sample_t samples[BMI088_FIFO_MAX_BATCH_SAMPLES];
    uint64_t sensor_time_ticks;
    uint64_t transfer_complete_timestamp_us;
    uint32_t flags;
    uint16_t count;
    uint16_t fifo_bytes;
    uint8_t skipped_frame_count;
} bmi088_accel_fifo_batch_t;

typedef struct
{
    imu_gyro_sample_t samples[BMI088_FIFO_MAX_BATCH_SAMPLES];
    uint64_t latest_capture_timestamp_us;
    uint64_t transfer_complete_timestamp_us;
    uint32_t flags;
    uint16_t count;
    uint16_t fifo_frames;
    uint16_t capture_event_count;
} bmi088_gyro_fifo_batch_t;

typedef struct
{
    uint32_t accel_request_count;
    uint32_t gyro_request_count;
    uint32_t accel_fallback_request_count;
    uint32_t gyro_fallback_request_count;
    uint32_t dma_submit_busy_count;
    uint32_t dma_error_count;
    uint32_t accel_batch_count;
    uint32_t gyro_batch_count;
    uint32_t accel_sample_count;
    uint32_t gyro_sample_count;
    uint32_t accel_skipped_frame_count;
    uint32_t accel_parse_error_count;
    uint32_t gyro_overrun_count;
    uint32_t gyro_capture_mismatch_count;
    uint32_t gyro_capture_queue_overflow_count;
    uint32_t gyro_capture_mismatch_reason;
    uint32_t gyro_warmup_discard_count;
    uint32_t accel_clock_stale_reset_count;
    uint32_t accel_clock_causal_reset_count;
    uint32_t accel_timeline_reset_count;
    uint32_t accel_length_read_count;
    uint32_t accel_empty_length_count;
    uint16_t accel_last_fifo_bytes;
    uint16_t accel_peak_fifo_bytes;
    uint8_t accel_last_length_response[4];
    uint8_t accel_register_snapshot[BMI088_ACCEL_REGISTER_SNAPSHOT_SIZE];
    uint32_t accel_sensor_time_before;
    uint32_t accel_sensor_time_after;
    int16_t accel_direct_raw[3];
    uint16_t accel_initial_fifo_bytes;
    bool accel_snapshot_valid;
    bool gyro_capture_sync_fault;
} bmi088_fifo_diagnostics_t;

typedef struct
{
    uint16_t accel_odr_hz;
    /* Nominal sensor bandwidth; verify the full installed response by sweep/PSD. */
    uint16_t accel_bandwidth_hz;
    uint16_t accel_range_g;
    uint16_t gyro_odr_hz;
    uint16_t gyro_bandwidth_hz;
    uint16_t gyro_range_dps;
    uint16_t accel_fifo_watermark_bytes;
    uint16_t gyro_fifo_watermark_frames;
    bool accel_fifo_enabled;
    bool gyro_fifo_enabled;
    bool register_readback_verified;
} bmi088_configuration_t;

#ifdef __cplusplus
extern "C" {
#endif

bool bmi088_init(void);
/* Starts gyro acquisition from the FIFO/capture boundary prepared by init. */
bool bmi088_fifo_start_gyro(void);
bool bmi088_check_configuration(void);
bool bmi088_check_accel_configuration(void);
bool bmi088_check_gyro_configuration(void);
bool bmi088_read_accel(uint64_t timestamp_us, imu_accel_sample_t *sample);
bool bmi088_read_gyro(uint64_t timestamp_us, imu_gyro_sample_t *sample);
bool bmi088_read(uint64_t timestamp_us, imu_sample_t *sample);

/* TIM5_CH2 stores each gyro capture in the ISR-safe ring. IRQ events consumed
 * by the main loop only request DMA; service and parsing remain thread-only. */
bool bmi088_fifo_request_accel(uint64_t capture_timestamp_us);
bool bmi088_fifo_request_gyro(uint64_t capture_timestamp_us);
void bmi088_fifo_capture_gyro_isr(uint64_t capture_timestamp_us,
                                  uint32_t capture_sequence);
void bmi088_fifo_service(uint64_t now_us);
bool bmi088_fifo_pop_accel_batch(bmi088_accel_fifo_batch_t *batch);
bool bmi088_fifo_pop_gyro_batch(bmi088_gyro_fifo_batch_t *batch);
bool bmi088_fifo_dma_busy(void);
bool bmi088_fifo_gyro_sync_faulted(void);
void bmi088_fifo_get_diagnostics(bmi088_fifo_diagnostics_t *diagnostics);
void bmi088_fifo_get_clock_sync_diagnostics(
    imu_clock_sync_diagnostics_t *diagnostics);

void bmi088_get_health(imu_health_t *health);
void bmi088_get_configuration(bmi088_configuration_t *configuration);
uint8_t bmi088_accel_chip_id(void);
uint8_t bmi088_gyro_chip_id(void);

#ifdef __cplusplus
}
#endif

#endif
