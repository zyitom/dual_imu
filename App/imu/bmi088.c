#include "bmi088.h"

#include "bmi08x.h"
#include "bmi088_fifo_guard.h"
#include "imu_clock_sync.h"
#include "imu_spi_dma.h"
#include "imu_stream_buffer.h"
#include "imu_time.h"
#include "main.h"
#include "spi.h"

#include <limits.h>
#include <string.h>

#define BMI088_SPI_TIMEOUT_MS               UINT32_C(5)
#define BMI088_CONFIG_WRITE_LEN             UINT16_C(32)
#define BMI088_POWER_UP_DELAY_US             UINT32_C(1000)
#define BMI088_ACCEL_SCALE_MPS2_PER_LSB      ((24.0f * 9.80665f) / 32768.0f)
#define BMI088_GYRO_SCALE_RAD_S_PER_LSB      ((2000.0f * 0.017453292519943295f) / 32768.0f)
#define BMI088_TEMPERATURE_READ_DIVIDER       UINT32_C(512)
#define BMI088_FIFO_DMA_TIMEOUT_MS             UINT32_C(5)
#define BMI088_ACCEL_FIFO_FRAME_BYTES          UINT16_C(7)
#define BMI088_GYRO_FIFO_FRAME_BYTES           UINT16_C(6)
#define BMI088_ACCEL_FIFO_OVERREAD_BYTES       UINT16_C(18)
#define BMI088_ACCEL_FIFO_MAX_CONTENT_BYTES    \
    ((BMI088_FIFO_MAX_BATCH_SAMPLES - UINT16_C(2)) * \
     BMI088_ACCEL_FIFO_FRAME_BYTES)
#define BMI088_ACCEL_SAMPLE_PERIOD_US           \
    (UINT64_C(1000000) / BMI088_CONFIG_ACCEL_ODR_HZ)
#define BMI088_GYRO_SAMPLE_PERIOD_US            \
    (UINT64_C(1000000) / BMI088_CONFIG_GYRO_ODR_HZ)
#define BMI088_ACCEL_FALLBACK_PERIOD_US         \
    (UINT64_C(2) * BMI088_ACCEL_FIFO_WATERMARK_FRAMES * \
     BMI088_ACCEL_SAMPLE_PERIOD_US)
#define BMI088_GYRO_FALLBACK_PERIOD_US          \
    (UINT64_C(2) * BMI088_GYRO_SAMPLE_PERIOD_US)
#define BMI088_SENSOR_TIME_MASK                 UINT32_C(0x00FFFFFF)
#define BMI088_ACCEL_SAMPLE_PERIOD_TICKS        \
    (UINT64_C(25600) / BMI088_CONFIG_ACCEL_ODR_HZ)
#define BMI088_ACCEL_SENSOR_TIME_TICK_US         (39.0625)
#define BMI088_CLOCK_REFERENCE_MAX_AGE_US         UINT64_C(20000)
#define BMI088_CLOCK_CAUSAL_TOLERANCE_US           UINT64_C(1000)
#define BMI088_ACCEL_FIFO_LENGTH_WIRE_BYTES      UINT16_C(4)
#define BMI088_GYRO_FIFO_STATUS_WIRE_BYTES       UINT16_C(2)
#define BMI088_SPI_COMMAND_BYTES                 UINT16_C(1)
#define BMI088_ACCEL_SPI_DUMMY_BYTES             UINT16_C(1)
#define BMI088_GYRO_FIFO_WM_INTERRUPT_DISABLED   UINT8_C(0x08)
#define BMI088_GYRO_START_SYNC_MAX_ATTEMPTS      UINT32_C(4)
#define BMI088_GYRO_START_SYNC_TIMEOUT_US        UINT64_C(2000)
#define BMI088_ACCEL_FIFO_DOWNS_REGISTER          UINT8_C(0x80)
#define BMI088_ACCEL_FIFO_CONFIG_0_REGISTER       UINT8_C(0x02)
#define BMI088_ACCEL_FIFO_CONFIG_1_REGISTER       UINT8_C(0x50)
#define BMI088_GYRO_CAPTURE_REASON_QUEUE_SHORT    (UINT32_C(1) << 0)
#define BMI088_GYRO_CAPTURE_REASON_SEQUENCE       (UINT32_C(1) << 1)
#define BMI088_GYRO_CAPTURE_REASON_TIMESTAMP      (UINT32_C(1) << 2)
#define BMI088_GYRO_CAPTURE_REASON_FRAME_COUNT    (UINT32_C(1) << 3)
#define BMI088_GYRO_CAPTURE_REASON_TRUNCATED      (UINT32_C(1) << 4)
#define BMI088_GYRO_CAPTURE_REASON_FIFO_OVERRUN   (UINT32_C(1) << 5)
#define BMI088_GYRO_CAPTURE_REASON_QUEUE_OVERFLOW (UINT32_C(1) << 6)
#define BMI088_GYRO_CAPTURE_REASON_DMA_TRANSFER   (UINT32_C(1) << 7)

#if BMI088_USE_ROBUST_HIGH_RATE_PROFILE
#define BMI088_ACCEL_ODR_SETTING              BMI08_ACCEL_ODR_1600_HZ
#define BMI088_GYRO_ODR_SETTING               BMI08_GYRO_BW_230_ODR_2000_HZ
#else
#define BMI088_ACCEL_ODR_SETTING              BMI08_ACCEL_ODR_400_HZ
#define BMI088_GYRO_ODR_SETTING               BMI08_GYRO_BW_47_ODR_400_HZ
#endif

#define BMI088_ACCEL_BW_SETTING               BMI08_ACCEL_BW_OSR4
#define BMI088_ACCEL_RANGE_SETTING            BMI088_ACCEL_RANGE_24G
#define BMI088_GYRO_RANGE_SETTING             BMI08_GYRO_RANGE_2000_DPS

typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} bmi088_spi_target_t;

typedef enum
{
    BMI088_DMA_IDLE = 0,
    BMI088_DMA_ACCEL_LENGTH,
    BMI088_DMA_ACCEL_DATA,
    BMI088_DMA_GYRO_STATUS,
    BMI088_DMA_GYRO_DATA
} bmi088_dma_state_t;

static struct bmi08_dev bmi088_device;
static imu_health_t bmi088_health = { .status = IMU_STATUS_NOT_INITIALIZED };
static bool bmi088_initialized;
static uint32_t bmi088_sequence;
static uint32_t bmi088_accel_sequence;
static uint32_t bmi088_gyro_sequence;
static uint32_t bmi088_temperature_countdown;
static float bmi088_temperature_c;
static uint8_t bmi088_spi_tx_buffer[BMI08_MAX_LEN + 1U];
static uint8_t bmi088_spi_rx_buffer[BMI08_MAX_LEN + 1U];
static bmi088_configuration_t bmi088_configuration;
static volatile bmi088_dma_state_t bmi088_dma_state;
static volatile bool bmi088_accel_request_pending;
static volatile bool bmi088_gyro_request_pending;
static volatile uint64_t bmi088_accel_request_timestamp_us;
static volatile uint64_t bmi088_gyro_request_timestamp_us;
static uint64_t bmi088_last_accel_request_us;
static uint64_t bmi088_last_gyro_request_us;
static uint64_t bmi088_active_request_timestamp_us;
static uint16_t bmi088_active_accel_fifo_bytes;
static uint16_t bmi088_active_gyro_fifo_frames;
static uint16_t bmi088_active_gyro_fifo_total_frames;
static uint16_t bmi088_accel_raw_wire_length;
static uint16_t bmi088_gyro_raw_wire_length;
static uint16_t bmi088_accel_raw_fifo_bytes;
static uint16_t bmi088_gyro_raw_fifo_frames;
static uint64_t bmi088_accel_raw_complete_us;
static uint64_t bmi088_gyro_raw_complete_us;
static uint64_t bmi088_accel_raw_request_timestamp_us;
static uint8_t bmi088_gyro_raw_overrun;
static bool bmi088_accel_raw_truncated;
static bool bmi088_gyro_raw_truncated;
static volatile bool bmi088_accel_raw_ready;
static volatile bool bmi088_gyro_raw_ready;
static bool bmi088_accel_batch_ready;
static bool bmi088_gyro_batch_ready;
static uint8_t bmi088_dma_tx[IMU_SPI_DMA_MAX_TRANSFER_SIZE];
static uint8_t bmi088_dma_control_rx[4];
static uint8_t bmi088_accel_raw_rx[IMU_SPI_DMA_MAX_TRANSFER_SIZE];
static uint8_t bmi088_gyro_raw_rx[IMU_SPI_DMA_MAX_TRANSFER_SIZE];
static bmi088_accel_fifo_batch_t bmi088_accel_batch;
static bmi088_gyro_fifo_batch_t bmi088_gyro_batch;
static bmi088_fifo_diagnostics_t bmi088_fifo_diagnostics;
static imu_timestamp_queue_t bmi088_gyro_capture_queue;
static volatile bool bmi088_gyro_capture_armed;
static volatile bool bmi088_gyro_capture_sync_fault;
static uint64_t bmi088_gyro_capture_timestamps[BMI088_FIFO_MAX_BATCH_SAMPLES];
static uint32_t bmi088_gyro_capture_sequences[BMI088_FIFO_MAX_BATCH_SAMPLES];
static imu_clock_sync_t bmi088_accel_clock_sync;
static struct bmi08_sensor_data
    bmi088_accel_parse_raw[BMI088_FIFO_MAX_BATCH_SAMPLES];
static uint64_t bmi088_last_accel_sample_tick;
static uint64_t bmi088_last_gyro_sample_us;
static uint64_t bmi088_gyro_valid_after_us;
static uint8_t bmi088_accel_sample_phase_tick;
static bool bmi088_last_accel_sample_tick_valid;
static bool bmi088_last_gyro_sample_us_valid;
static bool bmi088_last_gyro_capture_sequence_valid;
static uint32_t bmi088_last_gyro_capture_sequence;
static bool bmi088_accel_sample_phase_valid;
static bool bmi088_prefer_gyro;

_Static_assert((BMI088_SPI_COMMAND_BYTES + BMI088_ACCEL_SPI_DUMMY_BYTES +
                BMI088_ACCEL_FIFO_MAX_CONTENT_BYTES +
                BMI088_ACCEL_FIFO_OVERREAD_BYTES) <=
                   IMU_SPI_DMA_MAX_TRANSFER_SIZE,
               "BMI088 accel FIFO transfer must fit the DMA staging buffer");
_Static_assert((BMI088_SPI_COMMAND_BYTES +
                (BMI088_FIFO_MAX_BATCH_SAMPLES *
                 BMI088_GYRO_FIFO_FRAME_BYTES)) <=
                   IMU_SPI_DMA_MAX_TRANSFER_SIZE,
               "BMI088 gyro FIFO transfer must fit the DMA staging buffer");
_Static_assert((UINT32_C(1000000) % BMI088_CONFIG_ACCEL_ODR_HZ) == 0U,
               "BMI088 accel ODR must have an integral microsecond period");
_Static_assert((UINT32_C(1000000) % BMI088_CONFIG_GYRO_ODR_HZ) == 0U,
               "BMI088 gyro ODR must have an integral microsecond period");
_Static_assert((UINT32_C(25600) % BMI088_CONFIG_ACCEL_ODR_HZ) == 0U,
               "BMI088 accel ODR must align with the sensor-time clock");

static bmi088_spi_target_t bmi088_accel_target = {
    .spi = &hspi1,
    .cs_port = BMI088_0_ACS_GPIO_Port,
    .cs_pin = BMI088_0_ACS_Pin
};

static bmi088_spi_target_t bmi088_gyro_target = {
    .spi = &hspi1,
    .cs_port = BMI088_0_GCS_GPIO_Port,
    .cs_pin = BMI088_0_GCS_Pin
};

static BMI08_INTF_RET_TYPE bmi088_spi_read(uint8_t reg_addr,
                                            uint8_t *reg_data,
                                            uint32_t len,
                                            void *intf_ptr);
static BMI08_INTF_RET_TYPE bmi088_spi_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t len,
                                             void *intf_ptr);
static void bmi088_delay_us(uint32_t period_us, void *intf_ptr);
static bool bmi088_init_step(int8_t result, imu_status_t failure_status);
static bool bmi088_read_step(int8_t result);
static bool bmi088_verify_accel_configuration(void);
static bool bmi088_verify_gyro_configuration(void);
static bool bmi088_verify_fifo_configuration(void);
static bool bmi088_verify_configuration(void);
static bool bmi088_configure_fifo(void);
static bool bmi088_set_gyro_fifo_bypass(void);
static bool bmi088_sync_gyro_fifo_to_capture(uint32_t *mismatch_reason);
static bool bmi088_soft_reset_devices(void);
static bool bmi088_blocking_bus_available(void);
static void bmi088_fifo_kick(void);
static bool bmi088_submit_accel_length(void);
static bool bmi088_submit_accel_data(uint16_t fifo_bytes);
static bool bmi088_submit_gyro_status(void);
static bool bmi088_submit_gyro_data(uint16_t fifo_frames);
static void bmi088_dma_callback(SPI_HandleTypeDef *spi,
                                imu_spi_dma_status_t status,
                                uint16_t length,
                                void *context);
static void bmi088_process_accel_fifo(void);
static void bmi088_process_gyro_fifo(void);
static void bmi088_requeue_truncated_accel(void);
static bool bmi088_scan_accel_fifo_metadata(const struct bmi08_fifo_frame *fifo,
                                            uint32_t *sensor_time,
                                            bool *sensor_time_valid,
                                            uint8_t *skipped_frames,
                                            bool *sample_dropped,
                                            uint16_t *accel_frame_count);
static void bmi088_record_dma_error(imu_spi_dma_status_t status);
static void bmi088_requeue_active_request(bmi088_dma_state_t failed_state);
static bool bmi088_capture_accel_diagnostics(void);
static void bmi088_latch_gyro_capture_fault(uint32_t reason);
static bool bmi088_gyro_capture_epoch_is_current(uint32_t expected_epoch);

bool bmi088_init(void)
{
    struct bmi08_accel_int_channel_cfg accel_int_config = {
        .int_channel = BMI08_INT_CHANNEL_1,
        .int_type = BMI08_ACCEL_INT_FIFO_WM,
        .int_pin_cfg = {
            .lvl = BMI08_INT_ACTIVE_HIGH,
            .output_mode = BMI08_INT_MODE_PUSH_PULL,
            .enable_int_pin = BMI08_ENABLE
        }
    };
    struct bmi08_gyro_int_channel_cfg gyro_int_config = {
        /* DM-FC01 routes BMI088 gyro INT3 to PA1. */
        .int_channel = BMI08_INT_CHANNEL_3,
        .int_type = BMI08_GYRO_INT_DATA_RDY,
        .int_pin_cfg = {
            .lvl = BMI08_INT_ACTIVE_HIGH,
            .output_mode = BMI08_INT_MODE_PUSH_PULL,
            .enable_int_pin = BMI08_ENABLE
        }
    };
    int8_t result;
    const imu_clock_sync_config_t accel_clock_config = {
        .counter_bits = 24U,
        .nominal_tick_us = 39.0625,
        .minimum_clock_scale = 0.95,
        .maximum_clock_scale = 1.05,
        .phase_time_constant_us = 50000.0,
        .rate_time_constant_us = 2000000.0,
        .minimum_anchor_interval_us = 2000.0,
        .initial_residual_limit_us = 1000.0,
    };

    memset(&bmi088_device, 0, sizeof(bmi088_device));
    memset(&bmi088_health, 0, sizeof(bmi088_health));
    bmi088_health.status = IMU_STATUS_NOT_INITIALIZED;
    bmi088_initialized = false;
    bmi088_sequence = 0;
    bmi088_accel_sequence = 0U;
    bmi088_gyro_sequence = 0U;
    bmi088_temperature_countdown = 0U;
    bmi088_temperature_c = 0.0f;
    bmi088_dma_state = BMI088_DMA_IDLE;
    bmi088_accel_request_pending = false;
    bmi088_gyro_request_pending = false;
    bmi088_accel_request_timestamp_us = 0U;
    bmi088_gyro_request_timestamp_us = 0U;
    bmi088_last_accel_request_us = 0U;
    bmi088_last_gyro_request_us = 0U;
    bmi088_active_request_timestamp_us = 0U;
    bmi088_active_accel_fifo_bytes = 0U;
    bmi088_active_gyro_fifo_frames = 0U;
    bmi088_active_gyro_fifo_total_frames = 0U;
    bmi088_accel_raw_wire_length = 0U;
    bmi088_gyro_raw_wire_length = 0U;
    bmi088_accel_raw_fifo_bytes = 0U;
    bmi088_gyro_raw_fifo_frames = 0U;
    bmi088_accel_raw_complete_us = 0U;
    bmi088_gyro_raw_complete_us = 0U;
    bmi088_accel_raw_request_timestamp_us = 0U;
    bmi088_gyro_raw_overrun = 0U;
    bmi088_accel_raw_ready = false;
    bmi088_gyro_raw_ready = false;
    bmi088_accel_raw_truncated = false;
    bmi088_gyro_raw_truncated = false;
    bmi088_accel_batch_ready = false;
    bmi088_gyro_batch_ready = false;
    memset(&bmi088_accel_batch, 0, sizeof(bmi088_accel_batch));
    memset(&bmi088_gyro_batch, 0, sizeof(bmi088_gyro_batch));
    memset(&bmi088_fifo_diagnostics, 0, sizeof(bmi088_fifo_diagnostics));
    imu_timestamp_queue_reset(&bmi088_gyro_capture_queue);
    bmi088_gyro_capture_armed = false;
    bmi088_gyro_capture_sync_fault = false;
    memset(bmi088_dma_tx, 0, sizeof(bmi088_dma_tx));
    memset(bmi088_dma_control_rx, 0, sizeof(bmi088_dma_control_rx));
    memset(bmi088_accel_raw_rx, 0, sizeof(bmi088_accel_raw_rx));
    memset(bmi088_gyro_raw_rx, 0, sizeof(bmi088_gyro_raw_rx));
    memset(bmi088_accel_parse_raw, 0, sizeof(bmi088_accel_parse_raw));
    bmi088_last_accel_sample_tick = 0U;
    bmi088_last_gyro_sample_us = 0U;
    bmi088_gyro_valid_after_us = 0U;
    bmi088_accel_sample_phase_tick = 0U;
    bmi088_last_accel_sample_tick_valid = false;
    bmi088_last_gyro_sample_us_valid = false;
    bmi088_last_gyro_capture_sequence_valid = false;
    bmi088_last_gyro_capture_sequence = 0U;
    bmi088_accel_sample_phase_valid = false;
    bmi088_prefer_gyro = true;
    if (!imu_clock_sync_init(&bmi088_accel_clock_sync, &accel_clock_config))
    {
        bmi088_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }
    bmi088_configuration = (bmi088_configuration_t) {
        .accel_odr_hz = BMI088_CONFIG_ACCEL_ODR_HZ,
        .accel_bandwidth_hz = BMI088_CONFIG_ACCEL_BANDWIDTH_HZ,
        .accel_range_g = BMI088_CONFIG_ACCEL_RANGE_G,
        .gyro_odr_hz = BMI088_CONFIG_GYRO_ODR_HZ,
        .gyro_bandwidth_hz = BMI088_CONFIG_GYRO_BANDWIDTH_HZ,
        .gyro_range_dps = BMI088_CONFIG_GYRO_RANGE_DPS,
        .accel_fifo_watermark_bytes = BMI088_ACCEL_FIFO_WATERMARK_BYTES,
        .gyro_fifo_watermark_frames = BMI088_GYRO_FIFO_WATERMARK_FRAMES,
        .accel_fifo_enabled = true,
        .gyro_fifo_enabled = true,
        .register_readback_verified = false,
    };

    if ((hspi1.Instance != SPI1) || !imu_time_is_running())
    {
        bmi088_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    HAL_GPIO_WritePin(BMI088_0_ACS_GPIO_Port, BMI088_0_ACS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_0_GCS_GPIO_Port, BMI088_0_GCS_Pin, GPIO_PIN_SET);
    bmi088_delay_us(BMI088_POWER_UP_DELAY_US, NULL);

    bmi088_device.intf = BMI08_SPI_INTF;
    bmi088_device.variant = BMI088_VARIANT;
    bmi088_device.intf_ptr_accel = &bmi088_accel_target;
    bmi088_device.intf_ptr_gyro = &bmi088_gyro_target;
    bmi088_device.read = bmi088_spi_read;
    bmi088_device.write = bmi088_spi_write;
    bmi088_device.delay_us = bmi088_delay_us;
    bmi088_device.read_write_len = BMI088_CONFIG_WRITE_LEN;

    result = bmi08xa_init(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_BAD_ID))
    {
        return false;
    }

    if (bmi088_device.accel_chip_id != BMI088_ACCEL_CHIP_ID)
    {
        bmi088_health.status = IMU_STATUS_BAD_ID;
        return false;
    }

    result = bmi08g_init(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_BAD_ID))
    {
        return false;
    }

    if (bmi088_device.gyro_chip_id != BMI08_GYRO_CHIP_ID)
    {
        bmi088_health.status = IMU_STATUS_BAD_ID;
        return false;
    }

    if (!bmi088_soft_reset_devices())
    {
        return false;
    }

    result = bmi08a_load_config_file(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    bmi088_device.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    result = bmi08a_set_power_mode(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    bmi088_device.accel_cfg.odr = BMI088_ACCEL_ODR_SETTING;
    bmi088_device.accel_cfg.bw = BMI088_ACCEL_BW_SETTING;
    bmi088_device.accel_cfg.range = BMI088_ACCEL_RANGE_SETTING;
    result = bmi08xa_set_meas_conf(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    bmi088_device.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    result = bmi08g_set_power_mode(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    bmi088_device.gyro_cfg.odr = BMI088_GYRO_ODR_SETTING;
    bmi088_device.gyro_cfg.bw = BMI088_GYRO_ODR_SETTING;
    bmi088_device.gyro_cfg.range = BMI088_GYRO_RANGE_SETTING;
    result = bmi08g_set_meas_conf(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    if (!bmi088_configure_fifo())
    {
        return false;
    }

    /* DM-FC01 and its vendor PX4 target use BMI088 accel INT1 on PA0. */
    result = bmi08a_set_int_config(&accel_int_config, &bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    result = bmi08g_set_int_config(&gyro_int_config, &bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    if (!bmi088_capture_accel_diagnostics())
    {
        bmi088_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    if (!bmi088_verify_configuration())
    {
        return false;
    }

    /* Leave the gyro stopped. FIFO configuration writes do not take effect in
     * SUSPEND on this device. After wake and the NORMAL-mode settle delay,
     * bmi088_fifo_start_gyro() switches to BYPASS to clear the wake prefix and
     * establishes the capture/FIFO boundary before STREAM is enabled. */
    bmi088_device.gyro_cfg.power = BMI08_GYRO_PM_SUSPEND;
    result = bmi08g_set_power_mode(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }
    bmi088_delay_us(UINT32_C(1000), bmi088_device.intf_ptr_gyro);
    imu_timestamp_queue_reset(&bmi088_gyro_capture_queue);
    bmi088_last_gyro_capture_sequence = 0U;
    bmi088_last_gyro_capture_sequence_valid = false;
    bmi088_last_gyro_sample_us = 0U;
    bmi088_last_gyro_sample_us_valid = false;

    bmi088_initialized = true;
    bmi088_health.status = IMU_STATUS_OK;

    return true;
}

bool bmi088_fifo_start_gyro(void)
{
    if (!bmi088_initialized ||
        (bmi088_device.gyro_cfg.power != BMI08_GYRO_PM_SUSPEND) ||
        (bmi088_dma_state != BMI088_DMA_IDLE) ||
        imu_spi_dma_is_busy(&hspi1))
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bmi088_gyro_capture_armed = false;
    imu_timestamp_queue_reset(&bmi088_gyro_capture_queue);
    bmi088_gyro_capture_sync_fault = false;
    bmi088_gyro_request_pending = false;
    bmi088_gyro_request_timestamp_us = 0U;
    bmi088_gyro_raw_ready = false;
    bmi088_gyro_batch_ready = false;
    bmi088_last_gyro_capture_sequence = 0U;
    bmi088_last_gyro_capture_sequence_valid = false;
    bmi088_last_gyro_sample_us = 0U;
    bmi088_last_gyro_sample_us_valid = false;
    bmi088_fifo_diagnostics.gyro_capture_mismatch_reason = 0U;
    bmi088_fifo_diagnostics.gyro_capture_sync_fault = false;
    __set_PRIMASK(primask);

    bmi088_device.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    if (!bmi088_init_step(bmi08g_set_power_mode(&bmi088_device),
                          IMU_STATUS_CONFIG_ERROR))
    {
        bmi088_latch_gyro_capture_fault(
            BMI088_GYRO_CAPTURE_REASON_FRAME_COUNT);
        return false;
    }

    uint32_t mismatch_reason = 0U;
    if (!bmi088_sync_gyro_fifo_to_capture(&mismatch_reason))
    {
        bmi088_latch_gyro_capture_fault(mismatch_reason);
        return false;
    }
    if (!bmi088_verify_fifo_configuration())
    {
        bmi088_latch_gyro_capture_fault(
            BMI088_GYRO_CAPTURE_REASON_FRAME_COUNT);
        return false;
    }

    /* The wake-up prefix was cleared at the BYPASS boundary. Only samples
     * produced after final configuration readback may enter the estimator. */
    bmi088_gyro_valid_after_us = imu_time_now_us();
    bmi088_last_gyro_request_us = bmi088_gyro_valid_after_us;
    return true;
}

bool bmi088_check_configuration(void)
{
    if (!bmi088_initialized)
        return false;

    const bool accel_ok = bmi088_check_accel_configuration();
    const bool gyro_ok = bmi088_check_gyro_configuration();
    if (!accel_ok || !gyro_ok)
        return false;

    bmi088_health.status = IMU_STATUS_OK;
    return true;
}

bool bmi088_check_accel_configuration(void)
{
    if (!bmi088_initialized)
        return false;
    if (!bmi088_blocking_bus_available())
        return bmi088_configuration.register_readback_verified;

    const int8_t result = bmi08a_init(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_BAD_ID) ||
        (bmi088_device.accel_chip_id != BMI088_ACCEL_CHIP_ID)) {
        bmi088_health.status = IMU_STATUS_BAD_ID;
        return false;
    }
    return bmi088_verify_accel_configuration();
}

bool bmi088_check_gyro_configuration(void)
{
    if (!bmi088_initialized)
        return false;
    if (!bmi088_blocking_bus_available())
        return bmi088_configuration.register_readback_verified;

    const int8_t result = bmi08g_init(&bmi088_device);
    if (!bmi088_init_step(result, IMU_STATUS_BAD_ID) ||
        (bmi088_device.gyro_chip_id != BMI08_GYRO_CHIP_ID)) {
        bmi088_health.status = IMU_STATUS_BAD_ID;
        return false;
    }
    return bmi088_verify_gyro_configuration();
}

bool bmi088_read_accel(uint64_t timestamp_us, imu_accel_sample_t *sample)
{
    struct bmi08_sensor_data accel_raw = { 0 };

    if (sample == NULL)
    {
        return false;
    }

    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = timestamp_us;
    sample->source = IMU_SOURCE_BMI088;

    if (!bmi088_initialized)
    {
        bmi088_health.status = IMU_STATUS_NOT_INITIALIZED;
        return false;
    }
    if (!bmi088_blocking_bus_available())
        return false;

    if (!bmi088_read_step(bmi08a_get_data(&accel_raw, &bmi088_device)))
    {
        return false;
    }

    if (bmi088_temperature_countdown == 0U)
    {
        int32_t temperature_milli_c;
        if (!bmi088_read_step(
                bmi08a_get_sensor_temperature(&bmi088_device, &temperature_milli_c)))
        {
            return false;
        }
        bmi088_temperature_c = (float)temperature_milli_c / 1000.0f;
        bmi088_temperature_countdown = BMI088_TEMPERATURE_READ_DIVIDER;
    }
    else
    {
        bmi088_temperature_countdown--;
    }

    sample->accel_mps2[0] = (float)accel_raw.x * BMI088_ACCEL_SCALE_MPS2_PER_LSB;
    sample->accel_mps2[1] = (float)accel_raw.y * BMI088_ACCEL_SCALE_MPS2_PER_LSB;
    sample->accel_mps2[2] = (float)accel_raw.z * BMI088_ACCEL_SCALE_MPS2_PER_LSB;
    sample->temperature_c = bmi088_temperature_c;
    sample->sequence = ++bmi088_accel_sequence;
    sample->valid = true;

    bmi088_health.status = IMU_STATUS_OK;
    bmi088_health.read_count++;
    bmi088_health.last_sample_us = timestamp_us;

    return true;
}

bool bmi088_read_gyro(uint64_t timestamp_us, imu_gyro_sample_t *sample)
{
    struct bmi08_sensor_data gyro_raw;

    if (sample == NULL)
    {
        return false;
    }

    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = timestamp_us;
    sample->source = IMU_SOURCE_BMI088;

    if (!bmi088_initialized)
    {
        bmi088_health.status = IMU_STATUS_NOT_INITIALIZED;
        return false;
    }
    if (!bmi088_blocking_bus_available())
        return false;

    if (!bmi088_read_step(bmi08g_get_data(&gyro_raw, &bmi088_device)))
    {
        return false;
    }

    sample->gyro_rad_s[0] = (float)gyro_raw.x * BMI088_GYRO_SCALE_RAD_S_PER_LSB;
    sample->gyro_rad_s[1] = (float)gyro_raw.y * BMI088_GYRO_SCALE_RAD_S_PER_LSB;
    sample->gyro_rad_s[2] = (float)gyro_raw.z * BMI088_GYRO_SCALE_RAD_S_PER_LSB;
    sample->temperature_c = bmi088_temperature_c;
    sample->sequence = ++bmi088_gyro_sequence;
    sample->valid = true;

    bmi088_health.status = IMU_STATUS_OK;
    bmi088_health.read_count++;
    bmi088_health.last_sample_us = timestamp_us;

    return true;
}

bool bmi088_read(uint64_t timestamp_us, imu_sample_t *sample)
{
    imu_accel_sample_t accel;
    imu_gyro_sample_t gyro;

    if (sample == NULL)
    {
        return false;
    }

    memset(sample, 0, sizeof(*sample));
    if (!bmi088_read_accel(timestamp_us, &accel) ||
        !bmi088_read_gyro(timestamp_us, &gyro))
    {
        return false;
    }

    sample->timestamp_us = timestamp_us;
    sample->accel_timestamp_us = accel.timestamp_us;
    sample->gyro_timestamp_us = gyro.timestamp_us;
    sample->sequence = ++bmi088_sequence;
    sample->accel_sequence = accel.sequence;
    sample->gyro_sequence = gyro.sequence;
    memcpy(sample->accel_mps2, accel.accel_mps2, sizeof(sample->accel_mps2));
    memcpy(sample->gyro_rad_s, gyro.gyro_rad_s, sizeof(sample->gyro_rad_s));
    sample->temperature_c = accel.temperature_c;
    sample->source = IMU_SOURCE_BMI088;
    sample->accel_valid = true;
    sample->gyro_valid = true;
    sample->valid = true;
    return true;
}

bool bmi088_fifo_request_accel(uint64_t capture_timestamp_us)
{
    if (!bmi088_initialized)
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!bmi088_accel_request_pending ||
        ((bmi088_accel_request_timestamp_us == 0U) &&
         (capture_timestamp_us != 0U)))
    {
        bmi088_accel_request_timestamp_us = capture_timestamp_us;
        bmi088_accel_request_pending = true;
    }
    if (capture_timestamp_us != 0U)
        bmi088_last_accel_request_us = capture_timestamp_us;
    bmi088_fifo_diagnostics.accel_request_count++;
    __set_PRIMASK(primask);
    return true;
}

static void bmi088_latch_gyro_capture_fault(uint32_t reason)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bmi088_gyro_capture_armed = false;
    bmi088_gyro_capture_sync_fault = true;
    bmi088_gyro_request_pending = false;
    bmi088_gyro_request_timestamp_us = 0U;
    bmi088_last_gyro_capture_sequence_valid = false;
    bmi088_last_gyro_sample_us_valid = false;
    bmi088_fifo_diagnostics.gyro_capture_mismatch_reason |= reason;
    bmi088_fifo_diagnostics.gyro_capture_sync_fault = true;
    imu_timestamp_queue_discard(&bmi088_gyro_capture_queue);
    __set_PRIMASK(primask);
}

static bool bmi088_gyro_capture_epoch_is_current(uint32_t expected_epoch)
{
    return bmi088_fifo_capture_epoch_valid(
        expected_epoch,
        imu_timestamp_queue_discard_generation(&bmi088_gyro_capture_queue),
        bmi088_gyro_capture_sync_fault);
}

bool bmi088_fifo_request_gyro(uint64_t capture_timestamp_us)
{
    if (!bmi088_initialized || bmi088_gyro_capture_sync_fault)
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (bmi088_gyro_capture_sync_fault)
    {
        __set_PRIMASK(primask);
        return false;
    }
    bmi088_gyro_request_pending = true;
    bmi088_gyro_request_timestamp_us = capture_timestamp_us;
    if (capture_timestamp_us != 0U)
        bmi088_last_gyro_request_us = capture_timestamp_us;
    bmi088_fifo_diagnostics.gyro_request_count++;
    __set_PRIMASK(primask);
    return true;
}

void bmi088_fifo_capture_gyro_isr(uint64_t capture_timestamp_us,
                                  uint32_t capture_sequence)
{
    if (!bmi088_gyro_capture_armed || (capture_timestamp_us == 0U))
        return;

    if (!imu_timestamp_queue_push_event_isr(&bmi088_gyro_capture_queue,
                                            capture_timestamp_us,
                                            capture_sequence))
    {
        bmi088_fifo_diagnostics.gyro_capture_queue_overflow_count++;
        bmi088_latch_gyro_capture_fault(
            BMI088_GYRO_CAPTURE_REASON_QUEUE_OVERFLOW);
    }
}

void bmi088_fifo_service(uint64_t now_us)
{
    if (!bmi088_initialized)
        return;

    imu_spi_dma_service();

    if (bmi088_accel_raw_ready && !bmi088_accel_batch_ready)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const bool ready = bmi088_accel_raw_ready;
        bmi088_accel_raw_ready = false;
        __set_PRIMASK(primask);
        if (ready)
            bmi088_process_accel_fifo();
    }

    if (bmi088_gyro_raw_ready && !bmi088_gyro_batch_ready)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const bool ready = bmi088_gyro_raw_ready;
        bmi088_gyro_raw_ready = false;
        __set_PRIMASK(primask);
        if (ready)
            bmi088_process_gyro_fifo();
    }

    if ((now_us - bmi088_last_accel_request_us >=
         BMI088_ACCEL_FALLBACK_PERIOD_US) &&
        !bmi088_accel_request_pending &&
        (bmi088_dma_state != BMI088_DMA_ACCEL_LENGTH) &&
        (bmi088_dma_state != BMI088_DMA_ACCEL_DATA) &&
        !bmi088_accel_raw_ready && !bmi088_accel_batch_ready)
    {
        bmi088_last_accel_request_us = now_us;
        bmi088_fifo_diagnostics.accel_fallback_request_count++;
        (void)bmi088_fifo_request_accel(0U);
    }

    if (!bmi088_gyro_capture_sync_fault &&
        (now_us - bmi088_last_gyro_request_us >=
         BMI088_GYRO_FALLBACK_PERIOD_US) &&
        !bmi088_gyro_request_pending &&
        (bmi088_dma_state != BMI088_DMA_GYRO_STATUS) &&
        (bmi088_dma_state != BMI088_DMA_GYRO_DATA) &&
        !bmi088_gyro_raw_ready && !bmi088_gyro_batch_ready)
    {
        bmi088_last_gyro_request_us = now_us;
        bmi088_fifo_diagnostics.gyro_fallback_request_count++;
        (void)bmi088_fifo_request_gyro(0U);
    }

    bmi088_fifo_kick();
}

bool bmi088_fifo_pop_accel_batch(bmi088_accel_fifo_batch_t *batch)
{
    if (batch == NULL)
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!bmi088_accel_batch_ready)
    {
        __set_PRIMASK(primask);
        return false;
    }
    *batch = bmi088_accel_batch;
    __DMB();
    bmi088_accel_batch_ready = false;
    __set_PRIMASK(primask);
    return true;
}

bool bmi088_fifo_pop_gyro_batch(bmi088_gyro_fifo_batch_t *batch)
{
    if (batch == NULL)
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!bmi088_gyro_batch_ready)
    {
        __set_PRIMASK(primask);
        return false;
    }
    *batch = bmi088_gyro_batch;
    __DMB();
    bmi088_gyro_batch_ready = false;
    __set_PRIMASK(primask);
    return true;
}

bool bmi088_fifo_dma_busy(void)
{
    return (bmi088_dma_state != BMI088_DMA_IDLE) ||
           imu_spi_dma_is_busy(&hspi1);
}

bool bmi088_fifo_gyro_sync_faulted(void)
{
    return bmi088_gyro_capture_sync_fault;
}

void bmi088_fifo_get_diagnostics(bmi088_fifo_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
        return;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *diagnostics = bmi088_fifo_diagnostics;
    __set_PRIMASK(primask);
}

void bmi088_fifo_get_clock_sync_diagnostics(
    imu_clock_sync_diagnostics_t *diagnostics)
{
    imu_clock_sync_get_diagnostics(&bmi088_accel_clock_sync, diagnostics);
}

void bmi088_get_health(imu_health_t *health)
{
    if (health != NULL)
    {
        *health = bmi088_health;
    }
}

void bmi088_get_configuration(bmi088_configuration_t *configuration)
{
    if (configuration != NULL)
    {
        *configuration = bmi088_configuration;
    }
}

uint8_t bmi088_accel_chip_id(void)
{
    return bmi088_device.accel_chip_id;
}

uint8_t bmi088_gyro_chip_id(void)
{
    return bmi088_device.gyro_chip_id;
}

static BMI08_INTF_RET_TYPE bmi088_spi_read(uint8_t reg_addr,
                                            uint8_t *reg_data,
                                            uint32_t len,
                                            void *intf_ptr)
{
    bmi088_spi_target_t *target = (bmi088_spi_target_t *)intf_ptr;
    HAL_StatusTypeDef hal_result;

    if ((target == NULL) || (target->spi == NULL) || (reg_data == NULL) ||
        (len == 0U) || (len > BMI08_MAX_LEN))
    {
        return (BMI08_INTF_RET_TYPE)-1;
    }

    bmi088_spi_tx_buffer[0] = reg_addr;
    memset(&bmi088_spi_tx_buffer[1], 0, len);

    HAL_GPIO_WritePin(target->cs_port, target->cs_pin, GPIO_PIN_RESET);
    hal_result = HAL_SPI_TransmitReceive(target->spi, bmi088_spi_tx_buffer,
                                         bmi088_spi_rx_buffer, (uint16_t)(len + 1U),
                                         BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(target->cs_port, target->cs_pin, GPIO_PIN_SET);

    if (hal_result == HAL_OK)
    {
        /* Bosch includes the accel dummy byte in len; return it unchanged. */
        memcpy(reg_data, &bmi088_spi_rx_buffer[1], len);
    }

    return (hal_result == HAL_OK) ? BMI08_INTF_RET_SUCCESS : (BMI08_INTF_RET_TYPE)-1;
}

static BMI08_INTF_RET_TYPE bmi088_spi_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t len,
                                             void *intf_ptr)
{
    bmi088_spi_target_t *target = (bmi088_spi_target_t *)intf_ptr;
    HAL_StatusTypeDef hal_result;

    if ((target == NULL) || (target->spi == NULL) || (reg_data == NULL) ||
        (len == 0U) || (len > BMI08_MAX_LEN))
    {
        return (BMI08_INTF_RET_TYPE)-1;
    }

    bmi088_spi_tx_buffer[0] = reg_addr;
    memcpy(&bmi088_spi_tx_buffer[1], reg_data, len);

    HAL_GPIO_WritePin(target->cs_port, target->cs_pin, GPIO_PIN_RESET);
    hal_result = HAL_SPI_Transmit(target->spi, bmi088_spi_tx_buffer, (uint16_t)(len + 1U),
                                 BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(target->cs_port, target->cs_pin, GPIO_PIN_SET);

    return (hal_result == HAL_OK) ? BMI08_INTF_RET_SUCCESS : (BMI08_INTF_RET_TYPE)-1;
}

static void bmi088_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;
    imu_time_delay_us(period_us);
}

static bool bmi088_init_step(int8_t result, imu_status_t failure_status)
{
    if (result == BMI08_OK)
    {
        return true;
    }

    if (result == BMI08_E_DEV_NOT_FOUND)
    {
        bmi088_health.status = IMU_STATUS_BAD_ID;
    }
    else if (result == BMI08_E_COM_FAIL)
    {
        bmi088_health.status = IMU_STATUS_BUS_ERROR;
        bmi088_health.bus_error_count++;
    }
    else
    {
        bmi088_health.status = failure_status;
    }

    return false;
}

static bool bmi088_read_step(int8_t result)
{
    if (result == BMI08_OK)
    {
        return true;
    }

    bmi088_health.status = IMU_STATUS_BUS_ERROR;
    bmi088_health.bus_error_count++;

    return false;
}

static bool bmi088_verify_accel_configuration(void)
{
    int8_t result = bmi08a_get_meas_conf(&bmi088_device);

    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }
    if ((bmi088_device.accel_cfg.odr != BMI088_ACCEL_ODR_SETTING) ||
        (bmi088_device.accel_cfg.bw != BMI088_ACCEL_BW_SETTING) ||
        (bmi088_device.accel_cfg.range != BMI088_ACCEL_RANGE_SETTING))
    {
        bmi088_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    return true;
}

static bool bmi088_soft_reset_devices(void)
{
    uint8_t reset_command = BMI08_SOFT_RESET_CMD;

    if (!bmi088_init_step(
            bmi08a_get_set_regs(BMI08_REG_ACCEL_SOFTRESET, &reset_command,
                                sizeof(reset_command), &bmi088_device,
                                SET_FUNC),
            IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }
    bmi088_delay_us(BMI08_MS_TO_US(BMI08_ACCEL_SOFTRESET_DELAY_MS),
                    bmi088_device.intf_ptr_accel);

    if (!bmi088_init_step(
            bmi08g_set_regs(BMI08_REG_GYRO_SOFTRESET, &reset_command,
                            sizeof(reset_command), &bmi088_device),
            IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }
    bmi088_delay_us(BMI08_MS_TO_US(BMI08_GYRO_SOFTRESET_DELAY),
                    bmi088_device.intf_ptr_gyro);

    if (!bmi088_init_step(bmi08xa_init(&bmi088_device), IMU_STATUS_BAD_ID) ||
        (bmi088_device.accel_chip_id != BMI088_ACCEL_CHIP_ID) ||
        !bmi088_init_step(bmi08g_init(&bmi088_device), IMU_STATUS_BAD_ID) ||
        (bmi088_device.gyro_chip_id != BMI08_GYRO_CHIP_ID))
    {
        bmi088_health.status = IMU_STATUS_BAD_ID;
        return false;
    }

    return true;
}

static bool bmi088_verify_gyro_configuration(void)
{
    const int8_t result = bmi08g_get_meas_conf(&bmi088_device);

    if (!bmi088_init_step(result, IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }
    if ((bmi088_device.gyro_cfg.odr != BMI088_GYRO_ODR_SETTING) ||
        (bmi088_device.gyro_cfg.bw != BMI088_GYRO_ODR_SETTING) ||
        (bmi088_device.gyro_cfg.range != BMI088_GYRO_RANGE_SETTING))
    {
        bmi088_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    return true;
}

static bool bmi088_verify_configuration(void)
{
    if (!bmi088_verify_accel_configuration() ||
        !bmi088_verify_gyro_configuration() ||
        !bmi088_verify_fifo_configuration())
        return false;

    bmi088_configuration.register_readback_verified = true;
    return true;
}

static bool bmi088_configure_fifo(void)
{
    struct bmi08_gyr_fifo_config gyro_fifo = {
        .mode = BMI08_GYRO_FIFO_MODE_STREAM,
        .data_select = BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED,
        .tag = BMI08_GYRO_FIFO_TAG_DISABLED,
        .frame_count = 0U,
        .wm_level = BMI088_GYRO_FIFO_WATERMARK_FRAMES,
    };
    uint16_t accel_watermark = BMI088_ACCEL_FIFO_WATERMARK_BYTES;
    uint8_t accel_downsample = BMI088_ACCEL_FIFO_DOWNS_REGISTER;
    uint8_t accel_fifo_config[2] = {
        BMI088_ACCEL_FIFO_CONFIG_0_REGISTER,
        BMI088_ACCEL_FIFO_CONFIG_1_REGISTER,
    };
    const uint8_t gyro_watermark_interrupt =
        BMI088_GYRO_FIFO_WM_INTERRUPT_DISABLED;

    if (!bmi088_init_step(
            bmi08a_get_set_regs(BMI08_FIFO_DOWNS_ADDR, &accel_downsample,
                                sizeof(accel_downsample), &bmi088_device,
                                SET_FUNC),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08a_get_set_fifo_wm(
                &accel_watermark, &bmi088_device, SET_FUNC),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08a_get_set_regs(BMI08_FIFO_CONFIG_0_ADDR, accel_fifo_config,
                                sizeof(accel_fifo_config), &bmi088_device,
                                SET_FUNC),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08g_set_fifo_config(&gyro_fifo, &bmi088_device),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08g_set_regs(BMI08_REG_GYRO_FIFO_WM_ENABLE,
                            &gyro_watermark_interrupt, 1U, &bmi088_device),
            IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    return bmi088_verify_fifo_configuration();
}

static bool bmi088_set_gyro_fifo_bypass(void)
{
    const struct bmi08_gyr_fifo_config bypass_config = {
        .mode = BMI08_GYRO_FIFO_MODE_BYPASS,
        .data_select = BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED,
        .tag = BMI08_GYRO_FIFO_TAG_DISABLED,
        .frame_count = 0U,
        .wm_level = BMI088_GYRO_FIFO_WATERMARK_FRAMES,
    };
    struct bmi08_gyr_fifo_config readback = {0};

    if (!bmi088_init_step(
            bmi08g_set_fifo_config(&bypass_config, &bmi088_device),
            IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    /* Gyro register writes need the suspended-mode write delay before they can
     * be verified. In NORMAL mode this delay would cross multiple 2 kHz DRDY
     * edges and break the BYPASS-to-STREAM phase handshake. */
    if (bmi088_device.gyro_cfg.power == BMI08_GYRO_PM_SUSPEND)
        bmi088_delay_us(UINT32_C(1000), bmi088_device.intf_ptr_gyro);

    if (!bmi088_init_step(
            bmi08g_get_fifo_config(&readback, &bmi088_device),
            IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    if ((readback.mode != BMI08_GYRO_FIFO_MODE_BYPASS) ||
        (readback.frame_count != 0U))
    {
        bmi088_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    return true;
}

static bool bmi088_sync_gyro_fifo_to_capture(uint32_t *mismatch_reason)
{
    if (mismatch_reason == NULL)
        return false;

    *mismatch_reason = 0U;
    const struct bmi08_gyr_fifo_config stream_config = {
        .mode = BMI08_GYRO_FIFO_MODE_STREAM,
        .data_select = BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED,
        .tag = BMI08_GYRO_FIFO_TAG_DISABLED,
        .frame_count = 0U,
        .wm_level = BMI088_GYRO_FIFO_WATERMARK_FRAMES,
    };

    for (uint32_t attempt = 0U;
         attempt < BMI088_GYRO_START_SYNC_MAX_ATTEMPTS;
         ++attempt)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        bmi088_gyro_capture_armed = false;
        imu_timestamp_queue_reset(&bmi088_gyro_capture_queue);
        bmi088_last_gyro_capture_sequence_valid = false;
        __set_PRIMASK(primask);

        uint8_t status = 0U;
        if (bmi08g_get_regs(BMI08_REG_GYRO_FIFO_STATUS, &status, 1U,
                            &bmi088_device) != BMI08_OK)
        {
            *mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_FRAME_COUNT;
            return false;
        }
        if ((status & BMI08_GYRO_FIFO_OVERRUN_MASK) != 0U)
        {
            *mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_FIFO_OVERRUN;
            return false;
        }

        const uint16_t fifo_frames =
            status & BMI08_GYRO_FIFO_FRAME_COUNT_MASK;
        bmi088_gyro_sequence += fifo_frames;
        bmi088_fifo_diagnostics.gyro_warmup_discard_count += fifo_frames;
        if (!bmi088_set_gyro_fifo_bypass())
        {
            *mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_FRAME_COUNT;
            return false;
        }

        const uint32_t arm_primask = __get_PRIMASK();
        __disable_irq();
        imu_timestamp_queue_reset(&bmi088_gyro_capture_queue);
        bmi088_gyro_capture_armed = true;
        __set_PRIMASK(arm_primask);

        const uint64_t deadline_us = imu_time_now_us() +
            BMI088_GYRO_START_SYNC_TIMEOUT_US;
        while ((imu_timestamp_queue_count(&bmi088_gyro_capture_queue) == 0U) &&
               (imu_time_now_us() < deadline_us))
        {
        }

        const uint32_t capture_count =
            imu_timestamp_queue_count(&bmi088_gyro_capture_queue);
        if ((capture_count == 0U) ||
            (capture_count > BMI088_FIFO_MAX_BATCH_SAMPLES) ||
            !imu_timestamp_queue_pop_batch(
                &bmi088_gyro_capture_queue,
                bmi088_gyro_capture_timestamps,
                bmi088_gyro_capture_sequences,
                capture_count))
        {
            *mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_QUEUE_SHORT;
            continue;
        }

        const uint64_t capture_complete_us = imu_time_now_us();
        bool sequence_valid = true;
        for (uint32_t index = 0U; index < capture_count; ++index)
        {
            if ((bmi088_gyro_capture_timestamps[index] == 0U) ||
                (bmi088_gyro_capture_timestamps[index] >
                 capture_complete_us))
            {
                sequence_valid = false;
                *mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_TIMESTAMP;
            }
            if ((index != 0U) &&
                (((uint32_t)(bmi088_gyro_capture_sequences[index] -
                             bmi088_gyro_capture_sequences[index - 1U]) !=
                  1U) ||
                 (bmi088_gyro_capture_timestamps[index] <=
                  bmi088_gyro_capture_timestamps[index - 1U])))
            {
                sequence_valid = false;
                *mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_SEQUENCE;
            }
        }
        if (!sequence_valid)
            continue;

        const uint32_t seed_sequence =
            bmi088_gyro_capture_sequences[capture_count - 1U];
        if (bmi08g_set_fifo_config(&stream_config, &bmi088_device) != BMI08_OK)
        {
            *mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_FRAME_COUNT;
            return false;
        }

        /* Any edge during the short BYPASS->STREAM register transaction has
         * ambiguous ownership. Retry from BYPASS instead of estimating it. */
        if (imu_timestamp_queue_count(&bmi088_gyro_capture_queue) != 0U)
            continue;

        bmi088_last_gyro_capture_sequence = seed_sequence;
        bmi088_last_gyro_capture_sequence_valid = true;
        *mismatch_reason = 0U;
        return true;
    }

    if (*mismatch_reason == 0U)
        *mismatch_reason = BMI088_GYRO_CAPTURE_REASON_SEQUENCE;
    return false;
}

static bool bmi088_verify_fifo_configuration(void)
{
    struct bmi08_accel_fifo_config accel_fifo = { 0 };
    struct bmi08_gyr_fifo_config gyro_fifo = { 0 };
    uint16_t accel_watermark = 0U;
    uint8_t accel_downsample = 0U;
    uint8_t accel_fifo_config_raw[2] = { 0U, 0U };
    uint8_t gyro_watermark_interrupt = 0U;

    if (!bmi088_init_step(
            bmi08a_get_set_regs(BMI08_FIFO_DOWNS_ADDR, &accel_downsample,
                                sizeof(accel_downsample), &bmi088_device,
                                GET_FUNC),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08a_get_set_fifo_wm(
                &accel_watermark, &bmi088_device, GET_FUNC),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08a_get_set_fifo_config(
                &accel_fifo, &bmi088_device, GET_FUNC),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08a_get_set_regs(BMI08_FIFO_CONFIG_0_ADDR,
                                accel_fifo_config_raw,
                                sizeof(accel_fifo_config_raw),
                                &bmi088_device, GET_FUNC),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08g_get_fifo_config(&gyro_fifo, &bmi088_device),
            IMU_STATUS_CONFIG_ERROR) ||
        !bmi088_init_step(
            bmi08g_get_regs(BMI08_REG_GYRO_FIFO_WM_ENABLE,
                            &gyro_watermark_interrupt, 1U, &bmi088_device),
            IMU_STATUS_CONFIG_ERROR))
    {
        return false;
    }

    if ((accel_downsample != BMI088_ACCEL_FIFO_DOWNS_REGISTER) ||
        (accel_watermark != BMI088_ACCEL_FIFO_WATERMARK_BYTES) ||
        (accel_fifo_config_raw[0] !=
         BMI088_ACCEL_FIFO_CONFIG_0_REGISTER) ||
        (accel_fifo_config_raw[1] !=
         BMI088_ACCEL_FIFO_CONFIG_1_REGISTER) ||
        (accel_fifo.mode != BMI08_ACC_STREAM_MODE) ||
        (accel_fifo.accel_en != BMI08_ENABLE) ||
        (accel_fifo.int1_en != BMI08_DISABLE) ||
        (accel_fifo.int2_en != BMI08_DISABLE) ||
        (gyro_fifo.mode != BMI08_GYRO_FIFO_MODE_STREAM) ||
        (gyro_fifo.data_select != BMI08_GYRO_FIFO_XYZ_AXIS_ENABLED) ||
        (gyro_fifo.tag != BMI08_GYRO_FIFO_TAG_DISABLED) ||
        (gyro_fifo.wm_level != BMI088_GYRO_FIFO_WATERMARK_FRAMES) ||
        (gyro_watermark_interrupt !=
         BMI088_GYRO_FIFO_WM_INTERRUPT_DISABLED))
    {
        bmi088_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    return true;
}

static bool bmi088_blocking_bus_available(void)
{
    return (bmi088_dma_state == BMI088_DMA_IDLE) &&
           !imu_spi_dma_is_busy(&hspi1) &&
           (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY);
}

static bool bmi088_capture_accel_diagnostics(void)
{
    static const uint8_t register_addresses[
        BMI088_ACCEL_REGISTER_SNAPSHOT_SIZE] = {
        BMI08_REG_ACCEL_ERR,
        BMI08_REG_ACCEL_STATUS,
        BMI08_REG_ACCEL_INTERNAL_STAT,
        BMI08_REG_ACCEL_CONF,
        BMI08_REG_ACCEL_RANGE,
        BMI08_FIFO_DOWNS_ADDR,
        BMI08_FIFO_WTM_0_ADDR,
        BMI08_FIFO_WTM_1_ADDR,
        BMI08_FIFO_CONFIG_0_ADDR,
        BMI08_FIFO_CONFIG_1_ADDR,
        BMI08_REG_ACCEL_INT1_IO_CONF,
        BMI08_REG_ACCEL_INT2_IO_CONF,
        BMI08_REG_ACCEL_INT1_INT2_MAP_DATA,
        BMI08_REG_ACCEL_PWR_CONF,
        BMI08_REG_ACCEL_PWR_CTRL,
        BMI08_REG_ACCEL_INIT_CTRL,
    };
    struct bmi08_sensor_data accel_raw;
    uint32_t sensor_time_before = 0U;
    uint32_t sensor_time_after = 0U;
    uint16_t fifo_bytes = 0U;

    bool valid = bmi08a_get_sensor_time(
        &bmi088_device, &sensor_time_before) == BMI08_OK;
    bmi088_delay_us(UINT32_C(1000), bmi088_device.intf_ptr_accel);
    valid = valid &&
        (bmi08a_get_sensor_time(&bmi088_device, &sensor_time_after) ==
         BMI08_OK);
    valid = valid &&
        (bmi08a_get_data(&accel_raw, &bmi088_device) == BMI08_OK);
    valid = valid &&
        (bmi08a_get_fifo_length(&fifo_bytes, &bmi088_device) == BMI08_OK);

    for (size_t index = 0U;
         index < BMI088_ACCEL_REGISTER_SNAPSHOT_SIZE; ++index)
    {
        valid = valid &&
            (bmi08a_get_set_regs(register_addresses[index],
                                 &bmi088_fifo_diagnostics
                                      .accel_register_snapshot[index],
                                 1U, &bmi088_device, GET_FUNC) == BMI08_OK);
    }

    bmi088_fifo_diagnostics.accel_sensor_time_before = sensor_time_before;
    bmi088_fifo_diagnostics.accel_sensor_time_after = sensor_time_after;
    bmi088_fifo_diagnostics.accel_direct_raw[0] = accel_raw.x;
    bmi088_fifo_diagnostics.accel_direct_raw[1] = accel_raw.y;
    bmi088_fifo_diagnostics.accel_direct_raw[2] = accel_raw.z;
    bmi088_fifo_diagnostics.accel_initial_fifo_bytes = fifo_bytes;
    bmi088_fifo_diagnostics.accel_snapshot_valid = valid;
    return valid;
}

static void bmi088_requeue_active_request(bmi088_dma_state_t failed_state)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if ((failed_state == BMI088_DMA_ACCEL_LENGTH) ||
        (failed_state == BMI088_DMA_ACCEL_DATA))
    {
        if (!bmi088_accel_request_pending ||
            (bmi088_accel_request_timestamp_us == 0U))
        {
            bmi088_accel_request_timestamp_us =
                bmi088_active_request_timestamp_us;
        }
        bmi088_accel_request_pending = true;
    }
    else if ((failed_state == BMI088_DMA_GYRO_STATUS) ||
             (failed_state == BMI088_DMA_GYRO_DATA))
    {
        if (!bmi088_gyro_capture_sync_fault)
        {
            if (!bmi088_gyro_request_pending ||
                (bmi088_gyro_request_timestamp_us == 0U))
            {
                bmi088_gyro_request_timestamp_us =
                    bmi088_active_request_timestamp_us;
            }
            bmi088_gyro_request_pending = true;
        }
    }

    __set_PRIMASK(primask);
}

static void bmi088_record_dma_error(imu_spi_dma_status_t status)
{
    if (status == IMU_SPI_DMA_STATUS_BUSY)
    {
        bmi088_fifo_diagnostics.dma_submit_busy_count++;
    }
    else
    {
        bmi088_fifo_diagnostics.dma_error_count++;
        bmi088_health.bus_error_count++;
        bmi088_health.status = IMU_STATUS_BUS_ERROR;
    }
    bmi088_dma_state = BMI088_DMA_IDLE;
}

static bool bmi088_submit_dma(const bmi088_spi_target_t *target,
                              uint16_t length)
{
    if ((target == NULL) || (length == 0U) ||
        (length > IMU_SPI_DMA_MAX_TRANSFER_SIZE))
    {
        bmi088_record_dma_error(IMU_SPI_DMA_STATUS_INVALID_ARGUMENT);
        return false;
    }

    const imu_spi_dma_request_t request = {
        .spi = target->spi,
        .cs_port = target->cs_port,
        .cs_pin = target->cs_pin,
        .tx_data = bmi088_dma_tx,
        .rx_data = bmi088_dma_control_rx,
        .length = length,
        .timeout_ms = BMI088_FIFO_DMA_TIMEOUT_MS,
        .callback = bmi088_dma_callback,
        .context = &bmi088_device,
    };
    const imu_spi_dma_status_t status = imu_spi_dma_submit(&request);
    if (status != IMU_SPI_DMA_STATUS_OK)
    {
        bmi088_record_dma_error(status);
        return false;
    }
    return true;
}

static bool bmi088_submit_accel_length(void)
{
    memset(bmi088_dma_tx, 0, BMI088_ACCEL_FIFO_LENGTH_WIRE_BYTES);
    memset(bmi088_dma_control_rx, 0, sizeof(bmi088_dma_control_rx));
    bmi088_dma_tx[0] = BMI08_FIFO_LENGTH_0_ADDR | BMI08_SPI_RD_MASK;
    HAL_GPIO_WritePin(bmi088_gyro_target.cs_port, bmi088_gyro_target.cs_pin,
                      GPIO_PIN_SET);
    return bmi088_submit_dma(&bmi088_accel_target,
                             BMI088_ACCEL_FIFO_LENGTH_WIRE_BYTES);
}

static bool bmi088_submit_accel_data(uint16_t fifo_bytes)
{
    const uint32_t wire_length =
        (uint32_t)BMI088_SPI_COMMAND_BYTES +
        BMI088_ACCEL_SPI_DUMMY_BYTES + fifo_bytes +
        BMI088_ACCEL_FIFO_OVERREAD_BYTES;
    if ((fifo_bytes == 0U) ||
        (wire_length > IMU_SPI_DMA_MAX_TRANSFER_SIZE))
    {
        bmi088_record_dma_error(IMU_SPI_DMA_STATUS_INVALID_ARGUMENT);
        return false;
    }

    bmi088_accel_raw_wire_length = (uint16_t)wire_length;
    memset(bmi088_dma_tx, 0, bmi088_accel_raw_wire_length);
    bmi088_dma_tx[0] = BMI08_FIFO_DATA_ADDR | BMI08_SPI_RD_MASK;

    const imu_spi_dma_request_t request = {
        .spi = bmi088_accel_target.spi,
        .cs_port = bmi088_accel_target.cs_port,
        .cs_pin = bmi088_accel_target.cs_pin,
        .tx_data = bmi088_dma_tx,
        .rx_data = bmi088_accel_raw_rx,
        .length = bmi088_accel_raw_wire_length,
        .timeout_ms = BMI088_FIFO_DMA_TIMEOUT_MS,
        .callback = bmi088_dma_callback,
        .context = &bmi088_device,
    };
    HAL_GPIO_WritePin(bmi088_gyro_target.cs_port, bmi088_gyro_target.cs_pin,
                      GPIO_PIN_SET);
    const imu_spi_dma_status_t status = imu_spi_dma_submit(&request);
    if (status != IMU_SPI_DMA_STATUS_OK)
    {
        bmi088_record_dma_error(status);
        return false;
    }
    return true;
}

static bool bmi088_submit_gyro_status(void)
{
    memset(bmi088_dma_tx, 0, BMI088_GYRO_FIFO_STATUS_WIRE_BYTES);
    memset(bmi088_dma_control_rx, 0, sizeof(bmi088_dma_control_rx));
    bmi088_dma_tx[0] = BMI08_REG_GYRO_FIFO_STATUS | BMI08_SPI_RD_MASK;
    HAL_GPIO_WritePin(bmi088_accel_target.cs_port, bmi088_accel_target.cs_pin,
                      GPIO_PIN_SET);
    return bmi088_submit_dma(&bmi088_gyro_target,
                             BMI088_GYRO_FIFO_STATUS_WIRE_BYTES);
}

static bool bmi088_submit_gyro_data(uint16_t fifo_frames)
{
    const uint32_t wire_length =
        (uint32_t)BMI088_SPI_COMMAND_BYTES +
        ((uint32_t)fifo_frames * BMI088_GYRO_FIFO_FRAME_BYTES);
    if ((fifo_frames == 0U) ||
        (wire_length > IMU_SPI_DMA_MAX_TRANSFER_SIZE))
    {
        bmi088_record_dma_error(IMU_SPI_DMA_STATUS_INVALID_ARGUMENT);
        return false;
    }

    bmi088_gyro_raw_wire_length = (uint16_t)wire_length;
    memset(bmi088_dma_tx, 0, bmi088_gyro_raw_wire_length);
    bmi088_dma_tx[0] = BMI08_REG_GYRO_FIFO_DATA | BMI08_SPI_RD_MASK;

    const imu_spi_dma_request_t request = {
        .spi = bmi088_gyro_target.spi,
        .cs_port = bmi088_gyro_target.cs_port,
        .cs_pin = bmi088_gyro_target.cs_pin,
        .tx_data = bmi088_dma_tx,
        .rx_data = bmi088_gyro_raw_rx,
        .length = bmi088_gyro_raw_wire_length,
        .timeout_ms = BMI088_FIFO_DMA_TIMEOUT_MS,
        .callback = bmi088_dma_callback,
        .context = &bmi088_device,
    };
    HAL_GPIO_WritePin(bmi088_accel_target.cs_port, bmi088_accel_target.cs_pin,
                      GPIO_PIN_SET);
    const imu_spi_dma_status_t status = imu_spi_dma_submit(&request);
    if (status != IMU_SPI_DMA_STATUS_OK)
    {
        bmi088_record_dma_error(status);
        return false;
    }
    return true;
}

static void bmi088_fifo_kick(void)
{
    if ((bmi088_dma_state != BMI088_DMA_IDLE) ||
        imu_spi_dma_is_busy(&hspi1))
    {
        return;
    }

    const bool accel_available = bmi088_accel_request_pending &&
        !bmi088_accel_raw_ready && !bmi088_accel_batch_ready;
    const bool gyro_available = !bmi088_gyro_capture_sync_fault &&
        bmi088_gyro_request_pending &&
        !bmi088_gyro_raw_ready && !bmi088_gyro_batch_ready;
    if (!accel_available && !gyro_available)
        return;

    const bool select_gyro = gyro_available &&
        (!accel_available || bmi088_prefer_gyro);
    bmi088_dma_state_t claimed_state;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (select_gyro)
    {
        if (bmi088_gyro_capture_sync_fault)
        {
            __set_PRIMASK(primask);
            return;
        }
        bmi088_active_request_timestamp_us =
            bmi088_gyro_request_timestamp_us;
        bmi088_gyro_request_pending = false;
        bmi088_gyro_request_timestamp_us = 0U;
        claimed_state = BMI088_DMA_GYRO_STATUS;
        bmi088_dma_state = claimed_state;
        bmi088_prefer_gyro = false;
    }
    else
    {
        bmi088_active_request_timestamp_us =
            bmi088_accel_request_timestamp_us;
        bmi088_accel_request_pending = false;
        bmi088_accel_request_timestamp_us = 0U;
        claimed_state = BMI088_DMA_ACCEL_LENGTH;
        bmi088_dma_state = claimed_state;
        bmi088_prefer_gyro = true;
    }
    __set_PRIMASK(primask);

    const bool submitted = select_gyro ? bmi088_submit_gyro_status()
                                       : bmi088_submit_accel_length();
    if (!submitted)
        bmi088_requeue_active_request(claimed_state);
}

static void bmi088_dma_callback(SPI_HandleTypeDef *spi,
                                imu_spi_dma_status_t status,
                                uint16_t length,
                                void *context)
{
    const bmi088_dma_state_t completed_state = bmi088_dma_state;
    if ((spi != &hspi1) || (context != &bmi088_device) ||
        (status != IMU_SPI_DMA_STATUS_OK))
    {
        const bool gyro_data_ambiguous =
            bmi088_fifo_gyro_failure_is_ambiguous(
                completed_state == BMI088_DMA_GYRO_DATA);
        if (gyro_data_ambiguous)
        {
            bmi088_gyro_sequence += bmi088_active_gyro_fifo_frames;
            bmi088_latch_gyro_capture_fault(
                BMI088_GYRO_CAPTURE_REASON_DMA_TRANSFER);
        }
        else
        {
            bmi088_requeue_active_request(completed_state);
        }
        bmi088_record_dma_error((status == IMU_SPI_DMA_STATUS_OK)
                                    ? IMU_SPI_DMA_STATUS_TRANSFER_ERROR
                                    : status);
        return;
    }

    if (completed_state == BMI088_DMA_ACCEL_LENGTH)
    {
        if (length != BMI088_ACCEL_FIFO_LENGTH_WIRE_BYTES)
        {
            bmi088_requeue_active_request(completed_state);
            bmi088_record_dma_error(IMU_SPI_DMA_STATUS_TRANSFER_ERROR);
            return;
        }

        bmi088_fifo_diagnostics.accel_length_read_count++;
        memcpy(bmi088_fifo_diagnostics.accel_last_length_response,
               bmi088_dma_control_rx,
               sizeof(bmi088_fifo_diagnostics.accel_last_length_response));
        const uint16_t fifo_bytes = (uint16_t)(
            ((uint16_t)(bmi088_dma_control_rx[3] &
                        BMI08_FIFO_BYTE_COUNTER_MSB_MASK) << 8) |
            bmi088_dma_control_rx[2]);
        bmi088_fifo_diagnostics.accel_last_fifo_bytes = fifo_bytes;
        if (fifo_bytes > bmi088_fifo_diagnostics.accel_peak_fifo_bytes)
            bmi088_fifo_diagnostics.accel_peak_fifo_bytes = fifo_bytes;
        if (fifo_bytes == 0U)
        {
            bmi088_fifo_diagnostics.accel_empty_length_count++;
            bmi088_dma_state = BMI088_DMA_IDLE;
            return;
        }

        bmi088_active_accel_fifo_bytes = fifo_bytes;
        bmi088_accel_raw_truncated =
            fifo_bytes > BMI088_ACCEL_FIFO_MAX_CONTENT_BYTES;
        const uint16_t bytes_to_read = bmi088_accel_raw_truncated
            ? BMI088_ACCEL_FIFO_MAX_CONTENT_BYTES
            : fifo_bytes;
        bmi088_dma_state = BMI088_DMA_ACCEL_DATA;
        if (!bmi088_submit_accel_data(bytes_to_read))
            bmi088_requeue_active_request(BMI088_DMA_ACCEL_DATA);
        return;
    }

    if (completed_state == BMI088_DMA_ACCEL_DATA)
    {
        if (length != bmi088_accel_raw_wire_length)
        {
            bmi088_requeue_active_request(completed_state);
            bmi088_record_dma_error(IMU_SPI_DMA_STATUS_TRANSFER_ERROR);
            return;
        }
        bmi088_accel_raw_fifo_bytes = bmi088_active_accel_fifo_bytes;
        bmi088_accel_raw_request_timestamp_us =
            bmi088_active_request_timestamp_us;
        bmi088_accel_raw_complete_us = imu_time_now_us();
        bmi088_dma_state = BMI088_DMA_IDLE;
        __DMB();
        bmi088_accel_raw_ready = true;
        return;
    }

    if (completed_state == BMI088_DMA_GYRO_STATUS)
    {
        if (length != BMI088_GYRO_FIFO_STATUS_WIRE_BYTES)
        {
            bmi088_requeue_active_request(completed_state);
            bmi088_record_dma_error(IMU_SPI_DMA_STATUS_TRANSFER_ERROR);
            return;
        }

        if (bmi088_gyro_capture_sync_fault)
        {
            bmi088_dma_state = BMI088_DMA_IDLE;
            return;
        }

        const uint8_t status_byte = bmi088_dma_control_rx[1];
        const uint16_t fifo_frames =
            status_byte & BMI08_GYRO_FIFO_FRAME_COUNT_MASK;
        bmi088_gyro_raw_overrun =
            (uint8_t)((status_byte & BMI08_GYRO_FIFO_OVERRUN_MASK) != 0U);
        if (fifo_frames == 0U)
        {
            bmi088_dma_state = BMI088_DMA_IDLE;
            return;
        }

        bmi088_active_gyro_fifo_total_frames = fifo_frames;
        bmi088_gyro_raw_truncated =
            fifo_frames > BMI088_FIFO_MAX_BATCH_SAMPLES;
        bmi088_active_gyro_fifo_frames = bmi088_gyro_raw_truncated
            ? BMI088_FIFO_MAX_BATCH_SAMPLES
            : fifo_frames;
        bmi088_dma_state = BMI088_DMA_GYRO_DATA;
        if (!bmi088_submit_gyro_data(bmi088_active_gyro_fifo_frames))
            bmi088_requeue_active_request(BMI088_DMA_GYRO_DATA);
        return;
    }

    if (completed_state == BMI088_DMA_GYRO_DATA)
    {
        if (length != bmi088_gyro_raw_wire_length)
        {
            bmi088_gyro_sequence += bmi088_active_gyro_fifo_frames;
            bmi088_latch_gyro_capture_fault(
                BMI088_GYRO_CAPTURE_REASON_DMA_TRANSFER);
            bmi088_record_dma_error(IMU_SPI_DMA_STATUS_TRANSFER_ERROR);
            return;
        }
        if (bmi088_gyro_capture_sync_fault)
        {
            bmi088_gyro_sequence += bmi088_active_gyro_fifo_frames;
            bmi088_dma_state = BMI088_DMA_IDLE;
            return;
        }
        bmi088_gyro_raw_fifo_frames =
            bmi088_active_gyro_fifo_total_frames;
        bmi088_gyro_raw_complete_us = imu_time_now_us();
        bmi088_dma_state = BMI088_DMA_IDLE;
        __DMB();
        bmi088_gyro_raw_ready = true;
        return;
    }

    bmi088_record_dma_error(IMU_SPI_DMA_STATUS_TRANSFER_ERROR);
}

static bool bmi088_scan_accel_fifo_metadata(const struct bmi08_fifo_frame *fifo,
                                            uint32_t *sensor_time,
                                            bool *sensor_time_valid,
                                            uint8_t *skipped_frames,
                                            bool *sample_dropped,
                                            uint16_t *accel_frame_count)
{
    if ((fifo == NULL) || (fifo->data == NULL) ||
        (sensor_time == NULL) || (sensor_time_valid == NULL) ||
        (skipped_frames == NULL) || (sample_dropped == NULL) ||
        (accel_frame_count == NULL) ||
        (fifo->length <= bmi088_device.dummy_byte))
    {
        return false;
    }

    *sensor_time = 0U;
    *sensor_time_valid = false;
    *skipped_frames = 0U;
    *sample_dropped = false;
    *accel_frame_count = 0U;

    size_t index = bmi088_device.dummy_byte;
    while (index < fifo->length)
    {
        const uint8_t header = fifo->data[index++];
        const uint8_t frame_type = header & UINT8_C(0xFC);
        size_t payload_length;

        if (frame_type == BMI08_FIFO_HEADER_ACC_FRM)
        {
            payload_length = BMI08_FIFO_ACCEL_LENGTH;
            if ((index + payload_length) > fifo->length)
                return false;
            if (*accel_frame_count == UINT16_MAX)
                return false;
            (*accel_frame_count)++;
        }
        else if (frame_type == BMI08_FIFO_HEADER_SENS_TIME_FRM)
        {
            payload_length = BMI08_SENSOR_TIME_LENGTH;
            if ((index + payload_length) > fifo->length)
                return false;
            *sensor_time =
                (uint32_t)fifo->data[index] |
                ((uint32_t)fifo->data[index + 1U] << 8) |
                ((uint32_t)fifo->data[index + 2U] << 16);
            *sensor_time_valid = true;
        }
        else if (frame_type == BMI08_FIFO_HEADER_SKIP_FRM)
        {
            payload_length = BMI08_FIFO_SKIP_FRM_LENGTH;
            if ((index + payload_length) > fifo->length)
                return false;
            const uint16_t skipped_total =
                (uint16_t)*skipped_frames + fifo->data[index];
            *skipped_frames = (skipped_total > UINT8_MAX)
                ? UINT8_MAX
                : (uint8_t)skipped_total;
        }
        else if (frame_type == BMI08_FIFO_HEADER_INPUT_CFG_FRM)
        {
            payload_length = BMI08_FIFO_INPUT_CFG_LENGTH;
        }
        else if (frame_type == BMI08_FIFO_SAMPLE_DROP_FRM)
        {
            payload_length = BMI08_FIFO_INPUT_CFG_LENGTH;
            *sample_dropped = true;
        }
        else if (header == BMI08_FIFO_HEAD_OVER_READ_MSB)
        {
            return true;
        }
        else
        {
            return false;
        }

        if ((index + payload_length) > fifo->length)
            return false;
        index += payload_length;
    }

    return true;
}

static uint64_t bmi088_backdate_timestamp(uint64_t newest_timestamp_us,
                                          uint16_t sample_index,
                                          uint16_t sample_count,
                                          uint64_t sample_period_us)
{
    const uint64_t intervals = (uint64_t)(sample_count - 1U - sample_index);
    const uint64_t offset_us = intervals * sample_period_us;
    return (newest_timestamp_us > offset_us)
        ? newest_timestamp_us - offset_us
        : 1U;
}

static void bmi088_update_fifo_temperature(uint16_t sample_count)
{
    if (sample_count == 0U)
        return;

    if (bmi088_temperature_countdown > sample_count)
    {
        bmi088_temperature_countdown -= sample_count;
        return;
    }

    if (!bmi088_blocking_bus_available())
    {
        bmi088_temperature_countdown = 0U;
        return;
    }

    int32_t temperature_milli_c;
    if (bmi08a_get_sensor_temperature(
            &bmi088_device, &temperature_milli_c) == BMI08_OK)
    {
        bmi088_temperature_c = (float)temperature_milli_c / 1000.0f;
        bmi088_temperature_countdown = BMI088_TEMPERATURE_READ_DIVIDER;
    }
    else
    {
        bmi088_health.status = IMU_STATUS_BUS_ERROR;
        bmi088_health.bus_error_count++;
        bmi088_temperature_countdown = 0U;
    }
}

static void bmi088_requeue_truncated_accel(void)
{
    if (!bmi088_accel_raw_truncated)
        return;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!bmi088_accel_request_pending)
    {
        bmi088_accel_request_timestamp_us = 0U;
        bmi088_accel_request_pending = true;
    }
    __set_PRIMASK(primask);
}

static void bmi088_process_accel_fifo(void)
{
    bmi088_accel_fifo_batch_t batch;
    struct bmi08_fifo_frame fifo = { 0 };
    uint32_t sensor_time_raw = 0U;
    uint16_t metadata_frame_count = 0U;
    uint16_t parsed_count = BMI088_FIFO_MAX_BATCH_SAMPLES;
    uint8_t skipped_frames = 0U;
    bool sensor_time_present = false;
    bool sample_dropped = false;
    bool metadata_valid;
    uint64_t sensor_time_ticks = 0U;
    bool sensor_time_valid = false;
    bool sensor_sample_ticks_valid = false;
    uint64_t first_sample_tick = 0U;

    memset(&batch, 0, sizeof(batch));
    fifo.data = &bmi088_accel_raw_rx[BMI088_SPI_COMMAND_BYTES];
    fifo.length = (uint16_t)(bmi088_accel_raw_wire_length -
                             BMI088_SPI_COMMAND_BYTES);
    fifo.acc_data_enable = BMI08_ACCEL_EN_MASK;

    metadata_valid = bmi088_scan_accel_fifo_metadata(
        &fifo, &sensor_time_raw, &sensor_time_present, &skipped_frames,
        &sample_dropped, &metadata_frame_count);
    const int8_t parse_result = bmi08a_extract_accel(
        bmi088_accel_parse_raw, &parsed_count, &fifo, &bmi088_device);
    const bool parse_structure_valid = metadata_valid &&
        (parse_result >= BMI08_OK) &&
        (parsed_count == metadata_frame_count) &&
        (parsed_count <= BMI088_FIFO_MAX_BATCH_SAMPLES);
    if (parsed_count > BMI088_FIFO_MAX_BATCH_SAMPLES)
        parsed_count = BMI088_FIFO_MAX_BATCH_SAMPLES;

    batch.transfer_complete_timestamp_us = bmi088_accel_raw_complete_us;
    batch.fifo_bytes = bmi088_accel_raw_fifo_bytes;
    batch.skipped_frame_count = skipped_frames;
    if (bmi088_accel_raw_truncated)
        batch.flags |= BMI088_FIFO_BATCH_FLAG_TRUNCATED;
    if ((skipped_frames != 0U) || sample_dropped)
        batch.flags |= BMI088_FIFO_BATCH_FLAG_SKIPPED;
    if (skipped_frames != 0U)
        batch.flags |= BMI088_FIFO_BATCH_FLAG_OVERRUN;
    if (!parse_structure_valid)
    {
        batch.flags |= BMI088_FIFO_BATCH_FLAG_PARSE_ERROR;
    }

    if (parse_structure_valid && sensor_time_present &&
        imu_clock_sync_unwrap(&bmi088_accel_clock_sync,
                              sensor_time_raw & BMI088_SENSOR_TIME_MASK,
                              &sensor_time_ticks))
    {
        sensor_time_valid = true;
        batch.sensor_time_ticks = sensor_time_ticks;
        batch.flags |= BMI088_FIFO_BATCH_FLAG_SENSOR_TIME_VALID;
    }
    else if (sensor_time_present)
    {
        batch.flags |= BMI088_FIFO_BATCH_FLAG_PARSE_ERROR;
    }

    const bool tick_timeline_valid = parse_structure_valid &&
        (!sensor_time_present || sensor_time_valid);
    if (!tick_timeline_valid)
    {
        bmi088_fifo_diagnostics.accel_timeline_reset_count++;
        imu_clock_sync_reset(&bmi088_accel_clock_sync);
        bmi088_last_accel_sample_tick_valid = false;
        bmi088_accel_sample_phase_valid = false;

        bmi088_accel_sequence += bmi088_fifo_accel_invalid_frame_count(
            metadata_valid, metadata_frame_count, parsed_count,
            skipped_frames, sample_dropped);
        bmi088_fifo_diagnostics.accel_parse_error_count++;
        bmi088_fifo_diagnostics.accel_skipped_frame_count += skipped_frames;
        bmi088_requeue_truncated_accel();
        return;
    }

    bmi088_accel_sequence += bmi088_fifo_accel_missing_frame_count(
        skipped_frames, sample_dropped);

    const bool anchored_watermark_batch =
        tick_timeline_valid && sensor_time_valid && !sample_dropped &&
        (skipped_frames == 0U) && !bmi088_accel_raw_truncated &&
        (bmi088_accel_raw_fifo_bytes ==
         BMI088_ACCEL_FIFO_WATERMARK_BYTES) &&
        (parsed_count == BMI088_ACCEL_FIFO_WATERMARK_FRAMES) &&
        (((uint32_t)parsed_count * BMI088_ACCEL_FIFO_FRAME_BYTES) ==
         bmi088_accel_raw_fifo_bytes) &&
        (bmi088_accel_raw_request_timestamp_us != 0U) &&
        (bmi088_accel_raw_complete_us >=
         bmi088_accel_raw_request_timestamp_us);

    uint64_t watermark_sample_tick = 0U;
    bool watermark_sample_tick_valid = false;
    if (anchored_watermark_batch)
    {
        if (bmi088_last_accel_sample_tick_valid)
        {
            watermark_sample_tick = bmi088_last_accel_sample_tick +
                ((uint64_t)BMI088_ACCEL_FIFO_WATERMARK_FRAMES *
                 BMI088_ACCEL_SAMPLE_PERIOD_TICKS);
            watermark_sample_tick_valid = true;
        }
        else
        {
            const uint64_t elapsed_us = bmi088_accel_raw_complete_us -
                bmi088_accel_raw_request_timestamp_us;
            const uint64_t elapsed_ticks = (uint64_t)(
                ((double)elapsed_us / BMI088_ACCEL_SENSOR_TIME_TICK_US) +
                 0.5);
            if (sensor_time_ticks >= elapsed_ticks)
            {
                watermark_sample_tick = sensor_time_ticks - elapsed_ticks;
                bmi088_accel_sample_phase_tick =
                    (uint8_t)(watermark_sample_tick %
                              BMI088_ACCEL_SAMPLE_PERIOD_TICKS);
                bmi088_accel_sample_phase_valid = true;
                watermark_sample_tick_valid = true;
            }
        }
        if (watermark_sample_tick_valid)
        {
            (void)imu_clock_sync_observe(
                &bmi088_accel_clock_sync, watermark_sample_tick,
                bmi088_accel_raw_request_timestamp_us);
        }
    }

    if ((parsed_count >= BMI088_ACCEL_FIFO_WATERMARK_FRAMES) &&
        watermark_sample_tick_valid &&
        (watermark_sample_tick >=
         ((uint64_t)(BMI088_ACCEL_FIFO_WATERMARK_FRAMES - 1U) *
          BMI088_ACCEL_SAMPLE_PERIOD_TICKS)))
    {
        first_sample_tick = watermark_sample_tick -
            ((uint64_t)(BMI088_ACCEL_FIFO_WATERMARK_FRAMES - 1U) *
             BMI088_ACCEL_SAMPLE_PERIOD_TICKS);
        sensor_sample_ticks_valid = true;
    }
    else if (tick_timeline_valid && (parsed_count != 0U) &&
             bmi088_last_accel_sample_tick_valid && !sample_dropped)
    {
        first_sample_tick = bmi088_last_accel_sample_tick +
            ((uint64_t)skipped_frames + 1U) *
            BMI088_ACCEL_SAMPLE_PERIOD_TICKS;
        sensor_sample_ticks_valid = true;
    }
    else if (tick_timeline_valid && (parsed_count != 0U) && sensor_time_valid &&
             bmi088_accel_sample_phase_valid)
    {
        const uint64_t newest_tick = sensor_time_ticks -
            ((sensor_time_ticks - bmi088_accel_sample_phase_tick) %
             BMI088_ACCEL_SAMPLE_PERIOD_TICKS);
        const uint64_t history_ticks =
            (uint64_t)(parsed_count - 1U) *
            BMI088_ACCEL_SAMPLE_PERIOD_TICKS;
        if (newest_tick >= history_ticks)
        {
            first_sample_tick = newest_tick - history_ticks;
            sensor_sample_ticks_valid = true;
        }
    }

    bmi088_update_fifo_temperature(parsed_count);
    batch.count = parsed_count;
    uint64_t fallback_newest_us = bmi088_accel_raw_request_timestamp_us;
    if (fallback_newest_us == 0U)
        fallback_newest_us = bmi088_accel_raw_complete_us;

    imu_clock_sync_diagnostics_t clock_diagnostics;
    imu_clock_sync_get_diagnostics(&bmi088_accel_clock_sync,
                                   &clock_diagnostics);
    const bool stale_clock_reference =
        clock_diagnostics.valid &&
        !imu_clock_sync_reference_is_fresh(
            &bmi088_accel_clock_sync, bmi088_accel_raw_complete_us,
            BMI088_CLOCK_REFERENCE_MAX_AGE_US);
    bool causal_mapping_failure = false;
    bool reset_clock_sync = stale_clock_reference;
    bool all_timestamps_mapped = sensor_sample_ticks_valid &&
                                 clock_diagnostics.valid &&
                                 !reset_clock_sync;
    if (all_timestamps_mapped)
    {
        const uint64_t newest_sample_tick = first_sample_tick +
            ((uint64_t)(parsed_count - 1U) *
             BMI088_ACCEL_SAMPLE_PERIOD_TICKS);
        causal_mapping_failure = !imu_clock_sync_map_bounded(
            &bmi088_accel_clock_sync, newest_sample_tick,
            bmi088_accel_raw_complete_us +
                BMI088_CLOCK_CAUSAL_TOLERANCE_US,
            NULL);
        reset_clock_sync = causal_mapping_failure;
        all_timestamps_mapped = !reset_clock_sync;
    }
    if (reset_clock_sync)
    {
        if (stale_clock_reference)
            bmi088_fifo_diagnostics.accel_clock_stale_reset_count++;
        if (causal_mapping_failure)
            bmi088_fifo_diagnostics.accel_clock_causal_reset_count++;
        imu_clock_sync_reset(&bmi088_accel_clock_sync);
        bmi088_last_accel_sample_tick_valid = false;
        bmi088_accel_sample_phase_valid = false;
        sensor_sample_ticks_valid = false;
    }
    for (uint16_t index = 0U; index < parsed_count; ++index)
    {
        imu_accel_sample_t *sample = &batch.samples[index];
        const struct bmi08_sensor_data *raw = &bmi088_accel_parse_raw[index];
        memset(sample, 0, sizeof(*sample));

        if (sensor_sample_ticks_valid && clock_diagnostics.valid)
        {
            const uint64_t sample_tick = first_sample_tick +
                ((uint64_t)index * BMI088_ACCEL_SAMPLE_PERIOD_TICKS);
            if (!imu_clock_sync_map_bounded(
                    &bmi088_accel_clock_sync, sample_tick,
                    bmi088_accel_raw_complete_us +
                        BMI088_CLOCK_CAUSAL_TOLERANCE_US,
                    &sample->timestamp_us))
            {
                all_timestamps_mapped = false;
            }
        }
        if (!all_timestamps_mapped)
        {
            sample->timestamp_us = bmi088_backdate_timestamp(
                fallback_newest_us, index, parsed_count,
                BMI088_ACCEL_SAMPLE_PERIOD_US);
        }

        sample->accel_mps2[0] =
            (float)raw->x * BMI088_ACCEL_SCALE_MPS2_PER_LSB;
        sample->accel_mps2[1] =
            (float)raw->y * BMI088_ACCEL_SCALE_MPS2_PER_LSB;
        sample->accel_mps2[2] =
            (float)raw->z * BMI088_ACCEL_SCALE_MPS2_PER_LSB;
        sample->temperature_c = bmi088_temperature_c;
        sample->sequence = ++bmi088_accel_sequence;
        sample->source = IMU_SOURCE_BMI088;
        sample->valid = true;
    }

    if (!all_timestamps_mapped && (parsed_count != 0U))
        batch.flags |= BMI088_FIFO_BATCH_FLAG_TIMESTAMP_ESTIMATED;
    if (tick_timeline_valid && sensor_sample_ticks_valid &&
        (parsed_count != 0U))
    {
        bmi088_last_accel_sample_tick = first_sample_tick +
            ((uint64_t)(parsed_count - 1U) *
             BMI088_ACCEL_SAMPLE_PERIOD_TICKS);
        bmi088_last_accel_sample_tick_valid = true;
    }

    if ((batch.flags & BMI088_FIFO_BATCH_FLAG_PARSE_ERROR) != 0U)
        bmi088_fifo_diagnostics.accel_parse_error_count++;
    bmi088_fifo_diagnostics.accel_skipped_frame_count += skipped_frames;

    if (parsed_count != 0U)
    {
        bmi088_fifo_diagnostics.accel_batch_count++;
        bmi088_fifo_diagnostics.accel_sample_count += parsed_count;
        bmi088_health.read_count += parsed_count;
        bmi088_health.last_sample_us =
            batch.samples[parsed_count - 1U].timestamp_us;
        bmi088_health.status = IMU_STATUS_OK;
        bmi088_accel_batch = batch;
        __DMB();
        bmi088_accel_batch_ready = true;
    }

    bmi088_requeue_truncated_accel();
}

static int16_t bmi088_read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] |
                     ((uint16_t)data[1] << 8));
}

static void bmi088_process_gyro_fifo(void)
{
    bmi088_gyro_fifo_batch_t batch;
    memset(&batch, 0, sizeof(batch));

    const uint16_t count = bmi088_active_gyro_fifo_frames;
    const uint16_t fifo_total = bmi088_gyro_raw_fifo_frames;
    if ((count == 0U) || (count > BMI088_FIFO_MAX_BATCH_SAMPLES) ||
        (bmi088_gyro_raw_wire_length !=
         (BMI088_SPI_COMMAND_BYTES +
          (count * BMI088_GYRO_FIFO_FRAME_BYTES))))
    {
        if (count <= BMI088_FIFO_MAX_BATCH_SAMPLES)
            bmi088_gyro_sequence += count;
        bmi088_fifo_diagnostics.dma_error_count++;
        bmi088_health.bus_error_count++;
        bmi088_health.status = IMU_STATUS_BUS_ERROR;
        bmi088_latch_gyro_capture_fault(
            BMI088_GYRO_CAPTURE_REASON_FRAME_COUNT |
            BMI088_GYRO_CAPTURE_REASON_DMA_TRANSFER);
        return;
    }

    const uint32_t capture_epoch =
        imu_timestamp_queue_discard_generation(&bmi088_gyro_capture_queue);
    if (!bmi088_gyro_capture_epoch_is_current(capture_epoch))
    {
        bmi088_gyro_sequence += count;
        return;
    }

    uint32_t mismatch_reason = 0U;
    const uint32_t queued_captures =
        imu_timestamp_queue_count(&bmi088_gyro_capture_queue);
    if (queued_captures < count)
        mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_QUEUE_SHORT;

    const bool capture_batch_available = (mismatch_reason == 0U) &&
        imu_timestamp_queue_pop_batch(&bmi088_gyro_capture_queue,
                                      bmi088_gyro_capture_timestamps,
                                      bmi088_gyro_capture_sequences,
                                      count);
    if (!capture_batch_available)
        mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_QUEUE_SHORT;
    if (!bmi088_gyro_capture_epoch_is_current(capture_epoch))
    {
        bmi088_gyro_sequence += count;
        return;
    }

    bool capture_sequence_valid = capture_batch_available;
    if (capture_sequence_valid && bmi088_last_gyro_capture_sequence_valid &&
        ((uint32_t)(bmi088_gyro_capture_sequences[0] -
                    bmi088_last_gyro_capture_sequence) != 1U))
    {
        capture_sequence_valid = false;
        mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_SEQUENCE;
    }
    for (uint16_t index = 0U;
         capture_sequence_valid && (index < count);
         ++index)
    {
        if ((bmi088_gyro_capture_timestamps[index] == 0U) ||
            (bmi088_gyro_capture_timestamps[index] >
             bmi088_gyro_raw_complete_us))
        {
            capture_sequence_valid = false;
            mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_TIMESTAMP;
        }
        if ((index != 0U) &&
            (((uint32_t)(bmi088_gyro_capture_sequences[index] -
                         bmi088_gyro_capture_sequences[index - 1U]) != 1U) ||
             (bmi088_gyro_capture_timestamps[index] <=
              bmi088_gyro_capture_timestamps[index - 1U])))
        {
            capture_sequence_valid = false;
            mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_SEQUENCE;
        }
    }

    if (count != fifo_total)
        mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_FRAME_COUNT;
    if (bmi088_gyro_raw_truncated)
        mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_TRUNCATED;
    if (bmi088_gyro_raw_overrun != 0U)
    {
        mismatch_reason |= BMI088_GYRO_CAPTURE_REASON_FIFO_OVERRUN;
        bmi088_fifo_diagnostics.gyro_overrun_count++;
    }
    if (!capture_sequence_valid || (mismatch_reason != 0U))
    {
        bmi088_gyro_sequence += count;
        bmi088_fifo_diagnostics.gyro_capture_mismatch_count++;
        bmi088_latch_gyro_capture_fault(mismatch_reason);
        return;
    }

    const uint32_t capture_commit_primask = __get_PRIMASK();
    __disable_irq();
    if (!bmi088_gyro_capture_epoch_is_current(capture_epoch))
    {
        __set_PRIMASK(capture_commit_primask);
        bmi088_gyro_sequence += count;
        return;
    }
    bmi088_last_gyro_capture_sequence =
        bmi088_gyro_capture_sequences[count - 1U];
    bmi088_last_gyro_capture_sequence_valid = true;
    __set_PRIMASK(capture_commit_primask);

    batch.transfer_complete_timestamp_us = bmi088_gyro_raw_complete_us;
    batch.fifo_frames = fifo_total;
    batch.capture_event_count = count;

    uint16_t valid_count = 0U;
    for (uint16_t index = 0U; index < count; ++index)
    {
        const uint8_t *raw = &bmi088_gyro_raw_rx[
            BMI088_SPI_COMMAND_BYTES +
            ((size_t)index * BMI088_GYRO_FIFO_FRAME_BYTES)];
        const uint32_t physical_sequence = ++bmi088_gyro_sequence;
        if (bmi088_gyro_capture_timestamps[index] <
            bmi088_gyro_valid_after_us)
        {
            bmi088_fifo_diagnostics.gyro_warmup_discard_count++;
            continue;
        }

        imu_gyro_sample_t *sample = &batch.samples[valid_count++];
        memset(sample, 0, sizeof(*sample));
        sample->timestamp_us = bmi088_gyro_capture_timestamps[index];
        sample->gyro_rad_s[0] =
            (float)bmi088_read_i16_le(&raw[0]) *
            BMI088_GYRO_SCALE_RAD_S_PER_LSB;
        sample->gyro_rad_s[1] =
            (float)bmi088_read_i16_le(&raw[2]) *
            BMI088_GYRO_SCALE_RAD_S_PER_LSB;
        sample->gyro_rad_s[2] =
            (float)bmi088_read_i16_le(&raw[4]) *
            BMI088_GYRO_SCALE_RAD_S_PER_LSB;
        sample->temperature_c = bmi088_temperature_c;
        sample->sequence = physical_sequence;
        sample->source = IMU_SOURCE_BMI088;
        sample->valid = true;
    }

    if (valid_count == 0U)
        return;

    batch.count = valid_count;
    batch.latest_capture_timestamp_us =
        batch.samples[valid_count - 1U].timestamp_us;
    bmi088_gyro_batch = batch;
    __DMB();

    const uint32_t publish_primask = __get_PRIMASK();
    __disable_irq();
    if (!bmi088_gyro_capture_epoch_is_current(capture_epoch))
    {
        __set_PRIMASK(publish_primask);
        return;
    }
    bmi088_last_gyro_sample_us = batch.latest_capture_timestamp_us;
    bmi088_last_gyro_sample_us_valid = true;
    bmi088_fifo_diagnostics.gyro_batch_count++;
    bmi088_fifo_diagnostics.gyro_sample_count += valid_count;
    bmi088_health.read_count += valid_count;
    bmi088_health.last_sample_us = batch.latest_capture_timestamp_us;
    bmi088_health.status = IMU_STATUS_OK;
    bmi088_gyro_batch_ready = true;
    __set_PRIMASK(publish_primask);
}
