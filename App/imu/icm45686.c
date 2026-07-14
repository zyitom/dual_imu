#include "icm45686.h"

#include "imu_clock_sync.h"
#include "imu_spi_dma.h"
#include "imu_time.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "imu/inv_imu_driver_advanced.h"

#include <stddef.h>
#include <string.h>

#define ICM45686_SPI_READ_BIT       0x80U
#define ICM45686_SPI_ADDRESS_MASK   0x7FU
#define ICM45686_SPI_TIMEOUT_MS     5U
#define ICM45686_STARTUP_DELAY_US   70000U
#define ICM45686_ACCEL_SCALE_MPS2       ((32.0f * 9.80665f) / 32768.0f)
#define ICM45686_GYRO_SCALE_RAD_S       ((4000.0f * 0.017453292519943295f) / 32768.0f)
#define ICM45686_FIFO_ACCEL_SCALE_MPS2  ((32.0f * 9.80665f) / 524288.0f)
#define ICM45686_FIFO_GYRO_SCALE_RAD_S  ((4000.0f * 0.017453292519943295f) / 524288.0f)
#define ICM45686_TEMPERATURE_SCALE      (1.0f / 128.0f)
/* The 20-byte high-resolution frame carries the 16-bit temperature value. */
#define ICM45686_FIFO_TEMPERATURE_SCALE (1.0f / 128.0f)
#define ICM45686_TEMPERATURE_OFFSET     25.0f
#define ICM45686_SPI_MAX_TRANSFER   256U
#define ICM45686_FIFO_COUNT_TRANSFER_BYTES 3U
#define ICM45686_FIFO_DMA_TIMEOUT_MS       2U
#define ICM45686_FIFO_OUTPUT_DEPTH         64U
#define ICM45686_FIFO_MAX_TIMESTAMP_GAP_US 10000U
#define ICM45686_CLOCK_REFERENCE_MAX_AGE_US 20000U
#define ICM45686_CLOCK_CAUSAL_TOLERANCE_US   1000U
#define ICM45686_FIFO_FULL_FRAME_COUNT     409U

#define ICM45686_FIFO_HEADER_EXTENDED       0x80U
#define ICM45686_FIFO_HEADER_ACCEL          0x40U
#define ICM45686_FIFO_HEADER_GYRO           0x20U
#define ICM45686_FIFO_HEADER_HIGH_RES       0x10U
#define ICM45686_FIFO_HEADER_TIMESTAMP      0x08U
#define ICM45686_FIFO_HEADER_FSYNC          0x04U

#if ICM45686_USE_ROBUST_HIGH_RATE_PROFILE
#define ICM45686_ACCEL_ODR_SETTING  ACCEL_CONFIG0_ACCEL_ODR_1600_HZ
#define ICM45686_ACCEL_BW_SETTING   IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_16
#define ICM45686_GYRO_ODR_SETTING   GYRO_CONFIG0_GYRO_ODR_3200_HZ
#define ICM45686_GYRO_BW_SETTING    IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_16
#else
#define ICM45686_ACCEL_ODR_SETTING  ACCEL_CONFIG0_ACCEL_ODR_400_HZ
#define ICM45686_ACCEL_BW_SETTING   IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8
#define ICM45686_GYRO_ODR_SETTING   GYRO_CONFIG0_GYRO_ODR_400_HZ
#define ICM45686_GYRO_BW_SETTING    IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8
#endif

#define ICM45686_ACCEL_FSR_SETTING  ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G
#define ICM45686_GYRO_FSR_SETTING   GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS

typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} icm45686_bus_t;

typedef enum
{
    ICM45686_FIFO_ASYNC_IDLE = 0,
    ICM45686_FIFO_ASYNC_COUNT_FIRST,
    ICM45686_FIFO_ASYNC_COUNT_SECOND,
    ICM45686_FIFO_ASYNC_DATA,
    ICM45686_FIFO_ASYNC_RAW_READY,
    ICM45686_FIFO_ASYNC_ERROR
} icm45686_fifo_async_state_t;

typedef struct
{
    volatile icm45686_fifo_async_state_t state;
    uint64_t watermark_anchor_us;
    uint64_t pending_anchor_us;
    uint16_t requested_frame_count;
    uint16_t fifo_frame_count;
    volatile bool request_pending;
    uint8_t tx_data[ICM45686_FIFO_DMA_MAX_TRANSFER_BYTES];
    uint8_t rx_data[ICM45686_FIFO_DMA_MAX_TRANSFER_BYTES];
} icm45686_fifo_async_t;

static inv_imu_device_t s_device;
static icm45686_bus_t s_bus = {
    .spi = &hspi4,
    .cs_port = ICM45686_1_ACS_GPIO_Port,
    .cs_pin = ICM45686_1_ACS_Pin,
};
static imu_health_t s_health = {
    .status = IMU_STATUS_NOT_INITIALIZED,
};
static uint32_t s_sequence;
static uint8_t s_who_am_i;
static bool s_initialized;
static bool s_transport_failed;
static uint8_t s_spi_tx_buffer[ICM45686_SPI_MAX_TRANSFER + 1U];
static uint8_t s_spi_rx_buffer[ICM45686_SPI_MAX_TRANSFER + 1U];
static icm45686_configuration_t s_configuration;
static icm45686_fifo_async_t s_fifo_async;
static icm45686_fifo_frame_t s_fifo_parse_frames[ICM45686_FIFO_DMA_MAX_FRAMES];
static icm45686_fifo_frame_t s_fifo_output[ICM45686_FIFO_OUTPUT_DEPTH];
static volatile uint16_t s_fifo_output_head;
static volatile uint16_t s_fifo_output_tail;
static icm45686_fifo_diagnostics_t s_fifo_diagnostics;
static imu_clock_sync_t s_fifo_clock_sync;
static uint64_t s_fifo_previous_unwrapped_timestamp;
static bool s_fifo_previous_timestamp_valid;

_Static_assert(ICM45686_FIFO_DMA_MAX_TRANSFER_BYTES <= IMU_SPI_DMA_MAX_TRANSFER_SIZE,
               "ICM FIFO batch must fit the shared SPI DMA staging buffer");

static void icm45686_sleep_us(uint32_t delay_us)
{
    imu_time_delay_us(delay_us);
}

static void icm45686_chip_select_idle(void)
{
    HAL_GPIO_WritePin(ICM45686_1_GCS_GPIO_Port, ICM45686_1_GCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(s_bus.cs_port, s_bus.cs_pin, GPIO_PIN_SET);
}

static int icm45686_transport_error(void)
{
    s_transport_failed = true;
    s_health.bus_error_count++;
    icm45686_chip_select_idle();
    return -1;
}

static int icm45686_spi_write(void *context, uint8_t reg, const uint8_t *buffer, uint32_t length)
{
    icm45686_bus_t *bus = (icm45686_bus_t *)context;
    uint8_t command = reg & ICM45686_SPI_ADDRESS_MASK;
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (bus->spi == NULL) || ((buffer == NULL) && (length != 0U)) ||
        (length > ICM45686_SPI_MAX_TRANSFER) || imu_spi_dma_is_busy(bus->spi))
    {
        return icm45686_transport_error();
    }

    s_spi_tx_buffer[0] = command;
    if (length != 0U)
    {
        memcpy(&s_spi_tx_buffer[1], buffer, length);
    }

    HAL_GPIO_WritePin(ICM45686_1_GCS_GPIO_Port, ICM45686_1_GCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(bus->spi, s_spi_tx_buffer, (uint16_t)(length + 1U),
                              ICM45686_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_SET);
    if (status != HAL_OK)
    {
        return icm45686_transport_error();
    }

    return 0;
}

static int icm45686_spi_read(void *context, uint8_t reg, uint8_t *buffer, uint32_t length)
{
    icm45686_bus_t *bus = (icm45686_bus_t *)context;
    uint8_t command = reg | ICM45686_SPI_READ_BIT;
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (bus->spi == NULL) || ((buffer == NULL) && (length != 0U)) ||
        (length > ICM45686_SPI_MAX_TRANSFER) || imu_spi_dma_is_busy(bus->spi))
    {
        return icm45686_transport_error();
    }

    s_spi_tx_buffer[0] = command;
    if (length != 0U)
    {
        memset(&s_spi_tx_buffer[1], 0, length);
    }

    HAL_GPIO_WritePin(ICM45686_1_GCS_GPIO_Port, ICM45686_1_GCS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(bus->spi, s_spi_tx_buffer, s_spi_rx_buffer,
                                     (uint16_t)(length + 1U), ICM45686_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(bus->cs_port, bus->cs_pin, GPIO_PIN_SET);
    if (status != HAL_OK)
    {
        return icm45686_transport_error();
    }

    if (length != 0U)
    {
        memcpy(buffer, &s_spi_rx_buffer[1], length);
    }
    return 0;
}

static uint16_t icm45686_fifo_read_u16(const uint8_t *data)
{
    if (s_device.endianness_data != 0U)
        return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    return (uint16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static int32_t icm45686_fifo_extend_20(uint16_t upper, uint8_t lower)
{
    uint32_t value = ((uint32_t)upper << 4) | (uint32_t)(lower & 0x0FU);
    if ((value & UINT32_C(0x00080000)) != 0U)
        value |= UINT32_C(0xFFF00000);
    return (int32_t)value;
}

static bool icm45686_fifo_parse_frame(const uint8_t *data,
                                      uint64_t watermark_anchor_us,
                                      icm45686_fifo_frame_t *frame,
                                      bool *timestamp_discontinuity)
{
    uint16_t accel_upper[3];
    uint16_t gyro_upper[3];
    bool accel_valid = true;
    bool gyro_valid = true;

    memset(frame, 0, sizeof(*frame));
    *timestamp_discontinuity = false;
    frame->header = data[0];
    frame->watermark_anchor_us = watermark_anchor_us;
    frame->fsync_event = (frame->header & ICM45686_FIFO_HEADER_FSYNC) != 0U;

    const bool frame_layout_valid =
        ((frame->header & ICM45686_FIFO_HEADER_EXTENDED) == 0U) &&
        ((frame->header & ICM45686_FIFO_HEADER_HIGH_RES) != 0U) &&
        ((frame->header & (ICM45686_FIFO_HEADER_ACCEL |
                           ICM45686_FIFO_HEADER_GYRO)) ==
         (ICM45686_FIFO_HEADER_ACCEL |
          ICM45686_FIFO_HEADER_GYRO)) &&
        ((frame->header & (ICM45686_FIFO_HEADER_TIMESTAMP |
                           ICM45686_FIFO_HEADER_FSYNC)) != 0U);

    for (size_t axis = 0U; axis < 3U; ++axis)
    {
        accel_upper[axis] = icm45686_fifo_read_u16(&data[1U + (axis * 2U)]);
        gyro_upper[axis] = icm45686_fifo_read_u16(&data[7U + (axis * 2U)]);
        if (accel_upper[axis] == UINT16_C(0x8000))
            accel_valid = false;
        if (gyro_upper[axis] == UINT16_C(0x8000))
            gyro_valid = false;

        frame->accel_raw[axis] = icm45686_fifo_extend_20(
            accel_upper[axis], (uint8_t)(data[17U + axis] >> 4));
        frame->gyro_raw[axis] = icm45686_fifo_extend_20(
            gyro_upper[axis], data[17U + axis]);
    }

    frame->temperature_raw = (int16_t)icm45686_fifo_read_u16(&data[13]);
    frame->sensor_timestamp_us = icm45686_fifo_read_u16(&data[15]);
    frame->accel_valid = frame_layout_valid && accel_valid &&
        ((frame->header & ICM45686_FIFO_HEADER_ACCEL) != 0U);
    frame->gyro_valid = frame_layout_valid && gyro_valid &&
        ((frame->header & ICM45686_FIFO_HEADER_GYRO) != 0U);
    frame->temperature_valid =
        frame_layout_valid && (frame->temperature_raw != INT16_MIN);
    frame->timestamp_valid = frame_layout_valid && !frame->fsync_event &&
        ((frame->header & ICM45686_FIFO_HEADER_TIMESTAMP) != 0U);

    if (frame->accel_valid)
    {
        for (size_t axis = 0U; axis < 3U; ++axis)
            frame->accel_mps2[axis] =
                (float)frame->accel_raw[axis] * ICM45686_FIFO_ACCEL_SCALE_MPS2;
    }
    if (frame->gyro_valid)
    {
        for (size_t axis = 0U; axis < 3U; ++axis)
            frame->gyro_rad_s[axis] =
                (float)frame->gyro_raw[axis] * ICM45686_FIFO_GYRO_SCALE_RAD_S;
    }
    if (frame->temperature_valid)
    {
        frame->temperature_c =
            ((float)frame->temperature_raw * ICM45686_FIFO_TEMPERATURE_SCALE) +
            ICM45686_TEMPERATURE_OFFSET;
    }

    if (frame->timestamp_valid)
    {
        uint64_t unwrapped_timestamp;
        if (!imu_clock_sync_unwrap(&s_fifo_clock_sync,
                                   frame->sensor_timestamp_us,
                                   &unwrapped_timestamp))
        {
            frame->timestamp_valid = false;
            *timestamp_discontinuity = true;
        }
        else
        {
            if (s_fifo_previous_timestamp_valid &&
                ((unwrapped_timestamp - s_fifo_previous_unwrapped_timestamp) >
                 ICM45686_FIFO_MAX_TIMESTAMP_GAP_US))
            {
                *timestamp_discontinuity = true;
            }
            frame->sensor_timestamp_unwrapped_us = unwrapped_timestamp;
            s_fifo_previous_unwrapped_timestamp = unwrapped_timestamp;
            s_fifo_previous_timestamp_valid = true;
        }
    }

    return frame_layout_valid;
}

static int icm45686_verify_accel_configuration(void)
{
    accel_config0_t accel_config = {0};
    ipreg_sys2_reg_131_t accel_bw = {0};
    pwr_mgmt0_t power = {0};
    int status = INV_IMU_OK;

    status |= inv_imu_read_reg(&s_device, ACCEL_CONFIG0, 1U,
                               (uint8_t *)&accel_config);
    status |= inv_imu_read_reg(&s_device, IPREG_SYS2_REG_131, 1U,
                               (uint8_t *)&accel_bw);
    status |= inv_imu_read_reg(&s_device, PWR_MGMT0, 1U,
                               (uint8_t *)&power);
    if (status != INV_IMU_OK)
        return status;

    if ((accel_config.accel_odr != ICM45686_ACCEL_ODR_SETTING) ||
        (accel_config.accel_ui_fs_sel != ICM45686_ACCEL_FSR_SETTING) ||
        (accel_bw.accel_ui_lpfbw_sel != ICM45686_ACCEL_BW_SETTING) ||
        (power.accel_mode != PWR_MGMT0_ACCEL_MODE_LN))
        return INV_IMU_ERROR_BAD_ARG;

    return INV_IMU_OK;
}

static int icm45686_verify_gyro_configuration(void)
{
    gyro_config0_t gyro_config = {0};
    ipreg_sys1_reg_172_t gyro_bw = {0};
    pwr_mgmt0_t power = {0};
    int status = INV_IMU_OK;

    status |= inv_imu_read_reg(&s_device, GYRO_CONFIG0, 1U,
                               (uint8_t *)&gyro_config);
    status |= inv_imu_read_reg(&s_device, IPREG_SYS1_REG_172, 1U,
                               (uint8_t *)&gyro_bw);
    status |= inv_imu_read_reg(&s_device, PWR_MGMT0, 1U,
                               (uint8_t *)&power);
    if (status != INV_IMU_OK)
        return status;

    if ((gyro_config.gyro_odr != ICM45686_GYRO_ODR_SETTING) ||
        (gyro_config.gyro_ui_fs_sel != ICM45686_GYRO_FSR_SETTING) ||
        (gyro_bw.gyro_ui_lpfbw_sel != ICM45686_GYRO_BW_SETTING) ||
        (power.gyro_mode != PWR_MGMT0_GYRO_MODE_LN))
        return INV_IMU_ERROR_BAD_ARG;

    return INV_IMU_OK;
}

static int icm45686_configure_interface_and_timestamp(void)
{
    tmst_wom_config_t timestamp = {0};
    ioc_pad_scenario_aux_ovrd_t aux_override = {0};
    intf_config0_t interface_config = {0};
    int status = INV_IMU_OK;

    status |= inv_imu_adv_set_int2_pin_usage(
        &s_device, IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_INT2);
    status |= inv_imu_adv_configure_fsync_ap_tag(
        &s_device, FSYNC_CONFIG0_AP_FSYNC_NO);
    status |= inv_imu_read_reg(&s_device, TMST_WOM_CONFIG, 1U,
                               (uint8_t *)&timestamp);
    status |= inv_imu_read_reg(&s_device, IOC_PAD_SCENARIO_AUX_OVRD, 1U,
                               (uint8_t *)&aux_override);
    status |= inv_imu_read_reg(&s_device, INTF_CONFIG0, 1U,
                               (uint8_t *)&interface_config);
    if (status != INV_IMU_OK)
        return status;

    timestamp.tmst_delta_en = INV_IMU_DISABLE;
    aux_override.aux1_enable_ovrd = INV_IMU_ENABLE;
    aux_override.aux1_enable_ovrd_val = INV_IMU_DISABLE;
    interface_config.virtual_access_aux1_en = INV_IMU_DISABLE;
    status |= inv_imu_write_reg(&s_device, TMST_WOM_CONFIG, 1U,
                                (uint8_t *)&timestamp);
    status |= inv_imu_write_reg(&s_device, IOC_PAD_SCENARIO_AUX_OVRD, 1U,
                                (uint8_t *)&aux_override);
    status |= inv_imu_write_reg(&s_device, INTF_CONFIG0, 1U,
                                (uint8_t *)&interface_config);
    return status;
}

static int icm45686_verify_shared_configuration(void)
{
    inv_imu_int_state_t interrupt_source = {0};
    int1_config2_t interrupt_pin = {0};
    inv_imu_adv_fifo_config_t fifo = {0};
    tmst_wom_config_t timestamp = {0};
    smc_control_0_t timestamp_control = {0};
    fsync_config0_t fsync = {0};
    ioc_pad_scenario_aux_ovrd_t aux_override = {0};
    ioc_pad_scenario_ovrd_t pin_override = {0};
    intf_config0_t interface_config = {0};
    int status = INV_IMU_OK;

    status |= inv_imu_get_config_int(&s_device, INV_IMU_INT1,
                                     &interrupt_source);
    status |= inv_imu_read_reg(&s_device, INT1_CONFIG2, 1U,
                               (uint8_t *)&interrupt_pin);
    status |= inv_imu_adv_get_fifo_config(&s_device, &fifo);
    status |= inv_imu_read_reg(&s_device, TMST_WOM_CONFIG, 1U,
                               (uint8_t *)&timestamp);
    status |= inv_imu_read_reg(&s_device, SMC_CONTROL_0, 1U,
                               (uint8_t *)&timestamp_control);
    status |= inv_imu_read_reg(&s_device, FSYNC_CONFIG0, 1U,
                               (uint8_t *)&fsync);
    status |= inv_imu_read_reg(&s_device, IOC_PAD_SCENARIO_AUX_OVRD, 1U,
                               (uint8_t *)&aux_override);
    status |= inv_imu_read_reg(&s_device, IOC_PAD_SCENARIO_OVRD, 1U,
                               (uint8_t *)&pin_override);
    status |= inv_imu_read_reg(&s_device, INTF_CONFIG0, 1U,
                               (uint8_t *)&interface_config);
    if (status != INV_IMU_OK)
        return status;

    if ((interrupt_source.INV_FIFO_THS != INV_IMU_ENABLE) ||
        (interrupt_source.INV_FIFO_FULL != INV_IMU_ENABLE) ||
        (interrupt_source.INV_UI_DRDY != INV_IMU_DISABLE) ||
        (interrupt_pin.int1_polarity != INTX_CONFIG2_INTX_POLARITY_HIGH) ||
        (interrupt_pin.int1_mode != INTX_CONFIG2_INTX_MODE_PULSE) ||
        (interrupt_pin.int1_drive != INTX_CONFIG2_INTX_DRIVE_PP) ||
        (fifo.base_conf.fifo_mode != FIFO_CONFIG0_FIFO_MODE_SNAPSHOT) ||
        (fifo.base_conf.fifo_depth != FIFO_CONFIG0_FIFO_DEPTH_MAX) ||
        (fifo.base_conf.fifo_wm_th != ICM45686_FIFO_WATERMARK_FRAMES) ||
        (fifo.base_conf.accel_en != INV_IMU_ENABLE) ||
        (fifo.base_conf.gyro_en != INV_IMU_ENABLE) ||
        (fifo.base_conf.hires_en != INV_IMU_ENABLE) ||
        (fifo.fifo_wr_wm_gt_th != FIFO_CONFIG2_FIFO_WR_WM_EQ_TH) ||
        (fifo.tmst_fsync_en != INV_IMU_ENABLE) ||
        (fifo.comp_en != INV_IMU_DISABLE) ||
        (fifo.es0_en != INV_IMU_DISABLE) ||
        (fifo.es1_en != INV_IMU_DISABLE) ||
        (fifo.accel_dec != ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1) ||
        (fifo.gyro_dec != ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1) ||
        (timestamp.tmst_delta_en != INV_IMU_DISABLE) ||
        (timestamp_control.tmst_en != INV_IMU_ENABLE) ||
        (timestamp_control.tmst_fsync_en != INV_IMU_ENABLE) ||
        (fsync.ap_fsync_sel != FSYNC_CONFIG0_AP_FSYNC_NO) ||
        (aux_override.aux1_enable_ovrd != INV_IMU_ENABLE) ||
        (aux_override.aux1_enable_ovrd_val != INV_IMU_DISABLE) ||
        (pin_override.pads_int2_cfg_ovrd != INV_IMU_ENABLE) ||
        (pin_override.pads_int2_cfg_ovrd_val !=
         IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_INT2) ||
        (interface_config.virtual_access_aux1_en != INV_IMU_DISABLE) ||
        (s_device.fifo_frame_size != ICM45686_FIFO_FRAME_SIZE_BYTES) ||
        (inv_imu_adv_get_timestamp_resolution_us(&s_device) !=
         ICM45686_FIFO_TIMESTAMP_RESOLUTION_US))
        return INV_IMU_ERROR_BAD_ARG;

    return INV_IMU_OK;
}

static int icm45686_verify_configuration(void)
{
    int status = icm45686_verify_accel_configuration();
    status |= icm45686_verify_gyro_configuration();
    status |= icm45686_verify_shared_configuration();
    if (status != INV_IMU_OK)
        return status;

    s_configuration.register_readback_verified = true;
    return INV_IMU_OK;
}

static int icm45686_configure_sensor(void)
{
    int status;
    inv_imu_int_state_t interrupt_sources = {0};
    const inv_imu_adv_fifo_config_t fifo_config = {
        .base_conf = {
            .gyro_en = INV_IMU_ENABLE,
            .accel_en = INV_IMU_ENABLE,
            .hires_en = INV_IMU_ENABLE,
            .fifo_wm_th = ICM45686_FIFO_WATERMARK_FRAMES,
            .fifo_mode = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT,
            .fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_MAX,
        },
        /* Pulse exactly when the second packet is written.  In >= mode every
         * later write can generate another pulse, so a PE4 edge no longer
         * identifies the second packet of the batch used as the clock anchor. */
        .fifo_wr_wm_gt_th = FIFO_CONFIG2_FIFO_WR_WM_EQ_TH,
        .tmst_fsync_en = INV_IMU_ENABLE,
        .es1_en = INV_IMU_DISABLE,
        .es0_en = INV_IMU_DISABLE,
        .es0_6b_9b = FIFO_CONFIG4_FIFO_ES0_6B,
        .comp_en = INV_IMU_DISABLE,
        .comp_nc_flow_cfg = FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_DIS,
        .gyro_dec = ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1,
        .accel_dec = ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1,
    };
    const inv_imu_int_pin_config_t interrupt_pin = {
        .int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH,
        .int_mode = INTX_CONFIG2_INTX_MODE_PULSE,
        .int_drive = INTX_CONFIG2_INTX_DRIVE_PP,
    };

    status = inv_imu_set_accel_fsr(&s_device, ICM45686_ACCEL_FSR_SETTING);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_set_gyro_fsr(&s_device, ICM45686_GYRO_FSR_SETTING);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_set_accel_frequency(&s_device, ICM45686_ACCEL_ODR_SETTING);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_set_gyro_frequency(&s_device, ICM45686_GYRO_ODR_SETTING);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_set_accel_ln_bw(&s_device, ICM45686_ACCEL_BW_SETTING);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_set_gyro_ln_bw(&s_device, ICM45686_GYRO_BW_SETTING);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_adv_set_timestamp_resolution(
        &s_device, TMST_WOM_CONFIG_TMST_RESOL_1_US);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = icm45686_configure_interface_and_timestamp();
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_set_pin_config_int(&s_device, INV_IMU_INT1,
                                        &interrupt_pin);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    interrupt_sources.INV_FIFO_THS = INV_IMU_ENABLE;
    interrupt_sources.INV_FIFO_FULL = INV_IMU_ENABLE;
    status = inv_imu_set_config_int(&s_device, INV_IMU_INT1,
                                    &interrupt_sources);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    /* Configure FIFO/FDR while both sensors are still off, as required. */
    status = inv_imu_adv_set_fifo_config(&s_device, &fifo_config);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_set_accel_mode(&s_device, PWR_MGMT0_ACCEL_MODE_LN);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    status = inv_imu_set_gyro_mode(&s_device, PWR_MGMT0_GYRO_MODE_LN);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    icm45686_sleep_us(ICM45686_STARTUP_DELAY_US);

    /* Remove startup transients/backlog before exposing the watermark stream. */
    status = inv_imu_adv_reset_fifo(&s_device);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    return icm45686_verify_configuration();
}

bool icm45686_init(void)
{
    int status;
    const imu_clock_sync_config_t clock_sync_config = {
        .counter_bits = 16U,
        .nominal_tick_us = 1.0,
        .minimum_clock_scale = 0.95,
        .maximum_clock_scale = 1.05,
        .phase_time_constant_us = 20000.0,
        .rate_time_constant_us = 2000000.0,
        .minimum_anchor_interval_us = 500.0,
        .initial_residual_limit_us = 1000.0,
    };

    memset(&s_device, 0, sizeof(s_device));
    memset(&s_health, 0, sizeof(s_health));
    memset(&s_fifo_async, 0, sizeof(s_fifo_async));
    memset(&s_fifo_diagnostics, 0, sizeof(s_fifo_diagnostics));
    s_health.status = IMU_STATUS_NOT_INITIALIZED;
    s_fifo_async.state = ICM45686_FIFO_ASYNC_IDLE;
    s_fifo_output_head = 0U;
    s_fifo_output_tail = 0U;
    s_fifo_previous_unwrapped_timestamp = 0U;
    s_fifo_previous_timestamp_valid = false;
    s_sequence = 0U;
    s_who_am_i = 0U;
    s_initialized = false;
    s_transport_failed = false;
    s_configuration = (icm45686_configuration_t) {
        .accel_odr_hz = ICM45686_CONFIG_ACCEL_ODR_HZ,
        .accel_bandwidth_hz = ICM45686_CONFIG_ACCEL_BANDWIDTH_HZ,
        .accel_range_g = ICM45686_CONFIG_ACCEL_RANGE_G,
        .gyro_odr_hz = ICM45686_CONFIG_GYRO_ODR_HZ,
        .gyro_bandwidth_hz = ICM45686_CONFIG_GYRO_BANDWIDTH_HZ,
        .gyro_range_dps = ICM45686_CONFIG_GYRO_RANGE_DPS,
        .fifo_frame_size_bytes = ICM45686_FIFO_FRAME_SIZE_BYTES,
        .fifo_watermark_frames = ICM45686_FIFO_WATERMARK_FRAMES,
        .fifo_timestamp_resolution_us = ICM45686_FIFO_TIMESTAMP_RESOLUTION_US,
        .fifo_enabled = true,
        .register_readback_verified = false,
    };

    if (!imu_clock_sync_init(&s_fifo_clock_sync, &clock_sync_config))
    {
        s_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    icm45686_chip_select_idle();

    if ((hspi4.Instance != SPI4) || !imu_time_is_running())
    {
        s_health.status = IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    s_device.transport.context = &s_bus;
    s_device.transport.read_reg = icm45686_spi_read;
    s_device.transport.write_reg = icm45686_spi_write;
    s_device.transport.serif_type = UI_SPI4;
    s_device.transport.sleep_us = icm45686_sleep_us;

    s_transport_failed = false;
    status = inv_imu_adv_init(&s_device);
    if (status == INV_IMU_OK)
    {
        status = inv_imu_get_who_am_i(&s_device, &s_who_am_i);
    }
    if ((status == INV_IMU_OK) && (s_who_am_i != INV_IMU_WHOAMI))
    {
        s_health.status = IMU_STATUS_BAD_ID;
        return false;
    }
    if (status == INV_IMU_OK)
    {
        status = icm45686_configure_sensor();
    }
    if (status != INV_IMU_OK)
    {
        s_health.status = s_transport_failed ? IMU_STATUS_BUS_ERROR : IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    s_initialized = true;
    s_health.status = IMU_STATUS_OK;
    return true;
}

static bool icm45686_check_identity(void)
{
    uint8_t who_am_i = 0U;
    s_transport_failed = false;
    const int status = inv_imu_get_who_am_i(&s_device, &who_am_i);
    if ((status != INV_IMU_OK) || (who_am_i != INV_IMU_WHOAMI)) {
        s_health.status = s_transport_failed ? IMU_STATUS_BUS_ERROR
                                             : IMU_STATUS_BAD_ID;
        return false;
    }
    return true;
}

static bool icm45686_check_result(int status)
{
    if (status != INV_IMU_OK) {
        s_health.status = s_transport_failed ? IMU_STATUS_BUS_ERROR
                                             : IMU_STATUS_CONFIG_ERROR;
        return false;
    }
    return true;
}

bool icm45686_check_accel_configuration(void)
{
    if (!s_initialized || !icm45686_check_identity())
        return false;

    s_transport_failed = false;
    int status = icm45686_verify_accel_configuration();
    status |= icm45686_verify_shared_configuration();
    return icm45686_check_result(status);
}

bool icm45686_check_gyro_configuration(void)
{
    if (!s_initialized || !icm45686_check_identity())
        return false;

    s_transport_failed = false;
    int status = icm45686_verify_gyro_configuration();
    status |= icm45686_verify_shared_configuration();
    return icm45686_check_result(status);
}

bool icm45686_check_configuration(void)
{
    if (!s_initialized)
        return false;

    const bool accel_ok = icm45686_check_accel_configuration();
    const bool gyro_ok = icm45686_check_gyro_configuration();
    if (!accel_ok || !gyro_ok)
        return false;

    s_health.status = IMU_STATUS_OK;
    return true;
}

bool icm45686_read(uint64_t timestamp_us, imu_sample_t *sample)
{
    inv_imu_sensor_data_t raw_data;
    int status;

    if (sample == NULL)
    {
        return false;
    }

    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = timestamp_us;
    sample->accel_timestamp_us = timestamp_us;
    sample->gyro_timestamp_us = timestamp_us;
    sample->source = IMU_SOURCE_ICM45686;

    if (!s_initialized)
    {
        s_health.status = IMU_STATUS_NOT_INITIALIZED;
        return false;
    }

    s_transport_failed = false;
    status = inv_imu_get_register_data(&s_device, &raw_data);
    if (status != INV_IMU_OK)
    {
        s_health.status = s_transport_failed ? IMU_STATUS_BUS_ERROR : IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    for (uint32_t axis = 0U; axis < 3U; axis++)
    {
        sample->accel_mps2[axis] = (float)raw_data.accel_data[axis] * ICM45686_ACCEL_SCALE_MPS2;
        sample->gyro_rad_s[axis] = (float)raw_data.gyro_data[axis] * ICM45686_GYRO_SCALE_RAD_S;
    }
    sample->temperature_c = ((float)raw_data.temp_data * ICM45686_TEMPERATURE_SCALE) +
                            ICM45686_TEMPERATURE_OFFSET;
    sample->sequence = ++s_sequence;
    sample->accel_sequence = sample->sequence;
    sample->gyro_sequence = sample->sequence;
    sample->accel_valid = true;
    sample->gyro_valid = true;
    sample->valid = true;

    s_health.status = IMU_STATUS_OK;
    s_health.read_count++;
    s_health.last_sample_us = timestamp_us;
    return true;
}

size_t icm45686_fifo_dma_transfer_size(uint16_t frame_count)
{
    if ((frame_count == 0U) ||
        (frame_count > ICM45686_FIFO_DMA_MAX_FRAMES))
    {
        return 0U;
    }

    return (size_t)ICM45686_FIFO_SPI_OVERHEAD_BYTES +
           ((size_t)frame_count * ICM45686_FIFO_FRAME_SIZE_BYTES);
}

bool icm45686_fifo_prepare_count_read(uint8_t *tx_data, size_t tx_capacity)
{
    if ((tx_data == NULL) ||
        (tx_capacity < ICM45686_FIFO_COUNT_TRANSFER_BYTES))
    {
        return false;
    }

    memset(tx_data, 0, ICM45686_FIFO_COUNT_TRANSFER_BYTES);
    tx_data[0] = FIFO_COUNT_0 | ICM45686_SPI_READ_BIT;
    return true;
}

bool icm45686_fifo_parse_count_response(const uint8_t *rx_data,
                                        size_t rx_length,
                                        uint16_t *frame_count)
{
    if ((rx_data == NULL) || (frame_count == NULL) ||
        (rx_length < ICM45686_FIFO_COUNT_TRANSFER_BYTES))
    {
        return false;
    }

    *frame_count = icm45686_fifo_read_u16(&rx_data[1]);
    return true;
}

bool icm45686_fifo_prepare_data_read(uint16_t frame_count,
                                     uint8_t *tx_data,
                                     size_t tx_capacity)
{
    const size_t transfer_size =
        icm45686_fifo_dma_transfer_size(frame_count);
    if ((tx_data == NULL) || (transfer_size == 0U) ||
        (tx_capacity < transfer_size))
    {
        return false;
    }

    memset(tx_data, 0, transfer_size);
    tx_data[0] = FIFO_DATA | ICM45686_SPI_READ_BIT;
    return true;
}

bool icm45686_fifo_parse_dma_response(
    const uint8_t *rx_data,
    size_t rx_length,
    uint16_t frame_count,
    uint64_t watermark_anchor_us,
    icm45686_fifo_frame_t *frames,
    size_t frame_capacity,
    icm45686_fifo_batch_report_t *report)
{
    const size_t transfer_size =
        icm45686_fifo_dma_transfer_size(frame_count);
    icm45686_fifo_batch_report_t local_report = {
        .watermark_anchor_us = watermark_anchor_us,
        .requested_frame_count = frame_count,
    };

    if ((rx_data == NULL) || (frames == NULL) || (transfer_size == 0U) ||
        (rx_length < transfer_size) || (frame_capacity < frame_count))
    {
        return false;
    }

    for (uint16_t index = 0U; index < frame_count; ++index)
    {
        bool timestamp_discontinuity;
        const uint8_t *frame_data =
            &rx_data[ICM45686_FIFO_SPI_OVERHEAD_BYTES +
                     ((size_t)index * ICM45686_FIFO_FRAME_SIZE_BYTES)];
        const bool header_valid = icm45686_fifo_parse_frame(
            frame_data, watermark_anchor_us, &frames[index],
            &timestamp_discontinuity);

        local_report.parsed_frame_count++;
        if (frames[index].accel_valid)
            local_report.accel_frame_count++;
        if (frames[index].gyro_valid)
            local_report.gyro_frame_count++;
        if (!header_valid)
            local_report.malformed_frame_count++;
        if (timestamp_discontinuity)
            local_report.timestamp_discontinuity_count++;
    }

    /* In EQ mode the PE4 pulse is emitted when packet WM is written.  Packets
     * arriving before DMA starts do not change that ownership: the edge still
     * identifies packet WM-1 in the oldest-first FIFO batch. */
    if ((frame_count >= ICM45686_FIFO_WATERMARK_FRAMES) &&
        (watermark_anchor_us != 0U) &&
        (local_report.malformed_frame_count == 0U) &&
        (local_report.timestamp_discontinuity_count == 0U) &&
        frames[0].timestamp_valid && frames[1].timestamp_valid)
    {
        icm45686_fifo_frame_t *anchor_frame =
            &frames[ICM45686_FIFO_WATERMARK_FRAMES - 1U];
        if (anchor_frame->timestamp_valid)
        {
            (void)imu_clock_sync_observe(
                &s_fifo_clock_sync,
                anchor_frame->sensor_timestamp_unwrapped_us,
                watermark_anchor_us);
        }
    }

    imu_clock_sync_diagnostics_t clock_diagnostics;
    imu_clock_sync_get_diagnostics(&s_fifo_clock_sync, &clock_diagnostics);
    const uint64_t batch_arrival_us = imu_time_now_us();
    const uint64_t clock_mapping_limit_us =
        batch_arrival_us + ICM45686_CLOCK_CAUSAL_TOLERANCE_US;
    const bool stale_clock_reference =
        clock_diagnostics.valid &&
        !imu_clock_sync_reference_is_fresh(
            &s_fifo_clock_sync, batch_arrival_us,
            ICM45686_CLOCK_REFERENCE_MAX_AGE_US);
    bool causal_mapping_failure = false;
    bool reset_timestamp_state =
        (local_report.timestamp_discontinuity_count != 0U) ||
        stale_clock_reference;
    for (uint16_t index = 0U; index < frame_count; ++index)
    {
        if (!reset_timestamp_state && frames[index].timestamp_valid &&
            clock_diagnostics.valid)
        {
            frames[index].mcu_timestamp_valid = imu_clock_sync_map_bounded(
                &s_fifo_clock_sync,
                frames[index].sensor_timestamp_unwrapped_us,
                clock_mapping_limit_us,
                &frames[index].mcu_timestamp_us);
            if (!frames[index].mcu_timestamp_valid)
            {
                causal_mapping_failure = true;
                reset_timestamp_state = true;
            }
        }
    }

    if (reset_timestamp_state)
    {
        if (stale_clock_reference)
            s_fifo_diagnostics.clock_stale_reset_count++;
        if (causal_mapping_failure)
            s_fifo_diagnostics.clock_causal_reset_count++;
        for (uint16_t index = 0U; index < frame_count; ++index)
            frames[index].mcu_timestamp_valid = false;
        imu_clock_sync_reset(&s_fifo_clock_sync);
        s_fifo_previous_unwrapped_timestamp = 0U;
        s_fifo_previous_timestamp_valid = false;
    }

    s_fifo_diagnostics.malformed_frame_count +=
        local_report.malformed_frame_count;
    s_fifo_diagnostics.timestamp_discontinuity_count +=
        local_report.timestamp_discontinuity_count;
    if (report != NULL)
        *report = local_report;
    return true;
}

void icm45686_fifo_reset_timestamp_unwrap(void)
{
    imu_clock_sync_reset(&s_fifo_clock_sync);
    s_fifo_previous_unwrapped_timestamp = 0U;
    s_fifo_previous_timestamp_valid = false;
}

static void icm45686_fifo_dma_callback(SPI_HandleTypeDef *spi,
                                       imu_spi_dma_status_t dma_status,
                                       uint16_t length,
                                       void *context);

static imu_spi_dma_status_t icm45686_fifo_submit_dma(uint16_t length)
{
    const imu_spi_dma_request_t request = {
        .spi = &hspi4,
        .cs_port = s_bus.cs_port,
        .cs_pin = s_bus.cs_pin,
        .tx_data = s_fifo_async.tx_data,
        .rx_data = s_fifo_async.rx_data,
        .length = length,
        .timeout_ms = ICM45686_FIFO_DMA_TIMEOUT_MS,
        .callback = icm45686_fifo_dma_callback,
        .context = &s_fifo_async,
    };

    HAL_GPIO_WritePin(ICM45686_1_GCS_GPIO_Port, ICM45686_1_GCS_Pin,
                      GPIO_PIN_SET);
    return imu_spi_dma_submit(&request);
}

static void icm45686_fifo_record_dma_error(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    s_fifo_diagnostics.dma_error_count++;
    s_health.bus_error_count++;
    s_health.status = IMU_STATUS_BUS_ERROR;
    /* A failed or partial transaction makes edge-to-frame ownership ambiguous.
     * Schedule a drain, but never reuse that edge as a clock anchor. */
    s_fifo_async.pending_anchor_us = 0U;
    s_fifo_async.request_pending = true;
    s_fifo_async.state = ICM45686_FIFO_ASYNC_ERROR;

    __set_PRIMASK(primask);
}

static bool icm45686_fifo_start_count_dma(uint64_t watermark_anchor_us)
{
    if (!icm45686_fifo_prepare_count_read(
            s_fifo_async.tx_data, sizeof(s_fifo_async.tx_data)))
    {
        return false;
    }

    s_fifo_async.watermark_anchor_us = watermark_anchor_us;
    s_fifo_async.state = ICM45686_FIFO_ASYNC_COUNT_FIRST;
    const imu_spi_dma_status_t dma_status = icm45686_fifo_submit_dma(
        ICM45686_FIFO_COUNT_TRANSFER_BYTES);
    if (dma_status != IMU_SPI_DMA_STATUS_OK)
    {
        s_fifo_async.state = ICM45686_FIFO_ASYNC_IDLE;
        if (dma_status != IMU_SPI_DMA_STATUS_BUSY)
            icm45686_fifo_record_dma_error();
        return false;
    }
    return true;
}

static void icm45686_fifo_dma_callback(SPI_HandleTypeDef *spi,
                                       imu_spi_dma_status_t dma_status,
                                       uint16_t length,
                                       void *context)
{
    if ((spi != &hspi4) || (context != &s_fifo_async) ||
        (dma_status != IMU_SPI_DMA_STATUS_OK))
    {
        icm45686_fifo_record_dma_error();
        return;
    }

    if (s_fifo_async.state == ICM45686_FIFO_ASYNC_COUNT_FIRST)
    {
        if ((length != ICM45686_FIFO_COUNT_TRANSFER_BYTES) ||
            !icm45686_fifo_prepare_count_read(
                s_fifo_async.tx_data, sizeof(s_fifo_async.tx_data)))
        {
            icm45686_fifo_record_dma_error();
            return;
        }

        /* AN-000364 requires a second FIFO_COUNT read; only it is used. */
        s_fifo_async.state = ICM45686_FIFO_ASYNC_COUNT_SECOND;
        if (icm45686_fifo_submit_dma(ICM45686_FIFO_COUNT_TRANSFER_BYTES) !=
            IMU_SPI_DMA_STATUS_OK)
        {
            icm45686_fifo_record_dma_error();
        }
        return;
    }

    if (s_fifo_async.state == ICM45686_FIFO_ASYNC_COUNT_SECOND)
    {
        uint16_t fifo_frame_count;
        if ((length != ICM45686_FIFO_COUNT_TRANSFER_BYTES) ||
            !icm45686_fifo_parse_count_response(
                s_fifo_async.rx_data, sizeof(s_fifo_async.rx_data),
                &fifo_frame_count))
        {
            s_fifo_diagnostics.count_read_error_count++;
            icm45686_fifo_record_dma_error();
            return;
        }

        s_fifo_async.fifo_frame_count = fifo_frame_count;
        s_fifo_diagnostics.last_fifo_count = fifo_frame_count;
        if (fifo_frame_count > s_fifo_diagnostics.peak_fifo_count)
            s_fifo_diagnostics.peak_fifo_count = fifo_frame_count;
        if (fifo_frame_count >= ICM45686_FIFO_FULL_FRAME_COUNT)
            s_fifo_diagnostics.fifo_full_count++;

        if (fifo_frame_count == 0U)
        {
            s_fifo_async.state = ICM45686_FIFO_ASYNC_IDLE;
            return;
        }

        s_fifo_async.requested_frame_count = fifo_frame_count;
        if (s_fifo_async.requested_frame_count >
            ICM45686_FIFO_DMA_MAX_FRAMES)
        {
            /* A backlog larger than one DMA batch may have been woken by
             * FIFO_FULL or by an edge that no longer owns packet WM. Drain it
             * completely, but never turn that interrupt into a clock anchor. */
            s_fifo_async.watermark_anchor_us = 0U;
            s_fifo_async.requested_frame_count =
                ICM45686_FIFO_DMA_MAX_FRAMES;
            if (!s_fifo_async.request_pending)
            {
                s_fifo_async.pending_anchor_us = 0U;
                s_fifo_async.request_pending = true;
            }
        }

        const size_t transfer_size = icm45686_fifo_dma_transfer_size(
            s_fifo_async.requested_frame_count);
        if (!icm45686_fifo_prepare_data_read(
                s_fifo_async.requested_frame_count,
                s_fifo_async.tx_data, sizeof(s_fifo_async.tx_data)) ||
            (transfer_size > UINT16_MAX))
        {
            icm45686_fifo_record_dma_error();
            return;
        }

        s_fifo_async.state = ICM45686_FIFO_ASYNC_DATA;
        if (icm45686_fifo_submit_dma((uint16_t)transfer_size) !=
            IMU_SPI_DMA_STATUS_OK)
        {
            icm45686_fifo_record_dma_error();
        }
        return;
    }

    if (s_fifo_async.state == ICM45686_FIFO_ASYNC_DATA)
    {
        const size_t expected_length = icm45686_fifo_dma_transfer_size(
            s_fifo_async.requested_frame_count);
        if ((size_t)length != expected_length)
        {
            icm45686_fifo_record_dma_error();
            return;
        }

        s_fifo_diagnostics.dma_batch_count++;
        s_fifo_async.state = ICM45686_FIFO_ASYNC_RAW_READY;
        return;
    }

    icm45686_fifo_record_dma_error();
}

bool icm45686_fifo_request(uint64_t watermark_anchor_us)
{
    if (!s_initialized || (watermark_anchor_us == 0U))
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_fifo_diagnostics.irq_request_count++;
    if ((s_fifo_async.state != ICM45686_FIFO_ASYNC_IDLE) ||
        imu_spi_dma_is_busy(&hspi4))
    {
        if (s_fifo_async.request_pending)
            s_fifo_diagnostics.coalesced_irq_count++;
        /* The edge can belong to frames already covered by the active count or
         * data transaction. Drain the backlog without using a stale anchor. */
        s_fifo_async.pending_anchor_us = 0U;
        s_fifo_async.request_pending = true;
        __set_PRIMASK(primask);
        return true;
    }
    __set_PRIMASK(primask);

    if (!icm45686_fifo_start_count_dma(watermark_anchor_us))
    {
        const uint32_t retry_primask = __get_PRIMASK();
        __disable_irq();
        s_fifo_async.pending_anchor_us = 0U;
        s_fifo_async.request_pending = true;
        __set_PRIMASK(retry_primask);
    }
    return true;
}

static void icm45686_fifo_output_push(const icm45686_fifo_frame_t *frame)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint16_t next_head =
        (uint16_t)((s_fifo_output_head + 1U) % ICM45686_FIFO_OUTPUT_DEPTH);
    if (next_head == s_fifo_output_tail)
    {
        s_fifo_output_tail =
            (uint16_t)((s_fifo_output_tail + 1U) % ICM45686_FIFO_OUTPUT_DEPTH);
        s_fifo_diagnostics.output_overrun_count++;
    }
    s_fifo_output[s_fifo_output_head] = *frame;
    s_fifo_output_head = next_head;

    __set_PRIMASK(primask);
}

void icm45686_fifo_service(void)
{
    if (!s_initialized)
        return;

    if (s_fifo_async.state == ICM45686_FIFO_ASYNC_RAW_READY)
    {
        icm45686_fifo_batch_report_t report;
        const uint16_t frame_count = s_fifo_async.requested_frame_count;
        const size_t transfer_size =
            icm45686_fifo_dma_transfer_size(frame_count);
        const bool parsed = icm45686_fifo_parse_dma_response(
            s_fifo_async.rx_data, transfer_size, frame_count,
            s_fifo_async.watermark_anchor_us, s_fifo_parse_frames,
            ICM45686_FIFO_DMA_MAX_FRAMES, &report);

        if (!parsed)
        {
            icm45686_fifo_record_dma_error();
        }
        else
        {
            for (uint16_t index = 0U; index < report.parsed_frame_count;
                 ++index)
            {
                icm45686_fifo_output_push(&s_fifo_parse_frames[index]);
                if (s_fifo_parse_frames[index].accel_valid ||
                    s_fifo_parse_frames[index].gyro_valid)
                {
                    s_health.read_count++;
                    s_health.last_sample_us =
                        s_fifo_parse_frames[index].mcu_timestamp_valid
                            ? s_fifo_parse_frames[index].mcu_timestamp_us
                            : s_fifo_parse_frames[index].watermark_anchor_us;
                }
            }
            s_health.status = IMU_STATUS_OK;
            s_fifo_async.state = ICM45686_FIFO_ASYNC_IDLE;
        }
    }

    if (s_fifo_async.state == ICM45686_FIFO_ASYNC_ERROR)
        s_fifo_async.state = ICM45686_FIFO_ASYNC_IDLE;

    if ((s_fifo_async.state == ICM45686_FIFO_ASYNC_IDLE) &&
        s_fifo_async.request_pending && !imu_spi_dma_is_busy(&hspi4))
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const uint64_t pending_anchor_us = s_fifo_async.pending_anchor_us;
        s_fifo_async.request_pending = false;
        s_fifo_async.pending_anchor_us = 0U;
        __set_PRIMASK(primask);

        if (!icm45686_fifo_start_count_dma(pending_anchor_us))
        {
            const uint32_t retry_primask = __get_PRIMASK();
            __disable_irq();
            s_fifo_async.pending_anchor_us = pending_anchor_us;
            s_fifo_async.request_pending = true;
            __set_PRIMASK(retry_primask);
        }
    }
}

bool icm45686_fifo_pop_frame(icm45686_fifo_frame_t *frame)
{
    if (frame == NULL)
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_fifo_output_head == s_fifo_output_tail)
    {
        __set_PRIMASK(primask);
        return false;
    }

    *frame = s_fifo_output[s_fifo_output_tail];
    s_fifo_output_tail =
        (uint16_t)((s_fifo_output_tail + 1U) % ICM45686_FIFO_OUTPUT_DEPTH);
    __set_PRIMASK(primask);
    return true;
}

bool icm45686_fifo_read_frame_count(uint16_t *frame_count)
{
    if (!s_initialized || (frame_count == NULL) ||
        imu_spi_dma_is_busy(&hspi4))
    {
        return false;
    }

    s_transport_failed = false;
    const int status = inv_imu_get_frame_count(&s_device, frame_count);
    if (status != INV_IMU_OK)
    {
        s_fifo_diagnostics.count_read_error_count++;
        s_health.status = s_transport_failed ? IMU_STATUS_BUS_ERROR
                                             : IMU_STATUS_CONFIG_ERROR;
        return false;
    }
    return true;
}

bool icm45686_fifo_flush(void)
{
    if (!s_initialized ||
        (s_fifo_async.state != ICM45686_FIFO_ASYNC_IDLE) ||
        imu_spi_dma_is_busy(&hspi4))
    {
        return false;
    }

    s_transport_failed = false;
    if (inv_imu_adv_reset_fifo(&s_device) != INV_IMU_OK)
    {
        s_health.status = s_transport_failed ? IMU_STATUS_BUS_ERROR
                                             : IMU_STATUS_CONFIG_ERROR;
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_fifo_async.request_pending = false;
    s_fifo_async.pending_anchor_us = 0U;
    s_fifo_output_head = 0U;
    s_fifo_output_tail = 0U;
    __set_PRIMASK(primask);

    icm45686_fifo_reset_timestamp_unwrap();
    s_fifo_diagnostics.fifo_flush_count++;
    s_health.status = IMU_STATUS_OK;
    return true;
}

void icm45686_fifo_get_diagnostics(icm45686_fifo_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
        return;

    imu_clock_sync_diagnostics_t clock_diagnostics;
    imu_clock_sync_get_diagnostics(&s_fifo_clock_sync, &clock_diagnostics);

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *diagnostics = s_fifo_diagnostics;
    diagnostics->dma_active =
        s_fifo_async.state != ICM45686_FIFO_ASYNC_IDLE;
    diagnostics->request_pending = s_fifo_async.request_pending;
    __set_PRIMASK(primask);

    diagnostics->clock_anchor_accepted_count =
        clock_diagnostics.accepted_anchor_count;
    diagnostics->clock_anchor_rejected_count =
        clock_diagnostics.rejected_anchor_count;
    diagnostics->clock_nonmonotonic_reject_count =
        clock_diagnostics.nonmonotonic_reject_count;
    diagnostics->clock_interval_reject_count =
        clock_diagnostics.interval_reject_count;
    diagnostics->clock_slope_reject_count =
        clock_diagnostics.slope_reject_count;
    diagnostics->clock_residual_reject_count =
        clock_diagnostics.residual_reject_count;
    diagnostics->clock_reference_mcu_us =
        (clock_diagnostics.last_reference_mcu_us > 0.0)
            ? (uint64_t)(clock_diagnostics.last_reference_mcu_us + 0.5)
            : 0U;
    diagnostics->clock_scale = (float)clock_diagnostics.clock_scale;
    diagnostics->clock_last_observed_scale =
        (float)clock_diagnostics.last_observed_clock_scale;
    diagnostics->clock_last_residual_us =
        (float)clock_diagnostics.last_residual_us;
    diagnostics->clock_residual_sigma_us =
        (float)clock_diagnostics.residual_sigma_us;
    diagnostics->clock_sync_valid = clock_diagnostics.valid;
}

void icm45686_get_health(imu_health_t *health)
{
    if (health != NULL)
    {
        *health = s_health;
    }
}

void icm45686_get_configuration(icm45686_configuration_t *configuration)
{
    if (configuration != NULL)
    {
        *configuration = s_configuration;
    }
}

uint8_t icm45686_who_am_i(void)
{
    return s_who_am_i;
}
