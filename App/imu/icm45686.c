#include "icm45686.h"

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
#define ICM45686_ACCEL_SCALE_MPS2   ((32.0f * 9.80665f) / 32768.0f)
#define ICM45686_GYRO_SCALE_RAD_S   ((2000.0f * 0.017453292519943295f) / 32768.0f)
#define ICM45686_TEMPERATURE_SCALE  (1.0f / 128.0f)
#define ICM45686_TEMPERATURE_OFFSET 25.0f
#define ICM45686_SPI_MAX_TRANSFER   256U

#if ICM45686_USE_ROBUST_HIGH_RATE_PROFILE
#define ICM45686_ACCEL_ODR_SETTING  ACCEL_CONFIG0_ACCEL_ODR_1600_HZ
#define ICM45686_ACCEL_BW_SETTING   IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_16
#define ICM45686_GYRO_ODR_SETTING   GYRO_CONFIG0_GYRO_ODR_1600_HZ
#define ICM45686_GYRO_BW_SETTING    IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8
#else
#define ICM45686_ACCEL_ODR_SETTING  ACCEL_CONFIG0_ACCEL_ODR_400_HZ
#define ICM45686_ACCEL_BW_SETTING   IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8
#define ICM45686_GYRO_ODR_SETTING   GYRO_CONFIG0_GYRO_ODR_400_HZ
#define ICM45686_GYRO_BW_SETTING    IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8
#endif

#define ICM45686_ACCEL_FSR_SETTING  ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G
#define ICM45686_GYRO_FSR_SETTING   GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS

typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} icm45686_bus_t;

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
        (length > ICM45686_SPI_MAX_TRANSFER))
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
        (length > ICM45686_SPI_MAX_TRANSFER))
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

static int icm45686_verify_shared_configuration(void)
{
    int1_config0_t interrupt_source = {0};
    int1_config2_t interrupt_pin = {0};
    inv_imu_fifo_config_t fifo = {0};
    int status = INV_IMU_OK;

    status |= inv_imu_read_reg(&s_device, INT1_CONFIG0, 1U,
                               (uint8_t *)&interrupt_source);
    status |= inv_imu_read_reg(&s_device, INT1_CONFIG2, 1U,
                               (uint8_t *)&interrupt_pin);
    status |= inv_imu_get_fifo_config(&s_device, &fifo);
    if (status != INV_IMU_OK)
        return status;

    if ((interrupt_source.int1_status_en_drdy != INV_IMU_ENABLE) ||
        (interrupt_pin.int1_polarity != INTX_CONFIG2_INTX_POLARITY_HIGH) ||
        (interrupt_pin.int1_mode != INTX_CONFIG2_INTX_MODE_PULSE) ||
        (interrupt_pin.int1_drive != INTX_CONFIG2_INTX_DRIVE_PP) ||
        (fifo.fifo_mode != FIFO_CONFIG0_FIFO_MODE_BYPASS) ||
        (fifo.accel_en != INV_IMU_DISABLE) ||
        (fifo.gyro_en != INV_IMU_DISABLE) ||
        (fifo.hires_en != INV_IMU_DISABLE))
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
    intx_configx_t interrupt_sources = {0};
    const inv_imu_fifo_config_t fifo_config = {
        .gyro_en = INV_IMU_DISABLE,
        .accel_en = INV_IMU_DISABLE,
        .hires_en = INV_IMU_DISABLE,
        .fifo_wm_th = 0U,
        .fifo_mode = FIFO_CONFIG0_FIFO_MODE_BYPASS,
        .fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_MAX,
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

    /* The active read path is DRDY plus register burst, so keep FIFO explicitly bypassed. */
    status = inv_imu_set_fifo_config(&s_device, &fifo_config);
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

    status = inv_imu_set_pin_config_int(&s_device, INV_IMU_INT1, &interrupt_pin);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    /* Write both config bytes from zero so AUX1 and reserved sources stay disabled. */
    interrupt_sources.int1_config0.int1_status_en_drdy = INV_IMU_ENABLE;
    status = inv_imu_write_reg(&s_device, INT1_CONFIG0, sizeof(interrupt_sources),
                               (const uint8_t *)&interrupt_sources);
    if (status != INV_IMU_OK)
    {
        return status;
    }

    return icm45686_verify_configuration();
}

bool icm45686_init(void)
{
    int status;

    memset(&s_device, 0, sizeof(s_device));
    memset(&s_health, 0, sizeof(s_health));
    s_health.status = IMU_STATUS_NOT_INITIALIZED;
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
        .fifo_enabled = false,
        .register_readback_verified = false,
    };

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
