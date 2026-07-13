#include "bmi088.h"

#include "bmi08x.h"
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
static bool bmi088_verify_configuration(void);

bool bmi088_init(void)
{
    struct bmi08_accel_int_channel_cfg accel_int_config = {
        .int_channel = BMI08_INT_CHANNEL_1,
        .int_type = BMI08_ACCEL_INT_DATA_RDY,
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

    memset(&bmi088_device, 0, sizeof(bmi088_device));
    memset(&bmi088_health, 0, sizeof(bmi088_health));
    bmi088_health.status = IMU_STATUS_NOT_INITIALIZED;
    bmi088_initialized = false;
    bmi088_sequence = 0;
    bmi088_accel_sequence = 0U;
    bmi088_gyro_sequence = 0U;
    bmi088_temperature_countdown = 0U;
    bmi088_temperature_c = 0.0f;
    bmi088_configuration = (bmi088_configuration_t) {
        .accel_odr_hz = BMI088_CONFIG_ACCEL_ODR_HZ,
        .accel_bandwidth_hz = BMI088_CONFIG_ACCEL_BANDWIDTH_HZ,
        .accel_range_g = BMI088_CONFIG_ACCEL_RANGE_G,
        .gyro_odr_hz = BMI088_CONFIG_GYRO_ODR_HZ,
        .gyro_bandwidth_hz = BMI088_CONFIG_GYRO_BANDWIDTH_HZ,
        .gyro_range_dps = BMI088_CONFIG_GYRO_RANGE_DPS,
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

    if (!bmi088_verify_configuration())
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

    bmi088_initialized = true;
    bmi088_health.status = IMU_STATUS_OK;

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
    struct bmi08_sensor_data accel_raw;

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
        !bmi088_verify_gyro_configuration())
        return false;

    bmi088_configuration.register_readback_verified = true;
    return true;
}
