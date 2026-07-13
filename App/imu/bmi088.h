#ifndef BMI088_H
#define BMI088_H

#include "imu_types.h"

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

typedef struct
{
    uint16_t accel_odr_hz;
    /* Nominal sensor bandwidth; verify the full installed response by sweep/PSD. */
    uint16_t accel_bandwidth_hz;
    uint16_t accel_range_g;
    uint16_t gyro_odr_hz;
    uint16_t gyro_bandwidth_hz;
    uint16_t gyro_range_dps;
    bool register_readback_verified;
} bmi088_configuration_t;

#ifdef __cplusplus
extern "C" {
#endif

bool bmi088_init(void);
bool bmi088_check_configuration(void);
bool bmi088_check_accel_configuration(void);
bool bmi088_check_gyro_configuration(void);
bool bmi088_read_accel(uint64_t timestamp_us, imu_accel_sample_t *sample);
bool bmi088_read_gyro(uint64_t timestamp_us, imu_gyro_sample_t *sample);
bool bmi088_read(uint64_t timestamp_us, imu_sample_t *sample);
void bmi088_get_health(imu_health_t *health);
void bmi088_get_configuration(bmi088_configuration_t *configuration);
uint8_t bmi088_accel_chip_id(void);
uint8_t bmi088_gyro_chip_id(void);

#ifdef __cplusplus
}
#endif

#endif
