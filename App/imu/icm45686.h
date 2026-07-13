#ifndef ICM45686_H
#define ICM45686_H

#include "imu_types.h"

#include <stdbool.h>
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
#define ICM45686_CONFIG_GYRO_ODR_HZ           UINT16_C(1600)
#define ICM45686_CONFIG_GYRO_BANDWIDTH_HZ     UINT16_C(200)
#else
#define ICM45686_CONFIG_ACCEL_ODR_HZ          UINT16_C(400)
#define ICM45686_CONFIG_ACCEL_BANDWIDTH_HZ    UINT16_C(50)
#define ICM45686_CONFIG_GYRO_ODR_HZ           UINT16_C(400)
#define ICM45686_CONFIG_GYRO_BANDWIDTH_HZ     UINT16_C(50)
#endif

#define ICM45686_CONFIG_ACCEL_RANGE_G         UINT16_C(32)
#define ICM45686_CONFIG_GYRO_RANGE_DPS        UINT16_C(2000)
#define ICM45686_CONFIG_FIFO_ENABLED          0

typedef struct
{
    uint16_t accel_odr_hz;
    /* UI LPF setting; FIR AAF and mechanics can further shape the response. */
    uint16_t accel_bandwidth_hz;
    uint16_t accel_range_g;
    uint16_t gyro_odr_hz;
    uint16_t gyro_bandwidth_hz;
    uint16_t gyro_range_dps;
    bool fifo_enabled;
    bool register_readback_verified;
} icm45686_configuration_t;

#ifdef __cplusplus
extern "C" {
#endif

bool icm45686_init(void);
bool icm45686_check_configuration(void);
bool icm45686_check_accel_configuration(void);
bool icm45686_check_gyro_configuration(void);
bool icm45686_read(uint64_t timestamp_us, imu_sample_t *sample);
void icm45686_get_health(imu_health_t *health);
void icm45686_get_configuration(icm45686_configuration_t *configuration);
uint8_t icm45686_who_am_i(void);

#ifdef __cplusplus
}
#endif

#endif
