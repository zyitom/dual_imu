#ifndef IMU_TYPES_H
#define IMU_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    IMU_SOURCE_BMI088 = 0,
    IMU_SOURCE_ICM45686 = 1,
    IMU_SOURCE_COUNT,
    IMU_SOURCE_FUSED = IMU_SOURCE_COUNT
} imu_source_t;

typedef enum
{
    IMU_STATUS_OK = 0,
    IMU_STATUS_NOT_INITIALIZED,
    IMU_STATUS_BAD_ID,
    IMU_STATUS_BUS_ERROR,
    IMU_STATUS_CONFIG_ERROR,
    IMU_STATUS_STALE
} imu_status_t;

typedef struct
{
    uint64_t timestamp_us;
    uint64_t accel_timestamp_us;
    uint64_t gyro_timestamp_us;
    uint32_t sequence;
    uint32_t accel_sequence;
    uint32_t gyro_sequence;
    float accel_mps2[3];
    float gyro_rad_s[3];
    float temperature_c;
    imu_source_t source;
    bool accel_valid;
    bool gyro_valid;
    bool valid;
} imu_sample_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;
    float accel_mps2[3];
    float temperature_c;
    imu_source_t source;
    bool valid;
} imu_accel_sample_t;

typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;
    float gyro_rad_s[3];
    float temperature_c;
    imu_source_t source;
    bool valid;
} imu_gyro_sample_t;

typedef struct
{
    imu_status_t status;
    uint32_t read_count;
    uint32_t bus_error_count;
    uint32_t missed_interrupt_count;
    uint64_t last_sample_us;
} imu_health_t;

#endif
