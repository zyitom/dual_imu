#ifndef IMU_TIME_H
#define IMU_TIME_H

#include <stdbool.h>
#include <stdint.h>

bool imu_time_init(void);
bool imu_time_is_running(void);
bool imu_time_start_capture_channels_1_2(void);
bool imu_time_capture_is_running(void);
uint32_t imu_time_capture_overrun_count(uint32_t channel);
uint64_t imu_time_now_us(void);
void imu_time_delay_us(uint32_t delay_us);

/* Strong implementation is provided by the IMU manager. Channel is 1 or 2. */
void imu_time_capture_callback(uint32_t channel, uint64_t timestamp_us);

#endif
