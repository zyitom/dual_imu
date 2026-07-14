#ifndef IMU_TIME_H
#define IMU_TIME_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool running;
    uint32_t period_us;
    uint32_t pending_count;
    uint32_t compare_event_count;
    uint32_t missed_compare_count;
    uint32_t dropped_tick_count;
    uint64_t next_scheduled_us;
} imu_time_fast_tick_diagnostics_t;

bool imu_time_init(void);
bool imu_time_is_running(void);
bool imu_time_start_capture_channels_1_2(void);
bool imu_time_capture_is_running(void);
bool imu_time_capture_reset_channel(uint32_t channel);
uint32_t imu_time_capture_overrun_count(uint32_t channel);
uint64_t imu_time_now_us(void);
void imu_time_delay_us(uint32_t delay_us);

/* Starts a phase-stable TIM5 CH3 grid. Publication runs at the exact scheduled
 * epoch in the compare ISR; the following compare is armed first. */
bool imu_time_fast_tick_start(uint32_t period_us);
bool imu_time_fast_tick_is_running(void);
void imu_time_fast_tick_get_diagnostics(
    imu_time_fast_tick_diagnostics_t *diagnostics);
void imu_time_fast_tick_reset_diagnostics(void);

/* Strong implementation is provided by the IMU manager. Channel is 1 or 2. */
void imu_time_capture_callback(uint32_t channel, uint64_t timestamp_us);
/* Runs from TIM5 CH3 ISR after the following compare has already been armed. */
void imu_time_fast_tick_callback(uint64_t scheduled_us);

#endif
