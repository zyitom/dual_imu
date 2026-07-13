#ifndef IMU_STREAM_BUFFER_H
#define IMU_STREAM_BUFFER_H

#include "imu_types.h"

#include <stdbool.h>
#include <stdint.h>

#define IMU_TIMESTAMP_QUEUE_CAPACITY 32U
#define IMU_SAMPLE_BUFFER_CAPACITY   64U

typedef struct
{
    uint64_t entries[IMU_TIMESTAMP_QUEUE_CAPACITY];
    uint32_t sequences[IMU_TIMESTAMP_QUEUE_CAPACITY];
    volatile uint32_t write_index;
    volatile uint32_t read_index;
    volatile uint32_t dropped_count;
} imu_timestamp_queue_t;

typedef struct
{
    imu_accel_sample_t entries[IMU_SAMPLE_BUFFER_CAPACITY];
    volatile uint32_t write_index;
    volatile uint32_t read_index;
    volatile uint32_t overwritten_count;
} imu_accel_buffer_t;

typedef struct
{
    imu_gyro_sample_t entries[IMU_SAMPLE_BUFFER_CAPACITY];
    volatile uint32_t write_index;
    volatile uint32_t read_index;
    volatile uint32_t overwritten_count;
} imu_gyro_buffer_t;

void imu_timestamp_queue_reset(imu_timestamp_queue_t *queue);
bool imu_timestamp_queue_push_isr(imu_timestamp_queue_t *queue, uint64_t timestamp_us);
bool imu_timestamp_queue_push_event_isr(imu_timestamp_queue_t *queue,
                                        uint64_t timestamp_us,
                                        uint32_t sequence);
bool imu_timestamp_queue_peek(const imu_timestamp_queue_t *queue, uint64_t *timestamp_us);
bool imu_timestamp_queue_pop(imu_timestamp_queue_t *queue, uint64_t *timestamp_us);
bool imu_timestamp_queue_pop_event(imu_timestamp_queue_t *queue,
                                   uint64_t *timestamp_us,
                                   uint32_t *sequence);
uint32_t imu_timestamp_queue_dropped(const imu_timestamp_queue_t *queue);

void imu_accel_buffer_reset(imu_accel_buffer_t *buffer);
void imu_accel_buffer_push(imu_accel_buffer_t *buffer, const imu_accel_sample_t *sample);
bool imu_accel_buffer_pop(imu_accel_buffer_t *buffer, imu_accel_sample_t *sample);
uint32_t imu_accel_buffer_overwritten(const imu_accel_buffer_t *buffer);

void imu_gyro_buffer_reset(imu_gyro_buffer_t *buffer);
void imu_gyro_buffer_push(imu_gyro_buffer_t *buffer, const imu_gyro_sample_t *sample);
bool imu_gyro_buffer_pop(imu_gyro_buffer_t *buffer, imu_gyro_sample_t *sample);
uint32_t imu_gyro_buffer_overwritten(const imu_gyro_buffer_t *buffer);

#endif
