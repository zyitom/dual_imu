#ifndef IMU_STREAM_BUFFER_H
#define IMU_STREAM_BUFFER_H

#include "imu_types.h"

#include <stdbool.h>
#include <stdint.h>

#define IMU_TIMESTAMP_QUEUE_CAPACITY 128U

typedef struct
{
    uint64_t entries[IMU_TIMESTAMP_QUEUE_CAPACITY];
    uint32_t sequences[IMU_TIMESTAMP_QUEUE_CAPACITY];
    volatile uint32_t write_index;
    volatile uint32_t read_index;
    volatile uint32_t dropped_count;
    volatile uint32_t discard_generation;
} imu_timestamp_queue_t;

void imu_timestamp_queue_reset(imu_timestamp_queue_t *queue);
void imu_timestamp_queue_discard(imu_timestamp_queue_t *queue);
bool imu_timestamp_queue_push_event_isr(imu_timestamp_queue_t *queue,
                                        uint64_t timestamp_us,
                                        uint32_t sequence);
bool imu_timestamp_queue_pop_event(imu_timestamp_queue_t *queue,
                                   uint64_t *timestamp_us,
                                   uint32_t *sequence);
uint32_t imu_timestamp_queue_count(const imu_timestamp_queue_t *queue);
uint32_t imu_timestamp_queue_discard_generation(
    const imu_timestamp_queue_t *queue);
bool imu_timestamp_queue_pop_batch(imu_timestamp_queue_t *queue,
                                   uint64_t *timestamps_us,
                                   uint32_t *sequences,
                                   uint32_t count);
uint32_t imu_timestamp_queue_dropped(const imu_timestamp_queue_t *queue);

#endif
