#include "imu_stream_buffer.h"

#include "main.h"

#include <string.h>

#define TIMESTAMP_QUEUE_MASK (IMU_TIMESTAMP_QUEUE_CAPACITY - 1U)

_Static_assert((IMU_TIMESTAMP_QUEUE_CAPACITY & TIMESTAMP_QUEUE_MASK) == 0U,
               "timestamp queue capacity must be a power of two");

void imu_timestamp_queue_reset(imu_timestamp_queue_t *queue)
{
    if (queue != NULL)
    {
        memset(queue, 0, sizeof(*queue));
    }
}

void imu_timestamp_queue_discard(imu_timestamp_queue_t *queue)
{
    if (queue == NULL)
        return;

    queue->read_index = queue->write_index;
    __DMB();
    queue->discard_generation++;
    __DMB();
}

bool imu_timestamp_queue_push_event_isr(imu_timestamp_queue_t *queue,
                                        uint64_t timestamp_us,
                                        uint32_t sequence)
{
    if (queue == NULL)
    {
        return false;
    }

    const uint32_t write = queue->write_index;
    const uint32_t next = (write + 1U) & TIMESTAMP_QUEUE_MASK;
    if (next == queue->read_index)
    {
        queue->dropped_count++;
        return false;
    }

    queue->entries[write] = timestamp_us;
    queue->sequences[write] = sequence;
    __DMB();
    queue->write_index = next;
    return true;
}

bool imu_timestamp_queue_pop_event(imu_timestamp_queue_t *queue,
                                   uint64_t *timestamp_us,
                                   uint32_t *sequence)
{
    if ((queue == NULL) || (timestamp_us == NULL) || (sequence == NULL))
    {
        return false;
    }

    const uint32_t read = queue->read_index;
    if (read == queue->write_index)
    {
        return false;
    }

    __DMB();
    *timestamp_us = queue->entries[read];
    *sequence = queue->sequences[read];
    queue->read_index = (read + 1U) & TIMESTAMP_QUEUE_MASK;
    return true;
}

uint32_t imu_timestamp_queue_count(const imu_timestamp_queue_t *queue)
{
    if (queue == NULL)
        return 0U;

    return (queue->write_index - queue->read_index) & TIMESTAMP_QUEUE_MASK;
}

uint32_t imu_timestamp_queue_discard_generation(
    const imu_timestamp_queue_t *queue)
{
    return (queue != NULL) ? queue->discard_generation : 0U;
}

bool imu_timestamp_queue_pop_batch(imu_timestamp_queue_t *queue,
                                   uint64_t *timestamps_us,
                                   uint32_t *sequences,
                                   uint32_t count)
{
    if ((queue == NULL) || (timestamps_us == NULL) || (sequences == NULL) ||
        (count == 0U) || (imu_timestamp_queue_count(queue) < count))
        return false;

    uint32_t read = queue->read_index;
    __DMB();
    for (uint32_t index = 0U; index < count; ++index)
    {
        timestamps_us[index] = queue->entries[read];
        sequences[index] = queue->sequences[read];
        read = (read + 1U) & TIMESTAMP_QUEUE_MASK;
    }
    queue->read_index = read;
    return true;
}

uint32_t imu_timestamp_queue_dropped(const imu_timestamp_queue_t *queue)
{
    return (queue != NULL) ? queue->dropped_count : 0U;
}
