#include "imu_stream_buffer.h"

#include "main.h"

#include <string.h>

#define TIMESTAMP_QUEUE_MASK (IMU_TIMESTAMP_QUEUE_CAPACITY - 1U)
#define SAMPLE_BUFFER_MASK    (IMU_SAMPLE_BUFFER_CAPACITY - 1U)

_Static_assert((IMU_TIMESTAMP_QUEUE_CAPACITY & TIMESTAMP_QUEUE_MASK) == 0U,
               "timestamp queue capacity must be a power of two");
_Static_assert((IMU_SAMPLE_BUFFER_CAPACITY & SAMPLE_BUFFER_MASK) == 0U,
               "sample buffer capacity must be a power of two");

void imu_timestamp_queue_reset(imu_timestamp_queue_t *queue)
{
    if (queue != NULL)
    {
        memset(queue, 0, sizeof(*queue));
    }
}

bool imu_timestamp_queue_push_isr(imu_timestamp_queue_t *queue, uint64_t timestamp_us)
{
    return imu_timestamp_queue_push_event_isr(queue, timestamp_us, 0U);
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

bool imu_timestamp_queue_pop(imu_timestamp_queue_t *queue, uint64_t *timestamp_us)
{
    uint32_t sequence;
    return imu_timestamp_queue_pop_event(queue, timestamp_us, &sequence);
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

bool imu_timestamp_queue_peek(const imu_timestamp_queue_t *queue, uint64_t *timestamp_us)
{
    if ((queue == NULL) || (timestamp_us == NULL))
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
    return true;
}

uint32_t imu_timestamp_queue_dropped(const imu_timestamp_queue_t *queue)
{
    return (queue != NULL) ? queue->dropped_count : 0U;
}

void imu_accel_buffer_reset(imu_accel_buffer_t *buffer)
{
    if (buffer != NULL)
    {
        memset(buffer, 0, sizeof(*buffer));
    }
}

void imu_accel_buffer_push(imu_accel_buffer_t *buffer, const imu_accel_sample_t *sample)
{
    if ((buffer == NULL) || (sample == NULL))
    {
        return;
    }

    const uint32_t write = buffer->write_index;
    const uint32_t next = (write + 1U) & SAMPLE_BUFFER_MASK;
    if (next == buffer->read_index)
    {
        buffer->read_index = (buffer->read_index + 1U) & SAMPLE_BUFFER_MASK;
        buffer->overwritten_count++;
    }

    buffer->entries[write] = *sample;
    __DMB();
    buffer->write_index = next;
}

bool imu_accel_buffer_pop(imu_accel_buffer_t *buffer, imu_accel_sample_t *sample)
{
    if ((buffer == NULL) || (sample == NULL))
    {
        return false;
    }

    const uint32_t read = buffer->read_index;
    if (read == buffer->write_index)
    {
        return false;
    }

    __DMB();
    *sample = buffer->entries[read];
    buffer->read_index = (read + 1U) & SAMPLE_BUFFER_MASK;
    return true;
}

uint32_t imu_accel_buffer_overwritten(const imu_accel_buffer_t *buffer)
{
    return (buffer != NULL) ? buffer->overwritten_count : 0U;
}

void imu_gyro_buffer_reset(imu_gyro_buffer_t *buffer)
{
    if (buffer != NULL)
    {
        memset(buffer, 0, sizeof(*buffer));
    }
}

void imu_gyro_buffer_push(imu_gyro_buffer_t *buffer, const imu_gyro_sample_t *sample)
{
    if ((buffer == NULL) || (sample == NULL))
    {
        return;
    }

    const uint32_t write = buffer->write_index;
    const uint32_t next = (write + 1U) & SAMPLE_BUFFER_MASK;
    if (next == buffer->read_index)
    {
        buffer->read_index = (buffer->read_index + 1U) & SAMPLE_BUFFER_MASK;
        buffer->overwritten_count++;
    }

    buffer->entries[write] = *sample;
    __DMB();
    buffer->write_index = next;
}

bool imu_gyro_buffer_pop(imu_gyro_buffer_t *buffer, imu_gyro_sample_t *sample)
{
    if ((buffer == NULL) || (sample == NULL))
    {
        return false;
    }

    const uint32_t read = buffer->read_index;
    if (read == buffer->write_index)
    {
        return false;
    }

    __DMB();
    *sample = buffer->entries[read];
    buffer->read_index = (read + 1U) & SAMPLE_BUFFER_MASK;
    return true;
}

uint32_t imu_gyro_buffer_overwritten(const imu_gyro_buffer_t *buffer)
{
    return (buffer != NULL) ? buffer->overwritten_count : 0U;
}
