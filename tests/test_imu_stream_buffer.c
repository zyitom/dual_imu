#include "imu_stream_buffer.h"

#include <stdio.h>
#include <stdlib.h>

static unsigned int failures;

#define TEST_EXPECT(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            (void)fprintf(stderr, "%s:%d: failed: %s\n",                       \
                          __FILE__, __LINE__, #condition);                        \
            failures++;                                                           \
        }                                                                         \
    } while (0)

static void test_event_sequence_survives_queue(void)
{
    imu_timestamp_queue_t queue;
    uint64_t timestamp;
    uint32_t sequence;
    imu_timestamp_queue_reset(&queue);

    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 100U, 7U));
    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 200U, 9U));
    TEST_EXPECT(imu_timestamp_queue_pop_event(&queue, &timestamp, &sequence));
    TEST_EXPECT(timestamp == 100U);
    TEST_EXPECT(sequence == 7U);
    TEST_EXPECT(imu_timestamp_queue_pop_event(&queue, &timestamp, &sequence));
    TEST_EXPECT(timestamp == 200U);
    TEST_EXPECT(sequence == 9U);
}

static void test_overflow_preserves_future_gap_sequence(void)
{
    imu_timestamp_queue_t queue;
    uint64_t timestamp;
    uint32_t sequence;
    imu_timestamp_queue_reset(&queue);

    for (uint32_t index = 1U; index < IMU_TIMESTAMP_QUEUE_CAPACITY; ++index)
        TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, index, index));
    TEST_EXPECT(!imu_timestamp_queue_push_event_isr(
        &queue, IMU_TIMESTAMP_QUEUE_CAPACITY, IMU_TIMESTAMP_QUEUE_CAPACITY));
    TEST_EXPECT(imu_timestamp_queue_dropped(&queue) == 1U);

    while (imu_timestamp_queue_pop_event(&queue, &timestamp, &sequence)) {
    }
    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 1000U, 33U));
    TEST_EXPECT(imu_timestamp_queue_pop_event(&queue, &timestamp, &sequence));
    TEST_EXPECT(sequence == 33U);
}

static void test_batch_pop_preserves_fifo_capture_ownership(void)
{
    imu_timestamp_queue_t queue;
    uint64_t timestamps[2] = {0U, 0U};
    uint32_t sequences[2] = {0U, 0U};
    imu_timestamp_queue_reset(&queue);

    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 1000U, 10U));
    const uint32_t fifo_frame_count = 1U;
    TEST_EXPECT(imu_timestamp_queue_count(&queue) == fifo_frame_count);
    TEST_EXPECT(!imu_timestamp_queue_pop_batch(
        &queue, timestamps, sequences, fifo_frame_count + 1U));
    TEST_EXPECT(imu_timestamp_queue_count(&queue) == fifo_frame_count);

    /* A frame/capture arriving during FIFO_DATA remains newer than the N
     * oldest frames selected by FIFO_STATUS. */
    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 1500U, 11U));
    TEST_EXPECT(imu_timestamp_queue_pop_batch(
        &queue, timestamps, sequences, fifo_frame_count));
    TEST_EXPECT(timestamps[0] == 1000U);
    TEST_EXPECT(sequences[0] == 10U);
    TEST_EXPECT(imu_timestamp_queue_count(&queue) == 1U);
    TEST_EXPECT(imu_timestamp_queue_pop_batch(&queue, timestamps, sequences, 1U));
    TEST_EXPECT(timestamps[0] == 1500U);
    TEST_EXPECT(sequences[0] == 11U);

    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 2000U, 12U));
    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 2500U, 13U));
    TEST_EXPECT(imu_timestamp_queue_pop_batch(&queue, timestamps, sequences, 1U));
    TEST_EXPECT(timestamps[0] == 2000U);
    TEST_EXPECT(imu_timestamp_queue_count(&queue) == 1U);
    TEST_EXPECT(imu_timestamp_queue_pop_batch(&queue, timestamps, sequences, 1U));
    TEST_EXPECT(timestamps[0] == 2500U);
}

static void test_batch_pop_across_ring_wrap(void)
{
    imu_timestamp_queue_t queue;
    uint64_t timestamp;
    uint32_t sequence;
    uint64_t timestamps[40];
    uint32_t sequences[40];
    imu_timestamp_queue_reset(&queue);

    for (uint32_t index = 0U; index < 100U; ++index)
        TEST_EXPECT(imu_timestamp_queue_push_event_isr(
            &queue, 1000U + index, index + 1U));
    for (uint32_t index = 0U; index < 100U; ++index)
        TEST_EXPECT(imu_timestamp_queue_pop_event(
            &queue, &timestamp, &sequence));

    for (uint32_t index = 0U; index < 40U; ++index)
        TEST_EXPECT(imu_timestamp_queue_push_event_isr(
            &queue, 5000U + index, 200U + index));
    TEST_EXPECT(imu_timestamp_queue_pop_batch(
        &queue, timestamps, sequences, 40U));
    for (uint32_t index = 0U; index < 40U; ++index)
    {
        TEST_EXPECT(timestamps[index] == 5000U + index);
        TEST_EXPECT(sequences[index] == 200U + index);
    }
    TEST_EXPECT(imu_timestamp_queue_count(&queue) == 0U);
}

static void test_discard_invalidates_an_inflight_snapshot(void)
{
    imu_timestamp_queue_t queue;
    uint64_t timestamp;
    uint32_t sequence;
    imu_timestamp_queue_reset(&queue);

    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 1000U, 1U));
    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 1500U, 2U));
    const uint32_t snapshot_generation =
        imu_timestamp_queue_discard_generation(&queue);

    imu_timestamp_queue_discard(&queue);
    TEST_EXPECT(imu_timestamp_queue_count(&queue) == 0U);
    TEST_EXPECT(imu_timestamp_queue_discard_generation(&queue) ==
                snapshot_generation + 1U);
    TEST_EXPECT(!imu_timestamp_queue_pop_event(&queue, &timestamp, &sequence));

    TEST_EXPECT(imu_timestamp_queue_push_event_isr(&queue, 2000U, 3U));
    TEST_EXPECT(imu_timestamp_queue_pop_event(&queue, &timestamp, &sequence));
    TEST_EXPECT(timestamp == 2000U);
    TEST_EXPECT(sequence == 3U);
    TEST_EXPECT(imu_timestamp_queue_discard_generation(&queue) ==
                snapshot_generation + 1U);
}

int main(void)
{
    test_event_sequence_survives_queue();
    test_overflow_preserves_future_gap_sequence();
    test_batch_pop_preserves_fifo_capture_ownership();
    test_batch_pop_across_ring_wrap();
    test_discard_invalidates_an_inflight_snapshot();
    if (failures != 0U)
        return EXIT_FAILURE;
    (void)puts("imu_stream_buffer: all tests passed");
    return EXIT_SUCCESS;
}
