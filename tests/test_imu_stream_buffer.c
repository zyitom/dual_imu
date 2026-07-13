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

int main(void)
{
    test_event_sequence_survives_queue();
    test_overflow_preserves_future_gap_sequence();
    if (failures != 0U)
        return EXIT_FAILURE;
    (void)puts("imu_stream_buffer: all tests passed");
    return EXIT_SUCCESS;
}
