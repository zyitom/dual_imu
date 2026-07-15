#include "bmi088_fifo_guard.h"

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

static void test_capture_epoch_rejects_fault_or_discard(void)
{
    TEST_EXPECT(bmi088_fifo_capture_epoch_valid(4U, 4U, false));
    TEST_EXPECT(!bmi088_fifo_capture_epoch_valid(4U, 5U, false));
    TEST_EXPECT(!bmi088_fifo_capture_epoch_valid(4U, 4U, true));
}

static void test_only_started_fifo_data_failure_is_ambiguous(void)
{
    TEST_EXPECT(!bmi088_fifo_gyro_failure_is_ambiguous(false));
    TEST_EXPECT(bmi088_fifo_gyro_failure_is_ambiguous(true));
}

static void test_recovery_quiesces_new_normal_fifo_transactions(void)
{
    TEST_EXPECT(bmi088_fifo_normal_service_allowed(false, false));
    TEST_EXPECT(!bmi088_fifo_normal_service_allowed(true, false));
    TEST_EXPECT(!bmi088_fifo_normal_service_allowed(false, true));
    TEST_EXPECT(!bmi088_fifo_normal_service_allowed(true, true));
}

static void test_strict_gyro_boundary_requires_empty_non_overrun_status(void)
{
    TEST_EXPECT(bmi088_fifo_gyro_boundary_status_valid(0x00U));
    TEST_EXPECT(!bmi088_fifo_gyro_boundary_status_valid(0x01U));
    TEST_EXPECT(!bmi088_fifo_gyro_boundary_status_valid(0x7FU));
    TEST_EXPECT(!bmi088_fifo_gyro_boundary_status_valid(0x80U));
    TEST_EXPECT(!bmi088_fifo_gyro_boundary_status_valid(0xFFU));
}

static void test_gyro_recovery_always_creates_a_sequence_gap(void)
{
    TEST_EXPECT(bmi088_fifo_gyro_recovery_gap_count(0U, false) == 1U);
    TEST_EXPECT(bmi088_fifo_gyro_recovery_gap_count(7U, false) == 7U);
    TEST_EXPECT(bmi088_fifo_gyro_recovery_gap_count(0U, true) == 1U);
    TEST_EXPECT(bmi088_fifo_gyro_recovery_gap_count(127U, true) == 128U);
}

static void test_gyro_runtime_configuration_requires_every_owned_field(void)
{
    TEST_EXPECT(bmi088_fifo_gyro_core_config_valid(
        0x00U, 0x01U, 0x00U, 0x00U, 0x01U, 0x00U));
    TEST_EXPECT(!bmi088_fifo_gyro_core_config_valid(
        0x01U, 0x01U, 0x00U, 0x00U, 0x01U, 0x00U));
    TEST_EXPECT(!bmi088_fifo_gyro_core_config_valid(
        0x00U, 0x02U, 0x00U, 0x00U, 0x01U, 0x00U));
    TEST_EXPECT(!bmi088_fifo_gyro_core_config_valid(
        0x00U, 0x01U, 0x80U, 0x00U, 0x01U, 0x00U));

    TEST_EXPECT(bmi088_fifo_gyro_interrupt_config_valid(0x80U, 0x01U,
                                                        0x01U));
    TEST_EXPECT(!bmi088_fifo_gyro_interrupt_config_valid(0x00U, 0x01U,
                                                         0x01U));
    TEST_EXPECT(!bmi088_fifo_gyro_interrupt_config_valid(0xC0U, 0x01U,
                                                         0x01U));
    TEST_EXPECT(!bmi088_fifo_gyro_interrupt_config_valid(0x80U, 0x00U,
                                                         0x01U));
    TEST_EXPECT(!bmi088_fifo_gyro_interrupt_config_valid(0x80U, 0x03U,
                                                         0x01U));
    TEST_EXPECT(!bmi088_fifo_gyro_interrupt_config_valid(0x80U, 0x01U,
                                                         0x81U));
    TEST_EXPECT(!bmi088_fifo_gyro_interrupt_config_valid(0x80U, 0x01U,
                                                         0x05U));
}

static void test_accel_missing_frames_create_sequence_gap(void)
{
    TEST_EXPECT(bmi088_fifo_accel_missing_frame_count(0U, false) == 0U);
    TEST_EXPECT(bmi088_fifo_accel_missing_frame_count(2U, false) == 2U);
    TEST_EXPECT(bmi088_fifo_accel_missing_frame_count(2U, true) == 3U);

    TEST_EXPECT(bmi088_fifo_accel_invalid_frame_count(
                    true, 4U, 3U, 2U, false) == 6U);
    TEST_EXPECT(bmi088_fifo_accel_invalid_frame_count(
                    false, 4U, 3U, 2U, true) == 6U);
    TEST_EXPECT(bmi088_fifo_accel_invalid_frame_count(
                    false, 0U, 0U, 0U, false) == 1U);
}

int main(void)
{
    test_capture_epoch_rejects_fault_or_discard();
    test_only_started_fifo_data_failure_is_ambiguous();
    test_recovery_quiesces_new_normal_fifo_transactions();
    test_strict_gyro_boundary_requires_empty_non_overrun_status();
    test_gyro_recovery_always_creates_a_sequence_gap();
    test_gyro_runtime_configuration_requires_every_owned_field();
    test_accel_missing_frames_create_sequence_gap();
    if (failures != 0U)
        return EXIT_FAILURE;
    (void)puts("bmi088_fifo_guard: all tests passed");
    return EXIT_SUCCESS;
}
