#include "imu_causal_accel_history.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned int failure_count;

#define TEST_EXPECT(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",           \
                          __FILE__, __LINE__, #condition);                        \
            failure_count++;                                                      \
        }                                                                         \
    } while (0)

static imu_accel_sample_t make_sample(imu_source_t source,
                                      uint64_t timestamp_us,
                                      float value)
{
    const imu_accel_sample_t sample = {
        .timestamp_us = timestamp_us,
        .accel_mps2 = {value, value + 1.0f, value + 2.0f},
        .source = source,
        .valid = true,
    };
    return sample;
}

static void test_latest_causal_ignores_future_and_accepts_out_of_order(void)
{
    imu_causal_accel_history_t history;
    TEST_EXPECT(imu_causal_accel_history_init(
        &history, IMU_SOURCE_BMI088, 1000U, 100U));

    imu_accel_sample_t future =
        make_sample(IMU_SOURCE_BMI088, UINT64_C(200), 2.0f);
    imu_accel_sample_t causal =
        make_sample(IMU_SOURCE_BMI088, UINT64_C(100), 1.0f);
    TEST_EXPECT(imu_causal_accel_history_push(&history, &future, true, false));
    TEST_EXPECT(imu_causal_accel_history_push(&history, &causal, true, false));

    imu_causal_accel_query_result_t result;
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(150), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_OK);
    TEST_EXPECT(result.sample.timestamp_us == UINT64_C(100));
    TEST_EXPECT(result.age_us == UINT64_C(50));

    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(200), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_OK);
    TEST_EXPECT(result.sample.timestamp_us == UINT64_C(200));

    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(50), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_FUTURE_ONLY);
    TEST_EXPECT(result.sample.timestamp_us == UINT64_C(100));

    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_ICM45686, UINT64_C(150), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_SOURCE_MISMATCH);
    TEST_EXPECT(!imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, 0U, &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_INVALID_ARGUMENT);
    TEST_EXPECT(history.diagnostics.out_of_order_count == 1U);
}

static void test_age_boundary_and_gap_invalidate_old_evidence(void)
{
    imu_causal_accel_history_t history;
    TEST_EXPECT(imu_causal_accel_history_init(
        &history, IMU_SOURCE_ICM45686, 100U, 100U));
    imu_accel_sample_t sample =
        make_sample(IMU_SOURCE_ICM45686, UINT64_C(1000), 1.0f);
    TEST_EXPECT(imu_causal_accel_history_push(&history, &sample, true, false));

    imu_causal_accel_query_result_t result;
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_ICM45686, UINT64_C(1100), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_OK);
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_ICM45686, UINT64_C(1101), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_STALE);

    sample = make_sample(IMU_SOURCE_ICM45686, UINT64_C(1101), 2.0f);
    TEST_EXPECT(imu_causal_accel_history_push(&history, &sample, true, false));
    TEST_EXPECT(history.diagnostics.gap_reset_count == 1U);
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_ICM45686, UINT64_C(1050), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_FUTURE_ONLY);
    TEST_EXPECT(result.sample.timestamp_us == UINT64_C(1101));
}

static void test_saturation_and_invalid_barriers_reject_late_samples(void)
{
    imu_causal_accel_history_t history;
    TEST_EXPECT(imu_causal_accel_history_init(
        &history, IMU_SOURCE_BMI088, 1000U, 1000U));
    imu_accel_sample_t sample =
        make_sample(IMU_SOURCE_BMI088, UINT64_C(100), 1.0f);
    TEST_EXPECT(imu_causal_accel_history_push(&history, &sample, true, false));

    sample = make_sample(IMU_SOURCE_BMI088, UINT64_C(200), 20.0f);
    TEST_EXPECT(!imu_causal_accel_history_push(&history, &sample, true, true));
    imu_causal_accel_query_result_t result;
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(250), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_SATURATED);

    sample = make_sample(IMU_SOURCE_BMI088, UINT64_C(150), 1.5f);
    TEST_EXPECT(!imu_causal_accel_history_push(&history, &sample, true, false));
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(250), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_SATURATED);

    sample = make_sample(IMU_SOURCE_BMI088, UINT64_C(201), 2.0f);
    TEST_EXPECT(imu_causal_accel_history_push(&history, &sample, true, false));
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(250), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_OK);

    sample = make_sample(IMU_SOURCE_BMI088, UINT64_C(300), NAN);
    TEST_EXPECT(!imu_causal_accel_history_push(&history, &sample, true, false));
    sample = make_sample(IMU_SOURCE_BMI088, UINT64_C(250), 2.5f);
    TEST_EXPECT(!imu_causal_accel_history_push(&history, &sample, true, false));
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(350), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_INVALIDATED);

    sample = make_sample(IMU_SOURCE_BMI088, UINT64_C(301), 3.0f);
    TEST_EXPECT(imu_causal_accel_history_push(&history, &sample, true, false));
    imu_causal_accel_history_reset(&history);
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(350), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_INVALIDATED);
    TEST_EXPECT(history.diagnostics.invalidation_barrier_reject_count == 2U);
}

static void test_capacity_duplicate_and_too_old_behavior(void)
{
    imu_causal_accel_history_t history;
    TEST_EXPECT(imu_causal_accel_history_init(
        &history, IMU_SOURCE_BMI088, 1000U, 1000U));
    for (uint64_t index = 0U; index < IMU_CAUSAL_ACCEL_HISTORY_CAPACITY;
         ++index) {
        imu_accel_sample_t sample = make_sample(
            IMU_SOURCE_BMI088, UINT64_C(100) + index, (float)index);
        TEST_EXPECT(imu_causal_accel_history_push(
            &history, &sample, true, false));
    }

    imu_accel_sample_t duplicate =
        make_sample(IMU_SOURCE_BMI088, UINT64_C(110), 99.0f);
    TEST_EXPECT(imu_causal_accel_history_push(
        &history, &duplicate, true, false));
    imu_accel_sample_t newest =
        make_sample(IMU_SOURCE_BMI088, UINT64_C(116), 16.0f);
    TEST_EXPECT(imu_causal_accel_history_push(&history, &newest, true, false));
    imu_accel_sample_t too_old =
        make_sample(IMU_SOURCE_BMI088, UINT64_C(99), -1.0f);
    TEST_EXPECT(!imu_causal_accel_history_push(
        &history, &too_old, true, false));

    imu_causal_accel_query_result_t result;
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(110), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_OK);
    TEST_EXPECT(result.sample.accel_mps2[0] == 99.0f);
    TEST_EXPECT(history.diagnostics.duplicate_replacement_count == 1U);
    TEST_EXPECT(history.diagnostics.too_old_drop_count == 1U);
}

static void test_uint64_boundary_does_not_wrap_causality_or_age(void)
{
    imu_causal_accel_history_t history;
    TEST_EXPECT(imu_causal_accel_history_init(
        &history, IMU_SOURCE_BMI088, UINT32_MAX, 100U));
    imu_accel_sample_t sample = make_sample(
        IMU_SOURCE_BMI088, UINT64_MAX - UINT64_C(100), 1.0f);
    TEST_EXPECT(imu_causal_accel_history_push(&history, &sample, true, false));

    imu_causal_accel_query_result_t result;
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_MAX, &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_OK);
    TEST_EXPECT(result.age_us == UINT64_C(100));
    TEST_EXPECT(imu_causal_accel_history_query(
        &history, IMU_SOURCE_BMI088, UINT64_C(1), &result));
    TEST_EXPECT(result.status == IMU_CAUSAL_ACCEL_QUERY_FUTURE_ONLY);
}

int main(void)
{
    TEST_EXPECT(!imu_causal_accel_history_init(
        NULL, IMU_SOURCE_BMI088, 100U, 100U));
    test_latest_causal_ignores_future_and_accepts_out_of_order();
    test_age_boundary_and_gap_invalidate_old_evidence();
    test_saturation_and_invalid_barriers_reject_late_samples();
    test_capacity_duplicate_and_too_old_behavior();
    test_uint64_boundary_does_not_wrap_causality_or_age();

    if (failure_count != 0U) {
        (void)fprintf(stderr,
                      "imu_causal_accel_history: %u test failure(s)\n",
                      failure_count);
        return 1;
    }
    (void)puts("imu_causal_accel_history: all tests passed");
    return 0;
}
