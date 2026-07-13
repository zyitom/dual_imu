#include "dual_imu.h"

#include "bmi088.h"
#include "dual_imu_estimator.h"
#include "icm45686.h"
#include "imu_calibration.h"
#include "imu_stream_buffer.h"
#include "imu_time.h"
#include "main.h"

#include <math.h>
#include <string.h>

#define IMU_NOMINAL_PERIOD_US          2500U
#define IMU_IRQ_FALLBACK_DELAY_US     10000U
#define IMU_STALE_TIMEOUT_US          10000U
#define IMU_GRAVITY_MPS2               9.80665f
#define IMU_FROZEN_SAMPLE_LIMIT          8U
#define IMU_SATURATION_FRACTION         0.98f
#define BMI088_ACCEL_RANGE_MPS2        (24.0f * IMU_GRAVITY_MPS2)
#define ICM45686_ACCEL_RANGE_MPS2      (32.0f * IMU_GRAVITY_MPS2)
#define IMU_GYRO_RANGE_RAD_S           (2000.0f * 0.017453292519943295f)
#define IMU_READ_RETRY_BASE_US          2500U
#define IMU_READ_RETRY_MAX_US          40000U
#define IMU_MAX_EVENTS_PER_PROCESS        32U
#define IMU_MAX_WINDOWS_PER_PROCESS         8U
#define IMU_MAX_IMPACT_INHIBIT_US       1000000U
#define IMU_CONFIG_WATCHDOG_PERIOD_US   1000000U
#define IMU_ISOLATION_HINT_TTL_US        100000U
#define IMU_STREAM_ACCEL                     0U
#define IMU_STREAM_GYRO                      1U
#define IMU_STREAM_COUNT                     2U

/* Replace with P2 swept-sine/encoder measurements; zero keeps raw DRDY time. */
#ifndef BMI088_ACCEL_PIPELINE_DELAY_US
#define BMI088_ACCEL_PIPELINE_DELAY_US 0U
#endif
#ifndef BMI088_GYRO_PIPELINE_DELAY_US
#define BMI088_GYRO_PIPELINE_DELAY_US 0U
#endif
#ifndef ICM45686_ACCEL_PIPELINE_DELAY_US
#define ICM45686_ACCEL_PIPELINE_DELAY_US 0U
#endif
#ifndef ICM45686_GYRO_PIPELINE_DELAY_US
#define ICM45686_GYRO_PIPELINE_DELAY_US 0U
#endif

typedef struct
{
    imu_timestamp_queue_t timestamps;
    volatile uint32_t count;
    volatile uint64_t timestamp_us;
} irq_event_t;

typedef struct
{
    uint32_t count;
    uint64_t timestamp_us;
} irq_snapshot_t;

typedef struct
{
    float accel_mps2[3];
    float gyro_rad_s[3];
    uint8_t repeated_accel_samples;
    uint8_t repeated_gyro_samples;
    bool has_previous_accel;
    bool has_previous_gyro;
} sample_monitor_t;

static dual_imu_state_t state;
static dual_imu_estimator_t estimator;
static dual_imu_estimator_output_t estimator_output;
static sample_monitor_t sample_monitors[IMU_SOURCE_COUNT];
static irq_event_t bmi_accel_irq;
static irq_event_t bmi_gyro_irq;
static irq_event_t icm_irq;
static imu_accel_buffer_t accel_buffers[IMU_SOURCE_COUNT];
static imu_gyro_buffer_t gyro_buffers[IMU_SOURCE_COUNT];
static uint32_t bmi_combined_sequence;
static uint64_t last_bmi_accel_read_us;
static uint64_t last_bmi_gyro_read_us;
static uint64_t last_icm_read_us;
static uint64_t init_time_us;
static uint64_t retry_after_us[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint8_t read_error_streak[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint64_t estimator_stream_timestamp_us[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t coalesced_event_count[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t published_stream_sequence[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t last_irq_event_sequence[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t successful_stream_read_count[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t maximum_irq_to_read_us[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t capture_overrun_accounted_count[2];
static uint32_t capture_overrun_reported_count[2];
static uint64_t last_config_watchdog_us;
static uint64_t isolation_hint_expires_us;
static uint8_t config_watchdog_phase;
static bool estimator_initialized;

static void record_irq_event_isr(irq_event_t *event,
                                 uint64_t timestamp_us,
                                 uint32_t inferred_missed_events)
{
    if (event == NULL)
        return;

    event->timestamp_us = timestamp_us;
    event->count += 1U + inferred_missed_events;
    (void)imu_timestamp_queue_push_event_isr(&event->timestamps,
                                             timestamp_us,
                                             event->count);
}

static uint64_t time_us(void)
{
    return imu_time_now_us();
}

static irq_snapshot_t snapshot_irq(const irq_event_t *event)
{
    irq_snapshot_t snapshot;
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    snapshot.count = event->count;
    snapshot.timestamp_us = event->timestamp_us;
    __set_PRIMASK(primask);
    return snapshot;
}

static bool deadline_reached(uint64_t now_us, uint64_t deadline_us)
{
    return (deadline_us == 0U) || (now_us >= deadline_us);
}

static uint64_t compensate_pipeline_delay(uint64_t timestamp_us, uint32_t delay_us)
{
    return (timestamp_us > delay_us) ? (timestamp_us - delay_us) : 0U;
}

static uint32_t next_stream_sequence(imu_source_t source,
                                     uint32_t stream,
                                     bool has_irq_sequence,
                                     uint32_t irq_sequence)
{
    uint32_t increment = 1U;
    if (has_irq_sequence) {
        const uint32_t previous = last_irq_event_sequence[source][stream];
        increment = irq_sequence - previous;
        if ((increment == 0U) || (increment >= 0x80000000U))
            increment = 1U;
        last_irq_event_sequence[source][stream] = irq_sequence;
    }
    published_stream_sequence[source][stream] += increment;
    return published_stream_sequence[source][stream];
}

static void record_successful_stream_read(imu_source_t source,
                                          uint32_t stream,
                                          uint64_t irq_timestamp_us,
                                          bool has_irq_sequence)
{
    successful_stream_read_count[source][stream]++;
    if (!has_irq_sequence)
        return;

    const uint64_t completed_us = time_us();
    if (completed_us >= irq_timestamp_us) {
        const uint64_t latency_us = completed_us - irq_timestamp_us;
        const uint32_t bounded_latency = (latency_us > UINT32_MAX)
                                             ? UINT32_MAX
                                             : (uint32_t)latency_us;
        if (bounded_latency > maximum_irq_to_read_us[source][stream])
            maximum_irq_to_read_us[source][stream] = bounded_latency;
    }
}

static float vector_norm(const float vector[3])
{
    return sqrtf((vector[0] * vector[0]) + (vector[1] * vector[1]) +
                 (vector[2] * vector[2]));
}

static bool sample_is_sane(const imu_sample_t *sample)
{
    if ((sample == NULL) || !sample->valid || !isfinite(sample->temperature_c) ||
        (sample->temperature_c < -50.0f) || (sample->temperature_c > 125.0f))
        return false;

    for (uint32_t axis = 0; axis < 3U; axis++) {
        if (!isfinite(sample->accel_mps2[axis]) || !isfinite(sample->gyro_rad_s[axis]))
            return false;
    }

    return (vector_norm(sample->accel_mps2) < (40.0f * IMU_GRAVITY_MPS2)) &&
           (vector_norm(sample->gyro_rad_s) < 80.0f);
}

static void rotate_pitch_180(imu_sample_t *sample)
{
    sample->accel_mps2[0] = -sample->accel_mps2[0];
    sample->accel_mps2[2] = -sample->accel_mps2[2];
    sample->gyro_rad_s[0] = -sample->gyro_rad_s[0];
    sample->gyro_rad_s[2] = -sample->gyro_rad_s[2];
}

static void rotate_accel_pitch_180(imu_accel_sample_t *sample)
{
    sample->accel_mps2[0] = -sample->accel_mps2[0];
    sample->accel_mps2[2] = -sample->accel_mps2[2];
}

static void rotate_gyro_pitch_180(imu_gyro_sample_t *sample)
{
    sample->gyro_rad_s[0] = -sample->gyro_rad_s[0];
    sample->gyro_rad_s[2] = -sample->gyro_rad_s[2];
}

static void update_accel_monitor(imu_source_t source, const float accel_mps2[3])
{
    sample_monitor_t *monitor = &sample_monitors[source];
    bool repeated = monitor->has_previous_accel;

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        repeated = repeated && (accel_mps2[axis] == monitor->accel_mps2[axis]);
        monitor->accel_mps2[axis] = accel_mps2[axis];
    }
    if (repeated) {
        if (monitor->repeated_accel_samples < UINT8_MAX)
            monitor->repeated_accel_samples++;
    } else {
        monitor->repeated_accel_samples = 0U;
    }
    monitor->has_previous_accel = true;
}

static void update_gyro_monitor(imu_source_t source, const float gyro_rad_s[3])
{
    sample_monitor_t *monitor = &sample_monitors[source];
    bool repeated = monitor->has_previous_gyro;

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        repeated = repeated && (gyro_rad_s[axis] == monitor->gyro_rad_s[axis]);
        monitor->gyro_rad_s[axis] = gyro_rad_s[axis];
    }
    if (repeated) {
        if (monitor->repeated_gyro_samples < UINT8_MAX)
            monitor->repeated_gyro_samples++;
    } else {
        monitor->repeated_gyro_samples = 0U;
    }
    monitor->has_previous_gyro = true;
}

static bool accel_vector_is_saturated(imu_source_t source,
                                      const float accel_mps2[3])
{
    const float range = (source == IMU_SOURCE_BMI088)
                            ? BMI088_ACCEL_RANGE_MPS2
                            : ICM45686_ACCEL_RANGE_MPS2;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        if (fabsf(accel_mps2[axis]) >=
            (IMU_SATURATION_FRACTION * range))
            return true;
    }
    return false;
}

static bool gyro_vector_is_saturated(const float gyro_rad_s[3])
{
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        if (fabsf(gyro_rad_s[axis]) >=
            (IMU_SATURATION_FRACTION * IMU_GYRO_RANGE_RAD_S))
            return true;
    }
    return false;
}

static bool estimator_accel_sample_is_eligible(imu_source_t source,
                                               bool timestamp_trusted)
{
    if (!timestamp_trusted ||
        (sample_monitors[source].repeated_accel_samples >=
         IMU_FROZEN_SAMPLE_LIMIT))
        return false;

    if (source == IMU_SOURCE_BMI088)
        return !state.bmi088_accel_configuration_fault;
    return !state.icm45686_accel_configuration_fault;
}

static bool estimator_gyro_sample_is_eligible(imu_source_t source,
                                              bool timestamp_trusted)
{
    if (!timestamp_trusted ||
        (sample_monitors[source].repeated_gyro_samples >=
         IMU_FROZEN_SAMPLE_LIMIT))
        return false;

    if (source == IMU_SOURCE_BMI088)
        return !state.bmi088_gyro_configuration_fault;
    return !state.icm45686_gyro_configuration_fault;
}

static bool sample_is_fresh(const imu_sample_t *sample, uint64_t now_us)
{
    return sample_is_sane(sample) && (sample->accel_timestamp_us != 0U) &&
           (sample->gyro_timestamp_us != 0U) &&
           (now_us >= sample->accel_timestamp_us) &&
           (now_us >= sample->gyro_timestamp_us) &&
           ((now_us - sample->accel_timestamp_us) <= IMU_STALE_TIMEOUT_US) &&
           ((now_us - sample->gyro_timestamp_us) <= IMU_STALE_TIMEOUT_US);
}

static uint32_t sample_fault_flags(imu_source_t source,
                                   const imu_sample_t *sample,
                                   const imu_health_t *health,
                                   uint64_t now_us)
{
    uint32_t faults = DUAL_IMU_FAULT_NONE;

    if (!sample_is_sane(sample))
        faults |= DUAL_IMU_FAULT_INVALID;
    else if (!sample_is_fresh(sample, now_us))
        faults |= DUAL_IMU_FAULT_STALE;

    if (health->status == IMU_STATUS_STALE)
        faults |= DUAL_IMU_FAULT_STALE;
    else if (health->status != IMU_STATUS_OK)
        faults |= DUAL_IMU_FAULT_HEALTH;

    if ((sample != NULL) && sample->accel_valid &&
        accel_vector_is_saturated(sample->source, sample->accel_mps2))
        faults |= DUAL_IMU_FAULT_ACCEL_SATURATED;
    if ((sample != NULL) && sample->gyro_valid &&
        gyro_vector_is_saturated(sample->gyro_rad_s))
        faults |= DUAL_IMU_FAULT_GYRO_SATURATED;
    if (sample_monitors[source].repeated_accel_samples >= IMU_FROZEN_SAMPLE_LIMIT)
        faults |= DUAL_IMU_FAULT_ACCEL_FROZEN;
    if (sample_monitors[source].repeated_gyro_samples >= IMU_FROZEN_SAMPLE_LIMIT)
        faults |= DUAL_IMU_FAULT_GYRO_FROZEN;

    return faults;
}

static uint32_t timestamp_distance(uint64_t lhs, uint64_t rhs)
{
    const uint64_t delta = (lhs >= rhs) ? (lhs - rhs) : (rhs - lhs);
    return (delta > UINT32_MAX) ? UINT32_MAX : (uint32_t)delta;
}

static void record_read_result(imu_source_t source,
                               uint32_t stream,
                               bool success,
                               uint64_t now_us)
{
    if (success) {
        read_error_streak[source][stream] = 0U;
        retry_after_us[source][stream] = 0U;
        return;
    }

    if (read_error_streak[source][stream] < 5U)
        read_error_streak[source][stream]++;

    uint32_t delay_us =
        IMU_READ_RETRY_BASE_US << (read_error_streak[source][stream] - 1U);
    if (delay_us > IMU_READ_RETRY_MAX_US)
        delay_us = IMU_READ_RETRY_MAX_US;
    retry_after_us[source][stream] = now_us + delay_us;
    if (stream == IMU_STREAM_ACCEL) {
        sample_monitors[source].has_previous_accel = false;
        sample_monitors[source].repeated_accel_samples = 0U;
    } else {
        sample_monitors[source].has_previous_gyro = false;
        sample_monitors[source].repeated_gyro_samples = 0U;
    }
}

static bool irq_is_silent(const irq_snapshot_t *event, uint64_t now_us)
{
    if (event->count == 0U)
    {
        return (now_us >= init_time_us) &&
               ((now_us - init_time_us) > IMU_IRQ_FALLBACK_DELAY_US);
    }

    return (now_us >= event->timestamp_us) &&
           ((now_us - event->timestamp_us) > IMU_IRQ_FALLBACK_DELAY_US);
}

static void publish_bmi_sample(void)
{
    if (!state.bmi088_accel_sample.valid || !state.bmi088_gyro_sample.valid)
    {
        return;
    }

    imu_sample_t *const sample = &state.bmi088_sample;
    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = state.bmi088_gyro_sample.timestamp_us;
    sample->accel_timestamp_us = state.bmi088_accel_sample.timestamp_us;
    sample->gyro_timestamp_us = state.bmi088_gyro_sample.timestamp_us;
    sample->sequence = ++bmi_combined_sequence;
    sample->accel_sequence = state.bmi088_accel_sample.sequence;
    sample->gyro_sequence = state.bmi088_gyro_sample.sequence;
    memcpy(sample->accel_mps2, state.bmi088_accel_sample.accel_mps2,
           sizeof(sample->accel_mps2));
    memcpy(sample->gyro_rad_s, state.bmi088_gyro_sample.gyro_rad_s,
           sizeof(sample->gyro_rad_s));
    sample->temperature_c = state.bmi088_accel_sample.temperature_c;
    sample->source = IMU_SOURCE_BMI088;
    sample->accel_valid = state.bmi088_accel_sample.valid;
    sample->gyro_valid = state.bmi088_gyro_sample.valid;
    sample->valid = true;
}

static void read_bmi_accel_at(uint64_t timestamp_us,
                              uint64_t now_us,
                              bool has_irq_sequence,
                              uint32_t irq_sequence)
{
    if (!state.bmi088_initialized ||
        !deadline_reached(now_us,
                          retry_after_us[IMU_SOURCE_BMI088][IMU_STREAM_ACCEL]))
    {
        return;
    }

    const bool success = bmi088_read_accel(timestamp_us, &state.bmi088_accel_sample);
    if (success)
    {
        state.bmi088_accel_sample.sequence = next_stream_sequence(
            IMU_SOURCE_BMI088, IMU_STREAM_ACCEL,
            has_irq_sequence, irq_sequence);
        record_successful_stream_read(IMU_SOURCE_BMI088, IMU_STREAM_ACCEL,
                                      timestamp_us, has_irq_sequence);
        rotate_accel_pitch_180(&state.bmi088_accel_sample);
        state.bmi088_accel_sample.valid =
            state.bmi088_accel_sample.valid &&
            imu_calibration_apply_accel(
                IMU_SOURCE_BMI088,
                state.bmi088_accel_sample.temperature_c,
                state.bmi088_accel_sample.accel_mps2);
        if (state.bmi088_accel_sample.valid)
            update_accel_monitor(IMU_SOURCE_BMI088,
                                 state.bmi088_accel_sample.accel_mps2);
        imu_accel_buffer_push(&accel_buffers[IMU_SOURCE_BMI088],
                              &state.bmi088_accel_sample);
        if (estimator_initialized) {
            imu_accel_sample_t corrected = state.bmi088_accel_sample;
            corrected.timestamp_us = compensate_pipeline_delay(
                corrected.timestamp_us, BMI088_ACCEL_PIPELINE_DELAY_US);
            corrected.valid = corrected.valid &&
                              estimator_accel_sample_is_eligible(
                                  IMU_SOURCE_BMI088, has_irq_sequence) &&
                              !accel_vector_is_saturated(
                                  IMU_SOURCE_BMI088, corrected.accel_mps2);
            if (dual_imu_estimator_push_accel(&estimator, &corrected))
                estimator_stream_timestamp_us[IMU_SOURCE_BMI088]
                                             [IMU_STREAM_ACCEL] =
                    corrected.timestamp_us;
        }
    }
    record_read_result(IMU_SOURCE_BMI088, IMU_STREAM_ACCEL, success, now_us);
    last_bmi_accel_read_us = now_us;
}

static void read_bmi_gyro_at(uint64_t timestamp_us,
                             uint64_t now_us,
                             bool has_irq_sequence,
                             uint32_t irq_sequence)
{
    if (!state.bmi088_initialized ||
        !deadline_reached(now_us,
                          retry_after_us[IMU_SOURCE_BMI088][IMU_STREAM_GYRO]))
    {
        return;
    }

    const bool success = bmi088_read_gyro(timestamp_us, &state.bmi088_gyro_sample);
    if (success)
    {
        state.bmi088_gyro_sample.sequence = next_stream_sequence(
            IMU_SOURCE_BMI088, IMU_STREAM_GYRO,
            has_irq_sequence, irq_sequence);
        record_successful_stream_read(IMU_SOURCE_BMI088, IMU_STREAM_GYRO,
                                      timestamp_us, has_irq_sequence);
        rotate_gyro_pitch_180(&state.bmi088_gyro_sample);
        state.bmi088_gyro_sample.valid =
            state.bmi088_gyro_sample.valid &&
            imu_calibration_apply_gyro(
                IMU_SOURCE_BMI088,
                state.bmi088_gyro_sample.temperature_c,
                state.bmi088_gyro_sample.gyro_rad_s);
        if (state.bmi088_gyro_sample.valid)
            update_gyro_monitor(IMU_SOURCE_BMI088,
                                state.bmi088_gyro_sample.gyro_rad_s);
        imu_gyro_buffer_push(&gyro_buffers[IMU_SOURCE_BMI088],
                             &state.bmi088_gyro_sample);
        if (estimator_initialized) {
            imu_gyro_sample_t corrected = state.bmi088_gyro_sample;
            corrected.timestamp_us = compensate_pipeline_delay(
                corrected.timestamp_us, BMI088_GYRO_PIPELINE_DELAY_US);
            corrected.valid = corrected.valid &&
                              estimator_gyro_sample_is_eligible(
                                  IMU_SOURCE_BMI088, has_irq_sequence) &&
                              !gyro_vector_is_saturated(corrected.gyro_rad_s);
            if (dual_imu_estimator_push_gyro(&estimator, &corrected))
                estimator_stream_timestamp_us[IMU_SOURCE_BMI088]
                                             [IMU_STREAM_GYRO] =
                    corrected.timestamp_us;
        }
        publish_bmi_sample();
    }
    record_read_result(IMU_SOURCE_BMI088, IMU_STREAM_GYRO, success, now_us);
    last_bmi_gyro_read_us = now_us;
}

static void publish_icm_streams(const imu_sample_t *sample,
                                bool timestamp_trusted)
{
    imu_accel_sample_t accel = {
        .timestamp_us = sample->accel_timestamp_us,
        .sequence = sample->accel_sequence,
        .temperature_c = sample->temperature_c,
        .source = IMU_SOURCE_ICM45686,
        .valid = sample->accel_valid,
    };
    imu_gyro_sample_t gyro = {
        .timestamp_us = sample->gyro_timestamp_us,
        .sequence = sample->gyro_sequence,
        .temperature_c = sample->temperature_c,
        .source = IMU_SOURCE_ICM45686,
        .valid = sample->gyro_valid,
    };

    memcpy(accel.accel_mps2, sample->accel_mps2, sizeof(accel.accel_mps2));
    memcpy(gyro.gyro_rad_s, sample->gyro_rad_s, sizeof(gyro.gyro_rad_s));
    if (accel.valid)
        update_accel_monitor(IMU_SOURCE_ICM45686, accel.accel_mps2);
    if (gyro.valid)
        update_gyro_monitor(IMU_SOURCE_ICM45686, gyro.gyro_rad_s);
    imu_accel_buffer_push(&accel_buffers[IMU_SOURCE_ICM45686], &accel);
    imu_gyro_buffer_push(&gyro_buffers[IMU_SOURCE_ICM45686], &gyro);
    if (estimator_initialized) {
        imu_accel_sample_t corrected_accel = accel;
        imu_gyro_sample_t corrected_gyro = gyro;
        corrected_accel.timestamp_us = compensate_pipeline_delay(
            corrected_accel.timestamp_us, ICM45686_ACCEL_PIPELINE_DELAY_US);
        corrected_gyro.timestamp_us = compensate_pipeline_delay(
            corrected_gyro.timestamp_us, ICM45686_GYRO_PIPELINE_DELAY_US);
        corrected_accel.valid = corrected_accel.valid &&
                                  estimator_accel_sample_is_eligible(
                                      IMU_SOURCE_ICM45686,
                                      timestamp_trusted) &&
                                  !accel_vector_is_saturated(
                                      IMU_SOURCE_ICM45686,
                                      corrected_accel.accel_mps2);
        corrected_gyro.valid = corrected_gyro.valid &&
                               estimator_gyro_sample_is_eligible(
                                   IMU_SOURCE_ICM45686,
                                   timestamp_trusted) &&
                               !gyro_vector_is_saturated(
                                   corrected_gyro.gyro_rad_s);
        if (dual_imu_estimator_push_accel(&estimator, &corrected_accel))
            estimator_stream_timestamp_us[IMU_SOURCE_ICM45686]
                                         [IMU_STREAM_ACCEL] =
                corrected_accel.timestamp_us;
        if (dual_imu_estimator_push_gyro(&estimator, &corrected_gyro))
            estimator_stream_timestamp_us[IMU_SOURCE_ICM45686]
                                         [IMU_STREAM_GYRO] =
                corrected_gyro.timestamp_us;
    }
}

static void read_icm_at(uint64_t timestamp_us,
                        uint64_t now_us,
                        bool has_irq_sequence,
                        uint32_t irq_sequence)
{
    if (!state.icm45686_initialized ||
        !deadline_reached(now_us,
                          retry_after_us[IMU_SOURCE_ICM45686][IMU_STREAM_GYRO]))
    {
        return;
    }

    const bool success = icm45686_read(timestamp_us, &state.icm45686_sample);
    if (success)
    {
        const uint32_t sequence = next_stream_sequence(
            IMU_SOURCE_ICM45686, IMU_STREAM_GYRO,
            has_irq_sequence, irq_sequence);
        published_stream_sequence[IMU_SOURCE_ICM45686][IMU_STREAM_ACCEL] = sequence;
        last_irq_event_sequence[IMU_SOURCE_ICM45686][IMU_STREAM_ACCEL] =
            last_irq_event_sequence[IMU_SOURCE_ICM45686][IMU_STREAM_GYRO];
        state.icm45686_sample.sequence = sequence;
        state.icm45686_sample.accel_sequence = sequence;
        state.icm45686_sample.gyro_sequence = sequence;
        record_successful_stream_read(IMU_SOURCE_ICM45686, IMU_STREAM_ACCEL,
                                      timestamp_us, has_irq_sequence);
        record_successful_stream_read(IMU_SOURCE_ICM45686, IMU_STREAM_GYRO,
                                      timestamp_us, has_irq_sequence);
        rotate_pitch_180(&state.icm45686_sample);
        state.icm45686_sample.accel_valid =
            state.icm45686_sample.accel_valid &&
            imu_calibration_apply_accel(
                IMU_SOURCE_ICM45686,
                state.icm45686_sample.temperature_c,
                state.icm45686_sample.accel_mps2);
        state.icm45686_sample.gyro_valid =
            state.icm45686_sample.gyro_valid &&
            imu_calibration_apply_gyro(
                IMU_SOURCE_ICM45686,
                state.icm45686_sample.temperature_c,
                state.icm45686_sample.gyro_rad_s);
        state.icm45686_sample.valid =
            state.icm45686_sample.accel_valid &&
            state.icm45686_sample.gyro_valid;
        publish_icm_streams(&state.icm45686_sample, has_irq_sequence);
    }
    record_read_result(IMU_SOURCE_ICM45686, IMU_STREAM_ACCEL, success, now_us);
    record_read_result(IMU_SOURCE_ICM45686, IMU_STREAM_GYRO, success, now_us);
    last_icm_read_us = now_us;
}

static bool pop_latest_event(imu_timestamp_queue_t *queue,
                             uint64_t *latest_timestamp_us,
                             uint32_t *latest_sequence,
                             uint32_t *coalesced_count)
{
    uint32_t popped = 0U;
    uint64_t timestamp_us;

    uint32_t sequence;
    while ((popped < IMU_MAX_EVENTS_PER_PROCESS) &&
           imu_timestamp_queue_pop_event(queue, &timestamp_us, &sequence)) {
        *latest_timestamp_us = timestamp_us;
        *latest_sequence = sequence;
        popped++;
    }
    *coalesced_count = (popped > 1U) ? (popped - 1U) : 0U;
    return popped != 0U;
}

static void read_pending_bmi_events(uint64_t now_us)
{
    uint64_t accel_timestamp = 0U;
    uint64_t gyro_timestamp = 0U;
    uint32_t accel_sequence = 0U;
    uint32_t gyro_sequence = 0U;
    uint32_t accel_coalesced = 0U;
    uint32_t gyro_coalesced = 0U;
    const bool has_accel = pop_latest_event(&bmi_accel_irq.timestamps,
                                            &accel_timestamp,
                                            &accel_sequence,
                                            &accel_coalesced);
    const bool has_gyro = pop_latest_event(&bmi_gyro_irq.timestamps,
                                           &gyro_timestamp,
                                           &gyro_sequence,
                                           &gyro_coalesced);
    coalesced_event_count[IMU_SOURCE_BMI088][IMU_STREAM_ACCEL] +=
        accel_coalesced;
    coalesced_event_count[IMU_SOURCE_BMI088][IMU_STREAM_GYRO] +=
        gyro_coalesced;

    if (has_accel && (!has_gyro || (accel_timestamp <= gyro_timestamp))) {
        read_bmi_accel_at(accel_timestamp, now_us, true, accel_sequence);
        if (has_gyro)
            read_bmi_gyro_at(gyro_timestamp, now_us, true, gyro_sequence);
    } else if (has_gyro) {
        read_bmi_gyro_at(gyro_timestamp, now_us, true, gyro_sequence);
        if (has_accel)
            read_bmi_accel_at(accel_timestamp, now_us, true, accel_sequence);
    }
}

static void read_pending_icm_events(uint64_t now_us)
{
    uint64_t timestamp_us = 0U;
    uint32_t sequence = 0U;
    uint32_t coalesced = 0U;
    if (!pop_latest_event(&icm_irq.timestamps, &timestamp_us,
                          &sequence, &coalesced))
        return;

    coalesced_event_count[IMU_SOURCE_ICM45686][IMU_STREAM_ACCEL] += coalesced;
    coalesced_event_count[IMU_SOURCE_ICM45686][IMU_STREAM_GYRO] += coalesced;
    read_icm_at(timestamp_us, now_us, true, sequence);
}

static void read_pending_samples(uint64_t now_us)
{
    read_pending_bmi_events(now_us);
    read_pending_icm_events(now_us);

    const irq_snapshot_t bmi_accel_event = snapshot_irq(&bmi_accel_irq);
    const irq_snapshot_t bmi_gyro_event = snapshot_irq(&bmi_gyro_irq);
    const irq_snapshot_t icm_event = snapshot_irq(&icm_irq);

    if (irq_is_silent(&bmi_accel_event, now_us) &&
        ((now_us - last_bmi_accel_read_us) >= IMU_NOMINAL_PERIOD_US))
    {
        read_bmi_accel_at(now_us, now_us, false, 0U);
    }
    if (irq_is_silent(&bmi_gyro_event, now_us) &&
        ((now_us - last_bmi_gyro_read_us) >= IMU_NOMINAL_PERIOD_US))
    {
        read_bmi_gyro_at(now_us, now_us, false, 0U);
    }
    if (irq_is_silent(&icm_event, now_us) &&
        ((now_us - last_icm_read_us) >= IMU_NOMINAL_PERIOD_US))
    {
        read_icm_at(now_us, now_us, false, 0U);
    }
}

static void run_configuration_watchdog(uint64_t now_us)
{
    if ((now_us - last_config_watchdog_us) < IMU_CONFIG_WATCHDOG_PERIOD_US)
        return;
    last_config_watchdog_us = now_us;

    if (config_watchdog_phase == 0U) {
        if (state.bmi088_initialized) {
            state.bmi088_accel_configuration_fault =
                !bmi088_check_accel_configuration();
            if (state.bmi088_accel_configuration_fault) {
                state.bmi088_accel_watchdog_failure_count++;
                state.bmi088_watchdog_failure_count++;
            }
        }
    } else if (config_watchdog_phase == 1U) {
        if (state.bmi088_initialized) {
            state.bmi088_gyro_configuration_fault =
                !bmi088_check_gyro_configuration();
            if (state.bmi088_gyro_configuration_fault) {
                state.bmi088_gyro_watchdog_failure_count++;
                state.bmi088_watchdog_failure_count++;
            }
        }
    } else if (config_watchdog_phase == 2U) {
        if (state.icm45686_initialized) {
            state.icm45686_accel_configuration_fault =
                !icm45686_check_accel_configuration();
            if (state.icm45686_accel_configuration_fault) {
                state.icm45686_accel_watchdog_failure_count++;
                state.icm45686_watchdog_failure_count++;
            }
        }
    } else if (state.icm45686_initialized) {
        state.icm45686_gyro_configuration_fault =
            !icm45686_check_gyro_configuration();
        if (state.icm45686_gyro_configuration_fault) {
            state.icm45686_gyro_watchdog_failure_count++;
            state.icm45686_watchdog_failure_count++;
        }
    }
    state.icm45686_configuration_fault =
        state.icm45686_accel_configuration_fault ||
        state.icm45686_gyro_configuration_fault;
    config_watchdog_phase = (uint8_t)((config_watchdog_phase + 1U) % 4U);
}

static bool source_gyro_is_sane(imu_source_t source)
{
    if (source == IMU_SOURCE_BMI088) {
        return state.bmi088_gyro_sample.valid &&
               (state.bmi088_gyro_sample.timestamp_us != 0U) &&
               isfinite(state.bmi088_gyro_sample.gyro_rad_s[0]) &&
               isfinite(state.bmi088_gyro_sample.gyro_rad_s[1]) &&
               isfinite(state.bmi088_gyro_sample.gyro_rad_s[2]);
    }
    return state.icm45686_sample.gyro_valid &&
           (state.icm45686_sample.gyro_timestamp_us != 0U) &&
           isfinite(state.icm45686_sample.gyro_rad_s[0]) &&
           isfinite(state.icm45686_sample.gyro_rad_s[1]) &&
           isfinite(state.icm45686_sample.gyro_rad_s[2]);
}

static bool source_gyro_is_fresh(imu_source_t source, uint64_t now_us)
{
    if (!source_gyro_is_sane(source))
        return false;

    const uint64_t timestamp_us = (source == IMU_SOURCE_BMI088)
                                      ? state.bmi088_gyro_sample.timestamp_us
                                      : state.icm45686_sample.gyro_timestamp_us;
    return (now_us >= timestamp_us) &&
           ((now_us - timestamp_us) <= IMU_STALE_TIMEOUT_US);
}

static uint32_t selector_hard_faults(imu_source_t source, uint64_t now_us)
{
    const bool initialized = (source == IMU_SOURCE_BMI088)
                                 ? state.bmi088_initialized
                                 : state.icm45686_initialized;
    const uint32_t source_faults = (source == IMU_SOURCE_BMI088)
                                       ? state.bmi088_fault_flags
                                       : state.icm45686_fault_flags;
    if (!initialized)
        return DUAL_IMU_FAULT_HEALTH;

    uint32_t hard = source_faults &
                    (DUAL_IMU_FAULT_GYRO_SATURATED |
                     DUAL_IMU_FAULT_GYRO_FROZEN | DUAL_IMU_FAULT_FILTER |
                     DUAL_IMU_FAULT_GYRO_TIMING);
    if (source == IMU_SOURCE_BMI088) {
        if ((read_error_streak[IMU_SOURCE_BMI088][IMU_STREAM_GYRO] != 0U) ||
            state.bmi088_gyro_configuration_fault)
            hard |= DUAL_IMU_FAULT_HEALTH;
    } else if ((read_error_streak[IMU_SOURCE_ICM45686][IMU_STREAM_GYRO] != 0U) ||
               state.icm45686_gyro_configuration_fault) {
        hard |= DUAL_IMU_FAULT_HEALTH;
    }
    /* Expected impact clipping invalidates data but must not hard-latch a lane. */
    if (now_us < estimator.accel_inhibit_until_us)
        hard &= ~((uint32_t)DUAL_IMU_FAULT_GYRO_SATURATED);
    if ((now_us - init_time_us) > IMU_STALE_TIMEOUT_US) {
        if (!source_gyro_is_sane(source))
            hard |= DUAL_IMU_FAULT_INVALID;
        else if (!source_gyro_is_fresh(source, now_us))
            hard |= DUAL_IMU_FAULT_STALE;
    }
    return hard;
}

static uint64_t estimator_event_watermark(uint64_t now_us)
{
    uint64_t watermark_us = now_us;
    for (uint32_t source = 0U; source < IMU_SOURCE_COUNT; ++source) {
        const bool source_initialized = (source == IMU_SOURCE_BMI088)
                                            ? state.bmi088_initialized
                                            : state.icm45686_initialized;
        if (!source_initialized)
            continue;

        for (uint32_t stream = 0U; stream < IMU_STREAM_COUNT; ++stream) {
            const uint64_t timestamp_us =
                estimator_stream_timestamp_us[source][stream];
            if (timestamp_us == 0U) {
                if ((now_us - init_time_us) <= IMU_STALE_TIMEOUT_US)
                    return 0U;
                continue;
            }

            const bool stream_stale = (now_us >= timestamp_us) &&
                                      ((now_us - timestamp_us) >
                                       IMU_STALE_TIMEOUT_US);
            if (!stream_stale && (timestamp_us < watermark_us))
                watermark_us = timestamp_us;
        }
    }
    return watermark_us;
}

static void publish_estimator_output(const dual_imu_estimator_output_t *output)
{
    const float dt_s = (float)(output->end_us - output->start_us) * 1.0e-6f;
    state.selector_state = output->selector.state;
    state.selector_reason_flags = output->selector.reason_flags;
    state.selector_residual_nis = output->selector.residual_nis;
    state.selection_changed = output->selector.selection_changed;
    state.alignment_blend_active = output->output_alignment_active;
    state.stationary_candidate = output->stationary_candidate;
    state.stationary_confirmed = output->stationary_confirmed;
    state.accel_update_inhibited = output->accel_inhibited;
    state.fused_accel_valid = output->specific_force_valid;
    state.accel_residual_mps2 = output->accel_pair_residual_mps2;

    for (uint32_t lane = 0U; lane < IMU_SOURCE_COUNT; ++lane) {
        memcpy(state.lane_quaternion[lane], output->lane_quaternion[lane],
               sizeof(state.lane_quaternion[lane]));
        memcpy(state.lane_gyro_bias_rad_s[lane], output->lane_bias_rad_s[lane],
               sizeof(state.lane_gyro_bias_rad_s[lane]));
        state.lane_window_flags[lane] = output->lane_window[lane].flags;
        const attitude_mekf_diagnostics_t *diagnostics =
            &estimator.mekf[lane].diagnostics;
        state.lane_accel_reject_count[lane] =
            diagnostics->accel_update_count - diagnostics->accel_accept_count;
        state.lane_zaru_accept_count[lane] = diagnostics->zaru_accept_count;
    }
    state.bmi088_calibrated = output->lane_calibrated[IMU_SOURCE_BMI088];
    state.icm45686_calibrated = output->lane_calibrated[IMU_SOURCE_ICM45686];

    if (output->selector.residual_valid && (dt_s > 0.0f)) {
        state.gyro_residual_rad_s =
            vector_norm(output->selector.residual_rad) / dt_s;
    }
    state.pair_skew_us = timestamp_distance(
        state.bmi088_sample.gyro_timestamp_us,
        state.icm45686_sample.gyro_timestamp_us);

    if ((output->selector.reason_flags & IMU_SELECTOR_REASON_NIS_HIGH) != 0U)
        state.mismatch_count++;
    if (!output->lane_window[0].gyro_valid ||
        !output->lane_window[1].gyro_valid)
        state.unpaired_count++;
    if (output->selector.selection_changed)
        state.selection_count++;

    if (!output->output_valid ||
        (output->selector.selected_lane >= IMU_SELECTOR_LANE_COUNT)) {
        state.mode = DUAL_IMU_MODE_NONE;
        state.selected_source = IMU_SOURCE_FUSED;
        state.fused_sample.valid = false;
        state.filter_reject_count++;
        return;
    }

    const imu_source_t selected =
        (output->selector.selected_lane == IMU_SELECTOR_LANE_0)
            ? IMU_SOURCE_BMI088
            : IMU_SOURCE_ICM45686;
    const imu_sample_t *latest = (selected == IMU_SOURCE_BMI088)
                                     ? &state.bmi088_sample
                                     : &state.icm45686_sample;
    state.selected_source = selected;
    state.selected_accel_source =
        (output->selected_accel_lane == IMU_SELECTOR_LANE_0)
            ? IMU_SOURCE_BMI088
        : (output->selected_accel_lane == IMU_SELECTOR_LANE_1)
            ? IMU_SOURCE_ICM45686
            : IMU_SOURCE_FUSED;
    state.mode = (selected == IMU_SOURCE_BMI088)
                     ? DUAL_IMU_MODE_BMI088
                     : DUAL_IMU_MODE_ICM45686;

    imu_sample_t fused;
    memset(&fused, 0, sizeof(fused));
    const uint64_t measurement_midpoint_us =
        output->start_us + ((output->end_us - output->start_us) / 2U);
    fused.timestamp_us = output->end_us;
    fused.accel_timestamp_us = measurement_midpoint_us;
    fused.gyro_timestamp_us = measurement_midpoint_us;
    fused.sequence = state.fusion_count + 1U;
    const imu_sample_t *accel_latest =
        (state.selected_accel_source == IMU_SOURCE_BMI088)
            ? &state.bmi088_sample
        : (state.selected_accel_source == IMU_SOURCE_ICM45686)
            ? &state.icm45686_sample
            : latest;
    fused.accel_sequence = accel_latest->accel_sequence;
    fused.gyro_sequence = latest->gyro_sequence;
    memcpy(fused.accel_mps2, output->specific_force_mps2,
           sizeof(fused.accel_mps2));
    memcpy(fused.gyro_rad_s, output->angular_rate_rad_s,
           sizeof(fused.gyro_rad_s));
    fused.temperature_c = latest->temperature_c;
    fused.source = IMU_SOURCE_FUSED;
    fused.accel_valid = output->specific_force_valid;
    fused.gyro_valid = true;
    fused.valid = fused.gyro_valid;
    state.fused_sample = fused;
    memcpy(state.quaternion, output->quaternion, sizeof(state.quaternion));
    memcpy(state.euler_rad, output->euler_rad, sizeof(state.euler_rad));
    state.fusion_count++;
}

static void update_estimator(uint64_t now_us)
{
    if (!estimator_initialized)
        return;

    dual_imu_estimator_set_hard_faults(
        &estimator, IMU_SOURCE_BMI088,
        selector_hard_faults(IMU_SOURCE_BMI088, now_us));
    dual_imu_estimator_set_hard_faults(
        &estimator, IMU_SOURCE_ICM45686,
        selector_hard_faults(IMU_SOURCE_ICM45686, now_us));

    const uint64_t watermark_us = estimator_event_watermark(now_us);
    for (uint32_t count = 0U; count < IMU_MAX_WINDOWS_PER_PROCESS; ++count) {
        const imu_preintegrator_result_t result =
            dual_imu_estimator_process_next(&estimator,
                                            watermark_us,
                                            &estimator_output);
        if (result == IMU_PREINTEGRATOR_NOT_READY)
            break;
        if (result == IMU_PREINTEGRATOR_ERROR) {
            state.filter_reject_count++;
            break;
        }
        publish_estimator_output(&estimator_output);
    }
}

static void update_fast_rate_output(void)
{
    imu_source_t selected = state.selected_source;
    if (selected >= IMU_SOURCE_COUNT) {
        if (state.icm45686_initialized)
            selected = IMU_SOURCE_ICM45686;
        else if (state.bmi088_initialized)
            selected = IMU_SOURCE_BMI088;
        else
            return;
    }

    const float *raw_rate;
    uint64_t timestamp_us;
    if (selected == IMU_SOURCE_BMI088) {
        if (!state.bmi088_gyro_sample.valid)
            return;
        raw_rate = state.bmi088_gyro_sample.gyro_rad_s;
        timestamp_us = state.bmi088_gyro_sample.timestamp_us;
    } else {
        if (!state.icm45686_sample.gyro_valid)
            return;
        raw_rate = state.icm45686_sample.gyro_rad_s;
        timestamp_us = state.icm45686_sample.gyro_timestamp_us;
    }

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        state.control_gyro_rad_s[axis] =
            raw_rate[axis] - state.lane_gyro_bias_rad_s[selected][axis];
    }
    state.control_gyro_timestamp_us = timestamp_us;
}

bool dual_imu_init(void)
{
    imu_calibration_initialize();
    memset(&state, 0, sizeof(state));
    memset(&estimator, 0, sizeof(estimator));
    memset(&estimator_output, 0, sizeof(estimator_output));
    memset(sample_monitors, 0, sizeof(sample_monitors));
    memset(&bmi_accel_irq, 0, sizeof(bmi_accel_irq));
    memset(&bmi_gyro_irq, 0, sizeof(bmi_gyro_irq));
    memset(&icm_irq, 0, sizeof(icm_irq));
    memset(retry_after_us, 0, sizeof(retry_after_us));
    memset(read_error_streak, 0, sizeof(read_error_streak));
    memset(estimator_stream_timestamp_us, 0,
           sizeof(estimator_stream_timestamp_us));
    memset(coalesced_event_count, 0, sizeof(coalesced_event_count));
    memset(published_stream_sequence, 0, sizeof(published_stream_sequence));
    memset(last_irq_event_sequence, 0, sizeof(last_irq_event_sequence));
    memset(successful_stream_read_count, 0,
           sizeof(successful_stream_read_count));
    memset(maximum_irq_to_read_us, 0, sizeof(maximum_irq_to_read_us));
    memset(capture_overrun_accounted_count, 0,
           sizeof(capture_overrun_accounted_count));
    memset(capture_overrun_reported_count, 0,
           sizeof(capture_overrun_reported_count));
    for (uint32_t source = 0U; source < IMU_SOURCE_COUNT; source++) {
        imu_accel_buffer_reset(&accel_buffers[source]);
        imu_gyro_buffer_reset(&gyro_buffers[source]);
    }
    bmi_combined_sequence = 0U;
    last_bmi_accel_read_us = 0U;
    last_bmi_gyro_read_us = 0U;
    last_icm_read_us = 0U;
    estimator_initialized = false;
    last_config_watchdog_us = 0U;
    isolation_hint_expires_us = 0U;
    config_watchdog_phase = 0U;
    state.selected_source = IMU_SOURCE_FUSED;
    state.quaternion[0] = 1.0f;
    for (uint32_t source = 0U; source < IMU_SOURCE_COUNT; ++source)
        state.custom_calibration_loaded[source] =
            imu_calibration_has_custom((imu_source_t)source);

    if (!imu_time_init())
        return false;

    init_time_us = time_us();
    state.bmi088_initialized = bmi088_init();
    state.bmi088_accel_id = bmi088_accel_chip_id();
    state.bmi088_gyro_id = bmi088_gyro_chip_id();

    state.icm45686_initialized = icm45686_init();
    state.icm45686_id = icm45686_who_am_i();
    init_time_us = time_us();

    dual_imu_estimator_config_t estimator_config;
    dual_imu_estimator_default_config(&estimator_config);
    /* One missed high-rate sample may propagate with inflated Q; two may not. */
    estimator_config.preintegrator[IMU_SOURCE_BMI088].max_accel_gap_us = 1500U;
    estimator_config.preintegrator[IMU_SOURCE_BMI088].max_gyro_gap_us = 1250U;
    estimator_config.preintegrator[IMU_SOURCE_ICM45686].max_accel_gap_us = 1500U;
    estimator_config.preintegrator[IMU_SOURCE_ICM45686].max_gyro_gap_us = 1500U;
    const uint64_t window_us =
        estimator_config.preintegrator[IMU_SOURCE_BMI088].window_us;
    const uint64_t remainder_us = init_time_us % window_us;
    const uint64_t common_epoch_us = init_time_us + window_us - remainder_us;
    estimator_initialized = dual_imu_estimator_init(&estimator,
                                                    &estimator_config,
                                                    common_epoch_us);

    return estimator_initialized &&
           (state.bmi088_initialized || state.icm45686_initialized);
}

void dual_imu_process(void)
{
    uint64_t now_us = time_us();

    read_pending_samples(now_us);
    now_us = time_us();
    run_configuration_watchdog(now_us);
    now_us = time_us();
    if ((isolation_hint_expires_us != 0U) &&
        (now_us >= isolation_hint_expires_us)) {
        dual_imu_estimator_set_isolation_hint(&estimator,
                                              IMU_SELECTOR_HINT_NONE);
        isolation_hint_expires_us = 0U;
    }
    bmi088_get_health(&state.bmi088_health);
    icm45686_get_health(&state.icm45686_health);
    const irq_snapshot_t bmi_accel_event = snapshot_irq(&bmi_accel_irq);
    const irq_snapshot_t bmi_gyro_event = snapshot_irq(&bmi_gyro_irq);
    const irq_snapshot_t icm_event = snapshot_irq(&icm_irq);
    const bool bmi_hardware_timestamp = imu_time_capture_is_running();
    const uint32_t bmi_accel_capture_overruns =
        imu_time_capture_overrun_count(1U);
    const uint32_t bmi_gyro_capture_overruns =
        imu_time_capture_overrun_count(2U);
    const bool bmi_accel_capture_overrun_new =
        bmi_accel_capture_overruns != capture_overrun_reported_count[0];
    const bool bmi_gyro_capture_overrun_new =
        bmi_gyro_capture_overruns != capture_overrun_reported_count[1];
    capture_overrun_reported_count[0] = bmi_accel_capture_overruns;
    capture_overrun_reported_count[1] = bmi_gyro_capture_overruns;

    state.bmi088_health.missed_interrupt_count =
        imu_timestamp_queue_dropped(&bmi_accel_irq.timestamps) +
        imu_timestamp_queue_dropped(&bmi_gyro_irq.timestamps) +
        bmi_accel_capture_overruns + bmi_gyro_capture_overruns;
    state.icm45686_health.missed_interrupt_count =
        imu_timestamp_queue_dropped(&icm_irq.timestamps);
    if (state.bmi088_initialized &&
        (state.bmi088_health.status == IMU_STATUS_OK) &&
        ((now_us - init_time_us) > IMU_STALE_TIMEOUT_US) &&
        !sample_is_fresh(&state.bmi088_sample, now_us))
        state.bmi088_health.status = IMU_STATUS_STALE;
    if (state.icm45686_initialized &&
        (state.icm45686_health.status == IMU_STATUS_OK) &&
        ((now_us - init_time_us) > IMU_STALE_TIMEOUT_US) &&
        !sample_is_fresh(&state.icm45686_sample, now_us))
        state.icm45686_health.status = IMU_STATUS_STALE;

    state.bmi088_fault_flags = sample_fault_flags(IMU_SOURCE_BMI088,
                                                   &state.bmi088_sample,
                                                   &state.bmi088_health,
                                                   now_us);
    state.icm45686_fault_flags = sample_fault_flags(IMU_SOURCE_ICM45686,
                                                     &state.icm45686_sample,
                                                     &state.icm45686_health,
                                                     now_us);
    /* BMI accel/gyro are independent dies; never derive gyro clipping from
       the combined sample, which may be stale when only accel has failed. */
    state.bmi088_fault_flags &= ~((uint32_t)DUAL_IMU_FAULT_SATURATED);
    if (state.bmi088_accel_sample.valid &&
        accel_vector_is_saturated(IMU_SOURCE_BMI088,
                                  state.bmi088_accel_sample.accel_mps2))
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_ACCEL_SATURATED;
    if (state.bmi088_gyro_sample.valid &&
        gyro_vector_is_saturated(state.bmi088_gyro_sample.gyro_rad_s))
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_GYRO_SATURATED;
    if (state.bmi088_accel_configuration_fault)
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_ACCEL_TIMING;
    if (state.bmi088_gyro_configuration_fault)
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_HEALTH;
    if (state.icm45686_accel_configuration_fault)
        state.icm45686_fault_flags |= DUAL_IMU_FAULT_ACCEL_TIMING;
    if (state.icm45686_gyro_configuration_fault)
        state.icm45686_fault_flags |= DUAL_IMU_FAULT_HEALTH;
    if ((BMI088_CONFIG_ACCEL_ODR_HZ > 400U) &&
        irq_is_silent(&bmi_accel_event, now_us))
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_ACCEL_TIMING;
    if (bmi_accel_capture_overrun_new)
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_ACCEL_TIMING;
    if ((BMI088_CONFIG_GYRO_ODR_HZ > 400U) &&
        irq_is_silent(&bmi_gyro_event, now_us))
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_GYRO_TIMING;
    if (bmi_gyro_capture_overrun_new)
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_GYRO_TIMING;
    if ((ICM45686_CONFIG_GYRO_ODR_HZ > 400U) &&
        irq_is_silent(&icm_event, now_us))
        state.icm45686_fault_flags |= DUAL_IMU_FAULT_GYRO_TIMING;

    update_estimator(now_us);
    update_fast_rate_output();
    if ((estimator.hard_fault_flags[IMU_SOURCE_BMI088] & (1UL << 31)) != 0U)
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_FILTER;
    if ((estimator.hard_fault_flags[IMU_SOURCE_ICM45686] & (1UL << 31)) != 0U)
        state.icm45686_fault_flags |= DUAL_IMU_FAULT_FILTER;
    state.bmi088_accel_irq_count = bmi_accel_event.count;
    state.bmi088_gyro_irq_count = bmi_gyro_event.count;
    state.icm45686_irq_count = icm_event.count;
    state.bmi088_accel_capture_overrun_count = bmi_accel_capture_overruns;
    state.bmi088_gyro_capture_overrun_count = bmi_gyro_capture_overruns;
    state.bmi088_hardware_timestamp_enabled = bmi_hardware_timestamp;
    memcpy(state.stream_read_count, successful_stream_read_count,
           sizeof(state.stream_read_count));
    memcpy(state.stream_coalesced_count, coalesced_event_count,
           sizeof(state.stream_coalesced_count));
    memcpy(state.stream_max_irq_to_read_us, maximum_irq_to_read_us,
           sizeof(state.stream_max_irq_to_read_us));
    state.bmi088_accel_event_drop_count =
        imu_timestamp_queue_dropped(&bmi_accel_irq.timestamps) +
        coalesced_event_count[IMU_SOURCE_BMI088][IMU_STREAM_ACCEL];
    state.bmi088_gyro_event_drop_count =
        imu_timestamp_queue_dropped(&bmi_gyro_irq.timestamps) +
        coalesced_event_count[IMU_SOURCE_BMI088][IMU_STREAM_GYRO];
    state.icm45686_event_drop_count =
        imu_timestamp_queue_dropped(&icm_irq.timestamps) +
        coalesced_event_count[IMU_SOURCE_ICM45686][IMU_STREAM_GYRO];
    state.bmi088_accel_buffer_overwrite_count =
        imu_accel_buffer_overwritten(&accel_buffers[IMU_SOURCE_BMI088]);
    state.bmi088_gyro_buffer_overwrite_count =
        imu_gyro_buffer_overwritten(&gyro_buffers[IMU_SOURCE_BMI088]);
    state.icm45686_accel_buffer_overwrite_count =
        imu_accel_buffer_overwritten(&accel_buffers[IMU_SOURCE_ICM45686]);
    state.icm45686_gyro_buffer_overwrite_count =
        imu_gyro_buffer_overwritten(&gyro_buffers[IMU_SOURCE_ICM45686]);
}

void dual_imu_handle_exti(uint16_t gpio_pin)
{
    if (!imu_time_is_running())
        return;

    const uint64_t timestamp = time_us();

    if (gpio_pin == BMI088_0_ADR_Pin) {
        record_irq_event_isr(&bmi_accel_irq, timestamp, 0U);
    } else if (gpio_pin == BMI088_0_GDR_Pin) {
        record_irq_event_isr(&bmi_gyro_irq, timestamp, 0U);
    } else if (gpio_pin == ICM45686_1_ADR_Pin) {
        record_irq_event_isr(&icm_irq, timestamp, 0U);
    }
}

void imu_time_capture_callback(uint32_t channel, uint64_t timestamp_us)
{
    if ((channel < 1U) || (channel > 2U))
        return;

    const uint32_t index = channel - 1U;
    const uint32_t overrun_count = imu_time_capture_overrun_count(channel);
    const uint32_t inferred_missed =
        overrun_count - capture_overrun_accounted_count[index];
    capture_overrun_accounted_count[index] = overrun_count;
    record_irq_event_isr((channel == 1U) ? &bmi_accel_irq : &bmi_gyro_irq,
                         timestamp_us,
                         inferred_missed);
}

void dual_imu_notify_impact(uint32_t inhibit_duration_us)
{
    if (!estimator_initialized)
        return;

    if (inhibit_duration_us > IMU_MAX_IMPACT_INHIBIT_US)
        inhibit_duration_us = IMU_MAX_IMPACT_INHIBIT_US;
    const uint64_t now_us = time_us();
    dual_imu_estimator_inhibit_accel_until(&estimator,
                                           now_us + inhibit_duration_us);
}

void dual_imu_set_stationary_hint(bool stationary)
{
    state.stationary_hint_active = stationary;
    if (estimator_initialized)
        dual_imu_estimator_set_stationary_hint(&estimator, stationary);
}

void dual_imu_set_isolation_hint(dual_imu_isolation_hint_t hint)
{
    if (!estimator_initialized)
        return;

    imu_selector_hint_t selector_hint = IMU_SELECTOR_HINT_NONE;
    if (hint == DUAL_IMU_HINT_BMI088_BAD)
        selector_hint = IMU_SELECTOR_HINT_LANE_0_BAD;
    else if (hint == DUAL_IMU_HINT_ICM45686_BAD)
        selector_hint = IMU_SELECTOR_HINT_LANE_1_BAD;
    dual_imu_estimator_set_isolation_hint(&estimator, selector_hint);
    isolation_hint_expires_us = (selector_hint == IMU_SELECTOR_HINT_NONE)
                                    ? 0U
                                    : time_us() + IMU_ISOLATION_HINT_TTL_US;
}

bool dual_imu_get_control_gyro(imu_gyro_sample_t *sample)
{
    if ((sample == NULL) || (state.control_gyro_timestamp_us == 0U) ||
        (state.selected_source >= IMU_SOURCE_COUNT))
        return false;

    const uint32_t source_faults =
        (state.selected_source == IMU_SOURCE_BMI088)
            ? state.bmi088_fault_flags
            : state.icm45686_fault_flags;
    if ((selector_hard_faults(state.selected_source, time_us()) != 0U) ||
        ((source_faults & DUAL_IMU_FAULT_GYRO_SATURATED) != 0U))
        return false;

    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = state.control_gyro_timestamp_us;
    memcpy(sample->gyro_rad_s, state.control_gyro_rad_s,
           sizeof(sample->gyro_rad_s));
    sample->source = state.selected_source;
    if (state.selected_source == IMU_SOURCE_BMI088) {
        sample->sequence = state.bmi088_gyro_sample.sequence;
        sample->temperature_c = state.bmi088_gyro_sample.temperature_c;
    } else {
        sample->sequence = state.icm45686_sample.gyro_sequence;
        sample->temperature_c = state.icm45686_sample.temperature_c;
    }
    sample->valid = true;
    return true;
}

bool dual_imu_set_calibration(imu_source_t source,
                              const imu_calibration_t *calibration)
{
    if (source >= IMU_SOURCE_COUNT)
        return false;
    for (uint32_t stream = 0U; stream < IMU_STREAM_COUNT; ++stream) {
        if (successful_stream_read_count[source][stream] != 0U)
            return false;
    }
    const bool accepted = imu_calibration_set(source, calibration);
    if (accepted)
        state.custom_calibration_loaded[source] = true;
    return accepted;
}

bool dual_imu_get_calibration(imu_source_t source,
                              imu_calibration_t *calibration)
{
    return imu_calibration_get(source, calibration);
}

bool dual_imu_pop_accel(imu_source_t source, imu_accel_sample_t *sample)
{
    if ((source >= IMU_SOURCE_COUNT) || (sample == NULL))
    {
        return false;
    }
    return imu_accel_buffer_pop(&accel_buffers[source], sample);
}

bool dual_imu_pop_gyro(imu_source_t source, imu_gyro_sample_t *sample)
{
    if ((source >= IMU_SOURCE_COUNT) || (sample == NULL))
    {
        return false;
    }
    return imu_gyro_buffer_pop(&gyro_buffers[source], sample);
}

const dual_imu_state_t *dual_imu_get_state(void)
{
    return &state;
}
