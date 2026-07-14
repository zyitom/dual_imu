#include "dual_imu.h"

#include "bmi088.h"
#include "dual_imu_estimator.h"
#include "fast_attitude_predictor.h"
#include "icm45686.h"
#include "imu_calibration.h"
#include "imu_spi_dma.h"
#include "imu_stream_buffer.h"
#include "imu_time.h"
#include "main.h"

#include <math.h>
#include <string.h>

#define IMU_IRQ_FALLBACK_DELAY_US     10000U
#define IMU_STALE_TIMEOUT_US          10000U
#define IMU_GRAVITY_MPS2               9.80665f
#define IMU_FROZEN_SAMPLE_LIMIT          8U
#define IMU_SATURATION_FRACTION         0.98f
#define BMI088_ACCEL_RANGE_MPS2        (24.0f * IMU_GRAVITY_MPS2)
#define ICM45686_ACCEL_RANGE_MPS2      (32.0f * IMU_GRAVITY_MPS2)
#define BMI088_GYRO_RANGE_RAD_S        (2000.0f * 0.017453292519943295f)
#define ICM45686_GYRO_RANGE_RAD_S      (4000.0f * 0.017453292519943295f)
#define IMU_MAX_EVENTS_PER_PROCESS        32U
#define IMU_MAX_WINDOWS_PER_PROCESS         1U
#define IMU_MAX_ICM_FRAMES_PER_PROCESS      64U
#define IMU_ATTITUDE_OUTPUT_QUEUE_CAPACITY   32U
#define IMU_ATTITUDE_OUTPUT_QUEUE_MASK       \
    (IMU_ATTITUDE_OUTPUT_QUEUE_CAPACITY - 1U)
#define IMU_MAX_IMPACT_INHIBIT_US       1000000U
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
static fast_attitude_predictor_t fast_predictor;
static dual_imu_attitude_output_t fast_output_buffers[2];
static volatile uint32_t fast_output_buffer_index;
static dual_imu_attitude_output_t
    fast_output_queue[IMU_ATTITUDE_OUTPUT_QUEUE_CAPACITY];
static volatile uint32_t fast_output_queue_head;
static volatile uint32_t fast_output_queue_tail;
static volatile uint32_t fast_output_queue_drop_count;
static sample_monitor_t sample_monitors[IMU_SOURCE_COUNT];
static irq_event_t bmi_accel_irq;
static irq_event_t bmi_gyro_irq;
static irq_event_t icm_irq;
static imu_accel_buffer_t accel_buffers[IMU_SOURCE_COUNT];
static imu_gyro_buffer_t gyro_buffers[IMU_SOURCE_COUNT];
static uint32_t bmi_combined_sequence;
static uint64_t init_time_us;
static uint8_t read_error_streak[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint64_t estimator_stream_timestamp_us[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t coalesced_event_count[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t published_stream_sequence[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t last_irq_event_sequence[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t successful_stream_read_count[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t maximum_irq_to_read_us[IMU_SOURCE_COUNT][IMU_STREAM_COUNT];
static uint32_t capture_overrun_accounted_count[2];
static uint32_t capture_overrun_reported_count[2];
static bool bmi_capture_accepting;
static uint64_t isolation_hint_expires_us;
static bool estimator_initialized;
static bool fast_predictor_initialized;
static uint32_t fast_output_sequence;
static uint32_t fast_tick_missed_compare_count;
static uint32_t fast_tick_dropped_count;

_Static_assert((IMU_ATTITUDE_OUTPUT_QUEUE_CAPACITY &
                IMU_ATTITUDE_OUTPUT_QUEUE_MASK) == 0U,
               "attitude output queue capacity must be a power of two");

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

static void reset_irq_event(irq_event_t *event)
{
    if (event == NULL)
        return;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    imu_timestamp_queue_reset(&event->timestamps);
    event->count = 0U;
    event->timestamp_us = 0U;
    __set_PRIMASK(primask);
}

static uint64_t compensate_pipeline_delay(uint64_t timestamp_us, uint32_t delay_us)
{
    return (timestamp_us > delay_us) ? (timestamp_us - delay_us) : 0U;
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

static bool gyro_vector_is_saturated(imu_source_t source,
                                     const float gyro_rad_s[3])
{
    const float range = (source == IMU_SOURCE_BMI088)
                            ? BMI088_GYRO_RANGE_RAD_S
                            : ICM45686_GYRO_RANGE_RAD_S;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        if (fabsf(gyro_rad_s[axis]) >=
            (IMU_SATURATION_FRACTION * range))
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
        gyro_vector_is_saturated(source, sample->gyro_rad_s))
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
    if ((state.bmi088_accel_sample.timestamp_us == 0U) ||
        (state.bmi088_gyro_sample.timestamp_us == 0U)) {
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
    sample->valid = sample->accel_valid && sample->gyro_valid;
}

static void account_fifo_event(imu_source_t source,
                               uint32_t stream,
                               uint32_t event_sequence)
{
    const uint32_t previous = last_irq_event_sequence[source][stream];
    if (previous != 0U) {
        const uint32_t delta = event_sequence - previous;
        if ((delta > 1U) && (delta < UINT32_C(0x80000000)))
            coalesced_event_count[source][stream] += delta - 1U;
    }
    last_irq_event_sequence[source][stream] = event_sequence;
}

static void assign_fifo_sequence(imu_source_t source,
                                 uint32_t stream,
                                 uint32_t *sequence)
{
    if (*sequence == 0U) {
        *sequence = ++published_stream_sequence[source][stream];
        return;
    }

    published_stream_sequence[source][stream] = *sequence;
}

static void record_fifo_sample(imu_source_t source,
                               uint32_t stream,
                               uint64_t sample_timestamp_us,
                               uint64_t arrival_timestamp_us,
                               bool timestamp_trusted)
{
    successful_stream_read_count[source][stream]++;
    if (!timestamp_trusted || (arrival_timestamp_us < sample_timestamp_us))
        return;

    const uint64_t latency_us = arrival_timestamp_us - sample_timestamp_us;
    const uint32_t bounded_latency = (latency_us > UINT32_MAX)
                                         ? UINT32_MAX
                                         : (uint32_t)latency_us;
    if (bounded_latency > maximum_irq_to_read_us[source][stream])
        maximum_irq_to_read_us[source][stream] = bounded_latency;
}

static void ingest_accel_sample(imu_accel_sample_t *sample,
                                bool timestamp_trusted,
                                uint64_t arrival_timestamp_us)
{
    if ((sample == NULL) || (sample->source >= IMU_SOURCE_COUNT))
        return;

    const imu_source_t source = sample->source;
    const bool timestamp_causal = (sample->timestamp_us != 0U) &&
                                  (arrival_timestamp_us >=
                                   sample->timestamp_us);
    timestamp_trusted = timestamp_trusted && timestamp_causal;
    sample->valid = sample->valid && timestamp_causal;
    assign_fifo_sequence(source, IMU_STREAM_ACCEL, &sample->sequence);
    record_fifo_sample(source, IMU_STREAM_ACCEL, sample->timestamp_us,
                       arrival_timestamp_us, timestamp_trusted);
    rotate_accel_pitch_180(sample);
    sample->valid = sample->valid && (sample->timestamp_us != 0U) &&
                    imu_calibration_apply_accel(source,
                                                sample->temperature_c,
                                                sample->accel_mps2);
    if (sample->valid)
        update_accel_monitor(source, sample->accel_mps2);
    imu_accel_buffer_push(&accel_buffers[source], sample);

    if (estimator_initialized && timestamp_trusted) {
        imu_accel_sample_t corrected = *sample;
        const uint32_t pipeline_delay_us =
            (source == IMU_SOURCE_BMI088)
                ? BMI088_ACCEL_PIPELINE_DELAY_US
                : ICM45686_ACCEL_PIPELINE_DELAY_US;
        corrected.timestamp_us = compensate_pipeline_delay(
            corrected.timestamp_us, pipeline_delay_us);
        corrected.valid = corrected.valid &&
                          estimator_accel_sample_is_eligible(
                              source, timestamp_trusted) &&
                          !accel_vector_is_saturated(source,
                                                     corrected.accel_mps2);
        if (dual_imu_estimator_push_accel(&estimator, &corrected))
            estimator_stream_timestamp_us[source][IMU_STREAM_ACCEL] =
                corrected.timestamp_us;
    }
}

static void ingest_gyro_sample(imu_gyro_sample_t *sample,
                               bool timestamp_trusted,
                               uint64_t arrival_timestamp_us)
{
    if ((sample == NULL) || (sample->source >= IMU_SOURCE_COUNT))
        return;

    const imu_source_t source = sample->source;
    const bool timestamp_causal = (sample->timestamp_us != 0U) &&
                                  (arrival_timestamp_us >=
                                   sample->timestamp_us);
    timestamp_trusted = timestamp_trusted && timestamp_causal;
    sample->valid = sample->valid && timestamp_causal;
    assign_fifo_sequence(source, IMU_STREAM_GYRO, &sample->sequence);
    record_fifo_sample(source, IMU_STREAM_GYRO, sample->timestamp_us,
                       arrival_timestamp_us, timestamp_trusted);
    rotate_gyro_pitch_180(sample);
    sample->valid = sample->valid && (sample->timestamp_us != 0U) &&
                    imu_calibration_apply_gyro(source,
                                               sample->temperature_c,
                                               sample->gyro_rad_s);
    if (sample->valid)
        update_gyro_monitor(source, sample->gyro_rad_s);
    imu_gyro_buffer_push(&gyro_buffers[source], sample);

    if (estimator_initialized && timestamp_trusted) {
        imu_gyro_sample_t corrected = *sample;
        const uint32_t pipeline_delay_us =
            (source == IMU_SOURCE_BMI088)
                ? BMI088_GYRO_PIPELINE_DELAY_US
                : ICM45686_GYRO_PIPELINE_DELAY_US;
        corrected.timestamp_us = compensate_pipeline_delay(
            corrected.timestamp_us, pipeline_delay_us);
        corrected.valid = corrected.valid &&
                          estimator_gyro_sample_is_eligible(
                              source, timestamp_trusted) &&
                          !gyro_vector_is_saturated(source,
                                                    corrected.gyro_rad_s);
        if (fast_predictor_initialized) {
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            (void)fast_attitude_predictor_push_gyro(&fast_predictor,
                                                    &corrected);
            __set_PRIMASK(primask);
        }
        if (dual_imu_estimator_push_gyro(&estimator, &corrected))
            estimator_stream_timestamp_us[source][IMU_STREAM_GYRO] =
                corrected.timestamp_us;
    }
}

static bool bmi_accel_batch_timestamp_is_trusted(
    const bmi088_accel_fifo_batch_t *batch)
{
    const uint32_t rejected = BMI088_FIFO_BATCH_FLAG_SKIPPED |
                              BMI088_FIFO_BATCH_FLAG_OVERRUN |
                              BMI088_FIFO_BATCH_FLAG_TRUNCATED |
                              BMI088_FIFO_BATCH_FLAG_PARSE_ERROR |
                              BMI088_FIFO_BATCH_FLAG_TIMESTAMP_ESTIMATED;
    return ((batch->flags & BMI088_FIFO_BATCH_FLAG_SENSOR_TIME_VALID) != 0U) &&
           ((batch->flags & rejected) == 0U);
}

static bool bmi_gyro_batch_timestamp_is_trusted(
    const bmi088_gyro_fifo_batch_t *batch)
{
    const uint32_t rejected = BMI088_FIFO_BATCH_FLAG_OVERRUN |
                              BMI088_FIFO_BATCH_FLAG_TRUNCATED |
                              BMI088_FIFO_BATCH_FLAG_PARSE_ERROR |
                              BMI088_FIFO_BATCH_FLAG_TIMESTAMP_ESTIMATED |
                              BMI088_FIFO_BATCH_FLAG_CAPTURE_MISMATCH;
    return (batch->latest_capture_timestamp_us != 0U) &&
           ((batch->flags & rejected) == 0U);
}

static void ingest_bmi_fifo_batches(void)
{
    static bmi088_accel_fifo_batch_t accel_batch;
    static bmi088_gyro_fifo_batch_t gyro_batch;

    if (bmi088_fifo_pop_accel_batch(&accel_batch)) {
        const bool batch_trusted =
            bmi_accel_batch_timestamp_is_trusted(&accel_batch);
        for (uint16_t index = 0U; index < accel_batch.count; ++index) {
            imu_accel_sample_t sample = accel_batch.samples[index];
            ingest_accel_sample(&sample,
                                batch_trusted &&
                                    (sample.timestamp_us != 0U),
                                accel_batch.transfer_complete_timestamp_us);
            state.bmi088_accel_sample = sample;
            publish_bmi_sample();
        }
    }

    if (bmi088_fifo_pop_gyro_batch(&gyro_batch)) {
        const bool batch_trusted =
            bmi_gyro_batch_timestamp_is_trusted(&gyro_batch);
        for (uint16_t index = 0U; index < gyro_batch.count; ++index) {
            imu_gyro_sample_t sample = gyro_batch.samples[index];
            ingest_gyro_sample(&sample,
                               batch_trusted &&
                                   (sample.timestamp_us != 0U),
                               gyro_batch.transfer_complete_timestamp_us);
            state.bmi088_gyro_sample = sample;
            publish_bmi_sample();
        }
    }
}

static void ingest_icm_fifo_frame(const icm45686_fifo_frame_t *frame,
                                  uint64_t arrival_timestamp_us)
{
    if ((frame == NULL) || !frame->timestamp_valid ||
        !frame->mcu_timestamp_valid || (frame->mcu_timestamp_us == 0U)) {
        return;
    }

    const uint64_t timestamp_us = frame->mcu_timestamp_us;
    if (frame->temperature_valid)
        state.icm45686_sample.temperature_c = frame->temperature_c;
    state.icm45686_sample.source = IMU_SOURCE_ICM45686;

    if (frame->accel_valid) {
        imu_accel_sample_t accel = {
            .timestamp_us = timestamp_us,
            .temperature_c = state.icm45686_sample.temperature_c,
            .source = IMU_SOURCE_ICM45686,
            .valid = true,
        };
        memcpy(accel.accel_mps2, frame->accel_mps2,
               sizeof(accel.accel_mps2));
        ingest_accel_sample(&accel, true, arrival_timestamp_us);
        state.icm45686_sample.accel_timestamp_us = accel.timestamp_us;
        state.icm45686_sample.accel_sequence = accel.sequence;
        memcpy(state.icm45686_sample.accel_mps2, accel.accel_mps2,
               sizeof(accel.accel_mps2));
        state.icm45686_sample.accel_valid = accel.valid;
    }

    if (frame->gyro_valid) {
        imu_gyro_sample_t gyro = {
            .timestamp_us = timestamp_us,
            .temperature_c = state.icm45686_sample.temperature_c,
            .source = IMU_SOURCE_ICM45686,
            .valid = true,
        };
        memcpy(gyro.gyro_rad_s, frame->gyro_rad_s,
               sizeof(gyro.gyro_rad_s));
        ingest_gyro_sample(&gyro, true, arrival_timestamp_us);
        state.icm45686_sample.timestamp_us = gyro.timestamp_us;
        state.icm45686_sample.gyro_timestamp_us = gyro.timestamp_us;
        state.icm45686_sample.sequence = gyro.sequence;
        state.icm45686_sample.gyro_sequence = gyro.sequence;
        memcpy(state.icm45686_sample.gyro_rad_s, gyro.gyro_rad_s,
               sizeof(gyro.gyro_rad_s));
        state.icm45686_sample.gyro_valid = gyro.valid;
    }

    state.icm45686_sample.valid = state.icm45686_sample.accel_valid &&
                                  state.icm45686_sample.gyro_valid;
}

static void submit_fifo_requests(void)
{
    uint64_t timestamp_us;
    uint32_t sequence;
    uint32_t count = 0U;

    while ((count < IMU_MAX_EVENTS_PER_PROCESS) &&
           imu_timestamp_queue_pop_event(&bmi_accel_irq.timestamps,
                                         &timestamp_us, &sequence)) {
        account_fifo_event(IMU_SOURCE_BMI088, IMU_STREAM_ACCEL, sequence);
        (void)bmi088_fifo_request_accel(timestamp_us);
        count++;
    }

    count = 0U;
    while ((count < IMU_MAX_EVENTS_PER_PROCESS) &&
           imu_timestamp_queue_pop_event(&bmi_gyro_irq.timestamps,
                                         &timestamp_us, &sequence)) {
        account_fifo_event(IMU_SOURCE_BMI088, IMU_STREAM_GYRO, sequence);
        /* Per-sample timestamps are already in the ISR-owned capture ring.
         * This event only requests a FIFO_STATUS/FIFO_DATA DMA transaction. */
        (void)bmi088_fifo_request_gyro(timestamp_us);
        count++;
    }

    count = 0U;
    while (state.icm45686_initialized &&
           (count < IMU_MAX_EVENTS_PER_PROCESS) &&
           imu_timestamp_queue_pop_event(&icm_irq.timestamps,
                                         &timestamp_us, &sequence)) {
        account_fifo_event(IMU_SOURCE_ICM45686, IMU_STREAM_ACCEL, sequence);
        account_fifo_event(IMU_SOURCE_ICM45686, IMU_STREAM_GYRO, sequence);
        (void)icm45686_fifo_request(timestamp_us);
        count++;
    }
}

static void service_fifo_data(uint64_t now_us)
{
    imu_spi_dma_service();
    submit_fifo_requests();
    bmi088_fifo_service(now_us);
    icm45686_fifo_service();
    ingest_bmi_fifo_batches();

    icm45686_fifo_frame_t frame;
    uint32_t frame_count = 0U;
    while ((frame_count < IMU_MAX_ICM_FRAMES_PER_PROCESS) &&
           icm45686_fifo_pop_frame(&frame)) {
        ingest_icm_fifo_frame(&frame, time_us());
        frame_count++;
    }

    /* Popping the single-slot BMI batches releases SPI1 for the next queued
     * die without waiting for another main-loop iteration. */
    bmi088_fifo_service(time_us());
    icm45686_fifo_service();
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
    /* A missing/stale window is already excluded by selector window_valid.
     * It is not attributable evidence of a permanent lane failure and must
     * not enter the selector's hard-latched mask. */
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
    state.stationary_last_reject_reason =
        (uint8_t)output->stationary_last_reject_reason;
    state.stationary_streak = output->stationary_streak;
    state.stationary_max_streak = output->stationary_max_streak;
    memcpy(state.stationary_temporal_gyro_variance_rad2_s2,
           output->stationary_temporal_gyro_variance_rad2_s2,
           sizeof(state.stationary_temporal_gyro_variance_rad2_s2));
    memcpy(state.stationary_temporal_accel_variance_m2_s4,
           output->stationary_temporal_accel_variance_m2_s4,
           sizeof(state.stationary_temporal_accel_variance_m2_s4));
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
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        fast_attitude_predictor_invalidate_anchor(&fast_predictor);
        __set_PRIMASK(primask);
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

    bool anchor_accepted = false;
    if (fast_predictor_initialized &&
        output->lane_window[selected].gyro_valid) {
        fast_attitude_anchor_t anchor;
        memset(&anchor, 0, sizeof(anchor));
        anchor.timestamp_us = output->end_us;
        anchor.selected_source = selected;
        anchor.degraded = output->output_alignment_active ||
                          (output->selector.state != IMU_SELECTOR_HEALTHY);
        memcpy(anchor.quaternion, output->quaternion,
               sizeof(anchor.quaternion));
        memcpy(anchor.gyro_rate_rad_s,
               output->lane_window[selected].gyro_end_rad_s,
               sizeof(anchor.gyro_rate_rad_s));
        memcpy(anchor.gyro_bias_rad_s,
               output->lane_bias_rad_s[selected],
               sizeof(anchor.gyro_bias_rad_s));
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        anchor_accepted =
            fast_attitude_predictor_set_anchor(&fast_predictor, &anchor);
        if (!anchor_accepted)
            fast_attitude_predictor_invalidate_anchor(&fast_predictor);
        __set_PRIMASK(primask);
    }
    else if (fast_predictor_initialized) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        fast_attitude_predictor_invalidate_anchor(&fast_predictor);
        __set_PRIMASK(primask);
    }

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
    memcpy(state.mekf_quaternion, output->quaternion,
           sizeof(state.mekf_quaternion));
    memcpy(state.mekf_euler_rad, output->euler_rad,
           sizeof(state.mekf_euler_rad));
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

static void fast_attitude_queue_push_isr(
    const dual_imu_attitude_output_t *output)
{
    const uint32_t head = fast_output_queue_head;
    const uint32_t next = (head + 1U) & IMU_ATTITUDE_OUTPUT_QUEUE_MASK;

    if (next == fast_output_queue_tail) {
        fast_output_queue_drop_count++;
        state.fast_attitude_queue_drop_count = fast_output_queue_drop_count;
        return;
    }

    fast_output_queue[head] = *output;
    __DMB();
    fast_output_queue_head = next;
}

static void publish_fast_attitude_at(uint64_t scheduled_us)
{
    if (!fast_predictor_initialized)
        return;

    dual_imu_attitude_output_t output;
    (void)fast_attitude_predictor_predict(&fast_predictor,
                                          scheduled_us,
                                          &output);
    output.sequence = ++fast_output_sequence;

    const uint32_t next_buffer = fast_output_buffer_index ^ 1U;
    fast_output_buffers[next_buffer] = output;
    dual_imu_attitude_output_t *const published =
        &fast_output_buffers[next_buffer];
    const uint64_t publish_us = time_us();
    published->publish_timestamp_us = publish_us;
    const uint64_t latency_us = (publish_us >= scheduled_us)
                                    ? (publish_us - scheduled_us)
                                    : 0U;
    published->publish_latency_us =
        (latency_us > UINT32_MAX) ? UINT32_MAX : (uint32_t)latency_us;
    if (latency_us > DUAL_IMU_ATTITUDE_PUBLISH_DEADLINE_US) {
        published->deadline_miss = true;
        published->valid = false;
        published->integrity_flags |= FAST_ATTITUDE_FLAG_DEADLINE_MISS;
    }
    __DMB();
    fast_output_buffer_index = next_buffer;
    fast_attitude_queue_push_isr(published);

    state.fast_attitude_output_count++;
    state.fast_attitude_last_integrity_flags = published->integrity_flags;
    state.fast_attitude_last_publish_latency_us =
        published->publish_latency_us;
    if (published->publish_latency_us >
        state.fast_attitude_max_publish_latency_us) {
        state.fast_attitude_max_publish_latency_us =
            published->publish_latency_us;
    }
    state.fast_attitude_last_prediction_horizon_us =
        published->prediction_horizon_us;
    if (published->valid) {
        memcpy(state.quaternion, published->quaternion,
               sizeof(state.quaternion));
        memcpy(state.euler_rad, published->euler_rad,
               sizeof(state.euler_rad));
    } else {
        state.fast_attitude_invalid_count++;
    }
}

void imu_time_fast_tick_callback(uint64_t scheduled_us)
{
    publish_fast_attitude_at(scheduled_us);
}

static void update_fast_tick_diagnostics(void)
{
    imu_time_fast_tick_diagnostics_t diagnostics;
    imu_time_fast_tick_get_diagnostics(&diagnostics);

    const uint32_t missed_delta =
        diagnostics.missed_compare_count - fast_tick_missed_compare_count;
    const uint32_t dropped_delta =
        diagnostics.dropped_tick_count - fast_tick_dropped_count;
    fast_tick_missed_compare_count = diagnostics.missed_compare_count;
    fast_tick_dropped_count = diagnostics.dropped_tick_count;
    state.fast_attitude_missed_tick_count += missed_delta + dropped_delta;
    state.fast_attitude_compare_event_count = diagnostics.compare_event_count;
    state.fast_attitude_compare_missed_count =
        diagnostics.missed_compare_count;
    state.fast_attitude_tick_drop_count = diagnostics.dropped_tick_count;
    state.fast_attitude_scheduler_running = diagnostics.running;
}

bool dual_imu_init(void)
{
    imu_calibration_initialize();
    memset(&state, 0, sizeof(state));
    memset(&estimator, 0, sizeof(estimator));
    memset(&estimator_output, 0, sizeof(estimator_output));
    memset(&fast_predictor, 0, sizeof(fast_predictor));
    memset(fast_output_buffers, 0, sizeof(fast_output_buffers));
    memset(fast_output_queue, 0, sizeof(fast_output_queue));
    memset(sample_monitors, 0, sizeof(sample_monitors));
    memset(&bmi_accel_irq, 0, sizeof(bmi_accel_irq));
    memset(&bmi_gyro_irq, 0, sizeof(bmi_gyro_irq));
    memset(&icm_irq, 0, sizeof(icm_irq));
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
    estimator_initialized = false;
    fast_predictor_initialized = false;
    fast_output_sequence = 0U;
    fast_tick_missed_compare_count = 0U;
    fast_tick_dropped_count = 0U;
    bmi_capture_accepting = false;
    fast_output_buffer_index = 0U;
    fast_output_queue_head = 0U;
    fast_output_queue_tail = 0U;
    fast_output_queue_drop_count = 0U;
    isolation_hint_expires_us = 0U;
    state.selected_source = IMU_SOURCE_FUSED;
    for (uint32_t index = 0U; index < 2U; ++index) {
        fast_output_buffers[index].selected_source = IMU_SOURCE_FUSED;
        fast_output_buffers[index].integrity_flags =
            FAST_ATTITUDE_FLAG_NO_ANCHOR;
        fast_output_buffers[index].degraded = true;
    }
    state.mekf_quaternion[0] = 1.0f;
    state.quaternion[0] = 1.0f;
    for (uint32_t source = 0U; source < IMU_SOURCE_COUNT; ++source)
        state.custom_calibration_loaded[source] =
            imu_calibration_has_custom((imu_source_t)source);

    if (!imu_time_init())
        return false;
    if (!imu_spi_dma_init())
        return false;

    fast_attitude_predictor_config_t predictor_config;
    fast_attitude_predictor_default_config(&predictor_config);
    predictor_config.max_prediction_horizon_us =
        DUAL_IMU_ATTITUDE_MAX_PREDICTION_HORIZON_US;
    fast_predictor_initialized =
        fast_attitude_predictor_init(&fast_predictor, &predictor_config);
    if (!fast_predictor_initialized)
        return false;

    /* ICM startup contains the long blocking settle delay. Run it before BMI
     * starts so the 2 kHz BMI capture queue cannot overflow during init. */
    state.icm45686_initialized = icm45686_init();
    state.icm45686_id = icm45686_who_am_i();

    if (!imu_time_start_capture_channels_1_2())
        return false;
    bmi_capture_accepting = true;
    state.bmi088_initialized = bmi088_init();
    if (!state.bmi088_initialized)
        bmi_capture_accepting = false;
    state.bmi088_accel_id = bmi088_accel_chip_id();
    state.bmi088_gyro_id = bmi088_gyro_chip_id();
    if (state.bmi088_initialized)
    {
        bmi_capture_accepting = false;
        reset_irq_event(&bmi_gyro_irq);
        capture_overrun_accounted_count[1] = 0U;
        capture_overrun_reported_count[1] = 0U;
        if (!imu_time_capture_reset_channel(2U))
        {
            state.bmi088_initialized = false;
        }
        else
        {
            bmi_capture_accepting = true;
            if (!bmi088_fifo_start_gyro())
            {
                bmi_capture_accepting = false;
                state.bmi088_initialized = false;
            }
        }
    }
    if (state.icm45686_initialized && !icm45686_fifo_flush())
        state.icm45686_initialized = false;
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

    const bool fast_tick_started = estimator_initialized &&
        imu_time_fast_tick_start(DUAL_IMU_ATTITUDE_OUTPUT_PERIOD_US);
    return fast_tick_started &&
           (state.bmi088_initialized || state.icm45686_initialized);
}

bool dual_imu_start_streaming(void)
{
    if (!state.icm45686_initialized)
        return state.bmi088_initialized;

    /* In EQ watermark mode the first pulse occurs only once, at count==WM.
     * The caller enables EXTI immediately after this flush, well before the
     * second 3.2 kHz packet can be written. */
    if (!icm45686_fifo_flush()) {
        state.icm45686_initialized = false;
        return false;
    }

    init_time_us = time_us();
    return true;
}

void dual_imu_process(void)
{
    uint64_t now_us = time_us();

    update_fast_tick_diagnostics();
    const uint64_t fifo_service_start_us = time_us();
    service_fifo_data(now_us);
    const uint64_t fifo_service_duration_us =
        time_us() - fifo_service_start_us;
    state.fifo_service_last_us =
        (fifo_service_duration_us > UINT32_MAX)
            ? UINT32_MAX
            : (uint32_t)fifo_service_duration_us;
    if (state.fifo_service_last_us > state.fifo_service_max_us)
        state.fifo_service_max_us = state.fifo_service_last_us;
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
        gyro_vector_is_saturated(IMU_SOURCE_BMI088,
                                 state.bmi088_gyro_sample.gyro_rad_s))
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_GYRO_SATURATED;
    if (state.bmi088_accel_configuration_fault)
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_ACCEL_TIMING;
    if (state.bmi088_gyro_configuration_fault)
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_HEALTH;
    if (bmi088_fifo_gyro_sync_faulted())
        state.bmi088_fault_flags |= DUAL_IMU_FAULT_GYRO_TIMING;
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

    const uint64_t estimator_start_us = time_us();
    update_estimator(now_us);
    const uint64_t estimator_duration_us = time_us() - estimator_start_us;
    state.estimator_process_last_us =
        (estimator_duration_us > UINT32_MAX)
            ? UINT32_MAX
            : (uint32_t)estimator_duration_us;
    if (state.estimator_process_last_us > state.estimator_process_max_us)
        state.estimator_process_max_us = state.estimator_process_last_us;
    update_fast_rate_output();
    update_fast_tick_diagnostics();
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

    bmi088_fifo_diagnostics_t bmi_fifo_diagnostics;
    icm45686_fifo_diagnostics_t icm_fifo_diagnostics;
    imu_clock_sync_diagnostics_t bmi_clock_diagnostics;
    bmi088_fifo_get_diagnostics(&bmi_fifo_diagnostics);
    bmi088_fifo_get_clock_sync_diagnostics(&bmi_clock_diagnostics);
    icm45686_fifo_get_diagnostics(&icm_fifo_diagnostics);
    state.bmi088_accel_fifo_batch_count =
        bmi_fifo_diagnostics.accel_batch_count;
    state.bmi088_gyro_fifo_batch_count =
        bmi_fifo_diagnostics.gyro_batch_count;
    state.bmi088_fifo_dma_error_count =
        bmi_fifo_diagnostics.dma_error_count;
    state.bmi088_accel_fifo_length_read_count =
        bmi_fifo_diagnostics.accel_length_read_count;
    state.bmi088_accel_fifo_empty_length_count =
        bmi_fifo_diagnostics.accel_empty_length_count;
    state.bmi088_accel_timeline_reset_count =
        bmi_fifo_diagnostics.accel_timeline_reset_count;
    state.bmi088_gyro_capture_mismatch_count =
        bmi_fifo_diagnostics.gyro_capture_mismatch_count;
    state.bmi088_gyro_capture_queue_overflow_count =
        bmi_fifo_diagnostics.gyro_capture_queue_overflow_count;
    state.bmi088_gyro_capture_mismatch_reason =
        bmi_fifo_diagnostics.gyro_capture_mismatch_reason;
    state.bmi088_gyro_warmup_discard_count =
        bmi_fifo_diagnostics.gyro_warmup_discard_count;
    state.bmi088_gyro_capture_sync_fault =
        bmi_fifo_diagnostics.gyro_capture_sync_fault;
    state.bmi088_accel_fifo_last_bytes =
        bmi_fifo_diagnostics.accel_last_fifo_bytes;
    state.bmi088_accel_fifo_peak_bytes =
        bmi_fifo_diagnostics.accel_peak_fifo_bytes;
    memcpy(state.bmi088_accel_fifo_length_response,
           bmi_fifo_diagnostics.accel_last_length_response,
           sizeof(state.bmi088_accel_fifo_length_response));
    memcpy(state.bmi088_accel_register_snapshot,
           bmi_fifo_diagnostics.accel_register_snapshot,
           sizeof(state.bmi088_accel_register_snapshot));
    state.bmi088_accel_sensor_time_before =
        bmi_fifo_diagnostics.accel_sensor_time_before;
    state.bmi088_accel_sensor_time_after =
        bmi_fifo_diagnostics.accel_sensor_time_after;
    memcpy(state.bmi088_accel_direct_raw,
           bmi_fifo_diagnostics.accel_direct_raw,
           sizeof(state.bmi088_accel_direct_raw));
    state.bmi088_accel_initial_fifo_bytes =
        bmi_fifo_diagnostics.accel_initial_fifo_bytes;
    state.bmi088_accel_snapshot_valid =
        bmi_fifo_diagnostics.accel_snapshot_valid;
    state.bmi088_fifo_clock_sync_valid = bmi_clock_diagnostics.valid;
    state.fifo_clock_anchor_accepted_count[IMU_SOURCE_BMI088] =
        bmi_clock_diagnostics.accepted_anchor_count;
    state.fifo_clock_anchor_rejected_count[IMU_SOURCE_BMI088] =
        bmi_clock_diagnostics.rejected_anchor_count;
    state.fifo_clock_reference_mcu_us[IMU_SOURCE_BMI088] =
        (bmi_clock_diagnostics.last_reference_mcu_us > 0.0)
            ? (uint64_t)(bmi_clock_diagnostics.last_reference_mcu_us + 0.5)
            : 0U;
    state.fifo_clock_scale[IMU_SOURCE_BMI088] =
        (float)bmi_clock_diagnostics.clock_scale;
    state.fifo_clock_residual_sigma_us[IMU_SOURCE_BMI088] =
        (float)bmi_clock_diagnostics.residual_sigma_us;
    state.fifo_clock_reject_reason_count[IMU_SOURCE_BMI088][0] =
        bmi_clock_diagnostics.nonmonotonic_reject_count;
    state.fifo_clock_reject_reason_count[IMU_SOURCE_BMI088][1] =
        bmi_clock_diagnostics.interval_reject_count;
    state.fifo_clock_reject_reason_count[IMU_SOURCE_BMI088][2] =
        bmi_clock_diagnostics.slope_reject_count;
    state.fifo_clock_reject_reason_count[IMU_SOURCE_BMI088][3] =
        bmi_clock_diagnostics.residual_reject_count;
    state.fifo_clock_last_observed_scale[IMU_SOURCE_BMI088] =
        (float)bmi_clock_diagnostics.last_observed_clock_scale;
    state.fifo_clock_last_residual_us[IMU_SOURCE_BMI088] =
        (float)bmi_clock_diagnostics.last_residual_us;
    state.fifo_clock_stale_reset_count[IMU_SOURCE_BMI088] =
        bmi_fifo_diagnostics.accel_clock_stale_reset_count;
    state.fifo_clock_causal_reset_count[IMU_SOURCE_BMI088] =
        bmi_fifo_diagnostics.accel_clock_causal_reset_count;
    state.icm45686_fifo_batch_count = icm_fifo_diagnostics.dma_batch_count;
    state.icm45686_fifo_dma_error_count =
        icm_fifo_diagnostics.dma_error_count;
    state.icm45686_fifo_clock_sync_valid =
        icm_fifo_diagnostics.clock_sync_valid;
    state.fifo_clock_anchor_accepted_count[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_anchor_accepted_count;
    state.fifo_clock_anchor_rejected_count[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_anchor_rejected_count;
    state.fifo_clock_reference_mcu_us[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_reference_mcu_us;
    state.fifo_clock_scale[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_scale;
    state.fifo_clock_residual_sigma_us[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_residual_sigma_us;
    state.fifo_clock_reject_reason_count[IMU_SOURCE_ICM45686][0] =
        icm_fifo_diagnostics.clock_nonmonotonic_reject_count;
    state.fifo_clock_reject_reason_count[IMU_SOURCE_ICM45686][1] =
        icm_fifo_diagnostics.clock_interval_reject_count;
    state.fifo_clock_reject_reason_count[IMU_SOURCE_ICM45686][2] =
        icm_fifo_diagnostics.clock_slope_reject_count;
    state.fifo_clock_reject_reason_count[IMU_SOURCE_ICM45686][3] =
        icm_fifo_diagnostics.clock_residual_reject_count;
    state.fifo_clock_last_observed_scale[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_last_observed_scale;
    state.fifo_clock_last_residual_us[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_last_residual_us;
    state.fifo_clock_stale_reset_count[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_stale_reset_count;
    state.fifo_clock_causal_reset_count[IMU_SOURCE_ICM45686] =
        icm_fifo_diagnostics.clock_causal_reset_count;
    state.icm45686_timestamp_discontinuity_count =
        icm_fifo_diagnostics.timestamp_discontinuity_count;
    state.icm45686_fifo_last_frames = icm_fifo_diagnostics.last_fifo_count;
    state.icm45686_fifo_peak_frames = icm_fifo_diagnostics.peak_fifo_count;
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
    } else if ((gpio_pin == ICM45686_1_ADR_Pin) &&
               state.icm45686_initialized) {
        record_irq_event_isr(&icm_irq, timestamp, 0U);
    }
}

void imu_time_capture_callback(uint32_t channel, uint64_t timestamp_us)
{
    if ((channel < 1U) || (channel > 2U) || !bmi_capture_accepting)
        return;

    const uint32_t index = channel - 1U;
    const uint32_t overrun_count = imu_time_capture_overrun_count(channel);
    const uint32_t inferred_missed =
        overrun_count - capture_overrun_accounted_count[index];
    capture_overrun_accounted_count[index] = overrun_count;
    irq_event_t *const event =
        (channel == 1U) ? &bmi_accel_irq : &bmi_gyro_irq;
    record_irq_event_isr(event, timestamp_us, inferred_missed);
    if (channel == 2U)
        bmi088_fifo_capture_gyro_isr(timestamp_us, event->count);
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

bool dual_imu_get_attitude(dual_imu_attitude_output_t *output)
{
    if (output == NULL)
        return false;

    uint32_t index;
    do {
        index = fast_output_buffer_index;
        __DMB();
        *output = fast_output_buffers[index];
        __DMB();
    } while (index != fast_output_buffer_index);

    if (output->valid) {
        const uint64_t now_us = time_us();
        const uint64_t maximum_age_us =
            DUAL_IMU_ATTITUDE_OUTPUT_PERIOD_US +
            DUAL_IMU_ATTITUDE_PUBLISH_DEADLINE_US;
        if ((now_us < output->output_timestamp_us) ||
            ((now_us - output->output_timestamp_us) > maximum_age_us)) {
            output->integrity_flags |= FAST_ATTITUDE_FLAG_OUTPUT_STALE;
            output->degraded = true;
            output->valid = false;
        }
    }
    return output->valid;
}

bool dual_imu_pop_attitude(dual_imu_attitude_output_t *output)
{
    if (output == NULL)
        return false;

    const uint32_t tail = fast_output_queue_tail;
    if (tail == fast_output_queue_head)
        return false;

    __DMB();
    *output = fast_output_queue[tail];
    __DMB();
    fast_output_queue_tail =
        (tail + 1U) & IMU_ATTITUDE_OUTPUT_QUEUE_MASK;
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
