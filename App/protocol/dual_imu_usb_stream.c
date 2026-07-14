#include "dual_imu_usb_stream.h"

#include "hi91_frame.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DUAL_IMU_USB_FRAME_QUEUE_CAPACITY 32U
#define DUAL_IMU_USB_MAX_BATCH_FRAMES      4U
#define DUAL_IMU_USB_ACCEL_HISTORY_CAPACITY 8U
#define DUAL_IMU_USB_ACCEL_HISTORY_MASK     \
    (DUAL_IMU_USB_ACCEL_HISTORY_CAPACITY - 1U)
#define DUAL_IMU_USB_GRAVITY_MPS2          9.80665f
#define DUAL_IMU_USB_RAD_TO_DEG            57.29577951308232f

typedef struct
{
    float acceleration_mps2[3];
    uint64_t timestamp_us;
    bool valid;
} fused_accel_snapshot_t;

static uint8_t frame_queue[DUAL_IMU_USB_FRAME_QUEUE_CAPACITY][HI91_FRAME_SIZE];
static uint8_t transmit_batch[DUAL_IMU_USB_MAX_BATCH_FRAMES * HI91_FRAME_SIZE];
static uint16_t queue_head;
static uint16_t queue_tail;
static uint16_t queue_count;
static uint32_t fast_attitude_queue_drop_baseline;
static bool port_was_open;
static fused_accel_snapshot_t
    accel_history[DUAL_IMU_USB_ACCEL_HISTORY_CAPACITY];
static uint8_t accel_history_head;
static uint8_t accel_history_count;
static uint64_t last_captured_accel_timestamp_us;
static dual_imu_usb_stream_diagnostics_t stream_diagnostics;

_Static_assert((DUAL_IMU_USB_ACCEL_HISTORY_CAPACITY &
                DUAL_IMU_USB_ACCEL_HISTORY_MASK) == 0U,
               "accel history capacity must be a power of two");

static bool vector_is_finite(const float vector[3])
{
    return isfinite(vector[0]) && isfinite(vector[1]) && isfinite(vector[2]);
}

static bool quaternion_is_finite(const float quaternion[4])
{
    return isfinite(quaternion[0]) && isfinite(quaternion[1]) &&
           isfinite(quaternion[2]) && isfinite(quaternion[3]);
}

static int8_t temperature_to_int8(float temperature_c)
{
    if (!isfinite(temperature_c))
        return 0;
    if (temperature_c >= 127.0f)
        return INT8_MAX;
    if (temperature_c <= -128.0f)
        return INT8_MIN;

    const float rounded = temperature_c +
                          ((temperature_c >= 0.0f) ? 0.5f : -0.5f);
    return (int8_t)rounded;
}

static float selected_temperature(const dual_imu_state_t *state,
                                  imu_source_t source)
{
    if (source == IMU_SOURCE_BMI088)
        return state->bmi088_sample.temperature_c;
    if (source == IMU_SOURCE_ICM45686)
        return state->icm45686_sample.temperature_c;
    return 0.0f;
}

static void reset_accel_history(void)
{
    memset(accel_history, 0, sizeof(accel_history));
    accel_history_head = 0U;
    accel_history_count = 0U;
    last_captured_accel_timestamp_us = 0U;
}

static void capture_fused_accel(const dual_imu_state_t *state)
{
    const uint64_t timestamp_us = state->fused_sample.accel_timestamp_us;
    if ((timestamp_us == 0U) ||
        (timestamp_us == last_captured_accel_timestamp_us))
        return;

    fused_accel_snapshot_t *const snapshot =
        &accel_history[accel_history_head];
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_us = timestamp_us;
    snapshot->valid = state->fused_accel_valid &&
                      state->fused_sample.valid &&
                      state->fused_sample.accel_valid &&
                      vector_is_finite(state->fused_sample.accel_mps2);
    if (snapshot->valid) {
        memcpy(snapshot->acceleration_mps2,
               state->fused_sample.accel_mps2,
               sizeof(snapshot->acceleration_mps2));
    }

    accel_history_head =
        (uint8_t)((accel_history_head + 1U) &
                  DUAL_IMU_USB_ACCEL_HISTORY_MASK);
    if (accel_history_count < DUAL_IMU_USB_ACCEL_HISTORY_CAPACITY)
        accel_history_count++;
    last_captured_accel_timestamp_us = timestamp_us;
}

static const fused_accel_snapshot_t *find_fused_accel(
    const dual_imu_attitude_output_t *attitude)
{
    for (uint8_t offset = 0U; offset < accel_history_count; ++offset) {
        const uint8_t index =
            (uint8_t)((accel_history_head - 1U - offset) &
                      DUAL_IMU_USB_ACCEL_HISTORY_MASK);
        const fused_accel_snapshot_t *const snapshot = &accel_history[index];
        if (snapshot->timestamp_us > attitude->output_timestamp_us)
            continue;
        if (!snapshot->valid) {
            stream_diagnostics.accel_invalid_frame_count++;
            return NULL;
        }

        const uint64_t age_us = attitude->output_timestamp_us -
                                snapshot->timestamp_us;
        if (age_us > DUAL_IMU_USB_ACCEL_MAX_AGE_US) {
            stream_diagnostics.accel_stale_frame_count++;
            return NULL;
        }
        return snapshot;
    }
    stream_diagnostics.accel_no_causal_frame_count++;
    return NULL;
}

static uint16_t build_status(const dual_imu_state_t *state,
                             const dual_imu_attitude_output_t *attitude,
                             bool attitude_valid,
                             bool accel_valid,
                             bool gyro_valid)
{
    uint16_t status = DUAL_IMU_USB_STATUS_YAW_RELATIVE;
    if (attitude_valid)
        status |= DUAL_IMU_USB_STATUS_ATTITUDE_VALID;
    if (accel_valid)
        status |= DUAL_IMU_USB_STATUS_ACCEL_VALID;
    if (gyro_valid)
        status |= DUAL_IMU_USB_STATUS_GYRO_VALID;
    if (attitude->degraded)
        status |= DUAL_IMU_USB_STATUS_DEGRADED;
    if (attitude->deadline_miss)
        status |= DUAL_IMU_USB_STATUS_DEADLINE_MISS;
    if (attitude->euler_singular)
        status |= DUAL_IMU_USB_STATUS_EULER_SINGULAR;
    if (state->bmi088_initialized && state->icm45686_initialized)
        status |= DUAL_IMU_USB_STATUS_BOTH_INITIALIZED;
    if (state->selector_state == IMU_SELECTOR_HEALTHY)
        status |= DUAL_IMU_USB_STATUS_SELECTOR_HEALTHY;
    if (state->bmi088_calibrated)
        status |= DUAL_IMU_USB_STATUS_BMI_BIAS_CONVERGED;
    if (state->icm45686_calibrated)
        status |= DUAL_IMU_USB_STATUS_ICM_BIAS_CONVERGED;
    if (attitude->selected_source == IMU_SOURCE_ICM45686)
        status |= DUAL_IMU_USB_STATUS_SELECTED_ICM;
    if (state->stationary_confirmed)
        status |= DUAL_IMU_USB_STATUS_STATIONARY;
    if (state->bmi088_fault_flags != 0U)
        status |= DUAL_IMU_USB_STATUS_BMI_FAULT;
    if (state->icm45686_fault_flags != 0U)
        status |= DUAL_IMU_USB_STATUS_ICM_FAULT;
    if (stream_diagnostics.drop_sticky ||
        ((uint32_t)(state->fast_attitude_queue_drop_count -
                    fast_attitude_queue_drop_baseline) != 0U)) {
        status |= DUAL_IMU_USB_STATUS_STREAM_DROP;
    }
    return status;
}

static void fill_frame_data(const dual_imu_state_t *state,
                            const dual_imu_attitude_output_t *attitude,
                            hi91_frame_data_t *data)
{
    memset(data, 0, sizeof(*data));
    data->system_time_ms = (uint32_t)(attitude->output_timestamp_us / 1000U);
    data->temperature_c = temperature_to_int8(
        selected_temperature(state, attitude->selected_source));

    const bool attitude_valid = attitude->valid &&
                                vector_is_finite(attitude->euler_rad) &&
                                quaternion_is_finite(attitude->quaternion);
    if (attitude_valid) {
        for (size_t axis = 0U; axis < 3U; ++axis)
            data->euler_deg[axis] =
                attitude->euler_rad[axis] * DUAL_IMU_USB_RAD_TO_DEG;
        memcpy(data->quaternion, attitude->quaternion,
               sizeof(data->quaternion));
    }

    const fused_accel_snapshot_t *const accel = find_fused_accel(attitude);
    const bool accel_valid = accel != NULL;
    if (accel != NULL) {
        for (size_t axis = 0U; axis < 3U; ++axis)
            data->acceleration_g[axis] =
                accel->acceleration_mps2[axis] /
                DUAL_IMU_USB_GRAVITY_MPS2;
    }

    const bool gyro_valid = attitude_valid &&
                            vector_is_finite(attitude->gyro_rate_rad_s);
    if (gyro_valid) {
        for (size_t axis = 0U; axis < 3U; ++axis)
            data->angular_rate_deg_s[axis] =
                attitude->gyro_rate_rad_s[axis] * DUAL_IMU_USB_RAD_TO_DEG;
    }

    data->main_status = build_status(state, attitude, attitude_valid,
                                     accel_valid, gyro_valid);
}

static void enqueue_attitude(const dual_imu_state_t *state,
                             const dual_imu_attitude_output_t *attitude)
{
    if (queue_count == DUAL_IMU_USB_FRAME_QUEUE_CAPACITY) {
        queue_tail = (uint16_t)((queue_tail + 1U) %
                                DUAL_IMU_USB_FRAME_QUEUE_CAPACITY);
        queue_count--;
        stream_diagnostics.transport_queue_drop_count++;
        stream_diagnostics.drop_sticky = true;
    }

    hi91_frame_data_t data;
    fill_frame_data(state, attitude, &data);
    if (hi91_frame_encode(frame_queue[queue_head], HI91_FRAME_SIZE, &data) !=
        HI91_FRAME_SIZE) {
        stream_diagnostics.encoder_error_count++;
        return;
    }

    queue_head = (uint16_t)((queue_head + 1U) %
                            DUAL_IMU_USB_FRAME_QUEUE_CAPACITY);
    queue_count++;
    stream_diagnostics.encoded_frame_count++;
}

static void discard_transport_queue(bool record_drop)
{
    if (record_drop) {
        stream_diagnostics.transport_queue_drop_count += queue_count;
        if (queue_count != 0U)
            stream_diagnostics.drop_sticky = true;
    }
    queue_tail = queue_head;
    queue_count = 0U;
}

static void service_usb(void)
{
    if (queue_count == 0U)
        return;

    uint16_t batch_frames = queue_count;
    if (batch_frames > DUAL_IMU_USB_MAX_BATCH_FRAMES)
        batch_frames = DUAL_IMU_USB_MAX_BATCH_FRAMES;
    uint16_t index = queue_tail;
    for (uint16_t frame = 0U; frame < batch_frames; ++frame) {
        memcpy(&transmit_batch[(size_t)frame * HI91_FRAME_SIZE],
               frame_queue[index], HI91_FRAME_SIZE);
        index = (uint16_t)((index + 1U) %
                           DUAL_IMU_USB_FRAME_QUEUE_CAPACITY);
    }

    const uint16_t batch_bytes =
        (uint16_t)(batch_frames * HI91_FRAME_SIZE);
    const uint8_t result = CDC_Transmit_FS(transmit_batch, batch_bytes);
    if (result == USBD_OK) {
        queue_tail = index;
        queue_count = (uint16_t)(queue_count - batch_frames);
        stream_diagnostics.submitted_frame_count += batch_frames;
    } else if (result == USBD_BUSY) {
        stream_diagnostics.usb_busy_count++;
    } else {
        stream_diagnostics.usb_fail_count++;
        discard_transport_queue(true);
    }
}

static void drain_unpublished_attitudes(void)
{
    dual_imu_attitude_output_t attitude;
    while (dual_imu_pop_attitude(&attitude))
        stream_diagnostics.last_attitude_sequence = attitude.sequence;
}

static void begin_port_session(const dual_imu_state_t *state)
{
    discard_transport_queue(false);
    reset_accel_history();
    fast_attitude_queue_drop_baseline =
        state->fast_attitude_queue_drop_count;
    stream_diagnostics.attitude_frame_count = 0U;
    stream_diagnostics.encoded_frame_count = 0U;
    stream_diagnostics.submitted_frame_count = 0U;
    stream_diagnostics.transport_queue_drop_count = 0U;
    stream_diagnostics.usb_busy_count = 0U;
    stream_diagnostics.usb_fail_count = 0U;
    stream_diagnostics.encoder_error_count = 0U;
    stream_diagnostics.accel_no_causal_frame_count = 0U;
    stream_diagnostics.accel_invalid_frame_count = 0U;
    stream_diagnostics.accel_stale_frame_count = 0U;
    stream_diagnostics.drop_sticky = false;
}

void dual_imu_usb_stream_init(void)
{
    memset(frame_queue, 0, sizeof(frame_queue));
    memset(transmit_batch, 0, sizeof(transmit_batch));
    memset(&stream_diagnostics, 0, sizeof(stream_diagnostics));
    queue_head = 0U;
    queue_tail = 0U;
    queue_count = 0U;
    fast_attitude_queue_drop_baseline = 0U;
    port_was_open = false;
    reset_accel_history();
}

void dual_imu_usb_stream_process(const dual_imu_state_t *state)
{
    if (state == NULL)
        return;

    const bool port_open = CDC_IsPortOpen_FS() != 0U;
    stream_diagnostics.port_open = port_open;
    if (!port_open) {
        discard_transport_queue(false);
        drain_unpublished_attitudes();
        port_was_open = false;
        stream_diagnostics.queued_frame_count = 0U;
        return;
    }
    if (!port_was_open)
        begin_port_session(state);
    port_was_open = true;
    capture_fused_accel(state);

    service_usb();
    dual_imu_attitude_output_t attitude;
    uint32_t drained = 0U;
    while ((drained < DUAL_IMU_USB_FRAME_QUEUE_CAPACITY) &&
           dual_imu_pop_attitude(&attitude)) {
        stream_diagnostics.attitude_frame_count++;
        stream_diagnostics.last_attitude_sequence = attitude.sequence;
        enqueue_attitude(state, &attitude);
        drained++;
    }
    service_usb();
    stream_diagnostics.queued_frame_count = queue_count;
}

void dual_imu_usb_stream_get_diagnostics(
    dual_imu_usb_stream_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) {
        *diagnostics = stream_diagnostics;
        diagnostics->queued_frame_count = queue_count;
    }
}
