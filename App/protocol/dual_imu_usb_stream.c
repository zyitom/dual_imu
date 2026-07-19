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
#define DUAL_IMU_USB_FRAME_CRC_OFFSET       4U
#define DUAL_IMU_USB_FRAME_STATUS_OFFSET    (HI91_FRAME_HEADER_SIZE + 1U)

typedef struct
{
    float acceleration_mps2[3];
    uint64_t timestamp_us;
    bool valid;
} fused_accel_snapshot_t;

typedef struct
{
    const fused_accel_snapshot_t *payload;
    bool valid;
} fused_accel_lookup_t;

static uint8_t frame_queue[DUAL_IMU_USB_FRAME_QUEUE_CAPACITY][HI91_FRAME_SIZE];
static uint64_t frame_timestamp_queue[DUAL_IMU_USB_FRAME_QUEUE_CAPACITY];
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
static bool last_captured_accel_valid;
static bool captured_accel_state_available;
static dual_imu_attitude_output_t last_valid_attitude;
static bool last_valid_attitude_available;
static uint64_t reconciled_heading_loss_timestamp_us;
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

static uint16_t read_u16_le(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0] |
                      ((uint16_t)source[1] << 8U));
}

static void write_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8U);
}

static void refresh_frame_crc(uint8_t frame[HI91_FRAME_SIZE])
{
    uint16_t crc = hi91_crc16_ccitt_update(
        0U, frame, DUAL_IMU_USB_FRAME_CRC_OFFSET);
    crc = hi91_crc16_ccitt_update(
        crc, &frame[HI91_FRAME_HEADER_SIZE], HI91_FRAME_PAYLOAD_SIZE);
    write_u16_le(&frame[DUAL_IMU_USB_FRAME_CRC_OFFSET], crc);
}

static void mark_frame_heading_lost(uint8_t frame[HI91_FRAME_SIZE])
{
    uint8_t *const encoded_status =
        &frame[DUAL_IMU_USB_FRAME_STATUS_OFFSET];
    const uint16_t status = read_u16_le(encoded_status);
    /* Heading loss only raises the private bit7 diagnostic. It does NOT touch
     * the official ATT_CONV bit: heading continuity is sticky and independent
     * of tilt convergence, and the host reads bit7 to decide whether to
     * re-zero yaw. */
    const uint16_t updated =
        (uint16_t)(status | DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST);
    if (updated == status)
        return;

    write_u16_le(encoded_status, updated);
    refresh_frame_crc(frame);
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
    last_captured_accel_valid = false;
    captured_accel_state_available = false;
}

static void capture_fused_accel(const dual_imu_state_t *state)
{
    const uint64_t timestamp_us = state->fused_sample.accel_timestamp_us;
    const bool valid = state->fused_accel_valid &&
                       state->fused_sample.valid &&
                       state->fused_sample.accel_valid &&
                       vector_is_finite(state->fused_sample.accel_mps2);
    if (timestamp_us == 0U)
        return;
    if (captured_accel_state_available &&
        (timestamp_us == last_captured_accel_timestamp_us) &&
        (valid == last_captured_accel_valid)) {
        return;
    }

    fused_accel_snapshot_t *const snapshot =
        &accel_history[accel_history_head];
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->timestamp_us = timestamp_us;
    snapshot->valid = valid;
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
    last_captured_accel_valid = valid;
    captured_accel_state_available = true;
}

static fused_accel_lookup_t find_fused_accel(
    const dual_imu_attitude_output_t *attitude)
{
    fused_accel_lookup_t result = {0};
    bool causal_snapshot_found = false;
    for (uint8_t offset = 0U; offset < accel_history_count; ++offset) {
        const uint8_t index =
            (uint8_t)((accel_history_head - 1U - offset) &
                      DUAL_IMU_USB_ACCEL_HISTORY_MASK);
        const fused_accel_snapshot_t *const snapshot = &accel_history[index];
        if (snapshot->timestamp_us > attitude->output_timestamp_us)
            continue;

        const uint64_t age_us = attitude->output_timestamp_us -
                                snapshot->timestamp_us;
        if (!causal_snapshot_found) {
            causal_snapshot_found = true;
            if (!snapshot->valid)
                stream_diagnostics.accel_invalid_frame_count++;
            else if (age_us > DUAL_IMU_USB_ACCEL_MAX_AGE_US)
                stream_diagnostics.accel_stale_frame_count++;
            else {
                result.payload = snapshot;
                result.valid = true;
                return result;
            }
        }

        if ((result.payload == NULL) && snapshot->valid &&
            (age_us <= DUAL_IMU_USB_INVALID_HOLD_US)) {
            result.payload = snapshot;
        }
    }
    if (!causal_snapshot_found)
        stream_diagnostics.accel_no_causal_frame_count++;
    if (result.payload != NULL)
        stream_diagnostics.accel_held_frame_count++;
    return result;
}

static bool event_is_recent(bool seen,
                            uint64_t event_timestamp_us,
                            uint64_t frame_timestamp_us)
{
    return seen && (frame_timestamp_us >= event_timestamp_us) &&
           ((frame_timestamp_us - event_timestamp_us) <
            DUAL_IMU_USB_SATURATION_HOLD_US);
}

static bool saturation_history_is_recent(
    const imu_motion_guard_saturation_window_t
        history[IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT],
    bool seen,
    uint64_t latest_event_us,
    uint64_t frame_timestamp_us)
{
    bool history_available = false;
    for (size_t index = 0U;
         index < IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT; ++index) {
        history_available = history_available || history[index].valid;
        if (history[index].valid &&
            (frame_timestamp_us >= history[index].start_us) &&
            (frame_timestamp_us <= history[index].end_us)) {
            return true;
        }
    }
    return !history_available &&
           event_is_recent(seen, latest_event_us, frame_timestamp_us);
}

static bool selected_bias_is_converged(
    const dual_imu_state_t *state,
    const dual_imu_attitude_output_t *attitude)
{
    if (attitude->selected_source == IMU_SOURCE_BMI088)
        return state->bmi088_calibrated;
    if (attitude->selected_source == IMU_SOURCE_ICM45686)
        return state->icm45686_calibrated;
    return false;
}

static uint16_t build_status(const dual_imu_state_t *state,
                             const dual_imu_attitude_output_t *attitude,
                             bool attitude_valid,
                             bool accel_valid,
                             bool gyro_valid)
{
    uint16_t status = 0U;
    const bool heading_continuity_lost =
        state->heading_continuity_lost &&
        (state->heading_continuity_lost_timestamp_us != 0U) &&
        (attitude->output_timestamp_us >=
         state->heading_continuity_lost_timestamp_us);
    if (gyro_valid)
        status |= DUAL_IMU_USB_STATUS_PRIVATE_GYRO_VALID;
    if (accel_valid)
        status |= DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID;
    /* WB_CONV: positive polarity, gyro bias converged. */
    if (selected_bias_is_converged(state, attitude))
        status |= DUAL_IMU_USB_STATUS_WB_CONV;
    /* ACC_SAT: real accel over-range detection (1 = saturated). */
    if (attitude->accel_saturation_recent ||
        saturation_history_is_recent(
            state->motion_guard_accel_saturation_history,
            state->motion_guard_accel_saturation_seen,
            state->motion_guard_last_accel_saturation_us,
            attitude->output_timestamp_us)) {
        status |= DUAL_IMU_USB_STATUS_ACC_SAT;
    }
    /* GYR_SAT: real gyro over-range detection (1 = saturated). */
    if (attitude->gyro_saturation_recent ||
        saturation_history_is_recent(
            state->motion_guard_gyro_saturation_history,
            state->motion_guard_gyro_saturation_seen,
            state->motion_guard_last_gyro_saturation_us,
            attitude->output_timestamp_us)) {
        status |= DUAL_IMU_USB_STATUS_GYR_SAT;
    }
    /* ATT_CONV: positive polarity, tilt/attitude convergence only. It does
     * NOT include heading_continuity_lost: that flag is sticky (the estimator
     * only ever sets it, never clears it outside init), so folding it in would
     * pin ATT_CONV to 0 forever. Heading loss lives solely on the private
     * bit7; the host inspects bit7 to decide whether to re-zero yaw. */
    if (attitude_valid && attitude->attitude_converged &&
        !attitude->post_impact_reacquire_active &&
        !attitude->attitude_aiding_stale && !attitude->rotation_unobserved &&
        !attitude->euler_singular) {
        status |= DUAL_IMU_USB_STATUS_ATT_CONV;
    }
    if (heading_continuity_lost) {
        status |= DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST;
    }
    if (state->stationary_confirmed)
        status |= DUAL_IMU_USB_STATUS_PRIVATE_STATIONARY;
    if (state->bmi088_fault_flags != 0U)
        status |= DUAL_IMU_USB_STATUS_PRIVATE_BMI_FAULT;
    if (state->icm45686_fault_flags != 0U)
        status |= DUAL_IMU_USB_STATUS_PRIVATE_ICM_FAULT;
    if (stream_diagnostics.drop_sticky ||
        ((uint32_t)(state->fast_attitude_queue_drop_count -
                    fast_attitude_queue_drop_baseline) != 0U)) {
        status |= DUAL_IMU_USB_STATUS_PRIVATE_STREAM_DROP;
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
    const dual_imu_attitude_output_t *attitude_payload = NULL;
    if (attitude_valid) {
        last_valid_attitude = *attitude;
        last_valid_attitude_available = true;
        attitude_payload = attitude;
    } else if (last_valid_attitude_available &&
               (attitude->output_timestamp_us >=
                last_valid_attitude.output_timestamp_us)) {
        /* Hold the last valid attitude indefinitely: an all-zero quaternion is
         * not a legal orientation and would draw a false jump to zero on the
         * host. Validity bits (ATT_CONV) stay clear, so this is display-only. */
        attitude_payload = &last_valid_attitude;
        stream_diagnostics.attitude_held_frame_count++;
    }
    if (attitude_payload != NULL) {
        for (size_t axis = 0U; axis < 3U; ++axis)
            data->euler_deg[axis] =
                attitude_payload->euler_rad[axis] * DUAL_IMU_USB_RAD_TO_DEG;
        memcpy(data->quaternion, attitude_payload->quaternion,
               sizeof(data->quaternion));
    } else {
        /* Never had a valid attitude yet: emit the identity quaternion
         * (w=1, x=y=z=0) rather than the all-zero memset default. Euler stays
         * zero. HI91 quaternion is scalar-first (quaternion[0] = w). */
        data->quaternion[0] = 1.0f;
    }

    const fused_accel_lookup_t accel = find_fused_accel(attitude);
    if (accel.payload != NULL) {
        for (size_t axis = 0U; axis < 3U; ++axis)
            data->acceleration_g[axis] =
                accel.payload->acceleration_mps2[axis] /
                DUAL_IMU_USB_GRAVITY_MPS2;
    }

    const bool gyro_valid = attitude_valid &&
                            vector_is_finite(attitude->gyro_rate_rad_s);
    if ((attitude_payload != NULL) &&
        vector_is_finite(attitude_payload->gyro_rate_rad_s)) {
        for (size_t axis = 0U; axis < 3U; ++axis)
            data->angular_rate_deg_s[axis] =
                attitude_payload->gyro_rate_rad_s[axis] *
                DUAL_IMU_USB_RAD_TO_DEG;
    }

    data->main_status = build_status(state, attitude, attitude_valid,
                                     accel.valid, gyro_valid);
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
    frame_timestamp_queue[queue_head] = attitude->output_timestamp_us;

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

static void reconcile_heading_loss_event(const dual_imu_state_t *state)
{
    if (!state->heading_continuity_lost ||
        (state->heading_continuity_lost_timestamp_us == 0U)) {
        reconciled_heading_loss_timestamp_us = 0U;
        return;
    }

    const uint64_t event_timestamp_us =
        state->heading_continuity_lost_timestamp_us;
    if (event_timestamp_us == reconciled_heading_loss_timestamp_us)
        return;

    uint32_t patched = 0U;
    for (uint16_t offset = 0U; offset < queue_count; ++offset) {
        const uint16_t index = (uint16_t)(
            (queue_tail + offset) % DUAL_IMU_USB_FRAME_QUEUE_CAPACITY);
        if (frame_timestamp_queue[index] >= event_timestamp_us) {
            const uint16_t status_before = read_u16_le(
                &frame_queue[index][DUAL_IMU_USB_FRAME_STATUS_OFFSET]);
            mark_frame_heading_lost(frame_queue[index]);
            const uint16_t status_after = read_u16_le(
                &frame_queue[index][DUAL_IMU_USB_FRAME_STATUS_OFFSET]);
            if (status_after != status_before)
                patched++;
        }
    }
    stream_diagnostics.heading_event_queue_patch_count += patched;
    reconciled_heading_loss_timestamp_us = event_timestamp_us;
}

static bool frame_is_mature(uint64_t frame_timestamp_us,
                            uint64_t estimator_assessed_through_us)
{
    return (estimator_assessed_through_us >= frame_timestamp_us) &&
           ((estimator_assessed_through_us - frame_timestamp_us) >=
            DUAL_IMU_USB_INTEGRITY_HOLDBACK_US);
}

static void record_usb_busy(bool *busy_recorded)
{
    if (!*busy_recorded) {
        stream_diagnostics.usb_busy_count++;
        *busy_recorded = true;
    }
}

static void service_usb(const dual_imu_state_t *state,
                        bool *busy_recorded,
                        bool *maturity_wait_recorded)
{
    if (queue_count == 0U)
        return;
    if (!frame_is_mature(frame_timestamp_queue[queue_tail],
                         state->estimator_assessed_through_us)) {
        if (!*maturity_wait_recorded) {
            stream_diagnostics.maturity_wait_count++;
            *maturity_wait_recorded = true;
        }
        return;
    }
    if (CDC_TxReady_FS() == 0U) {
        record_usb_busy(busy_recorded);
        return;
    }

    uint16_t batch_frames = 0U;
    uint16_t index = queue_tail;
    while ((batch_frames < queue_count) &&
           (batch_frames < DUAL_IMU_USB_MAX_BATCH_FRAMES) &&
           frame_is_mature(frame_timestamp_queue[index],
                           state->estimator_assessed_through_us)) {
        memcpy(&transmit_batch[(size_t)batch_frames * HI91_FRAME_SIZE],
               frame_queue[index], HI91_FRAME_SIZE);
        index = (uint16_t)((index + 1U) %
                           DUAL_IMU_USB_FRAME_QUEUE_CAPACITY);
        batch_frames++;
    }

    const uint16_t batch_bytes =
        (uint16_t)(batch_frames * HI91_FRAME_SIZE);
    const uint8_t result = CDC_Transmit_FS(transmit_batch, batch_bytes);
    if (result == USBD_OK) {
        queue_tail = index;
        queue_count = (uint16_t)(queue_count - batch_frames);
        stream_diagnostics.submitted_frame_count += batch_frames;
    } else if (result == USBD_BUSY) {
        record_usb_busy(busy_recorded);
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
    memset(&last_valid_attitude, 0, sizeof(last_valid_attitude));
    last_valid_attitude_available = false;
    reconciled_heading_loss_timestamp_us = 0U;
    fast_attitude_queue_drop_baseline =
        state->fast_attitude_queue_drop_count;
    stream_diagnostics.attitude_frame_count = 0U;
    stream_diagnostics.encoded_frame_count = 0U;
    stream_diagnostics.submitted_frame_count = 0U;
    stream_diagnostics.transport_queue_drop_count = 0U;
    stream_diagnostics.usb_busy_count = 0U;
    stream_diagnostics.usb_fail_count = 0U;
    stream_diagnostics.encoder_error_count = 0U;
    stream_diagnostics.heading_event_queue_patch_count = 0U;
    stream_diagnostics.maturity_wait_count = 0U;
    stream_diagnostics.accel_no_causal_frame_count = 0U;
    stream_diagnostics.accel_invalid_frame_count = 0U;
    stream_diagnostics.accel_stale_frame_count = 0U;
    stream_diagnostics.attitude_held_frame_count = 0U;
    stream_diagnostics.accel_held_frame_count = 0U;
    stream_diagnostics.drop_sticky = false;
}

void dual_imu_usb_stream_init(void)
{
    memset(frame_queue, 0, sizeof(frame_queue));
    memset(frame_timestamp_queue, 0, sizeof(frame_timestamp_queue));
    memset(transmit_batch, 0, sizeof(transmit_batch));
    memset(&stream_diagnostics, 0, sizeof(stream_diagnostics));
    queue_head = 0U;
    queue_tail = 0U;
    queue_count = 0U;
    fast_attitude_queue_drop_baseline = 0U;
    port_was_open = false;
    reset_accel_history();
    memset(&last_valid_attitude, 0, sizeof(last_valid_attitude));
    last_valid_attitude_available = false;
    reconciled_heading_loss_timestamp_us = 0U;
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
    /* MAIN_STATUS and its checksum are repaired before any mature frame can
     * cross the USB ownership boundary. Event-before bytes remain untouched. */
    reconcile_heading_loss_event(state);
    capture_fused_accel(state);

    bool busy_recorded = false;
    bool maturity_wait_recorded = false;
    service_usb(state, &busy_recorded, &maturity_wait_recorded);
    dual_imu_attitude_output_t attitude;
    uint32_t drained = 0U;
    while ((drained < DUAL_IMU_USB_FRAME_QUEUE_CAPACITY) &&
           dual_imu_pop_attitude(&attitude)) {
        stream_diagnostics.attitude_frame_count++;
        stream_diagnostics.last_attitude_sequence = attitude.sequence;
        enqueue_attitude(state, &attitude);
        drained++;
    }
    service_usb(state, &busy_recorded, &maturity_wait_recorded);
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
