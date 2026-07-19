#include "dual_imu_usb_stream.h"
#include "hi91_frame.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_ATTITUDE_CAPACITY 64U
#define TEST_USB_CAPTURE_SIZE 2048U

static unsigned int failure_count;
static dual_imu_attitude_output_t attitude_input[TEST_ATTITUDE_CAPACITY];
static uint32_t attitude_head;
static uint32_t attitude_tail;
static uint8_t cdc_result;
static uint8_t cdc_capture[TEST_USB_CAPTURE_SIZE];
static uint8_t cdc_attempt_capture[TEST_USB_CAPTURE_SIZE];
static uint16_t cdc_capture_length;
static uint16_t cdc_attempt_capture_length;
static uint32_t cdc_call_count;
static bool cdc_port_open;
static bool cdc_tx_ready;
static bool cdc_auto_busy_after_submit;

#define TEST_EXPECT(condition)                                                   \
    do {                                                                         \
        if (!(condition)) {                                                       \
            (void)fprintf(stderr, "%s:%d: expectation failed: %s\n",          \
                          __FILE__, __LINE__, #condition);                        \
            failure_count++;                                                      \
        }                                                                         \
    } while (0)

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static float read_float_le(const uint8_t *data)
{
    const uint32_t bits = read_u32_le(data);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint16_t captured_status(uint16_t frame_index)
{
    return read_u16_le(&cdc_capture[(size_t)frame_index * HI91_FRAME_SIZE +
                                    7U]);
}

static bool captured_crc_is_valid(uint16_t frame_index)
{
    const uint8_t *const frame =
        &cdc_capture[(size_t)frame_index * HI91_FRAME_SIZE];
    uint16_t crc = hi91_crc16_ccitt_update(0U, frame, 4U);
    crc = hi91_crc16_ccitt_update(
        crc, &frame[HI91_FRAME_HEADER_SIZE], HI91_FRAME_PAYLOAD_SIZE);
    return crc == read_u16_le(&frame[4]);
}

static bool nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void reset_fakes(void)
{
    memset(attitude_input, 0, sizeof(attitude_input));
    memset(cdc_capture, 0, sizeof(cdc_capture));
    memset(cdc_attempt_capture, 0, sizeof(cdc_attempt_capture));
    attitude_head = 0U;
    attitude_tail = 0U;
    cdc_result = USBD_OK;
    cdc_capture_length = 0U;
    cdc_attempt_capture_length = 0U;
    cdc_call_count = 0U;
    cdc_port_open = true;
    cdc_tx_ready = true;
    cdc_auto_busy_after_submit = false;
}

static void initialize_state(dual_imu_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->estimator_assessed_through_us = UINT64_MAX;
}

static void push_attitude(dual_imu_attitude_output_t attitude)
{
    TEST_EXPECT(attitude_head < TEST_ATTITUDE_CAPACITY);
    if (attitude_head < TEST_ATTITUDE_CAPACITY)
        attitude_input[attitude_head++] = attitude;
}

bool dual_imu_pop_attitude(dual_imu_attitude_output_t *output)
{
    if ((output == NULL) || (attitude_tail == attitude_head))
        return false;
    *output = attitude_input[attitude_tail++];
    return true;
}

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
    cdc_call_count++;
    if ((buffer != NULL) && (length <= sizeof(cdc_attempt_capture))) {
        memcpy(cdc_attempt_capture, buffer, length);
        cdc_attempt_capture_length = length;
    }
    if ((cdc_result == USBD_OK) && (buffer != NULL) &&
        (length <= sizeof(cdc_capture))) {
        memcpy(cdc_capture, buffer, length);
        cdc_capture_length = length;
        if (cdc_auto_busy_after_submit)
            cdc_tx_ready = false;
    }
    return cdc_result;
}

uint8_t CDC_IsPortOpen_FS(void)
{
    return cdc_port_open ? 1U : 0U;
}

uint8_t CDC_TxReady_FS(void)
{
    return cdc_tx_ready ? 1U : 0U;
}

static dual_imu_attitude_output_t make_attitude(uint32_t sequence)
{
    dual_imu_attitude_output_t attitude;
    memset(&attitude, 0, sizeof(attitude));
    attitude.sequence = sequence;
    attitude.output_timestamp_us = UINT64_C(1234567) + sequence;
    attitude.selected_source = IMU_SOURCE_ICM45686;
    attitude.quaternion[0] = 1.0f;
    attitude.euler_rad[0] = 0.5f * TEST_PI;
    attitude.euler_rad[1] = -0.25f * TEST_PI;
    attitude.euler_rad[2] = 0.125f * TEST_PI;
    attitude.gyro_rate_rad_s[0] = TEST_PI;
    attitude.gyro_rate_rad_s[1] = -0.5f * TEST_PI;
    attitude.attitude_converged = true;
    attitude.valid = true;
    return attitude;
}

static void test_field_mapping_and_status(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();

    dual_imu_state_t state;
    initialize_state(&state);
    state.bmi088_initialized = true;
    state.icm45686_initialized = true;
    state.bmi088_calibrated = true;
    state.icm45686_calibrated = true;
    state.attitude_converged = true;
    state.selector_state = IMU_SELECTOR_HEALTHY;
    state.stationary_confirmed = true;
    state.fused_accel_valid = true;
    state.fused_sample.valid = true;
    state.fused_sample.accel_valid = true;
    state.fused_sample.accel_timestamp_us =
        make_attitude(7U).output_timestamp_us - 1000U;
    state.fused_sample.accel_mps2[0] = 9.80665f;
    state.fused_sample.accel_mps2[1] = -4.903325f;
    state.icm45686_sample.temperature_c = 35.4f;

    push_attitude(make_attitude(7U));
    dual_imu_usb_stream_process(&state);

    TEST_EXPECT(cdc_call_count == 1U);
    TEST_EXPECT(cdc_capture_length == HI91_FRAME_SIZE);
    TEST_EXPECT(cdc_capture[0] == 0x5AU && cdc_capture[1] == 0xA5U);
    TEST_EXPECT(read_u16_le(&cdc_capture[2]) == HI91_FRAME_PAYLOAD_SIZE);
    TEST_EXPECT(cdc_capture[6] == HI91_FRAME_TAG);
    const uint16_t expected_status =
        DUAL_IMU_USB_STATUS_PRIVATE_GYRO_VALID |
        DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID |
        DUAL_IMU_USB_STATUS_PRIVATE_STATIONARY |
        DUAL_IMU_USB_STATUS_ATT_CONV |
        DUAL_IMU_USB_STATUS_WB_CONV;
    TEST_EXPECT(read_u16_le(&cdc_capture[7]) == expected_status);
    /* Unsupported official HI91 bits must be a constant 0 on this hardware. */
    const uint16_t unsupported_official =
        DUAL_IMU_USB_STATUS_OD | DUAL_IMU_USB_STATUS_SOUT_PULSE |
        DUAL_IMU_USB_STATUS_UTC_TIME | DUAL_IMU_USB_STATUS_MAG_AIDING |
        DUAL_IMU_USB_STATUS_MAG_DIST;
    TEST_EXPECT((read_u16_le(&cdc_capture[7]) & unsupported_official) == 0U);
    /* Regression: a stationary board must NOT set the official GYR_SAT (bit9)
     * or ACC_SAT (bit10). The old layout aliased STATIONARY onto bit9. */
    TEST_EXPECT((read_u16_le(&cdc_capture[7]) &
                 (DUAL_IMU_USB_STATUS_GYR_SAT |
                  DUAL_IMU_USB_STATUS_ACC_SAT)) == 0U);
    TEST_EXPECT((int8_t)cdc_capture[9] == INT8_C(35));
    TEST_EXPECT(read_u32_le(&cdc_capture[14]) == UINT32_C(1234));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 1.0f, 1.0e-6f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[22]), -0.5f, 1.0e-6f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[30]), 180.0f, 1.0e-4f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[34]), -90.0f, 1.0e-4f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[54]), 90.0f, 1.0e-4f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[58]), -45.0f, 1.0e-4f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[62]), 22.5f, 1.0e-4f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[66]), 1.0f, 1.0e-6f));
    for (size_t byte = 10U; byte < 14U; ++byte)
        TEST_EXPECT(cdc_capture[byte] == 0U);
    for (size_t byte = 42U; byte < 54U; ++byte)
        TEST_EXPECT(cdc_capture[byte] == 0U);

    uint16_t crc = hi91_crc16_ccitt_update(0U, cdc_capture, 4U);
    crc = hi91_crc16_ccitt_update(crc, &cdc_capture[6],
                                  HI91_FRAME_PAYLOAD_SIZE);
    TEST_EXPECT(read_u16_le(&cdc_capture[4]) == crc);

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.attitude_frame_count == 1U);
    TEST_EXPECT(diagnostics.encoded_frame_count == 1U);
    TEST_EXPECT(diagnostics.submitted_frame_count == 1U);
    TEST_EXPECT(diagnostics.last_attitude_sequence == 7U);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
}

static void test_invalid_data_emits_identity_quaternion_when_never_valid(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.bmi088_initialized = true;
    state.icm45686_initialized = true;

    dual_imu_attitude_output_t attitude = make_attitude(1U);
    attitude.valid = false;
    attitude.deadline_miss = true;
    attitude.quaternion[0] = NAN;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);

    const uint16_t status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_GYRO_VALID) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_WB_CONV) == 0U);
    /* bit11 (MAG_DIST) must no longer be a constant 1 as the old
     * UTC_UNSYNC bit was. */
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_MAG_DIST) == 0U);

    /* No valid attitude has ever been seen: the quaternion field must carry
     * the identity (w=1, x=y=z=0), never an all-zero non-orientation. */
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[66]), 1.0f, 1.0e-6f));
    /* Accel, gyro and euler stay zero; only quaternion[0] is non-zero. */
    for (size_t byte = 18U; byte < 66U; ++byte)
        TEST_EXPECT(cdc_capture[byte] == 0U);
    for (size_t byte = 70U; byte < HI91_FRAME_SIZE; ++byte)
        TEST_EXPECT(cdc_capture[byte] == 0U);
}

static void test_brief_invalid_frame_holds_payload_but_clears_validity(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;
    state.attitude_converged = true;
    state.fused_accel_valid = true;
    state.fused_sample.valid = true;
    state.fused_sample.accel_valid = true;

    dual_imu_attitude_output_t valid = make_attitude(10U);
    state.fused_sample.accel_timestamp_us = valid.output_timestamp_us;
    state.fused_sample.accel_mps2[0] = 9.80665f;
    push_attitude(valid);
    dual_imu_usb_stream_process(&state);

    dual_imu_attitude_output_t invalid = make_attitude(11U);
    invalid.output_timestamp_us = valid.output_timestamp_us + 625U;
    invalid.valid = false;
    state.fused_accel_valid = false;
    state.fused_sample.valid = true;
    state.fused_sample.accel_valid = false;
    state.fused_sample.accel_timestamp_us = invalid.output_timestamp_us;
    push_attitude(invalid);
    dual_imu_usb_stream_process(&state);

    const uint16_t status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_GYRO_VALID) == 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 1.0f, 1.0e-6f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[30]), 180.0f, 1.0e-4f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[54]), 90.0f, 1.0e-4f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[66]), 1.0f, 1.0e-6f));

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.attitude_held_frame_count == 1U);
    TEST_EXPECT(diagnostics.accel_held_frame_count == 1U);
}

static void test_invalid_past_old_window_keeps_holding_attitude(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;
    state.attitude_converged = true;
    dual_imu_attitude_output_t valid = make_attitude(1U);
    push_attitude(valid);
    dual_imu_usb_stream_process(&state);

    dual_imu_attitude_output_t invalid = make_attitude(2U);
    invalid.valid = false;
    /* Well beyond the former 20 ms invalid-hold cap: the attitude must still
     * be held (never an all-zero quaternion) so the host does not draw a jump
     * to zero. ATT_CONV must stay clear because the frame is invalid. */
    invalid.output_timestamp_us = valid.output_timestamp_us +
                                  (10U * DUAL_IMU_USB_INVALID_HOLD_US);
    push_attitude(invalid);
    dual_imu_usb_stream_process(&state);

    const uint16_t status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_GYRO_VALID) == 0U);
    /* Held orientation is still present: quaternion[0]=1, euler[0]=90 deg. */
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[66]), 1.0f, 1.0e-6f));
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[54]), 90.0f, 1.0e-4f));

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.attitude_held_frame_count == 1U);
}

static void test_busy_queue_drop_and_disconnect_flush(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    cdc_result = USBD_BUSY;

    for (uint32_t sequence = 1U; sequence <= 40U; ++sequence)
        push_attitude(make_attitude(sequence));
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_process(&state);

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.attitude_frame_count == 40U);
    TEST_EXPECT(diagnostics.transport_queue_drop_count == 8U);
    TEST_EXPECT(diagnostics.queued_frame_count == 32U);
    TEST_EXPECT(diagnostics.drop_sticky);
    TEST_EXPECT(diagnostics.usb_busy_count >= 2U);

    cdc_result = USBD_FAIL;
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
    TEST_EXPECT(diagnostics.transport_queue_drop_count == 40U);
    TEST_EXPECT(diagnostics.usb_fail_count == 1U);
}

static void test_closed_port_discards_without_a_drop(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    cdc_port_open = false;
    push_attitude(make_attitude(1U));
    dual_imu_usb_stream_process(&state);

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 0U);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
    TEST_EXPECT(diagnostics.transport_queue_drop_count == 0U);
    TEST_EXPECT(!diagnostics.drop_sticky);
    TEST_EXPECT(!diagnostics.port_open);

    cdc_port_open = true;
    push_attitude(make_attitude(2U));
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 1U);
    TEST_EXPECT(diagnostics.attitude_frame_count == 1U);
    TEST_EXPECT(diagnostics.submitted_frame_count == 1U);
    TEST_EXPECT(!diagnostics.drop_sticky);
    TEST_EXPECT(diagnostics.port_open);
}

static void test_not_ready_skips_transmit_and_retries(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    cdc_tx_ready = false;

    push_attitude(make_attitude(1U));
    push_attitude(make_attitude(2U));
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_process(&state);

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 0U);
    TEST_EXPECT(cdc_capture_length == 0U);
    TEST_EXPECT(diagnostics.queued_frame_count == 2U);
    TEST_EXPECT(diagnostics.submitted_frame_count == 0U);
    TEST_EXPECT(diagnostics.usb_busy_count == 2U);

    cdc_tx_ready = true;
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 1U);
    TEST_EXPECT(cdc_capture_length == 2U * HI91_FRAME_SIZE);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
    TEST_EXPECT(diagnostics.submitted_frame_count == 2U);
    TEST_EXPECT(diagnostics.usb_busy_count == 2U);
}

static void test_backlog_uses_each_attitude_gyro(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.control_gyro_rad_s[0] = 99.0f;

    dual_imu_attitude_output_t first = make_attitude(1U);
    dual_imu_attitude_output_t second = make_attitude(2U);
    first.gyro_rate_rad_s[0] = 1.0f;
    second.gyro_rate_rad_s[0] = 2.0f;
    push_attitude(first);
    push_attitude(second);
    dual_imu_usb_stream_process(&state);

    TEST_EXPECT(cdc_capture_length == 2U * HI91_FRAME_SIZE);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[30]),
                             1.0f * (180.0f / TEST_PI), 1.0e-4f));
    TEST_EXPECT(nearly_equal(
        read_float_le(&cdc_capture[HI91_FRAME_SIZE + 30U]),
        2.0f * (180.0f / TEST_PI), 1.0e-4f));
}

static void test_future_accel_is_zeroed_and_stale_accel_is_held(void)
{
    dual_imu_state_t state;
    dual_imu_attitude_output_t attitude;
    uint16_t status;

    reset_fakes();
    dual_imu_usb_stream_init();
    initialize_state(&state);
    attitude = make_attitude(1U);
    state.fused_accel_valid = true;
    state.fused_sample.valid = true;
    state.fused_sample.accel_valid = true;
    state.fused_sample.accel_mps2[0] = 9.80665f;
    state.fused_sample.accel_timestamp_us = attitude.output_timestamp_us + 1U;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);
    status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID) == 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 0.0f, 0.0f));

    reset_fakes();
    dual_imu_usb_stream_init();
    initialize_state(&state);
    attitude = make_attitude(2U);
    state.fused_accel_valid = true;
    state.fused_sample.valid = true;
    state.fused_sample.accel_valid = true;
    state.fused_sample.accel_mps2[0] = 9.80665f;
    state.fused_sample.accel_timestamp_us =
        attitude.output_timestamp_us - (DUAL_IMU_USB_ACCEL_MAX_AGE_US + 1U);
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);
    status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID) == 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 1.0f, 1.0e-6f));
}

static void test_accel_history_selects_latest_causal_sample(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.fused_accel_valid = true;
    state.fused_sample.valid = true;
    state.fused_sample.accel_valid = true;
    state.fused_sample.accel_timestamp_us = 9000U;
    state.fused_sample.accel_mps2[0] = 9.80665f;
    dual_imu_usb_stream_process(&state);

    state.fused_sample.accel_timestamp_us = 11000U;
    state.fused_sample.accel_mps2[0] = 2.0f * 9.80665f;
    dual_imu_attitude_output_t attitude = make_attitude(1U);
    attitude.output_timestamp_us = 10000U;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);

    const uint16_t status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID) != 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 1.0f, 1.0e-6f));
}

static void test_accel_validity_transition_is_captured_without_new_timestamp(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.fused_accel_valid = true;
    state.fused_sample.valid = true;
    state.fused_sample.accel_valid = true;
    state.fused_sample.accel_timestamp_us = 9000U;
    state.fused_sample.accel_mps2[0] = 9.80665f;
    dual_imu_usb_stream_process(&state);

    state.fused_accel_valid = false;
    state.fused_sample.valid = false;
    dual_imu_attitude_output_t attitude = make_attitude(1U);
    attitude.output_timestamp_us = 10000U;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);

    const uint16_t status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_ACCEL_VALID) == 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 1.0f, 1.0e-6f));
}

static void test_producer_drop_status_is_session_relative(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.fast_attitude_queue_drop_count = 17U;

    push_attitude(make_attitude(1U));
    dual_imu_usb_stream_process(&state);
    uint16_t status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_STREAM_DROP) == 0U);

    state.fast_attitude_queue_drop_count++;
    push_attitude(make_attitude(2U));
    dual_imu_usb_stream_process(&state);
    status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_STREAM_DROP) != 0U);
}

static void test_convergence_status_uses_active_low_quality_semantics(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);

    dual_imu_attitude_output_t attitude = make_attitude(1U);
    state.icm45686_calibrated = false;
    attitude.attitude_converged = false;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);
    uint16_t status = captured_status(0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_WB_CONV) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);

    state.icm45686_calibrated = true;
    attitude = make_attitude(2U);
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);
    status = captured_status(0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_WB_CONV) != 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);

    attitude = make_attitude(3U);
    attitude.post_impact_reacquire_active = true;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);
    status = captured_status(0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);

    attitude = make_attitude(4U);
    attitude.euler_singular = true;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);
    status = captured_status(0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);
}

static void test_convergence_status_is_frame_causal_across_backlog(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;

    /* The encoder-time state is deliberately the opposite of the first
     * queued frame. Each status must follow its scheduled attitude frame. */
    state.attitude_converged = false;
    state.post_impact_reacquire_active = true;
    dual_imu_attitude_output_t old_good = make_attitude(1U);
    dual_imu_attitude_output_t new_bad = make_attitude(2U);
    new_bad.attitude_converged = false;
    new_bad.post_impact_reacquire_active = true;
    push_attitude(old_good);
    push_attitude(new_bad);
    dual_imu_usb_stream_process(&state);

    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);
    TEST_EXPECT((captured_status(1U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);

    reset_fakes();
    dual_imu_usb_stream_init();
    initialize_state(&state);
    state.icm45686_calibrated = true;
    state.attitude_converged = true;
    dual_imu_attitude_output_t old_bad = make_attitude(3U);
    dual_imu_attitude_output_t new_good = make_attitude(4U);
    old_bad.attitude_aiding_stale = true;
    push_attitude(old_bad);
    push_attitude(new_good);
    dual_imu_usb_stream_process(&state);

    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);
    TEST_EXPECT((captured_status(1U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);
}

static void test_heading_loss_sets_private_bit_without_touching_att_conv(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;
    state.heading_continuity_lost = true;
    state.heading_continuity_lost_timestamp_us = UINT64_C(5000000);

    dual_imu_attitude_output_t before = make_attitude(1U);
    dual_imu_attitude_output_t at_event = make_attitude(2U);
    before.output_timestamp_us = state.heading_continuity_lost_timestamp_us - 1U;
    at_event.output_timestamp_us = state.heading_continuity_lost_timestamp_us;
    push_attitude(before);
    push_attitude(at_event);
    dual_imu_usb_stream_process(&state);

    /* Before the event: no heading-lost bit. At/after the event: private bit7
     * is raised, but the sticky heading loss must NOT clear ATT_CONV. Both
     * frames carry a converged tilt attitude, so ATT_CONV stays set. */
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST) == 0U);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);
    TEST_EXPECT((captured_status(1U) &
                 DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST) != 0U);
    TEST_EXPECT((captured_status(1U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);
}

static void test_late_heading_event_patches_only_post_event_encoded_frames(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;

    const uint64_t event_us = UINT64_C(5000000);
    dual_imu_attitude_output_t before = make_attitude(1U);
    dual_imu_attitude_output_t stale_after = make_attitude(2U);
    before.output_timestamp_us = event_us - 1U;
    stale_after.output_timestamp_us = event_us;

    cdc_result = USBD_BUSY;
    push_attitude(before);
    push_attitude(stale_after);
    dual_imu_usb_stream_process(&state);

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_attempt_capture_length == 2U * HI91_FRAME_SIZE);
    TEST_EXPECT(diagnostics.queued_frame_count == 2U);
    TEST_EXPECT(diagnostics.heading_event_queue_patch_count == 0U);
    uint8_t event_before_frame[HI91_FRAME_SIZE];
    memcpy(event_before_frame, cdc_attempt_capture, sizeof(event_before_frame));

    state.heading_continuity_lost = true;
    state.heading_continuity_lost_timestamp_us = event_us;
    cdc_result = USBD_OK;
    dual_imu_usb_stream_process(&state);

    TEST_EXPECT(cdc_capture_length == 2U * HI91_FRAME_SIZE);
    TEST_EXPECT(memcmp(cdc_capture, event_before_frame,
                       HI91_FRAME_SIZE) == 0);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST) == 0U);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);
    TEST_EXPECT((captured_status(1U) &
                 DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST) != 0U);
    /* Patch only raises bit7; the converged ATT_CONV bit is left untouched. */
    TEST_EXPECT((captured_status(1U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);
    TEST_EXPECT(captured_crc_is_valid(0U));
    TEST_EXPECT(captured_crc_is_valid(1U));
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
    TEST_EXPECT(diagnostics.transport_queue_drop_count == 0U);
    TEST_EXPECT(diagnostics.heading_event_queue_patch_count == 1U);
    TEST_EXPECT(!diagnostics.drop_sticky);

    dual_imu_attitude_output_t rebuilt_after = make_attitude(3U);
    rebuilt_after.output_timestamp_us = event_us + 1U;
    state.estimator_assessed_through_us = UINT64_MAX;
    push_attitude(rebuilt_after);
    dual_imu_usb_stream_process(&state);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST) != 0U);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);
    TEST_EXPECT(captured_crc_is_valid(0U));
}

static void test_three_window_confirmation_backfills_first_suspect(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);

    const uint64_t suspect_us = UINT64_C(7000000);
    dual_imu_attitude_output_t first_suspect = make_attitude(1U);
    dual_imu_attitude_output_t later_suspect = make_attitude(2U);
    first_suspect.output_timestamp_us = suspect_us;
    later_suspect.output_timestamp_us = suspect_us + 625U;
    state.estimator_assessed_through_us =
        suspect_us + DUAL_IMU_USB_INTEGRITY_HOLDBACK_US - 1U;
    push_attitude(first_suspect);
    push_attitude(later_suspect);
    dual_imu_usb_stream_process(&state);

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 0U);
    TEST_EXPECT(diagnostics.queued_frame_count == 2U);

    /* The third estimator window reports the first suspect boundary. Both
     * already encoded frames are repaired before the first one matures. */
    state.heading_continuity_lost = true;
    state.heading_continuity_lost_timestamp_us = suspect_us;
    state.estimator_assessed_through_us =
        suspect_us + DUAL_IMU_USB_INTEGRITY_HOLDBACK_US;
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 1U);
    TEST_EXPECT(diagnostics.submitted_frame_count == 1U);
    TEST_EXPECT(diagnostics.queued_frame_count == 1U);
    TEST_EXPECT(diagnostics.heading_event_queue_patch_count == 2U);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST) != 0U);
    /* Patch raises bit7 only; ATT_CONV stays set on this converged frame. */
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ATT_CONV) != 0U);
    TEST_EXPECT(captured_crc_is_valid(0U));

    state.estimator_assessed_through_us =
        later_suspect.output_timestamp_us +
        DUAL_IMU_USB_INTEGRITY_HOLDBACK_US;
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.submitted_frame_count == 2U);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_PRIVATE_HEADING_LOST) != 0U);
    TEST_EXPECT(captured_crc_is_valid(0U));
}

static void test_integrity_holdback_releases_at_exact_assessment_boundary(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.estimator_assessed_through_us = 0U;

    const uint64_t first_us = UINT64_C(1000000);
    dual_imu_attitude_output_t first = make_attitude(1U);
    dual_imu_attitude_output_t second = make_attitude(2U);
    first.output_timestamp_us = first_us;
    second.output_timestamp_us = first_us + 625U;
    push_attitude(first);
    push_attitude(second);
    dual_imu_usb_stream_process(&state);

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 0U);
    TEST_EXPECT(diagnostics.queued_frame_count == 2U);
    TEST_EXPECT(diagnostics.maturity_wait_count == 1U);

    state.estimator_assessed_through_us =
        first_us + DUAL_IMU_USB_INTEGRITY_HOLDBACK_US - 1U;
    dual_imu_usb_stream_process(&state);
    TEST_EXPECT(cdc_call_count == 0U);

    state.estimator_assessed_through_us =
        first_us + DUAL_IMU_USB_INTEGRITY_HOLDBACK_US;
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 1U);
    TEST_EXPECT(diagnostics.submitted_frame_count == 1U);
    TEST_EXPECT(diagnostics.queued_frame_count == 1U);

    state.estimator_assessed_through_us =
        second.output_timestamp_us + DUAL_IMU_USB_INTEGRITY_HOLDBACK_US;
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(cdc_call_count == 2U);
    TEST_EXPECT(diagnostics.submitted_frame_count == 2U);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
}

static void test_mature_backlog_preserves_average_stream_throughput(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.estimator_assessed_through_us = 0U;

    for (uint32_t sequence = 1U; sequence <= 16U; ++sequence)
        push_attitude(make_attitude(sequence));
    dual_imu_usb_stream_process(&state);

    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.queued_frame_count == 16U);
    TEST_EXPECT(diagnostics.transport_queue_drop_count == 0U);

    state.estimator_assessed_through_us = UINT64_MAX;
    cdc_auto_busy_after_submit = true;
    for (uint32_t cycle = 1U; cycle <= 4U; ++cycle) {
        cdc_tx_ready = true;
        dual_imu_usb_stream_process(&state);
        dual_imu_usb_stream_get_diagnostics(&diagnostics);
        TEST_EXPECT(diagnostics.submitted_frame_count == cycle * 4U);
        TEST_EXPECT(diagnostics.queued_frame_count == 16U - cycle * 4U);
    }
    TEST_EXPECT(diagnostics.submitted_frame_count == 16U);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
    TEST_EXPECT(diagnostics.transport_queue_drop_count == 0U);
}

static void test_disconnect_flushes_immature_session_backlog(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.estimator_assessed_through_us = 0U;
    push_attitude(make_attitude(1U));
    push_attitude(make_attitude(2U));
    dual_imu_usb_stream_process(&state);

    cdc_port_open = false;
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_diagnostics_t diagnostics;
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
    TEST_EXPECT(diagnostics.transport_queue_drop_count == 0U);

    cdc_port_open = true;
    state.estimator_assessed_through_us = UINT64_MAX;
    push_attitude(make_attitude(3U));
    dual_imu_usb_stream_process(&state);
    dual_imu_usb_stream_get_diagnostics(&diagnostics);
    TEST_EXPECT(diagnostics.attitude_frame_count == 1U);
    TEST_EXPECT(diagnostics.submitted_frame_count == 1U);
    TEST_EXPECT(diagnostics.queued_frame_count == 0U);
    TEST_EXPECT(diagnostics.maturity_wait_count == 0U);
}

static void test_saturation_status_is_frame_causal_and_holds_two_seconds(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;
    state.attitude_converged = true;

    const uint64_t event_us = UINT64_C(10000000);
    state.motion_guard_accel_saturation_seen = true;
    state.motion_guard_gyro_saturation_seen = true;
    state.motion_guard_last_accel_saturation_us = event_us;
    state.motion_guard_last_gyro_saturation_us = event_us;

    dual_imu_attitude_output_t before = make_attitude(1U);
    dual_imu_attitude_output_t at_event = make_attitude(2U);
    dual_imu_attitude_output_t before_expiry = make_attitude(3U);
    dual_imu_attitude_output_t at_expiry = make_attitude(4U);
    before.output_timestamp_us = event_us - 1U;
    at_event.output_timestamp_us = event_us;
    before_expiry.output_timestamp_us =
        event_us + DUAL_IMU_USB_SATURATION_HOLD_US - 1U;
    at_expiry.output_timestamp_us =
        event_us + DUAL_IMU_USB_SATURATION_HOLD_US;
    push_attitude(before);
    push_attitude(at_event);
    push_attitude(before_expiry);
    push_attitude(at_expiry);
    dual_imu_usb_stream_process(&state);

    const uint16_t saturation_mask =
        DUAL_IMU_USB_STATUS_ACC_SAT |
        DUAL_IMU_USB_STATUS_GYR_SAT;
    TEST_EXPECT((captured_status(0U) & saturation_mask) == 0U);
    TEST_EXPECT((captured_status(1U) & saturation_mask) == saturation_mask);
    TEST_EXPECT((captured_status(2U) & saturation_mask) == saturation_mask);
    TEST_EXPECT((captured_status(3U) & saturation_mask) == 0U);

    /* A new event extends both windows even when the data payload is held. */
    const uint64_t second_event_us =
        event_us + DUAL_IMU_USB_SATURATION_HOLD_US + 100U;
    state.motion_guard_last_accel_saturation_us = second_event_us;
    state.motion_guard_last_gyro_saturation_us = second_event_us;
    dual_imu_attitude_output_t invalid = make_attitude(5U);
    invalid.output_timestamp_us = second_event_us;
    invalid.valid = false;
    push_attitude(invalid);
    dual_imu_usb_stream_process(&state);

    const uint16_t status = captured_status(0U);
    TEST_EXPECT((status & saturation_mask) == saturation_mask);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATT_CONV) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_PRIVATE_GYRO_VALID) == 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[30]),
                             180.0f, 1.0e-4f));
}

static void test_saturation_status_retains_prior_causal_window(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;
    state.attitude_converged = true;

    const uint64_t first_event_us = UINT64_C(10000000);
    const uint64_t second_event_us =
        first_event_us + DUAL_IMU_USB_SATURATION_HOLD_US + 100U;
    state.motion_guard_accel_saturation_seen = true;
    state.motion_guard_last_accel_saturation_us = second_event_us;
    state.motion_guard_accel_saturation_history[0] =
        (imu_motion_guard_saturation_window_t) {
            .start_us = second_event_us,
            .end_us = second_event_us + DUAL_IMU_USB_SATURATION_HOLD_US - 1U,
            .valid = true,
        };
    state.motion_guard_accel_saturation_history[1] =
        (imu_motion_guard_saturation_window_t) {
            .start_us = first_event_us,
            .end_us = first_event_us + DUAL_IMU_USB_SATURATION_HOLD_US - 1U,
            .valid = true,
        };

    dual_imu_attitude_output_t in_previous = make_attitude(1U);
    dual_imu_attitude_output_t in_gap = make_attitude(2U);
    dual_imu_attitude_output_t in_current = make_attitude(3U);
    in_previous.output_timestamp_us = first_event_us + 100U;
    in_gap.output_timestamp_us =
        first_event_us + DUAL_IMU_USB_SATURATION_HOLD_US + 50U;
    in_current.output_timestamp_us = second_event_us;
    push_attitude(in_previous);
    push_attitude(in_gap);
    push_attitude(in_current);
    dual_imu_usb_stream_process(&state);

    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ACC_SAT) != 0U);
    TEST_EXPECT((captured_status(1U) &
                 DUAL_IMU_USB_STATUS_ACC_SAT) == 0U);
    TEST_EXPECT((captured_status(2U) &
                 DUAL_IMU_USB_STATUS_ACC_SAT) != 0U);
}

static void test_frame_snapshot_survives_long_history_backlog(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;
    state.attitude_converged = true;

    const uint64_t spacing_us = DUAL_IMU_USB_SATURATION_HOLD_US + 100U;
    const uint64_t event_us[4] = {
        UINT64_C(10000000),
        UINT64_C(10000000) + spacing_us,
        UINT64_C(10000000) + (2U * spacing_us),
        UINT64_C(10000000) + (3U * spacing_us),
    };
    state.motion_guard_accel_saturation_seen = true;
    state.motion_guard_gyro_saturation_seen = true;
    state.motion_guard_last_accel_saturation_us = event_us[3];
    state.motion_guard_last_gyro_saturation_us = event_us[3];
    for (size_t index = 0U;
         index < IMU_MOTION_GUARD_SATURATION_HISTORY_COUNT; ++index) {
        const uint64_t retained_event_us = event_us[3U - index];
        const imu_motion_guard_saturation_window_t window = {
            .start_us = retained_event_us,
            .end_us = retained_event_us +
                      DUAL_IMU_USB_SATURATION_HOLD_US - 1U,
            .valid = true,
        };
        state.motion_guard_accel_saturation_history[index] = window;
        state.motion_guard_gyro_saturation_history[index] = window;
    }

    for (uint32_t index = 0U; index < 4U; ++index) {
        dual_imu_attitude_output_t attitude = make_attitude(index + 1U);
        attitude.output_timestamp_us = event_us[index] + 10U;
        if (index == 0U) {
            attitude.accel_saturation_recent = true;
            attitude.gyro_saturation_recent = true;
        }
        push_attitude(attitude);
    }
    dual_imu_usb_stream_process(&state);

    const uint16_t saturation_mask =
        DUAL_IMU_USB_STATUS_ACC_SAT |
        DUAL_IMU_USB_STATUS_GYR_SAT;
    for (uint16_t index = 0U; index < 4U; ++index)
        TEST_EXPECT((captured_status(index) & saturation_mask) ==
                    saturation_mask);
}

static void test_new_usb_session_does_not_clear_device_saturation_history(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    initialize_state(&state);
    state.icm45686_calibrated = true;
    state.attitude_converged = true;
    state.motion_guard_accel_saturation_seen = true;
    state.motion_guard_last_accel_saturation_us = 10000U;

    cdc_port_open = false;
    dual_imu_usb_stream_process(&state);
    cdc_port_open = true;
    dual_imu_attitude_output_t attitude = make_attitude(1U);
    attitude.output_timestamp_us = 10001U;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);

    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ACC_SAT) != 0U);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_GYR_SAT) == 0U);

    state.motion_guard_accel_saturation_seen = false;
    state.motion_guard_gyro_saturation_seen = true;
    state.motion_guard_last_gyro_saturation_us = 10002U;
    attitude = make_attitude(2U);
    attitude.output_timestamp_us = 10003U;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_ACC_SAT) == 0U);
    TEST_EXPECT((captured_status(0U) &
                 DUAL_IMU_USB_STATUS_GYR_SAT) != 0U);
}

int main(void)
{
    test_field_mapping_and_status();
    test_invalid_data_emits_identity_quaternion_when_never_valid();
    test_brief_invalid_frame_holds_payload_but_clears_validity();
    test_invalid_past_old_window_keeps_holding_attitude();
    test_busy_queue_drop_and_disconnect_flush();
    test_closed_port_discards_without_a_drop();
    test_not_ready_skips_transmit_and_retries();
    test_backlog_uses_each_attitude_gyro();
    test_future_accel_is_zeroed_and_stale_accel_is_held();
    test_accel_history_selects_latest_causal_sample();
    test_accel_validity_transition_is_captured_without_new_timestamp();
    test_producer_drop_status_is_session_relative();
    test_convergence_status_uses_active_low_quality_semantics();
    test_convergence_status_is_frame_causal_across_backlog();
    test_heading_loss_sets_private_bit_without_touching_att_conv();
    test_late_heading_event_patches_only_post_event_encoded_frames();
    test_three_window_confirmation_backfills_first_suspect();
    test_integrity_holdback_releases_at_exact_assessment_boundary();
    test_mature_backlog_preserves_average_stream_throughput();
    test_disconnect_flushes_immature_session_backlog();
    test_saturation_status_is_frame_causal_and_holds_two_seconds();
    test_saturation_status_retains_prior_causal_window();
    test_frame_snapshot_survives_long_history_backlog();
    test_new_usb_session_does_not_clear_device_saturation_history();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "dual_imu_usb_stream: %u failure(s)\n",
                      failure_count);
        return EXIT_FAILURE;
    }
    (void)puts("dual_imu_usb_stream: all tests passed");
    return EXIT_SUCCESS;
}
