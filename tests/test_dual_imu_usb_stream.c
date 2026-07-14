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
static uint16_t cdc_capture_length;
static uint32_t cdc_call_count;
static bool cdc_port_open;

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

static bool nearly_equal(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static void reset_fakes(void)
{
    memset(attitude_input, 0, sizeof(attitude_input));
    memset(cdc_capture, 0, sizeof(cdc_capture));
    attitude_head = 0U;
    attitude_tail = 0U;
    cdc_result = USBD_OK;
    cdc_capture_length = 0U;
    cdc_call_count = 0U;
    cdc_port_open = true;
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
    if ((cdc_result == USBD_OK) && (buffer != NULL) &&
        (length <= sizeof(cdc_capture))) {
        memcpy(cdc_capture, buffer, length);
        cdc_capture_length = length;
    }
    return cdc_result;
}

uint8_t CDC_IsPortOpen_FS(void)
{
    return cdc_port_open ? 1U : 0U;
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
    attitude.valid = true;
    return attitude;
}

static void test_field_mapping_and_status(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();

    dual_imu_state_t state;
    memset(&state, 0, sizeof(state));
    state.bmi088_initialized = true;
    state.icm45686_initialized = true;
    state.bmi088_calibrated = true;
    state.icm45686_calibrated = true;
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
        DUAL_IMU_USB_STATUS_ATTITUDE_VALID |
        DUAL_IMU_USB_STATUS_ACCEL_VALID |
        DUAL_IMU_USB_STATUS_GYRO_VALID |
        DUAL_IMU_USB_STATUS_BOTH_INITIALIZED |
        DUAL_IMU_USB_STATUS_SELECTOR_HEALTHY |
        DUAL_IMU_USB_STATUS_BMI_BIAS_CONVERGED |
        DUAL_IMU_USB_STATUS_ICM_BIAS_CONVERGED |
        DUAL_IMU_USB_STATUS_SELECTED_ICM |
        DUAL_IMU_USB_STATUS_STATIONARY |
        DUAL_IMU_USB_STATUS_YAW_RELATIVE;
    TEST_EXPECT(read_u16_le(&cdc_capture[7]) == expected_status);
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

static void test_invalid_data_is_zeroed(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    memset(&state, 0, sizeof(state));
    state.bmi088_initialized = true;
    state.icm45686_initialized = true;

    dual_imu_attitude_output_t attitude = make_attitude(1U);
    attitude.valid = false;
    attitude.deadline_miss = true;
    attitude.quaternion[0] = NAN;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);

    const uint16_t status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ATTITUDE_VALID) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ACCEL_VALID) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_GYRO_VALID) == 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_DEADLINE_MISS) != 0U);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_YAW_RELATIVE) != 0U);
    for (size_t byte = 18U; byte < HI91_FRAME_SIZE; ++byte)
        TEST_EXPECT(cdc_capture[byte] == 0U);
}

static void test_busy_queue_drop_and_disconnect_flush(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    memset(&state, 0, sizeof(state));
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
    memset(&state, 0, sizeof(state));
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

static void test_backlog_uses_each_attitude_gyro(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    memset(&state, 0, sizeof(state));
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

static void test_future_and_stale_accel_are_rejected(void)
{
    dual_imu_state_t state;
    dual_imu_attitude_output_t attitude;
    uint16_t status;

    reset_fakes();
    dual_imu_usb_stream_init();
    memset(&state, 0, sizeof(state));
    attitude = make_attitude(1U);
    state.fused_accel_valid = true;
    state.fused_sample.valid = true;
    state.fused_sample.accel_valid = true;
    state.fused_sample.accel_mps2[0] = 9.80665f;
    state.fused_sample.accel_timestamp_us = attitude.output_timestamp_us + 1U;
    push_attitude(attitude);
    dual_imu_usb_stream_process(&state);
    status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ACCEL_VALID) == 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 0.0f, 0.0f));

    reset_fakes();
    dual_imu_usb_stream_init();
    memset(&state, 0, sizeof(state));
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
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ACCEL_VALID) == 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 0.0f, 0.0f));
}

static void test_accel_history_selects_latest_causal_sample(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    memset(&state, 0, sizeof(state));
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
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_ACCEL_VALID) != 0U);
    TEST_EXPECT(nearly_equal(read_float_le(&cdc_capture[18]), 1.0f, 1.0e-6f));
}

static void test_producer_drop_status_is_session_relative(void)
{
    reset_fakes();
    dual_imu_usb_stream_init();
    dual_imu_state_t state;
    memset(&state, 0, sizeof(state));
    state.fast_attitude_queue_drop_count = 17U;

    push_attitude(make_attitude(1U));
    dual_imu_usb_stream_process(&state);
    uint16_t status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_STREAM_DROP) == 0U);

    state.fast_attitude_queue_drop_count++;
    push_attitude(make_attitude(2U));
    dual_imu_usb_stream_process(&state);
    status = read_u16_le(&cdc_capture[7]);
    TEST_EXPECT((status & DUAL_IMU_USB_STATUS_STREAM_DROP) != 0U);
}

int main(void)
{
    test_field_mapping_and_status();
    test_invalid_data_is_zeroed();
    test_busy_queue_drop_and_disconnect_flush();
    test_closed_port_discards_without_a_drop();
    test_backlog_uses_each_attitude_gyro();
    test_future_and_stale_accel_are_rejected();
    test_accel_history_selects_latest_causal_sample();
    test_producer_drop_status_is_session_relative();

    if (failure_count != 0U) {
        (void)fprintf(stderr, "dual_imu_usb_stream: %u failure(s)\n",
                      failure_count);
        return EXIT_FAILURE;
    }
    (void)puts("dual_imu_usb_stream: all tests passed");
    return EXIT_SUCCESS;
}
