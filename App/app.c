#include "app.h"

#include "dual_imu.h"
#include "protocol/dual_imu_usb_stream.h"
#include "imu_time.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define APP_DIAGNOSTIC_PERIOD_MS 1000U
#define APP_DIAGNOSTIC_BUFFER_SIZE 1024U
#define APP_DIAGNOSTIC_BUFFER_COUNT 2U
#define APP_LED_PERIOD_MS        100U
#define RAD_TO_MDEG              57295.7795f
#define APP_USB_BINARY_OUTPUT             1U

static uint32_t last_diagnostic_ms;
static uint32_t last_led_ms;
static uint32_t last_blue_toggle_ms;
static bool blue_state;
static volatile bool imu_exti_ready;
static uint32_t app_process_count;
static uint32_t app_process_max_us;

__weak bool app_load_imu_calibration(imu_source_t source,
                                     imu_calibration_t *calibration)
{
    (void)source;
    (void)calibration;
    return false;
}

static bool imu_timebase_is_running(void)
{
    return imu_time_is_running();
}

static void configure_imu_exti(bool enable)
{
    const uint32_t saved_primask = __get_PRIMASK();

    __disable_irq();
    imu_exti_ready = false;
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);

    __HAL_GPIO_EXTI_CLEAR_IT(ICM45686_1_ADR_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);

    if (enable && imu_timebase_is_running()) {
        imu_exti_ready = true;
        HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    }

    __DSB();
    __ISB();
    if (saved_primask == 0U)
        __enable_irq();
}

static int32_t scaled_float(float value, float scale)
{
    const float scaled = value * scale;

    if (!isfinite(scaled))
        return 0;
    if (scaled > 2147483000.0f)
        return INT32_MAX;
    if (scaled < -2147483000.0f)
        return INT32_MIN;
    return (int32_t)scaled;
}

static uint32_t counter_rate_hz(uint32_t delta, uint32_t elapsed_ms)
{
    if (elapsed_ms == 0U)
        return 0U;

    return (uint32_t)((((uint64_t)delta * 1000U) + (elapsed_ms / 2U)) /
                      elapsed_ms);
}

static void update_leds(const dual_imu_state_t *imu, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_led_ms) < APP_LED_PERIOD_MS)
        return;

    last_led_ms = now_ms;
    const bool any_sensor = imu->bmi088_initialized || imu->icm45686_initialized;
    const bool both_sensors = imu->bmi088_initialized && imu->icm45686_initialized;
    const bool healthy_fusion = both_sensors && imu->fused_sample.valid &&
                                imu->bmi088_calibrated &&
                                imu->icm45686_calibrated &&
                                (imu->selector_state == IMU_SELECTOR_HEALTHY);

    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, any_sensor ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,
                      healthy_fusion ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if ((uint32_t)(now_ms - last_blue_toggle_ms) >= 500U) {
        last_blue_toggle_ms = now_ms;
        blue_state = !blue_state;
        HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,
                          blue_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void send_diagnostics(const dual_imu_state_t *imu, uint32_t now_ms)
{
    static char messages[APP_DIAGNOSTIC_BUFFER_COUNT][APP_DIAGNOSTIC_BUFFER_SIZE];
    static uint32_t next_message_index;
    static uint32_t previous_diagnostic_ms;
    static uint32_t previous_irq_count[3];
    static uint32_t previous_read_count[4];
    static uint32_t previous_process_count;
    static uint32_t previous_fusion_count;
    static uint32_t previous_fast_output_count;
    static uint32_t previous_fast_invalid_count;
    static uint32_t previous_compare_missed_count;
    static uint32_t previous_tick_drop_count;

    if ((uint32_t)(now_ms - last_diagnostic_ms) < APP_DIAGNOSTIC_PERIOD_MS)
        return;
    last_diagnostic_ms = now_ms;

    const uint32_t irq_count[3] = {
        imu->bmi088_accel_irq_count,
        imu->bmi088_gyro_irq_count,
        imu->icm45686_irq_count,
    };
    const uint32_t read_count[4] = {
        imu->stream_read_count[IMU_SOURCE_BMI088][0],
        imu->stream_read_count[IMU_SOURCE_BMI088][1],
        imu->stream_read_count[IMU_SOURCE_ICM45686][0],
        imu->stream_read_count[IMU_SOURCE_ICM45686][1],
    };
    uint32_t fast_output_count;
    uint32_t fast_invalid_count;
    uint32_t fast_last_integrity_flags;
    uint32_t fast_last_publish_latency_us;
    uint32_t fast_max_publish_latency_us;
    uint32_t fast_last_prediction_horizon_us;
    const uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    fast_output_count = imu->fast_attitude_output_count;
    fast_invalid_count = imu->fast_attitude_invalid_count;
    fast_last_integrity_flags = imu->fast_attitude_last_integrity_flags;
    fast_last_publish_latency_us =
        imu->fast_attitude_last_publish_latency_us;
    fast_max_publish_latency_us = imu->fast_attitude_max_publish_latency_us;
    fast_last_prediction_horizon_us =
        imu->fast_attitude_last_prediction_horizon_us;
    __set_PRIMASK(saved_primask);
    uint32_t irq_rate_hz[3] = {0U, 0U, 0U};
    uint32_t read_rate_hz[4] = {0U, 0U, 0U, 0U};
    uint32_t process_rate_hz = 0U;
    uint32_t fusion_rate_hz = 0U;
    uint32_t fast_rate_hz = 0U;
    uint32_t fast_valid_rate_hz = 0U;
    uint32_t invalid_delta = 0U;
    uint32_t compare_missed_delta = 0U;
    uint32_t tick_drop_delta = 0U;
    const uint32_t elapsed_ms = now_ms - previous_diagnostic_ms;
    if ((previous_diagnostic_ms != 0U) && (elapsed_ms != 0U))
    {
        for (uint32_t index = 0U; index < 3U; ++index) {
            irq_rate_hz[index] = counter_rate_hz(
                irq_count[index] - previous_irq_count[index], elapsed_ms);
        }
        for (uint32_t index = 0U; index < 4U; ++index)
            read_rate_hz[index] = counter_rate_hz(
                read_count[index] - previous_read_count[index], elapsed_ms);

        const uint32_t fast_delta =
            fast_output_count - previous_fast_output_count;
        invalid_delta = fast_invalid_count - previous_fast_invalid_count;
        const uint32_t valid_delta =
            (invalid_delta <= fast_delta) ? fast_delta - invalid_delta : 0U;
        process_rate_hz = counter_rate_hz(
            app_process_count - previous_process_count, elapsed_ms);
        fusion_rate_hz = counter_rate_hz(
            imu->fusion_count - previous_fusion_count, elapsed_ms);
        fast_rate_hz = counter_rate_hz(fast_delta, elapsed_ms);
        fast_valid_rate_hz = counter_rate_hz(valid_delta, elapsed_ms);
        compare_missed_delta = imu->fast_attitude_compare_missed_count -
                               previous_compare_missed_count;
        tick_drop_delta = imu->fast_attitude_tick_drop_count -
                          previous_tick_drop_count;
    }

    char *const message = messages[next_message_index];
    const int length = snprintf(
        message, sizeof(messages[0]),
        "IMU id=%u:%02X/%02X,%u:%02X cfg=%u/%u/%u/%u cal_load=%u/%u "
        "rate_hz=%lu/%lu/%lu/%lu fusion=%lu fast=%lu/%lu "
        "irq=%lu/%lu/%lu mode=%u sel=%u/%u "
        "rpy_mdeg=%ld,%ld,%ld sw=%lu mm=%lu unpair=%lu reject=%lu "
        "stat=%u/%u sr=%u/%u/%u tv=%ld/%ld:%ld/%ld "
        "cal=%u/%u gz_mdps=%ld/%ld bz_mdps=%ld/%ld "
        "fault=%lx/%lx wf=%lx/%lx fast_err=%lu/%lu/%lu if=%lx "
        "lat_us=%lu/%lu horizon=%lu sched=%u "
        "fifo_err=%lu/%lu atl_reset=%lu gyro=%lu/%lu:%02lx/%u warm=%lu "
        "drop=%lu/%lu/%lu cap=%lu/%lu clock=%u/%u ppm=%ld/%ld "
        "arej=%lu/%lu reset=%lu/%lu:%lu/%lu disc=%lu depth=%u/%u "
        "stage_us=%lu/%lu:%lu/%lu loop=%lu/%lu\r\n",
        imu->bmi088_initialized, imu->bmi088_accel_id, imu->bmi088_gyro_id,
        imu->icm45686_initialized, imu->icm45686_id,
        imu->bmi088_accel_configuration_fault,
        imu->bmi088_gyro_configuration_fault,
        imu->icm45686_accel_configuration_fault,
        imu->icm45686_gyro_configuration_fault,
        imu->custom_calibration_loaded[IMU_SOURCE_BMI088],
        imu->custom_calibration_loaded[IMU_SOURCE_ICM45686],
        (unsigned long)read_rate_hz[0],
        (unsigned long)read_rate_hz[1],
        (unsigned long)read_rate_hz[2],
        (unsigned long)read_rate_hz[3],
        (unsigned long)fusion_rate_hz,
        (unsigned long)fast_rate_hz,
        (unsigned long)fast_valid_rate_hz,
        (unsigned long)irq_rate_hz[0],
        (unsigned long)irq_rate_hz[1],
        (unsigned long)irq_rate_hz[2],
        (unsigned int)imu->mode,
        (unsigned int)imu->selected_source,
        (unsigned int)imu->selector_state,
        (long)scaled_float(imu->euler_rad[0], RAD_TO_MDEG),
        (long)scaled_float(imu->euler_rad[1], RAD_TO_MDEG),
        (long)scaled_float(imu->euler_rad[2], RAD_TO_MDEG),
        (unsigned long)imu->selection_count,
        (unsigned long)imu->mismatch_count,
        (unsigned long)imu->unpaired_count,
        (unsigned long)imu->filter_reject_count,
        imu->stationary_candidate,
        imu->stationary_confirmed,
        (unsigned int)imu->stationary_last_reject_reason,
        (unsigned int)imu->stationary_streak,
        (unsigned int)imu->stationary_max_streak,
        (long)scaled_float(
            sqrtf(fmaxf(0.0f,
                        imu->stationary_temporal_gyro_variance_rad2_s2[0])),
            RAD_TO_MDEG),
        (long)scaled_float(
            sqrtf(fmaxf(0.0f,
                        imu->stationary_temporal_gyro_variance_rad2_s2[1])),
            RAD_TO_MDEG),
        (long)scaled_float(
            sqrtf(fmaxf(0.0f,
                        imu->stationary_temporal_accel_variance_m2_s4[0])),
            1000.0f),
        (long)scaled_float(
            sqrtf(fmaxf(0.0f,
                        imu->stationary_temporal_accel_variance_m2_s4[1])),
            1000.0f),
        imu->bmi088_calibrated,
        imu->icm45686_calibrated,
        (long)scaled_float(imu->bmi088_gyro_sample.gyro_rad_s[2],
                           RAD_TO_MDEG),
        (long)scaled_float(imu->icm45686_sample.gyro_rad_s[2],
                           RAD_TO_MDEG),
        (long)scaled_float(
            imu->lane_gyro_bias_rad_s[IMU_SOURCE_BMI088][2], RAD_TO_MDEG),
        (long)scaled_float(
            imu->lane_gyro_bias_rad_s[IMU_SOURCE_ICM45686][2], RAD_TO_MDEG),
        (unsigned long)imu->bmi088_fault_flags,
        (unsigned long)imu->icm45686_fault_flags,
        (unsigned long)imu->lane_window_flags[IMU_SOURCE_BMI088],
        (unsigned long)imu->lane_window_flags[IMU_SOURCE_ICM45686],
        (unsigned long)invalid_delta,
        (unsigned long)compare_missed_delta,
        (unsigned long)tick_drop_delta,
        (unsigned long)fast_last_integrity_flags,
        (unsigned long)fast_last_publish_latency_us,
        (unsigned long)fast_max_publish_latency_us,
        (unsigned long)fast_last_prediction_horizon_us,
        imu->fast_attitude_scheduler_running,
        (unsigned long)imu->bmi088_fifo_dma_error_count,
        (unsigned long)imu->icm45686_fifo_dma_error_count,
        (unsigned long)imu->bmi088_accel_timeline_reset_count,
        (unsigned long)imu->bmi088_gyro_capture_mismatch_count,
        (unsigned long)imu->bmi088_gyro_capture_queue_overflow_count,
        (unsigned long)imu->bmi088_gyro_capture_mismatch_reason,
        imu->bmi088_gyro_capture_sync_fault,
        (unsigned long)imu->bmi088_gyro_warmup_discard_count,
        (unsigned long)imu->bmi088_accel_event_drop_count,
        (unsigned long)imu->bmi088_gyro_event_drop_count,
        (unsigned long)imu->icm45686_event_drop_count,
        (unsigned long)imu->bmi088_accel_capture_overrun_count,
        (unsigned long)imu->bmi088_gyro_capture_overrun_count,
        imu->bmi088_fifo_clock_sync_valid, imu->icm45686_fifo_clock_sync_valid,
        (long)scaled_float(
            imu->fifo_clock_scale[IMU_SOURCE_BMI088] - 1.0f, 1000000.0f),
        (long)scaled_float(
            imu->fifo_clock_scale[IMU_SOURCE_ICM45686] - 1.0f, 1000000.0f),
        (unsigned long)imu->fifo_clock_anchor_rejected_count[IMU_SOURCE_BMI088],
        (unsigned long)imu->fifo_clock_anchor_rejected_count[IMU_SOURCE_ICM45686],
        (unsigned long)imu->fifo_clock_stale_reset_count[IMU_SOURCE_BMI088],
        (unsigned long)imu->fifo_clock_causal_reset_count[IMU_SOURCE_BMI088],
        (unsigned long)imu->fifo_clock_stale_reset_count[IMU_SOURCE_ICM45686],
        (unsigned long)imu->fifo_clock_causal_reset_count[IMU_SOURCE_ICM45686],
        (unsigned long)imu->icm45686_timestamp_discontinuity_count,
        imu->icm45686_fifo_last_frames, imu->icm45686_fifo_peak_frames,
        (unsigned long)imu->fifo_service_last_us,
        (unsigned long)imu->fifo_service_max_us,
        (unsigned long)imu->estimator_process_last_us,
        (unsigned long)imu->estimator_process_max_us,
        (unsigned long)process_rate_hz,
        (unsigned long)app_process_max_us);

    if ((length > 0) && (length < (int)sizeof(messages[0])) &&
        (CDC_Transmit_FS((uint8_t *)message, (uint16_t)length) == USBD_OK))
    {
        memcpy(previous_irq_count, irq_count, sizeof(previous_irq_count));
        memcpy(previous_read_count, read_count, sizeof(previous_read_count));
        previous_diagnostic_ms = now_ms;
        previous_process_count = app_process_count;
        previous_fusion_count = imu->fusion_count;
        previous_fast_output_count = fast_output_count;
        previous_fast_invalid_count = fast_invalid_count;
        previous_compare_missed_count =
            imu->fast_attitude_compare_missed_count;
        previous_tick_drop_count = imu->fast_attitude_tick_drop_count;
        next_message_index =
            (next_message_index + 1U) % APP_DIAGNOSTIC_BUFFER_COUNT;
        app_process_max_us = 0U;
    }
}

void app_init(void)
{
    configure_imu_exti(false);
    dual_imu_usb_stream_init();
    for (uint32_t source = 0U; source < IMU_SOURCE_COUNT; ++source) {
        imu_calibration_t calibration;
        if (app_load_imu_calibration((imu_source_t)source, &calibration))
            (void)dual_imu_set_calibration((imu_source_t)source, &calibration);
    }
    (void)dual_imu_init();
    (void)dual_imu_start_streaming();
    configure_imu_exti(true);
}

void app_process(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint64_t process_start_us = imu_time_now_us();

    dual_imu_process();
    const uint64_t process_duration_us = imu_time_now_us() - process_start_us;
    if (process_duration_us > app_process_max_us)
        app_process_max_us = (process_duration_us > UINT32_MAX)
                                 ? UINT32_MAX
                                 : (uint32_t)process_duration_us;
    app_process_count++;
    const dual_imu_state_t *imu = dual_imu_get_state();
    update_leds(imu, now_ms);
    if (APP_USB_BINARY_OUTPUT != 0U)
        dual_imu_usb_stream_process(imu);
    else
        send_diagnostics(imu, now_ms);
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (imu_exti_ready && (gpio_pin == ICM45686_1_ADR_Pin) &&
        imu_timebase_is_running())
        dual_imu_handle_exti(gpio_pin);
}
