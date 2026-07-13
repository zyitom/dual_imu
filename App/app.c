#include "app.h"

#include "dual_imu.h"
#include "imu_time.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define APP_DIAGNOSTIC_PERIOD_MS 250U
#define APP_DIAGNOSTIC_BUFFER_SIZE 640U
#define APP_DIAGNOSTIC_BUFFER_COUNT 2U
#define APP_LED_PERIOD_MS        100U
#define RAD_TO_MDEG              57295.7795f
#define RAD_S_TO_MDPS            57295.7795f
#define MPS2_TO_MG               101.971621f

static uint32_t last_diagnostic_ms;
static uint32_t last_led_ms;
static uint32_t last_blue_toggle_ms;
static bool blue_state;
static volatile bool imu_exti_ready;
static bool bmi_capture_ready;

__weak bool app_load_imu_calibration(imu_source_t source,
                                     imu_calibration_t *calibration)
{
    (void)source;
    (void)calibration;
    return false;
}

__weak bool app_imu_external_stationary_hint(void)
{
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
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);

    __HAL_GPIO_EXTI_CLEAR_IT(BMI088_0_ADR_Pin | BMI088_0_GDR_Pin |
                             ICM45686_1_GDR_Pin | ICM45686_1_ADR_Pin);
    HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
    HAL_NVIC_ClearPendingIRQ(EXTI4_IRQn);

    if (enable && imu_timebase_is_running()) {
        imu_exti_ready = true;
        if (!bmi_capture_ready) {
            HAL_NVIC_EnableIRQ(EXTI0_IRQn);
            HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        }
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
    static uint32_t previous_read_count[3];

    if ((uint32_t)(now_ms - last_diagnostic_ms) < APP_DIAGNOSTIC_PERIOD_MS)
        return;
    last_diagnostic_ms = now_ms;

    uint32_t irq_rate_hz[3] = {0U, 0U, 0U};
    const uint32_t irq_count[3] = {
        imu->bmi088_accel_irq_count,
        imu->bmi088_gyro_irq_count,
        imu->icm45686_irq_count,
    };
    const uint32_t read_count[3] = {
        imu->stream_read_count[IMU_SOURCE_BMI088][0],
        imu->stream_read_count[IMU_SOURCE_BMI088][1],
        imu->stream_read_count[IMU_SOURCE_ICM45686][1],
    };
    uint32_t read_rate_hz[3] = {0U, 0U, 0U};
    const uint32_t elapsed_ms = now_ms - previous_diagnostic_ms;
    if ((previous_diagnostic_ms != 0U) && (elapsed_ms != 0U)) {
        for (uint32_t index = 0U; index < 3U; ++index) {
            irq_rate_hz[index] =
                ((irq_count[index] - previous_irq_count[index]) * 1000U) /
                elapsed_ms;
            read_rate_hz[index] =
                ((read_count[index] - previous_read_count[index]) * 1000U) /
                elapsed_ms;
        }
    }
    memcpy(previous_irq_count, irq_count, sizeof(previous_irq_count));
    memcpy(previous_read_count, read_count, sizeof(previous_read_count));
    previous_diagnostic_ms = now_ms;

    char *const message = messages[next_message_index];
    const imu_sample_t *sample = &imu->fused_sample;
    const int length = snprintf(
        message, sizeof(messages[0]),
        "IMU bmi=%u:%02X/%02X icm=%u:%02X mode=%u sel=%u/%u cal=%u/%u seq=%lu "
        "a_mg=%ld,%ld,%ld g_mdps=%ld,%ld,%ld rpy_mdeg=%ld,%ld,%ld "
        "irq_hz=%lu/%lu/%lu read_hz=%lu/%lu/%lu err=%lu/%lu "
        "cap=%u:%lu/%lu cfgcal=%u/%u cfgfault=%u/%u/%u/%u "
        "fault=%lx/%lx wf=%lx/%lx nis_milli=%ld stat=%u/%u/%u "
        "gate=%u av=%u blend=%u "
        "sw=%lu mismatch=%lu unpair=%lu reject=%lu drop=%lu/%lu/%lu "
        "coal=%lu/%lu/%lu lat_us=%lu/%lu/%lu\r\n",
        imu->bmi088_initialized, imu->bmi088_accel_id, imu->bmi088_gyro_id,
        imu->icm45686_initialized, imu->icm45686_id, (unsigned int)imu->mode,
        (unsigned int)imu->selected_source, (unsigned int)imu->selector_state,
        imu->bmi088_calibrated, imu->icm45686_calibrated,
        (unsigned long)imu->fusion_count,
        (long)scaled_float(sample->accel_mps2[0], MPS2_TO_MG),
        (long)scaled_float(sample->accel_mps2[1], MPS2_TO_MG),
        (long)scaled_float(sample->accel_mps2[2], MPS2_TO_MG),
        (long)scaled_float(sample->gyro_rad_s[0], RAD_S_TO_MDPS),
        (long)scaled_float(sample->gyro_rad_s[1], RAD_S_TO_MDPS),
        (long)scaled_float(sample->gyro_rad_s[2], RAD_S_TO_MDPS),
        (long)scaled_float(imu->euler_rad[0], RAD_TO_MDEG),
        (long)scaled_float(imu->euler_rad[1], RAD_TO_MDEG),
        (long)scaled_float(imu->euler_rad[2], RAD_TO_MDEG),
        (unsigned long)irq_rate_hz[0],
        (unsigned long)irq_rate_hz[1],
        (unsigned long)irq_rate_hz[2],
        (unsigned long)read_rate_hz[0],
        (unsigned long)read_rate_hz[1],
        (unsigned long)read_rate_hz[2],
        (unsigned long)imu->bmi088_health.bus_error_count,
        (unsigned long)imu->icm45686_health.bus_error_count,
        imu->bmi088_hardware_timestamp_enabled,
        (unsigned long)imu->bmi088_accel_capture_overrun_count,
        (unsigned long)imu->bmi088_gyro_capture_overrun_count,
        imu->custom_calibration_loaded[IMU_SOURCE_BMI088],
        imu->custom_calibration_loaded[IMU_SOURCE_ICM45686],
        imu->bmi088_accel_configuration_fault,
        imu->bmi088_gyro_configuration_fault,
        imu->icm45686_accel_configuration_fault,
        imu->icm45686_gyro_configuration_fault,
        (unsigned long)imu->bmi088_fault_flags,
        (unsigned long)imu->icm45686_fault_flags,
        (unsigned long)imu->lane_window_flags[IMU_SOURCE_BMI088],
        (unsigned long)imu->lane_window_flags[IMU_SOURCE_ICM45686],
        (long)scaled_float(imu->selector_residual_nis, 1000.0f),
        imu->stationary_candidate, imu->stationary_confirmed,
        imu->stationary_hint_active, imu->accel_update_inhibited,
        imu->fused_accel_valid, imu->alignment_blend_active,
        (unsigned long)imu->selection_count,
        (unsigned long)imu->mismatch_count,
        (unsigned long)imu->unpaired_count,
        (unsigned long)imu->filter_reject_count,
        (unsigned long)imu->bmi088_accel_event_drop_count,
        (unsigned long)imu->bmi088_gyro_event_drop_count,
        (unsigned long)imu->icm45686_event_drop_count,
        (unsigned long)imu->stream_coalesced_count[IMU_SOURCE_BMI088][0],
        (unsigned long)imu->stream_coalesced_count[IMU_SOURCE_BMI088][1],
        (unsigned long)imu->stream_coalesced_count[IMU_SOURCE_ICM45686][1],
        (unsigned long)imu->stream_max_irq_to_read_us[IMU_SOURCE_BMI088][0],
        (unsigned long)imu->stream_max_irq_to_read_us[IMU_SOURCE_BMI088][1],
        (unsigned long)imu->stream_max_irq_to_read_us[IMU_SOURCE_ICM45686][1]);

    if (length > 0) {
        const uint16_t transmit_length = (length < (int)sizeof(messages[0]))
                                             ? (uint16_t)length
                                             : (uint16_t)(sizeof(messages[0]) - 1U);
        if (CDC_Transmit_FS((uint8_t *)message, transmit_length) == USBD_OK)
            next_message_index = (next_message_index + 1U) % APP_DIAGNOSTIC_BUFFER_COUNT;
    }
}

void app_init(void)
{
    configure_imu_exti(false);
    for (uint32_t source = 0U; source < IMU_SOURCE_COUNT; ++source) {
        imu_calibration_t calibration;
        if (app_load_imu_calibration((imu_source_t)source, &calibration))
            (void)dual_imu_set_calibration((imu_source_t)source, &calibration);
    }
    (void)dual_imu_init();
    bmi_capture_ready = imu_time_start_capture_channels_1_2();
    configure_imu_exti(true);
}

void app_process(void)
{
    const uint32_t now_ms = HAL_GetTick();

    dual_imu_set_stationary_hint(app_imu_external_stationary_hint());
    dual_imu_process();
    const dual_imu_state_t *imu = dual_imu_get_state();
    update_leds(imu, now_ms);
    send_diagnostics(imu, now_ms);
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    const bool bmi_pin = !bmi_capture_ready &&
                         ((gpio_pin == BMI088_0_ADR_Pin) ||
                          (gpio_pin == BMI088_0_GDR_Pin));
    const bool imu_pin = bmi_pin || (gpio_pin == ICM45686_1_ADR_Pin);

    if (imu_exti_ready && imu_pin && imu_timebase_is_running())
        dual_imu_handle_exti(gpio_pin);
}
