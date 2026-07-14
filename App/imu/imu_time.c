#include "imu_time.h"

#include "main.h"
#include "tim.h"

#define IMU_TIME_COUNTER_HALF_RANGE     UINT32_C(0x80000000)
static volatile uint32_t s_overflow_count;
static volatile uint32_t s_capture_overrun_count[2];
static bool s_initialized;
static bool s_capture_running;

static volatile uint32_t s_fast_tick_compare_event_count;
static volatile uint32_t s_fast_tick_missed_compare_count;
static volatile uint32_t s_fast_tick_dropped_count;
static volatile uint32_t s_fast_tick_period_us;
static volatile uint64_t s_fast_tick_next_scheduled_us;
static volatile bool s_fast_tick_running;

bool imu_time_init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    if ((htim5.Instance != TIM5) || (htim5.Init.Prescaler != 239U) ||
        (htim5.Init.Period != UINT32_MAX))
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (!s_initialized)
    {
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
        s_overflow_count = 0U;
        __HAL_TIM_SET_COUNTER(&htim5, 0U);
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);

        if ((TIM5->CR1 & TIM_CR1_CEN) == 0U)
        {
            status = HAL_TIM_Base_Start_IT(&htim5);
        }
        else
        {
            __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
        }

        if (status == HAL_OK)
        {
            HAL_NVIC_ClearPendingIRQ(TIM5_IRQn);
            HAL_NVIC_EnableIRQ(TIM5_IRQn);
            s_initialized = true;
        }
    }

    __set_PRIMASK(primask);
    return s_initialized && (status == HAL_OK);
}

bool imu_time_is_running(void)
{
    return s_initialized && (htim5.Instance == TIM5) &&
           ((TIM5->CR1 & TIM_CR1_CEN) != 0U);
}

bool imu_time_start_capture_channels_1_2(void)
{
    if (!imu_time_is_running())
        return false;
    if (imu_time_capture_is_running())
        return true;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    s_capture_overrun_count[0] = 0U;
    s_capture_overrun_count[1] = 0U;
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC1 | TIM_FLAG_CC2);
    const HAL_StatusTypeDef channel_1_status =
        HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
    const HAL_StatusTypeDef channel_2_status =
        HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
    s_capture_running = (channel_1_status == HAL_OK) &&
                        (channel_2_status == HAL_OK);
    if (!s_capture_running)
    {
        (void)HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
        (void)HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2);
        /* HAL stop may clear CEN when no capture/compare channel remains.
         * TIM5 is also the system IMU clock, so keep its base counter alive. */
        __HAL_TIM_ENABLE(&htim5);
    }

    __set_PRIMASK(primask);
    return s_capture_running;
}

bool imu_time_capture_is_running(void)
{
    return s_capture_running &&
           ((TIM5->DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE)) ==
            (TIM_DIER_CC1IE | TIM_DIER_CC2IE));
}

bool imu_time_capture_reset_channel(uint32_t channel)
{
    if (!imu_time_capture_is_running() ||
        (channel < 1U) || (channel > 2U))
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (channel == 1U)
    {
        (void)TIM5->CCR1;
        TIM5->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
    }
    else
    {
        (void)TIM5->CCR2;
        TIM5->SR = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);
    }
    s_capture_overrun_count[channel - 1U] = 0U;
    __set_PRIMASK(primask);
    return true;
}

uint32_t imu_time_capture_overrun_count(uint32_t channel)
{
    if ((channel < 1U) || (channel > 2U))
        return 0U;
    return s_capture_overrun_count[channel - 1U];
}

uint64_t imu_time_now_us(void)
{
    uint32_t high;
    uint32_t low;
    uint32_t update_pending;

    if (htim5.Instance != TIM5)
    {
        return 0U;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    high = s_overflow_count;
    low = TIM5->CNT;
    update_pending = TIM5->SR & TIM_SR_UIF;

    /* If UIF is pending and CNT is already in the new half-cycle, the ISR has
     * not yet accounted for this wrap. If CNT is still high, the wrap occurred
     * after the sampled count and this timestamp belongs to the old epoch. */
    if ((update_pending != 0U) && (low < IMU_TIME_COUNTER_HALF_RANGE))
    {
        high++;
    }

    __set_PRIMASK(primask);
    return ((uint64_t)high << 32) | low;
}

void imu_time_delay_us(uint32_t delay_us)
{
    const uint32_t start = TIM5->CNT;

    while ((uint32_t)(TIM5->CNT - start) < delay_us)
    {
    }
}

bool imu_time_fast_tick_start(uint32_t period_us)
{
    if (!imu_time_is_running() || (period_us == 0U) ||
        (period_us >= IMU_TIME_COUNTER_HALF_RANGE))
    {
        return false;
    }
    if ((TIM5->CCMR2 & TIM_CCMR2_CC3S) != 0U)
        return false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_fast_tick_running)
    {
        const bool unchanged = (s_fast_tick_period_us == period_us) &&
                               ((TIM5->DIER & TIM_DIER_CC3IE) != 0U);
        __set_PRIMASK(primask);
        return unchanged;
    }

    s_fast_tick_compare_event_count = 0U;
    s_fast_tick_missed_compare_count = 0U;
    s_fast_tick_dropped_count = 0U;
    s_fast_tick_period_us = period_us;

    const uint64_t now_us = imu_time_now_us();
    s_fast_tick_next_scheduled_us =
        now_us + (uint64_t)period_us - (now_us % (uint64_t)period_us);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,
                          (uint32_t)s_fast_tick_next_scheduled_us);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC3 | TIM_FLAG_CC3OF);

    const HAL_StatusTypeDef status =
        HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_3);
    s_fast_tick_running = (status == HAL_OK);
    if (s_fast_tick_running)
    {
        uint32_t next_compare = (uint32_t)s_fast_tick_next_scheduled_us;
        uint64_t next_scheduled_us = s_fast_tick_next_scheduled_us;
        bool compare_armed = false;

        /* HAL setup can cross a compare that was only a few cycles away.
         * Re-arm on the same phase grid instead of waiting for CNT wrap. */
        for (uint32_t attempt = 0U; attempt < 4U; ++attempt)
        {
            const uint32_t status_flags = TIM5->SR;
            const int32_t lateness = (int32_t)(TIM5->CNT - next_compare);
            if ((lateness < 0) && ((status_flags & TIM_SR_CC3IF) == 0U))
            {
                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, next_compare);
                __DMB();
                if (((int32_t)(TIM5->CNT - next_compare) < 0) &&
                    ((TIM5->SR & TIM_SR_CC3IF) == 0U))
                {
                    compare_armed = true;
                    break;
                }
            }

            __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC3 | TIM_FLAG_CC3OF);
            const int32_t current_lateness =
                (int32_t)(TIM5->CNT - next_compare);
            const uint32_t skipped = (current_lateness >= 0)
                ? ((uint32_t)current_lateness / period_us) + 1U
                : 1U;
            next_compare += skipped * period_us;
            next_scheduled_us +=
                (uint64_t)skipped * (uint64_t)period_us;
            s_fast_tick_missed_compare_count += skipped;
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, next_compare);
        }

        if (compare_armed)
        {
            s_fast_tick_next_scheduled_us = next_scheduled_us;
        }
        else
        {
            (void)HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_3);
            __HAL_TIM_ENABLE(&htim5);
            s_fast_tick_running = false;
        }
    }

    if (!s_fast_tick_running)
    {
        s_fast_tick_period_us = 0U;
        s_fast_tick_next_scheduled_us = 0U;
    }

    __set_PRIMASK(primask);
    return s_fast_tick_running;
}

bool imu_time_fast_tick_is_running(void)
{
    return s_fast_tick_running && imu_time_is_running() &&
           ((TIM5->DIER & TIM_DIER_CC3IE) != 0U) &&
           ((TIM5->CCER & TIM_CCER_CC3E) != 0U);
}

void imu_time_fast_tick_get_diagnostics(
    imu_time_fast_tick_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
        return;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    diagnostics->running = imu_time_fast_tick_is_running();
    diagnostics->period_us = s_fast_tick_period_us;
    diagnostics->pending_count = 0U;
    diagnostics->compare_event_count = s_fast_tick_compare_event_count;
    diagnostics->missed_compare_count = s_fast_tick_missed_compare_count;
    diagnostics->dropped_tick_count = s_fast_tick_dropped_count;
    diagnostics->next_scheduled_us = s_fast_tick_next_scheduled_us;

    __set_PRIMASK(primask);
}

void imu_time_fast_tick_reset_diagnostics(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_fast_tick_compare_event_count = 0U;
    s_fast_tick_missed_compare_count = 0U;
    s_fast_tick_dropped_count = 0U;
    __set_PRIMASK(primask);
}

__weak void imu_time_capture_callback(uint32_t channel, uint64_t timestamp_us)
{
    (void)channel;
    (void)timestamp_us;
}

__weak void imu_time_fast_tick_callback(uint64_t scheduled_us)
{
    (void)scheduled_us;
}

static uint64_t imu_time_expand_capture(uint32_t captured_count, uint32_t status)
{
    uint32_t high = s_overflow_count;
    if (((status & TIM_SR_UIF) != 0U) &&
        (captured_count < IMU_TIME_COUNTER_HALF_RANGE))
        high++;
    return ((uint64_t)high << 32) | captured_count;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == NULL) || (htim->Instance != TIM5))
        return;

    const uint32_t status = TIM5->SR;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        const uint32_t captured = TIM5->CCR1;
        if ((status & TIM_SR_CC1OF) != 0U)
            s_capture_overrun_count[0]++;
        TIM5->SR = ~TIM_SR_CC1OF;
        imu_time_capture_callback(1U, imu_time_expand_capture(captured, status));
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        const uint32_t captured = TIM5->CCR2;
        if ((status & TIM_SR_CC2OF) != 0U)
            s_capture_overrun_count[1]++;
        TIM5->SR = ~TIM_SR_CC2OF;
        imu_time_capture_callback(2U, imu_time_expand_capture(captured, status));
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == NULL) || (htim->Instance != TIM5) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_3) ||
        !s_fast_tick_running)
    {
        return;
    }

    const uint64_t scheduled_us = s_fast_tick_next_scheduled_us;
    s_fast_tick_compare_event_count++;

    uint32_t next_compare =
        TIM5->CCR3 + s_fast_tick_period_us;
    uint64_t next_scheduled_us =
        scheduled_us + (uint64_t)s_fast_tick_period_us;

    /* Move CCR3 past CNT without losing the original phase. Signed delta is
     * valid because the configured period is less than half the counter span. */
    for (;;)
    {
        const uint32_t now_count = TIM5->CNT;
        const int32_t compare_lateness =
            (int32_t)(now_count - next_compare);
        if (compare_lateness < 0)
        {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, next_compare);
            __DMB();
            if ((int32_t)(TIM5->CNT - next_compare) < 0)
                break;
        }

        const uint32_t skipped =
            ((uint32_t)((int32_t)(TIM5->CNT - next_compare)) /
             s_fast_tick_period_us) + 1U;
        next_compare += skipped * s_fast_tick_period_us;
        next_scheduled_us +=
            (uint64_t)skipped * (uint64_t)s_fast_tick_period_us;
        s_fast_tick_missed_compare_count += skipped;
    }

    s_fast_tick_next_scheduled_us = next_scheduled_us;
    imu_time_fast_tick_callback(scheduled_us);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) && (htim->Instance == TIM5))
        s_overflow_count++;
}
