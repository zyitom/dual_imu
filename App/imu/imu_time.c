#include "imu_time.h"

#include "main.h"
#include "tim.h"

#define IMU_TIME_OVERFLOW_IRQ_PRIORITY 4U
#define IMU_TIME_COUNTER_HALF_RANGE     UINT32_C(0x80000000)

static volatile uint32_t s_overflow_count;
static volatile uint32_t s_capture_overrun_count[2];
static bool s_initialized;
static bool s_capture_running;

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
            HAL_NVIC_SetPriority(TIM5_IRQn, IMU_TIME_OVERFLOW_IRQ_PRIORITY, 0U);
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

    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    CLEAR_BIT(TIM5->CCER,
              TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP |
                  TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP);
    MODIFY_REG(TIM5->CCMR1,
               TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC1F |
                   TIM_CCMR1_CC2S | TIM_CCMR1_IC2PSC | TIM_CCMR1_IC2F,
               TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
    TIM5->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF |
                 TIM_SR_CC2IF | TIM_SR_CC2OF);
    s_capture_overrun_count[0] = 0U;
    s_capture_overrun_count[1] = 0U;
    SET_BIT(TIM5->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    SET_BIT(TIM5->DIER, TIM_DIER_CC1IE | TIM_DIER_CC2IE);
    s_capture_running = true;

    __set_PRIMASK(primask);
    return true;
}

bool imu_time_capture_is_running(void)
{
    return s_capture_running &&
           ((TIM5->DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE)) ==
            (TIM_DIER_CC1IE | TIM_DIER_CC2IE));
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

__weak void imu_time_capture_callback(uint32_t channel, uint64_t timestamp_us)
{
    (void)channel;
    (void)timestamp_us;
}

static uint64_t imu_time_expand_capture(uint32_t captured_count, uint32_t status)
{
    uint32_t high = s_overflow_count;
    if (((status & TIM_SR_UIF) != 0U) &&
        (captured_count < IMU_TIME_COUNTER_HALF_RANGE))
        high++;
    return ((uint64_t)high << 32) | captured_count;
}

void TIM5_IRQHandler(void)
{
    const uint32_t status = TIM5->SR;
    const uint32_t enabled = TIM5->DIER;

    if (((status & TIM_SR_CC1IF) != 0U) && ((enabled & TIM_DIER_CC1IE) != 0U))
    {
        const uint32_t captured = TIM5->CCR1;
        if ((status & TIM_SR_CC1OF) != 0U)
            s_capture_overrun_count[0]++;
        TIM5->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
        imu_time_capture_callback(1U, imu_time_expand_capture(captured, status));
    }
    if (((status & TIM_SR_CC2IF) != 0U) && ((enabled & TIM_DIER_CC2IE) != 0U))
    {
        const uint32_t captured = TIM5->CCR2;
        if ((status & TIM_SR_CC2OF) != 0U)
            s_capture_overrun_count[1]++;
        TIM5->SR = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);
        imu_time_capture_callback(2U, imu_time_expand_capture(captured, status));
    }
    if (((status & TIM_SR_UIF) != 0U) && ((enabled & TIM_DIER_UIE) != 0U))
    {
        TIM5->SR = ~TIM_SR_UIF;
        s_overflow_count++;
    }
}
