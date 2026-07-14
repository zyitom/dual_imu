#include "imu_spi_dma.h"

#include "spi.h"

#include <stddef.h>
#include <string.h>

#define IMU_SPI_DMA_CHANNEL_COUNT UINT32_C(2)
#define IMU_SPI_DMA_ALIGNMENT     UINT32_C(32)
#define STM32H743_D2_SRAM3_END    UINT32_C(0x30048000)

typedef struct
{
    SPI_HandleTypeDef *spi;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    GPIO_TypeDef *cs_port;
    uint8_t *client_rx;
    imu_spi_dma_callback_t callback;
    void *context;
    uint32_t timeout_ms;
    uint32_t start_tick_ms;
    uint16_t cs_pin;
    uint16_t length;
    volatile bool busy;
    volatile bool aborting;
    imu_spi_dma_diagnostics_t diagnostics;
} imu_spi_dma_channel_t;

_Static_assert((IMU_SPI_DMA_MAX_TRANSFER_SIZE % IMU_SPI_DMA_ALIGNMENT) == 0U,
               "IMU SPI DMA buffers must preserve cache-line alignment");
_Static_assert((IMU_SPI_DMA_RESERVED_START_ADDR % IMU_SPI_DMA_ALIGNMENT) == 0U,
               "IMU SPI DMA reserved start must be cache-line aligned");
_Static_assert((IMU_SPI_DMA_SPI1_TX_ADDR + IMU_SPI_DMA_MAX_TRANSFER_SIZE) ==
                   IMU_SPI_DMA_SPI1_RX_ADDR,
               "SPI1 DMA buffers must be contiguous and non-overlapping");
_Static_assert((IMU_SPI_DMA_SPI1_RX_ADDR + IMU_SPI_DMA_MAX_TRANSFER_SIZE) ==
                   IMU_SPI_DMA_SPI4_TX_ADDR,
               "SPI1 and SPI4 DMA buffers must not overlap");
_Static_assert((IMU_SPI_DMA_SPI4_TX_ADDR + IMU_SPI_DMA_MAX_TRANSFER_SIZE) ==
                   IMU_SPI_DMA_SPI4_RX_ADDR,
               "SPI4 DMA buffers must be contiguous and non-overlapping");
_Static_assert((IMU_SPI_DMA_SPI4_RX_ADDR + IMU_SPI_DMA_MAX_TRANSFER_SIZE) ==
                   IMU_SPI_DMA_RESERVED_END_ADDR,
               "IMU SPI DMA reserved range must cover every buffer");
_Static_assert(IMU_SPI_DMA_RESERVED_END_ADDR <= STM32H743_D2_SRAM3_END,
               "IMU SPI DMA buffers must remain inside D2 SRAM3");

static imu_spi_dma_channel_t s_channels[IMU_SPI_DMA_CHANNEL_COUNT] = {
    {
        .spi = &hspi1,
        .tx_buffer = (uint8_t *)IMU_SPI_DMA_SPI1_TX_ADDR,
        .rx_buffer = (uint8_t *)IMU_SPI_DMA_SPI1_RX_ADDR,
    },
    {
        .spi = &hspi4,
        .tx_buffer = (uint8_t *)IMU_SPI_DMA_SPI4_TX_ADDR,
        .rx_buffer = (uint8_t *)IMU_SPI_DMA_SPI4_RX_ADDR,
    },
};

static volatile bool s_initialized;

static imu_spi_dma_channel_t *imu_spi_dma_find_channel(const SPI_HandleTypeDef *spi)
{
    for (size_t index = 0U; index < IMU_SPI_DMA_CHANNEL_COUNT; ++index)
    {
        if (s_channels[index].spi == spi)
        {
            return &s_channels[index];
        }
    }

    return NULL;
}

static bool imu_spi_dma_handle_is_ready(const SPI_HandleTypeDef *spi)
{
    return (spi != NULL) &&
           (((spi == &hspi1) && (spi->Instance == SPI1)) ||
            ((spi == &hspi4) && (spi->Instance == SPI4))) &&
           (spi->hdmarx != NULL) && (spi->hdmatx != NULL);
}

static bool imu_spi_dma_layout_is_valid(void)
{
    return ((uintptr_t)s_channels[0].tx_buffer == IMU_SPI_DMA_SPI1_TX_ADDR) &&
           ((uintptr_t)s_channels[0].rx_buffer == IMU_SPI_DMA_SPI1_RX_ADDR) &&
           ((uintptr_t)s_channels[1].tx_buffer == IMU_SPI_DMA_SPI4_TX_ADDR) &&
           ((uintptr_t)s_channels[1].rx_buffer == IMU_SPI_DMA_SPI4_RX_ADDR) &&
           (((uintptr_t)s_channels[0].tx_buffer % IMU_SPI_DMA_ALIGNMENT) == 0U) &&
           (((uintptr_t)s_channels[0].rx_buffer % IMU_SPI_DMA_ALIGNMENT) == 0U) &&
           (((uintptr_t)s_channels[1].tx_buffer % IMU_SPI_DMA_ALIGNMENT) == 0U) &&
           (((uintptr_t)s_channels[1].rx_buffer % IMU_SPI_DMA_ALIGNMENT) == 0U);
}

static void imu_spi_dma_clear_active(imu_spi_dma_channel_t *channel)
{
    channel->cs_port = NULL;
    channel->client_rx = NULL;
    channel->callback = NULL;
    channel->context = NULL;
    channel->timeout_ms = 0U;
    channel->start_tick_ms = 0U;
    channel->cs_pin = 0U;
    channel->length = 0U;
    channel->diagnostics.active_length = 0U;
    channel->aborting = false;
    __DMB();
    channel->busy = false;
    channel->diagnostics.busy = false;
}

static void imu_spi_dma_finish(imu_spi_dma_channel_t *channel,
                               imu_spi_dma_status_t status,
                               uint32_t hal_error,
                               bool copy_rx)
{
    imu_spi_dma_callback_t callback;
    void *context;
    SPI_HandleTypeDef *spi;
    uint16_t length;

    if ((channel == NULL) || !channel->busy)
    {
        return;
    }

    HAL_GPIO_WritePin(channel->cs_port, channel->cs_pin, GPIO_PIN_SET);

    if (copy_rx && (channel->client_rx != NULL))
    {
        __DMB();
        memcpy(channel->client_rx, channel->rx_buffer, channel->length);
    }

    callback = channel->callback;
    context = channel->context;
    spi = channel->spi;
    length = channel->length;

    channel->diagnostics.last_hal_error = hal_error;
    channel->diagnostics.last_finish_tick_ms = HAL_GetTick();
    if (status == IMU_SPI_DMA_STATUS_OK)
    {
        channel->diagnostics.completed_count++;
    }
    else if (status == IMU_SPI_DMA_STATUS_TRANSFER_ERROR)
    {
        channel->diagnostics.transfer_error_count++;
    }

    imu_spi_dma_clear_active(channel);

    if (callback != NULL)
    {
        callback(spi, status, length, context);
    }
}

bool imu_spi_dma_init(void)
{
    if (!imu_spi_dma_layout_is_valid() ||
        !imu_spi_dma_handle_is_ready(&hspi1) ||
        !imu_spi_dma_handle_is_ready(&hspi4))
    {
        return false;
    }

    if (s_initialized)
    {
        return true;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (size_t index = 0U; index < IMU_SPI_DMA_CHANNEL_COUNT; ++index)
    {
        if (s_channels[index].busy)
        {
            __set_PRIMASK(primask);
            return false;
        }
    }

    for (size_t index = 0U; index < IMU_SPI_DMA_CHANNEL_COUNT; ++index)
    {
        memset(s_channels[index].tx_buffer, 0, IMU_SPI_DMA_MAX_TRANSFER_SIZE);
        memset(s_channels[index].rx_buffer, 0, IMU_SPI_DMA_MAX_TRANSFER_SIZE);
        memset(&s_channels[index].diagnostics, 0,
               sizeof(s_channels[index].diagnostics));
        imu_spi_dma_clear_active(&s_channels[index]);
    }
    s_initialized = true;

    __set_PRIMASK(primask);
    return true;
}

imu_spi_dma_status_t imu_spi_dma_submit(const imu_spi_dma_request_t *request)
{
    imu_spi_dma_channel_t *channel = NULL;

    if (request != NULL)
    {
        channel = imu_spi_dma_find_channel(request->spi);
    }

    if (!s_initialized)
    {
        return IMU_SPI_DMA_STATUS_NOT_INITIALIZED;
    }

    if ((request == NULL) || (channel == NULL) ||
        !imu_spi_dma_handle_is_ready(request->spi) ||
        (request->cs_port == NULL) || (request->cs_pin == 0U) ||
        ((request->cs_pin & (uint16_t)(request->cs_pin - 1U)) != 0U) ||
        (request->tx_data == NULL) || (request->length == 0U) ||
        (request->length > IMU_SPI_DMA_MAX_TRANSFER_SIZE) ||
        (request->timeout_ms == 0U))
    {
        if (channel != NULL)
        {
            channel->diagnostics.invalid_argument_count++;
        }
        return IMU_SPI_DMA_STATUS_INVALID_ARGUMENT;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (channel->busy || (HAL_SPI_GetState(channel->spi) != HAL_SPI_STATE_READY))
    {
        channel->diagnostics.busy_reject_count++;
        __set_PRIMASK(primask);
        return IMU_SPI_DMA_STATUS_BUSY;
    }

    channel->busy = true;
    channel->diagnostics.busy = true;
    channel->cs_port = request->cs_port;
    channel->cs_pin = request->cs_pin;
    channel->client_rx = request->rx_data;
    channel->callback = request->callback;
    channel->context = request->context;
    channel->timeout_ms = request->timeout_ms;
    channel->start_tick_ms = HAL_GetTick();
    channel->length = request->length;
    channel->diagnostics.active_length = request->length;
    channel->diagnostics.last_start_tick_ms = channel->start_tick_ms;
    memcpy(channel->tx_buffer, request->tx_data, request->length);
    memset(channel->rx_buffer, 0, request->length);
    __DMB();

    HAL_GPIO_WritePin(channel->cs_port, channel->cs_pin, GPIO_PIN_RESET);
    const HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive_DMA(
        channel->spi, channel->tx_buffer, channel->rx_buffer, channel->length);

    if (hal_status != HAL_OK)
    {
        HAL_GPIO_WritePin(channel->cs_port, channel->cs_pin, GPIO_PIN_SET);
        channel->diagnostics.last_hal_error = HAL_SPI_GetError(channel->spi);

        if (hal_status == HAL_BUSY)
        {
            channel->diagnostics.busy_reject_count++;
            imu_spi_dma_clear_active(channel);
            __set_PRIMASK(primask);
            return IMU_SPI_DMA_STATUS_BUSY;
        }

        channel->diagnostics.start_error_count++;
        imu_spi_dma_clear_active(channel);
        __set_PRIMASK(primask);
        return IMU_SPI_DMA_STATUS_START_ERROR;
    }

    channel->diagnostics.submitted_count++;
    __set_PRIMASK(primask);
    return IMU_SPI_DMA_STATUS_OK;
}

void imu_spi_dma_service(void)
{
    for (size_t index = 0U; index < IMU_SPI_DMA_CHANNEL_COUNT; ++index)
    {
        imu_spi_dma_channel_t *channel = &s_channels[index];
        const uint32_t now_ms = HAL_GetTick();

        if (!channel->busy ||
            ((uint32_t)(now_ms - channel->start_tick_ms) < channel->timeout_ms))
        {
            continue;
        }

        imu_spi_dma_callback_t callback;
        void *context;
        SPI_HandleTypeDef *spi;
        uint16_t length;
        HAL_StatusTypeDef abort_status;

        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (!channel->busy ||
            ((uint32_t)(HAL_GetTick() - channel->start_tick_ms) <
             channel->timeout_ms))
        {
            __set_PRIMASK(primask);
            continue;
        }

        HAL_GPIO_WritePin(channel->cs_port, channel->cs_pin, GPIO_PIN_SET);
        callback = channel->callback;
        context = channel->context;
        spi = channel->spi;
        length = channel->length;
        channel->aborting = true;

        __set_PRIMASK(primask);

        /* This HAL call can wait for the peripheral, so keep IRQs enabled. */
        abort_status = HAL_SPI_Abort(channel->spi);

        const uint32_t finish_primask = __get_PRIMASK();
        __disable_irq();

        channel->diagnostics.timeout_count++;
        channel->diagnostics.last_hal_error = HAL_SPI_GetError(channel->spi);
        channel->diagnostics.last_finish_tick_ms = HAL_GetTick();
        if (abort_status != HAL_OK)
        {
            channel->diagnostics.abort_error_count++;
        }
        imu_spi_dma_clear_active(channel);

        __set_PRIMASK(finish_primask);

        if (callback != NULL)
        {
            callback(spi, IMU_SPI_DMA_STATUS_TIMEOUT, length, context);
        }
    }
}

bool imu_spi_dma_is_busy(const SPI_HandleTypeDef *spi)
{
    const imu_spi_dma_channel_t *channel = imu_spi_dma_find_channel(spi);
    return (channel != NULL) && channel->busy;
}

bool imu_spi_dma_get_diagnostics(const SPI_HandleTypeDef *spi,
                                 imu_spi_dma_diagnostics_t *diagnostics)
{
    imu_spi_dma_channel_t *channel = imu_spi_dma_find_channel(spi);
    if ((channel == NULL) || (diagnostics == NULL))
    {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *diagnostics = channel->diagnostics;
    diagnostics->busy = channel->busy;
    diagnostics->active_length = channel->length;
    __set_PRIMASK(primask);
    return true;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    imu_spi_dma_channel_t *channel = imu_spi_dma_find_channel(spi);
    if ((channel == NULL) || !channel->busy)
    {
        imu_spi_dma_unhandled_txrx_complete_callback(spi);
        return;
    }

    if (channel->aborting)
    {
        return;
    }

    imu_spi_dma_finish(channel, IMU_SPI_DMA_STATUS_OK, HAL_SPI_ERROR_NONE, true);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    imu_spi_dma_channel_t *channel = imu_spi_dma_find_channel(spi);
    if ((channel == NULL) || !channel->busy)
    {
        imu_spi_dma_unhandled_error_callback(spi);
        return;
    }

    if (channel->aborting)
    {
        return;
    }

    imu_spi_dma_finish(channel, IMU_SPI_DMA_STATUS_TRANSFER_ERROR,
                       HAL_SPI_GetError(spi), false);
}

__weak void imu_spi_dma_unhandled_txrx_complete_callback(SPI_HandleTypeDef *spi)
{
    (void)spi;
}

__weak void imu_spi_dma_unhandled_error_callback(SPI_HandleTypeDef *spi)
{
    (void)spi;
}
