#ifndef IMU_SPI_DMA_H
#define IMU_SPI_DMA_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_SPI_DMA_MAX_TRANSFER_SIZE UINT16_C(512)

/*
 * Reserved D2 SRAM3 range (inclusive start, exclusive end). Nothing else may
 * be linked or allocated in 0x30040000..0x300407ff.
 */
#define IMU_SPI_DMA_RESERVED_START_ADDR UINT32_C(0x30040000)
#define IMU_SPI_DMA_SPI1_TX_ADDR         UINT32_C(0x30040000)
#define IMU_SPI_DMA_SPI1_RX_ADDR         UINT32_C(0x30040200)
#define IMU_SPI_DMA_SPI4_TX_ADDR         UINT32_C(0x30040400)
#define IMU_SPI_DMA_SPI4_RX_ADDR         UINT32_C(0x30040600)
#define IMU_SPI_DMA_RESERVED_END_ADDR    UINT32_C(0x30040800)

typedef enum
{
    IMU_SPI_DMA_STATUS_OK = 0,
    IMU_SPI_DMA_STATUS_NOT_INITIALIZED,
    IMU_SPI_DMA_STATUS_INVALID_ARGUMENT,
    IMU_SPI_DMA_STATUS_BUSY,
    IMU_SPI_DMA_STATUS_START_ERROR,
    IMU_SPI_DMA_STATUS_TRANSFER_ERROR,
    IMU_SPI_DMA_STATUS_TIMEOUT
} imu_spi_dma_status_t;

typedef void (*imu_spi_dma_callback_t)(SPI_HandleTypeDef *spi,
                                       imu_spi_dma_status_t status,
                                       uint16_t length,
                                       void *context);

typedef struct
{
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    const uint8_t *tx_data;
    uint8_t *rx_data;
    uint16_t length;
    uint32_t timeout_ms;
    imu_spi_dma_callback_t callback;
    void *context;
} imu_spi_dma_request_t;

typedef struct
{
    uint32_t submitted_count;
    uint32_t completed_count;
    uint32_t busy_reject_count;
    uint32_t invalid_argument_count;
    uint32_t start_error_count;
    uint32_t transfer_error_count;
    uint32_t timeout_count;
    uint32_t abort_error_count;
    uint32_t last_hal_error;
    uint32_t last_start_tick_ms;
    uint32_t last_finish_tick_ms;
    uint16_t active_length;
    bool busy;
} imu_spi_dma_diagnostics_t;

/*
 * The module owns one in-flight transaction on each of SPI1 and SPI4. TX data
 * is copied into DMA-accessible staging memory before submit returns. On
 * completion RX data is copied into request.rx_data before callback is called.
 */
bool imu_spi_dma_init(void);
imu_spi_dma_status_t imu_spi_dma_submit(const imu_spi_dma_request_t *request);

/* Call regularly from thread/main-loop context to enforce request timeouts. */
void imu_spi_dma_service(void);

bool imu_spi_dma_is_busy(const SPI_HandleTypeDef *spi);
bool imu_spi_dma_get_diagnostics(const SPI_HandleTypeDef *spi,
                                 imu_spi_dma_diagnostics_t *diagnostics);

/* Optional hooks for SPI callbacks not owned by an active IMU DMA request. */
void imu_spi_dma_unhandled_txrx_complete_callback(SPI_HandleTypeDef *spi);
void imu_spi_dma_unhandled_error_callback(SPI_HandleTypeDef *spi);

#ifdef __cplusplus
}
#endif

#endif
