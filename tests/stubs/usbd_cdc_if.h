#ifndef TEST_USBD_CDC_IF_H
#define TEST_USBD_CDC_IF_H

#include <stdint.h>

#define USBD_OK   0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);
uint8_t CDC_IsPortOpen_FS(void);
uint8_t CDC_TxReady_FS(void);

#endif
