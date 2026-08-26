#ifndef STUB_CDC_IF_H
#define STUB_CDC_IF_H
#include "usb_device.h"
typedef struct { uint32_t TxState; uint32_t RxState; } USBD_CDC_HandleTypeDef;
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
#endif
