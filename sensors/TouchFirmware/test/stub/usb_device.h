#ifndef STUB_USB_DEVICE_H
#define STUB_USB_DEVICE_H
#include <stdint.h>
typedef enum { USBD_OK = 0, USBD_BUSY, USBD_FAIL } USBD_StatusTypeDef;
#define USBD_STATE_CONFIGURED 3u
typedef struct { uint32_t dev_state; void *pClassData; } USBD_HandleTypeDef;
void MX_USB_DEVICE_Init(void);
#endif
