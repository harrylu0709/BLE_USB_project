#ifndef USBD_FRAMEWORK_H_
#define USBD_FRAMEWORK_H_

#include "usbd_driver.h"

void usbd_initialize();
void usbd_poll();
void usbd_configure();
void cdc_transmit(uint8_t* Buf, uint16_t Len);
#endif /* USBD_FRAMEWORK_H_ */