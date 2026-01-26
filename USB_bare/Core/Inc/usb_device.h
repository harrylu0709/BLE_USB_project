#ifndef USB_DEVICE_H_
#define USB_DEVICE_H_

#include "usb_standards.h"
#include <stdint.h>
#define GPIO_J_K_STATE  0
typedef struct
{
    UsbDeviceState device_state;
    UsbDeviceState old_device_state;
    UsbControlTransferStage control_transfer_stage;
    uint8_t configuration_value;
    void *ptr_out_buffer;
    uint32_t out_data_size;
    const void *ptr_in_buffer;
    uint32_t in_data_size;
    uint8_t dev_remote_wakeup;
    uint8_t low_power_enable;
    uint8_t vbus_sensing_enable;
}UsbDevice;

#endif /* USB_DEVICE_H_ */