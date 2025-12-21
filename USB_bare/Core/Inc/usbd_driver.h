#ifndef USBD_DRIVER_H_
#define USBD_DRIVER_H_

#include "stm32f4xx.h"
#include "usb_standards.h"
#define USB_OTG_FS_GLOBAL ((USB_OTG_GlobalTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_GLOBAL_BASE))
#define USB_OTG_FS_DEVICE ((USB_OTG_DeviceTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define USB_OTG_FS_PCGCCTL ((uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))

// #define USB_OTG_HS_GLOBAL ((USB_OTG_GlobalTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE))
// #define USB_OTG_HS_DEVICE ((USB_OTG_DeviceTypeDef *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE))
// #define USB_OTG_HS_PCGCCTL ((uint32_t *)(USB_OTG_HS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))
#define ENDPOINT_COUNT          4

#define PKTSTS_OUT_NACK         1
#define PKTSTS_OUT_PACKET       2
#define PKTSTS_OUT_CMPLT        3
#define PKTSTS_SETUP_CMPLT      4
#define PKTSTS_SETUP_PACKET     6


static inline USB_OTG_INEndpointTypeDef* IN_ENDPOINT(uint8_t endpoint_number)
{
    // 0x20 is the interval of each endpoint, RM0090 p.1314
    return (USB_OTG_INEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (endpoint_number * 0x20));
}

static inline USB_OTG_OUTEndpointTypeDef* OUT_ENDPOINT(uint8_t endpoint_number)
{
    return (USB_OTG_OUTEndpointTypeDef *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (endpoint_number * 0x20));
}

static inline volatile uint32_t *FIFO(uint8_t endpoint_number)
{   
    // 0x1000 is the interval of each FIFO, RM0090 p.1272
    return (volatile uint32_t *)(USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (endpoint_number * 0x1000));
}

typedef struct
{
    void (*initialize_core)();
    void (*initialize_gpio_pins)();
    void (*set_device_address)(uint8_t address);
    void (*connect)();
    void (*disconnect)();
    void (*flush_rxfifo)();
    void (*flush_txfifo)(uint8_t endpoint_number);
    void (*configure_in_endpoint)(uint8_t endpoint_number, enum UsbEndpointType endpoint_type, uint16_t endpoint_size);
    void (*configure_out_endpoint)(uint8_t endpoint_number, enum UsbEndpointType endpoint_type, uint16_t endpoint_size);
    void (*read_packet)(void *buffer, uint16_t size);
    void (*write_packet)(uint8_t endpoint_number , const void *buffer, uint16_t size);
    void (*poll)();
} UsbDriver;
extern const UsbDriver usb_driver;
extern UsbEvents usb_events;
// static void initialize_gpio_pins();
// static void initialize_core();
// static void connect();
// static void disconnect();
// static void read_packet(void *buffer, uint16_t size);
// static void write_packet(uint8_t endpoint_number, void *buffer, uint16_t size);
// static void flush_rxfifo();
// static void flush_txfifo(uint8_t endpoint_number);
// static void configure_endpoint0(uint8_t endpoint_size);
// static void configure_in_endpoint(uint8_t endpoint_number, UsbEndpointType endpoint_type, uint16_t endpoint_size);
// static void deconfigure_endpoint(uint8_t endpoint_number);
// static void refresh_fifo_start_address();
// static void configure_rxfifo_size(uint16_t size);
// static void configure_txfifo_size(uint8_t endpoint_number, uint16_t size);
// static void usbrst_handler();
// static void enumdone_handler();
// static void rxflvl_handler();
// static void gintsts_handler();
#endif /* USBD_DRIVER_H_ */
