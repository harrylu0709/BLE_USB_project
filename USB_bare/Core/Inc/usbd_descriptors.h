#ifndef USBD_DESCRIPTORS_H_
#define USBD_DESCRIPTORS_H_

#include "usb_standards.h"
#include "usbd_cdc.h"
#include "Hid/usb_hid_standards.h"
#if 0
const UsbDeviceDescriptor device_descriptor = 
{
    .bLength            = sizeof(UsbDeviceDescriptor),
    .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB             = 0x0200, // 0xJJMN
    .bDeviceClass       = USB_CLASS_PER_INTERFACE,
    .bDeviceSubClass    = USB_SUBCLASS_NONE,
    .bDeviceProtocol    = USB_PROTOCOL_NONE,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x6666,
    .idProduct          = 0x13AA,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0,
    .iProduct           = 0,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1,
};

const uint8_t hid_report_descriptor [] = 
{
    HID_USAGE_PAGE(HID_PAGE_DESKTOP),   
    HID_USAGE(HID_DESKTOP_MOUSE),
    HID_COLLECTION(HID_APPLICATION_COLLECTION),
        HID_USAGE(HID_DESKTOP_POINTER),
        HID_COLLECTION(HID_PHYSICAL_COLLECTION),
            /* x & y axes */
            HID_USAGE_PAGE(HID_PAGE_DESKTOP),
            HID_USAGE(HID_DESKTOP_X),
            HID_USAGE(HID_DESKTOP_Y),
            HID_LOGICAL_MINIMUM(-128), 
            HID_LOGICAL_MAXIMUM(127),
            HID_REPORT_SIZE(8), 
            HID_REPORT_COUNT(2), 
            HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE), 

            /* button */
            HID_USAGE_PAGE(HID_PAGE_BUTTON),
            HID_USAGE_MINIMUM(1),
            HID_USAGE_MAXIMUM(3),
            HID_LOGICAL_MINIMUM(0), // button is unclicked
            HID_LOGICAL_MAXIMUM(1), // button is clicked
            HID_REPORT_SIZE(1), // 1 bit
            HID_REPORT_COUNT(3), // total 3 bits
            HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE), 
            HID_REPORT_SIZE(1), // for padding
            HID_REPORT_COUNT(5), // for padding
            HID_INPUT(HID_IOF_CONSTANT), // for padding
        HID_END_COLLECTION,
    HID_END_COLLECTION,
};

typedef struct 
{
    UsbConfigurationDescriptor  usb_configuration_descriptor;
    UsbInterfaceDescriptor      usb_interface_descriptor;
    UsbHidDescriptor            usb_mouse_hid_descriptor;
    UsbEndpointDescriptor       usb_mouse_endpoint_descriptor;
}UsbConfigurationDescriptorCombination;

const UsbConfigurationDescriptorCombination configuration_descriptor_combination =
{
    .usb_configuration_descriptor = 
    {
        .bLength                = sizeof(UsbConfigurationDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_CONFIGURATION,
        .wTotalLength           = sizeof(configuration_descriptor_combination),
        .bNumInterfaces         = 1,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0x80 | 0x40,  // Bus-powered, remote-wakeup
        .bMaxPower              = 25            // in unit of 2mA
    },
    .usb_interface_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber       = 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_HID,
        .bInterfaceSubClass     = USB_PROTOCOL_NONE,
        .bInterfaceProtocol     = USB_PROTOCOL_NONE,
        .iInterface             = 0
    },
    .usb_mouse_hid_descriptor = 
    {
        .bLength                = sizeof(UsbHidDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_HID,
        .bcdHID                 = 0x0100,
        .bCountryCode           = USB_HID_COUNTRY_NONE,
        .bNumDescriptors        = 1,
        .bDescriptorType0       = USB_DESCRIPTOR_TYPE_HID_REPORT,
        .wDescriptorLength0     = sizeof(hid_report_descriptor),
    },
    .usb_mouse_endpoint_descriptor = 
    {
        .bLength                = sizeof(UsbEndpointDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress       = 0x83, //0x80 means IN endpoint
        .bmAttributes           = USB_ENDPOINT_TYPE_INTERRUPT,
        .wMaxPacketSize         = 64,
        .bInterval              = 50
    }
};

typedef struct {
	int8_t      x;
	int8_t      y;
	uint8_t     buttons;
} __attribute__((__packed__)) HidReport;
#else
const UsbDeviceDescriptor device_descriptor = 
{
    .bLength            = sizeof(UsbDeviceDescriptor),
    .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB             = 0x0200, // 0xJJMN
    .bDeviceClass       = USB_CLASS_CDC_COMM,
    .bDeviceSubClass    = USB_SUBCLASS_IAD,
    .bDeviceProtocol    = USB_PROTOCOL_NONE,
    .bMaxPacketSize0    = 64,
    .idVendor           = 1155,
    .idProduct          = 22336,
    .bcdDevice          = 0x0200,
    .iManufacturer      = 0,
    .iProduct           = 0,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1,
};

const uint8_t hid_report_descriptor [] = 
{
    HID_USAGE_PAGE(HID_PAGE_DESKTOP),   
    HID_USAGE(HID_DESKTOP_MOUSE),
    HID_COLLECTION(HID_APPLICATION_COLLECTION),
        HID_USAGE(HID_DESKTOP_POINTER),
        HID_COLLECTION(HID_PHYSICAL_COLLECTION),
            /* x & y axes */
            HID_USAGE_PAGE(HID_PAGE_DESKTOP),
            HID_USAGE(HID_DESKTOP_X),
            HID_USAGE(HID_DESKTOP_Y),
            HID_LOGICAL_MINIMUM(-128), 
            HID_LOGICAL_MAXIMUM(127),
            HID_REPORT_SIZE(8), 
            HID_REPORT_COUNT(2), 
            HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE), 

            /* button */
            HID_USAGE_PAGE(HID_PAGE_BUTTON),
            HID_USAGE_MINIMUM(1),
            HID_USAGE_MAXIMUM(3),
            HID_LOGICAL_MINIMUM(0), // button is unclicked
            HID_LOGICAL_MAXIMUM(1), // button is clicked
            HID_REPORT_SIZE(1), // 1 bit
            HID_REPORT_COUNT(3), // total 3 bits
            HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE), 
            HID_REPORT_SIZE(1), // for padding
            HID_REPORT_COUNT(5), // for padding
            HID_INPUT(HID_IOF_CONSTANT), // for padding
        HID_END_COLLECTION,
    HID_END_COLLECTION,
};

typedef struct 
{
    UsbConfigurationDescriptor              usb_configuration_descriptor;
    UsbInterfaceDescriptor                  usb_interface_descriptor;
    UsbHeaderFunctionalDescriptor           usb_header_functional_descriptor;
    UsbCallManagementFunctionalDescriptor   usb_call_management_functional_descriptor;
    UsbACMFunctionalDescriptor              usb_acm_functional_descriptor;
    UsbUnionFunctionalDescriptor            usb_union_functional_descriptor;
    UsbEndpointDescriptor                   usb_cdc_cmd_endpoint_descriptor;
    UsbInterfaceDescriptor                  usb_class_interface_descriptor;
    UsbEndpointDescriptor                   usb_cdc_out_endpoint_descriptor;
    UsbEndpointDescriptor                   usb_cdc_in_endpoint_descriptor;
}UsbConfigurationDescriptorCombination;

const UsbConfigurationDescriptorCombination configuration_descriptor_combination =
{
    .usb_configuration_descriptor = 
    {
        .bLength                = sizeof(UsbConfigurationDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_CONFIGURATION,
        .wTotalLength           = sizeof(configuration_descriptor_combination),
        .bNumInterfaces         = 2,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0x80 | 0x40,  // Bus-powered, remote-wakeup
        .bMaxPower              = 50            // in unit of 2mA
    },
    .usb_interface_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_CDC_COMM,
        .bInterfaceSubClass     = USB_SUBCLASS_IAD,
        .bInterfaceProtocol     = USB_PROTOCOL_IAD,
        .iInterface             = 0
    },
    .usb_header_functional_descriptor = 
    {
        .bFunctionLength        = sizeof(UsbHeaderFunctionalDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .bDescriptorSubtype     = 0x00,
        .bcdCDC                 = 0x0110
    },
    .usb_call_management_functional_descriptor = 
    {
        .bFunctionLength        = sizeof(UsbCallManagementFunctionalDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .bDescriptorSubtype     = 0x01,
        .bmCapabilities         = 0x00,
        .bDataInterface         = 0x01
    },
    .usb_acm_functional_descriptor = 
    {
        .bFunctionLength        = sizeof(UsbACMFunctionalDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .bDescriptorSubtype     = 0x02,
        .bmCapabilities         = 0x02
    },
    .usb_union_functional_descriptor = 
    {
        .bFunctionLength        = sizeof(UsbUnionFunctionalDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_CS_INTERFACE,
        .bDescriptorSubtype     = 0x06,
        .bMasterInterface       = 0x00,
        .bSlaveInterface0       = 0x01
    },
    .usb_cdc_cmd_endpoint_descriptor = 
    {
        .bLength                = sizeof(UsbEndpointDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress       = CDC_CMD_EP,
        .bmAttributes           = USB_ENDPOINT_TYPE_INTERRUPT,
        .wMaxPacketSize         = 8,
        .bInterval              = 0x10
    },
    .usb_class_interface_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber       = 1,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 2,
        .bInterfaceClass        = USB_CLASS_CDC_DATA,
        .bInterfaceSubClass     = 0,
        .bInterfaceProtocol     = 0,
        .iInterface             = 0
    },
    .usb_cdc_out_endpoint_descriptor = 
    {
        .bLength                = sizeof(UsbEndpointDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress       = CDC_OUT_EP,
        .bmAttributes           = USB_ENDPOINT_TYPE_BULK,
        .wMaxPacketSize         = 64,
        .bInterval              = 0
    },
    .usb_cdc_in_endpoint_descriptor = 
    {
        .bLength                = sizeof(UsbEndpointDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress       = CDC_IN_EP,
        .bmAttributes           = USB_ENDPOINT_TYPE_BULK,
        .wMaxPacketSize         = 64,
        .bInterval              = 0
    },
};
typedef struct {
	int8_t      x;
	int8_t      y;
	uint8_t     buttons;
} __attribute__((__packed__)) HidReport;
#endif
#endif