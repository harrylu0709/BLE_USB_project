#ifndef USBD_DESCRIPTORS_H_
#define USBD_DESCRIPTORS_H_

#include "usb_standards.h"
#include "usbd_cdc.h"
#include "Hid/usb_hid_standards.h"
#if 0
const UsbDeviceDescriptor device_descriptor = 
{
#if MOUSE
    .bLength            = sizeof(UsbDeviceDescriptor),
    .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB             = 0x0200, // 0xJJMN
    .bDeviceClass       = USB_CLASS_PER_INTERFACE,
    .bDeviceSubClass    = USB_SUBCLASS_NONE,
    .bDeviceProtocol    = USB_PROTOCOL_NONE,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x046d,
    .idProduct          = 0xc077,
    .bcdDevice          = 0x7200,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1,
#else
    .bLength            = sizeof(UsbDeviceDescriptor),
    .bDescriptorType    = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB             = 0x0110, // 0xJJMN
    .bDeviceClass       = USB_CLASS_PER_INTERFACE,
    .bDeviceSubClass    = USB_SUBCLASS_NONE,
    .bDeviceProtocol    = USB_PROTOCOL_NONE,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x2a7b,
    .idProduct          = 0x8a57,
    .bcdDevice          = 0x0001,
    .iManufacturer      = 0,
    .iProduct           = 1,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1,
#endif
};

const uint8_t hid_report_descriptor [] = 
{
    // HID_USAGE_PAGE(HID_PAGE_DESKTOP),   
    // HID_USAGE(HID_DESKTOP_MOUSE),
    // HID_COLLECTION(HID_APPLICATION_COLLECTION),
    //     HID_USAGE(HID_DESKTOP_POINTER),
    //     HID_COLLECTION(HID_PHYSICAL_COLLECTION),
    //         /* x & y axes */
    //         HID_USAGE_PAGE(HID_PAGE_DESKTOP),
    //         HID_USAGE(HID_DESKTOP_X),
    //         HID_USAGE(HID_DESKTOP_Y),
    //         HID_LOGICAL_MINIMUM(-128), 
    //         HID_LOGICAL_MAXIMUM(127),
    //         HID_REPORT_SIZE(8), 
    //         HID_REPORT_COUNT(2), 
    //         HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE), 

    //         /* button */
    //         HID_USAGE_PAGE(HID_PAGE_BUTTON),
    //         HID_USAGE_MINIMUM(1),
    //         HID_USAGE_MAXIMUM(3),
    //         HID_LOGICAL_MINIMUM(0), // button is unclicked
    //         HID_LOGICAL_MAXIMUM(1), // button is clicked
    //         HID_REPORT_SIZE(1), // 1 bit
    //         HID_REPORT_COUNT(3), // total 3 bits
    //         HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE), 
    //         HID_REPORT_SIZE(1), // for padding
    //         HID_REPORT_COUNT(5), // for padding
    //         HID_INPUT(HID_IOF_CONSTANT), // for padding
    //     HID_END_COLLECTION,
    // HID_END_COLLECTION,

#if KEYBOARD
0x05,0x01,          		//80  	GLOBAL_USAGE_PAGE(Generic Desktop Controls)	
0x09,0x06,          		//82  	LOCAL_USAGE(Keyboard)	
0xA1,0x01,          		//84  	MAIN_COLLECTION(Applicatior)	
0x85,REPORTID_KEYBOARD,     //86  	GLOBAL_REPORT_ID(3)	
0x05,0x08,          		//6   	GLOBAL_USAGE_PAGE(LEDs)	
0x19,0x01,          		//8   	LOCAL_USAGE_MINIMUM(1)	
0x29,0x03,          		//10  	LOCAL_USAGE_MAXIMUM(3)	
0x15,0x00,          		//12  	GLOBAL_LOGICAL_MINIMUM(0)	
0x25,0x01,          		//14  	GLOBAL_LOCAL_MAXIMUM(1)	
0x75,0x01,          		//16  	GLOBAL_REPORT_SIZE(1)	
0x95,0x03,          		//18  	GLOBAL_REPORT_COUNT(3)	
0x91,0x02,          		//20  	MAIN_OUTPUT(data var absolute NoWrap linear PreferredState NoNullPosition NonVolatile )	Output 0.3
0x95,0x05,          		//22  	GLOBAL_REPORT_COUNT(5)	
0x91,0x01,          		//24  	MAIN_OUTPUT(const array absolute NoWrap linear PreferredState NoNullPosition NonVolatile )	Output 1.0
0x05,0x07,          		//26  	GLOBAL_USAGE_PAGE(Keyboard/Keypad)	
0x19,0xE0,          		//28  	LOCAL_USAGE_MINIMUM(224)	
0x29,0xE7,          		//30  	LOCAL_USAGE_MAXIMUM(231)	
0x95,0x08,          		//32  	GLOBAL_REPORT_COUNT(8)	
0x81,0x02,          		//34  	MAIN_INPUT(data var absolute NoWrap linear PreferredState NoNullPosition NonVolatile )	Input 1.0
0x75,0x08,          		//36  	GLOBAL_REPORT_SIZE(8)	
0x95,0x01,          		//38  	GLOBAL_REPORT_COUNT(1)	
0x81,0x01,          		//40  	MAIN_INPUT(const array absolute NoWrap linear PreferredState NoNullPosition NonVolatile )	Input 2.0
0x19,0x00,          		//42  	LOCAL_USAGE_MINIMUM(0)	
0x29,0x91,          		//44  	LOCAL_USAGE_MAXIMUM(145)	
0x26,0xFF,0x00,     		//46  	GLOBAL_LOCAL_MAXIMUM(255/255)	
0x95,0x06,          		//49  	GLOBAL_REPORT_COUNT(6)	
0x81,0x00,          		//51  	MAIN_INPUT(data array absolute NoWrap linear PreferredState NoNullPosition NonVolatile )	Input 8.0
0xC0,               		//53  	MAIN_COLLECTION_END	
#endif
#if MOUSE
	0x05, 0x01,                         // USAGE_PAGE (Generic Desktop) 0
	0x09, 0x02,                         // USAGE (Mouse) 2
	0xa1, 0x01,                         // COLLECTION (Application) 4
	0x85, REPORTID_MOUSE,               //   REPORT_ID (Mouse) 6
	0x09, 0x01,                         //   USAGE (Pointer) 8
	0xa1, 0x00,                         //   COLLECTION (Physical) 10
	0x05, 0x09,                         //     USAGE_PAGE (Button) 12
	0x19, 0x01,                         //     USAGE_MINIMUM (Button 1) 14
	0x29, 0x02,                         //     USAGE_MAXIMUM (Button 2) 16
	0x15, 0x00,                         //     LOGICAL_MINIMUM (0) 18
	0x25, 0x01,                         //     LOGICAL_MAXIMUM (1) 20
	0x75, 0x01,                         //     REPORT_SIZE (1) 22
	0x95, 0x02,                         //     REPORT_COUNT (2) 24
	0x81, 0x02,                         //     INPUT (Data,Var,Abs) 26
	0x95, 0x06,                         //     REPORT_COUNT (6) 28
	0x81, 0x03,                         //     INPUT (Cnst,Var,Abs) 30
	0x05, 0x01,                         //     USAGE_PAGE (Generic Desktop) 32
	0x09, 0x30,                         //     USAGE (X) 34
	0x09, 0x31,                         //     USAGE (Y) 36
	0x75, 0x10,                         //     REPORT_SIZE (16) 38
	0x95, 0x02,                         //     REPORT_COUNT (2) 40
	0x15, 0x00,                         //     LOGICAL_MINIMUM (0) 42
	0x26, 0xff, 0x7f,                   //     LOGICAL_MAXIMUM (32767) 44
	0x81, 0x02,                         //     INPUT (Data,Var,Abs) 47
	0xc0,                               //   END_COLLECTION 63
	0xc0,                               // END_COLLECTION 65
#endif
};

typedef struct 
{
    UsbConfigurationDescriptor  usb_configuration_descriptor;
    UsbInterfaceDescriptor      usb_interface_descriptor;
#if MOUSE
    UsbHidDescriptor            usb_mouse_hid_descriptor;
    UsbEndpointDescriptor       usb_mouse_endpoint_descriptor;
#endif
#if KEYBOARD
    UsbHidDescriptor            usb_keyboard_hid_descriptor;
    UsbEndpointDescriptor       usb_keyboard_endpoint_descriptor;
#endif
}UsbConfigurationDescriptorCombination;

const UsbConfigurationDescriptorCombination configuration_descriptor_combination =
{
#if MOUSE
    .usb_configuration_descriptor = 
    {
        .bLength                = sizeof(UsbConfigurationDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_CONFIGURATION,
        .wTotalLength           = sizeof(configuration_descriptor_combination),
        .bNumInterfaces         = 1,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0xa0,//0x80 | 0x20,  // Self-powered, remote-wakeup
        .bMaxPower              = 50            // in unit of 2mA
    },
    .usb_interface_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_HID,
        .bInterfaceSubClass     = 1,
        .bInterfaceProtocol     = 2,
        .iInterface             = 0
    },
    .usb_mouse_hid_descriptor = 
    {
        .bLength                = sizeof(UsbHidDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_HID,
        .bcdHID                 = 0x0111,
        .bCountryCode           = USB_HID_COUNTRY_NONE,
        .bNumDescriptors        = 1,
        .bDescriptorType0       = USB_DESCRIPTOR_TYPE_HID_REPORT,
        .wDescriptorLength0     = sizeof(hid_report_descriptor),
    },
    .usb_mouse_endpoint_descriptor = 
    {
        .bLength                = sizeof(UsbEndpointDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress       = 0x81, //0x80 means IN endpoint
        .bmAttributes           = USB_ENDPOINT_TYPE_INTERRUPT,
        .wMaxPacketSize         = 4,
        .bInterval              = 0x0a,
    }
#endif
#if KEYBOARD
   .usb_configuration_descriptor = 
    {
        .bLength                = sizeof(UsbConfigurationDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_CONFIGURATION,
        .wTotalLength           = sizeof(configuration_descriptor_combination),
        .bNumInterfaces         = 1,
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0xa0,//0x80 | 0x20,  // Self-powered, remote-wakeup
        .bMaxPower              = 50            // in unit of 2mA
    },
    .usb_interface_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber       = 0,
        .bAlternateSetting      = 0,
        .bNumEndpoints          = 1,
        .bInterfaceClass        = USB_CLASS_HID,
        .bInterfaceSubClass     = 1,
        .bInterfaceProtocol     = 1,
        .iInterface             = 0
    },
    .usb_keyboard_hid_descriptor = 
    {
        .bLength                = sizeof(UsbHidDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_HID,
        .bcdHID                 = 0x0110,
        .bCountryCode           = USB_HID_COUNTRY_NONE,
        .bNumDescriptors        = 1,
        .bDescriptorType0       = USB_DESCRIPTOR_TYPE_HID_REPORT,
        .wDescriptorLength0     = sizeof(hid_report_descriptor),
    },
    .usb_keyboard_endpoint_descriptor = 
    {
        .bLength                = sizeof(UsbEndpointDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_ENDPOINT,
        .bEndpointAddress       = 0x81, //0x80 means IN endpoint
        .bmAttributes           = USB_ENDPOINT_TYPE_INTERRUPT,
        .wMaxPacketSize         = 64,
        .bInterval              = 0x0a,
    },
#endif
};

const uint8_t language_id_string_descriptor[4] = {0x04, 0x03, 0x09, 0x04};
//const uint8_t manufacturer_string_descriptor[18] = {0x12, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00};
const uint8_t manufacturer_string_descriptor[26] = {0x1a, 0x03, 0x43, 0x00, 0x41, 0x00, 0x53, 0x00, 0x55, 0x00, 0x45, 0x00, 0x20, 0x00, 0x55, 0x00, 0x53, 0x00, 0x42, 0x00, 0x20, 0x00, 0x4b, 0x00, 0x42, 0x00};
const uint8_t mouse_string_descriptor[36] = {0x24, 0x03, 0x55, 0x00, 0x53, 0x00, 0x42, 0x00, 0x20, 0x00, 0x4f, 0x00, 0x70, 0x00, 0x74, 0x00, 0x69, 0x00, 0x63, 0x00, 0x61, 0x00, 0x6c, 0x00, 0x20, 0x00, 0x4d, 0x00, 0x6f, 0x00, 0x75, 0x00, 0x73, 0x00, 0x65, 0x00};
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
    .bDeviceClass       = USB_CLASS_IAD,//USB_CLASS_CDC_COMM,//USB_CLASS_PER_INTERFACE
    .bDeviceSubClass    = USB_SUBCLASS_IAD,
    .bDeviceProtocol    = USB_PROTOCOL_IAD,
    .bMaxPacketSize0    = 64,
    .idVendor           = 0x484,//0x483,
    .idProduct          = 0x5740,
    .bcdDevice          = 0x0200,
    .iManufacturer      = 0,
    .iProduct           = 0,
    .iSerialNumber      = 0,
    .bNumConfigurations = 1,
};

const uint8_t hid_report_descriptor [] = 
{
    // HID_USAGE_PAGE(HID_PAGE_DESKTOP),   
    // HID_USAGE(HID_DESKTOP_MOUSE),
    // HID_COLLECTION(HID_APPLICATION_COLLECTION),
    //     HID_USAGE(HID_DESKTOP_POINTER),
    //     HID_COLLECTION(HID_PHYSICAL_COLLECTION),
    //         /* x & y axes */
    //         HID_USAGE_PAGE(HID_PAGE_DESKTOP),
    //         HID_USAGE(HID_DESKTOP_X),
    //         HID_USAGE(HID_DESKTOP_Y),
    //         HID_LOGICAL_MINIMUM(-128), 
    //         HID_LOGICAL_MAXIMUM(127),
    //         HID_REPORT_SIZE(8), 
    //         HID_REPORT_COUNT(2), 
    //         HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_RELATIVE), 

    //         /* button */
    //         HID_USAGE_PAGE(HID_PAGE_BUTTON),
    //         HID_USAGE_MINIMUM(1),
    //         HID_USAGE_MAXIMUM(3),
    //         HID_LOGICAL_MINIMUM(0), // button is unclicked
    //         HID_LOGICAL_MAXIMUM(1), // button is clicked
    //         HID_REPORT_SIZE(1), // 1 bit
    //         HID_REPORT_COUNT(3), // total 3 bits
    //         HID_INPUT(HID_IOF_DATA | HID_IOF_VARIABLE | HID_IOF_ABSOLUTE), 
    //         HID_REPORT_SIZE(1), // for padding
    //         HID_REPORT_COUNT(5), // for padding
    //         HID_INPUT(HID_IOF_CONSTANT), // for padding
    //     HID_END_COLLECTION,
    // HID_END_COLLECTION,
    0x05, 0x0D,       // Usage Page (Digitizer)
    0x09, 0x0E,       // Usage (Configuration)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x43,     //   Report ID (67)
    0x09, 0x23,     //   Usage (Device Settings)
    0xA1, 0x02,     //   Collection (Logical)
        0x09, 0x52,   //     Usage (Device Mode)
        0x09, 0x53,   //     Usage (Device Identifier)
        0x25, 0x0A,   //     Logical Maximum (10)
        0x75, 0x08,   //     Report Size (8)
        0x95, 0x02,   //     Report Count (2)
        0xB1, 0x02,   //     Feature (Data,Var,Abs)
    0xC0,
    0xC0,             // End Collection

    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x03,     //   Report ID (3)
    0x09, 0x01,     //   Usage (Pointer)
    0xA1, 0x00,     //   Collection (Physical)
        0x05, 0x09,   //     Usage Page (Button)
        0x19, 0x01,   //     Usage Minimum (1)
        0x29, 0x02,   //     Usage Maximum (2)
        0x25, 0x01,   //     Logical Maximum (1)
        0x75, 0x01,   //     Report Size (1)
        0x95, 0x02,   //     Report Count (2)
        0x81, 0x02,   //     Input (Data,Var,Abs) -> 2 Buttons
        0x95, 0x06,   //     Report Count (6)
        0x81, 0x03,   //     Input (Const,Var,Abs) -> Padding
        0x05, 0x01,   //     Usage Page (Generic Desktop)
        0x09, 0x30,   //     Usage (X)
        0x09, 0x31,   //     Usage (Y)
        0x75, 0x10,   //     Report Size (16)
        0x95, 0x02,   //     Report Count (2)
        0x26, 0xFF, 0x7F, // Logical Maximum (32767)
        0x81, 0x02,   //     Input (Data,Var,Abs) -> 16-bit X/Y
    0xC0,
    0xC0              // End Collection
};

typedef struct 
{
    UsbConfigurationDescriptor              usb_configuration_descriptor;
#if MOUSE
    UsbInterfaceAssociationDescriptor       usb_mouse_interface_association_descriptor;
    UsbInterfaceDescriptor                  usb_mouse_interface_descriptor;
    UsbHidDescriptor                        usb_mouse_hid_descriptor;
    UsbEndpointDescriptor                   usb_mouse_endpoint_descriptor;
#endif
    UsbInterfaceAssociationDescriptor       usb_cdc_interface_association_descriptor;
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
        .wTotalLength           = sizeof(configuration_descriptor_combination),//0x64
#if MOUSE
        .bNumInterfaces         = 3,
#else
        .bNumInterfaces         = 2,
#endif
        .bConfigurationValue    = 1,
        .iConfiguration         = 0,
        .bmAttributes           = 0x80 | 0x20,  // Bus-powered, remote-wakeup
        .bMaxPower              = 50            // in unit of 2mA
    },
#if MOUSE
    .usb_mouse_interface_association_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceAssociationDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFASEASSOC,
        .bFirstInterface        = 0,
        .bInterfaceCount        = 1,
        .bFunctionClass         = USB_CLASS_HID,
        .bFunctionSubClass      = USB_PROTOCOL_NONE,
        .bFunctionProtocol      = USB_PROTOCOL_NONE,
        .iFunction              = 0,
    },
    .usb_mouse_interface_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber       = 0,
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
        .bcdHID                 = 0x0111,
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
        .wMaxPacketSize         = 4,
        .bInterval              = 10
    },
#endif
    .usb_cdc_interface_association_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceAssociationDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFASEASSOC,
        .bFirstInterface        = 1,
        .bInterfaceCount        = 2,
        .bFunctionClass         = 2,
        .bFunctionSubClass      = 2,
        .bFunctionProtocol      = 1,
        .iFunction              = 0,
    },
    .usb_interface_descriptor = 
    {
        .bLength                = sizeof(UsbInterfaceDescriptor),
        .bDescriptorType        = USB_DESCRIPTOR_TYPE_INTERFACE,
        .bInterfaceNumber       = 1,
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
        .bInterfaceNumber       = 2,
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