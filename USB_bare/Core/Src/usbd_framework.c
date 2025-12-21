#include "stddef.h"
#include "usbd_framework.h"
#include "usbd_driver.h"
#include "usbd_def.h"
#include "usb_device.h"
#include "usbd_descriptors.h"
#include "usb_standards.h"
#include "logger.h"

//#include "Helpers/math.h"
#define MOUSE                           0
#define USB_CDC_SET_LINE_CODING         0x20
#define USB_CDC_GET_LINE_CODING         0x21
#define USB_CDC_SET_CONTROL_LINE_STATE  0x22
#define CDC_CONNECT_THRESHOLD			2
static UsbDevice *usbd_handle;
uint8_t cdc_report[7]; 
volatile uint8_t set_line_coding_flag = 0;
volatile uint8_t cdc_connect_flag = 0;

void usbd_initialize(UsbDevice *usb_device)
{
    memset(cdc_report, 0, sizeof(cdc_report));
    usbd_handle = usb_device;
    usb_driver.initialize_gpio_pins();
    usb_driver.initialize_core();
    usb_driver.connect();
}

void usbd_poll()
{
    usb_driver.poll();
}

void usbd_configure()
{
#if MOUSE
    //TODO Configure the device (e.g. the endpoints active in this configuration)
    usb_driver.configure_in_endpoint(
        (configuration_descriptor_combination.usb_mouse_endpoint_descriptor.bEndpointAddress & 0x0F),
        (configuration_descriptor_combination.usb_mouse_endpoint_descriptor.bmAttributes & 0x03),
        configuration_descriptor_combination.usb_mouse_endpoint_descriptor.wMaxPacketSize
    );

    usb_driver.write_packet(
        (configuration_descriptor_combination.usb_mouse_endpoint_descriptor.bEndpointAddress & 0x0F),
        NULL,
        0
    );
#else
    usb_driver.configure_in_endpoint(
        (configuration_descriptor_combination.usb_cdc_in_endpoint_descriptor.bEndpointAddress & 0x0F),
        (configuration_descriptor_combination.usb_cdc_in_endpoint_descriptor.bmAttributes & 0x03),
        configuration_descriptor_combination.usb_cdc_in_endpoint_descriptor.wMaxPacketSize
    );

    usb_driver.write_packet(
        (configuration_descriptor_combination.usb_cdc_in_endpoint_descriptor.bEndpointAddress & 0x0F),
        NULL,
        0
    );

    usb_driver.configure_out_endpoint(
        (configuration_descriptor_combination.usb_cdc_out_endpoint_descriptor.bEndpointAddress & 0x0F),
        (configuration_descriptor_combination.usb_cdc_out_endpoint_descriptor.bmAttributes & 0x03),
        configuration_descriptor_combination.usb_cdc_out_endpoint_descriptor.wMaxPacketSize
    );

    usb_driver.configure_in_endpoint(
        (configuration_descriptor_combination.usb_cdc_cmd_endpoint_descriptor.bEndpointAddress & 0x0F),
        (configuration_descriptor_combination.usb_cdc_cmd_endpoint_descriptor.bmAttributes & 0x03),
        configuration_descriptor_combination.usb_cdc_cmd_endpoint_descriptor.wMaxPacketSize
    );

    usb_driver.write_packet(
        (configuration_descriptor_combination.usb_cdc_cmd_endpoint_descriptor.bEndpointAddress & 0x0F),
        NULL,
        0
    );
#endif
}

static void usb_reset_received_handler()
{
    usbd_handle->in_data_size = 0;
    usbd_handle->out_data_size = 0;
    usbd_handle->configuration_value = 0;
    usbd_handle->device_state = USB_DEVICE_STATE_DEFAULT;
    usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_SETUP;
    usb_driver.set_device_address(0);
}

static void process_standard_device_request()
{
    UsbRequest const *request = usbd_handle->ptr_out_buffer;

    switch(request->bRequest)
    {
        case USB_STANDARD_GET_DESCRIPTOR:
            log_info("Standard Get Descriptor request received.");
            const uint8_t descriptor_type = request->wValue >> 8;
            const uint16_t descriptor_length = request->wLength;
            //const uint8_t descriptor_index = request->wValue & 0xFF;

            switch(descriptor_type)
            {
                case USB_DESCRIPTOR_TYPE_DEVICE:
                    log_info("Get Device Descriptor.");
                    usbd_handle->ptr_in_buffer = &device_descriptor;
                    if(descriptor_length == 0x40) usbd_handle->in_data_size = sizeof(device_descriptor);  
                    else    usbd_handle->in_data_size = descriptor_length;
                    log_info("Switching control transfer stage to IN-DATA.");
                    usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
                    break;

                case USB_DESCRIPTOR_TYPE_CONFIGURATION:
                    log_info("Get Configuration Descriptor.");
                    usbd_handle->ptr_in_buffer = &configuration_descriptor_combination;
                    if(descriptor_length > sizeof(UsbConfigurationDescriptor)) usbd_handle->in_data_size = sizeof(configuration_descriptor_combination);  //??
                    else    usbd_handle->in_data_size = descriptor_length;  //??
                    log_info("Switching control transfer stage to IN-DATA.");
                    usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
                    break;
                default:
                    usbd_handle->in_data_size = 0;
                    log_info("Switching control transfer stage to IN-DATA.");
                    usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
                    break;
            }
            break;
        case USB_STANDARD_SET_ADDRESS:
            log_info("Standard set address request received.");
            const uint16_t device_address = request->wValue;
            usb_driver.set_device_address(device_address);
            usbd_handle->device_state = USB_DEVICE_STATE_ADDRESSED;
            log_info("Switching control transfer stage to IN-STATUS."); //set address request is an OUT request.
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_STATUS_IN;
            break;
        case USB_STANDARD_SET_CONFIG:
            log_info("Standard Set Configuration request received.");
            usbd_handle->configuration_value = request->wValue;
            usbd_configure();
            usbd_handle->device_state = USB_DEVICE_STATE_CONFIGURED;
            log_info("Switching control transfer stage to IN-STATUS."); 
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_STATUS_IN;
            break;
    }
}

static void process_class_interface_request()
{
    UsbRequest const *request = usbd_handle->ptr_out_buffer;
    
    switch(request->bRequest)
    {
        case USB_HID_SETIDLE: /* Device should only respond interrupt events */
            set_line_coding_flag = 0;
            log_info("Switching control transfer stage to IN-STATUS."); 
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_STATUS_IN;
            break;
        case USB_CDC_GET_LINE_CODING:
            set_line_coding_flag = 0;
            printf("get line coding\n");
            usbd_handle->ptr_in_buffer = &cdc_report;
            usbd_handle->in_data_size = 7;
            log_info("Switching stage to IN-DATA."); 
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;

            break;
        case USB_CDC_SET_CONTROL_LINE_STATE:
            if(request->wValue == 0x02) cdc_connect_flag = 0;
            set_line_coding_flag = 0;
            printf("set control line state\n");
            log_info("Switching control transfer stage to IN-STATUS.");
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_STATUS_IN;
            break;
        case USB_CDC_SET_LINE_CODING:
            set_line_coding_flag = 1;
            printf("set line coding\n");    
            log_info("Switching control transfer stage to IN-STATUS.");
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_STATUS_IN;       
            break;
    }
}

static void process_standard_interface_request()
{
    UsbRequest const *request = usbd_handle->ptr_out_buffer;

    switch(request->wValue >> 8)
    {
        case USB_DESCRIPTOR_TYPE_HID_REPORT:
            usbd_handle->ptr_in_buffer = &hid_report_descriptor;
            usbd_handle->in_data_size = sizeof(hid_report_descriptor);

            log_info("Switching stage to IN-DATA."); 
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
            break;
    }
}

static void process_request()
{
    UsbRequest const *request = usbd_handle->ptr_out_buffer;

    switch(request->bmRequestType & (USB_BM_REQUEST_TYPE_TYPE_MASK | USB_BM_REQUEST_TYPE_RECIPIENT_MASK))
    {
        case USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIPIENT_DEVICE:
            process_standard_device_request();
            break;
        case USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIPIENT_INTERFACE:
            process_class_interface_request();
            break;
        case USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIPIENT_INTERFACE:
            process_standard_interface_request();
            break;
    }
}

static void process_control_transfer_stage()
{
    switch(usbd_handle->control_transfer_stage)
    {
        case USB_CONTROL_STAGE_SETUP: /* Already done in usb_events.on_setup_data_received (setup_data_received_handler) */
            break;
        case USB_CONTROL_STAGE_DATA_IN:
            log_info("Processing IN-DATA stage.");
            uint8_t data_size = MIN(usbd_handle->in_data_size, device_descriptor.bMaxPacketSize0);
            usb_driver.write_packet(0, usbd_handle->ptr_in_buffer, data_size);
            usbd_handle->in_data_size -= data_size;
            usbd_handle->ptr_in_buffer += data_size;
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_DATA_IN_IDLE;
            /*
            We need to switch the stage of the control tranfer temporarily to a different sub stage and wait until the data is read or fetched by the host.
            And only after this we can continue running in the normal IN DATA stage and sending the rest of the data
            */

            if(usbd_handle->in_data_size == 0)
            {
                if(data_size == device_descriptor.bMaxPacketSize0)
                {
                    // Send another zero length of data packet.
                    log_info("Switching control transfer stage to IN-DATA ZERO");
                    usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_DATA_IN_ZERO;
                }
                else
                {
                    //Host will send us a zero length packet to tell us, everything has been received successfully.
                    log_info("Switching control transfer stage to OUT-STATUS");
                    usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_STATUS_OUT;
                }
            }
            break;
        case USB_CONTROL_STAGE_DATA_IN_IDLE: // Switch back to IN DATA stage function, we can use on_in_transfer_completed function pointer.
            break;
        case USB_CONTROL_STAGE_STATUS_OUT: 
            log_info("Switching control transfer stage to SETUP.");
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_SETUP;
            break;
        case USB_CONTROL_STAGE_STATUS_IN: //In STATUS-IN stage, we simply should send a zero length data packet to the host.
            usb_driver.write_packet(0, NULL, 0);
            log_info("Switching control transfer stage to SETUP.");
            usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_SETUP;
            break;
        default:
            break;
    }
}

static void usb_polled_handler()
{
    process_control_transfer_stage();
}
#if MOUSE
static void write_mouse_report()
{
    log_debug("Sending USB HID mouse report");

    HidReport hid_report = {
        .x = 5
    };

    usb_driver.write_packet(
        (configuration_descriptor_combination.usb_mouse_endpoint_descriptor.bEndpointAddress & 0x0F),
        &hid_report,
        sizeof(hid_report)
    );
}
#endif
void cdc_transmit(uint8_t* Buf, uint16_t Len)
{
	if(cdc_connect_flag >= CDC_CONNECT_THRESHOLD)	
		usb_driver.write_packet((CDC_IN_EP & 0x0F), Buf, Len);
}
static void in_transfer_completed_hander(uint8_t endpoint_number)
{
    if(usbd_handle->in_data_size)
    {
        // Keep sending data to the host
        log_info("Switching control transfer stage to IN-DATA");
        usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_DATA_IN;
    }
    else if(usbd_handle->control_transfer_stage == USB_CONTROL_STAGE_DATA_IN_ZERO)
    {
        // Transfer is completed by sending a zero length data packet to the host 
        usb_driver.write_packet(0, NULL, 0);
        log_info("Switching control transfer stage to OUT-STATUS");
        usbd_handle->control_transfer_stage = USB_CONTROL_STAGE_STATUS_OUT;
    }
#if MOUSE
    if(endpoint_number == (configuration_descriptor_combination.usb_mouse_endpoint_descriptor.bEndpointAddress & 0x0F))
    {
        write_mouse_report();
    }
#endif
    if(endpoint_number == (configuration_descriptor_combination.usb_cdc_in_endpoint_descriptor.bEndpointAddress & 0x0F))
    {
        ;
    }
}

static void out_transfer_completed_hander(uint8_t endpoint_number)
{
    unsigned char test_str[] = "echo back\n";
    if(endpoint_number == (CDC_OUT_EP & 0xf))
    {
        //cdc_transmit(test_str, sizeof(test_str));
    }
}

static void out_data_received_handler(uint8_t endpoint_number, uint16_t byte_count)
{
    usb_driver.read_packet(usbd_handle->ptr_out_buffer, byte_count);
    if(set_line_coding_flag)
        memcpy(cdc_report, usbd_handle->ptr_out_buffer, byte_count);
    else
        memset(cdc_report, 0, byte_count);
    if(set_line_coding_flag)
    {
        uint8_t *ptr = (uint8_t*)(usbd_handle->ptr_out_buffer);
        int i;
        for(i = 0; i < byte_count ; i++)
        {
            if(ptr[i] != 0)
            {
                cdc_connect_flag++;
                break;
            }
        }
    }
    log_debug_array("OUT packet data: ", usbd_handle->ptr_out_buffer, byte_count);
}

static void setup_data_received_handler(uint8_t endpoint_number, uint16_t byte_count)
{
    //Read the data out of the endpoint zero
    usb_driver.read_packet(usbd_handle->ptr_out_buffer, byte_count);

    // Prints out the received data
    log_debug_array("SETUP data: ", usbd_handle->ptr_out_buffer, byte_count);

    process_request();
}

UsbEvents usb_events = 
{
    .on_usb_reset_received = &usb_reset_received_handler,
    .on_setup_data_received = &setup_data_received_handler,
    .on_out_data_received = &out_data_received_handler,
    .on_usb_polled = &usb_polled_handler,
    .on_in_transfer_completed = &in_transfer_completed_hander,
    .on_out_transfer_completed = &out_transfer_completed_hander
};