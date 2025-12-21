#include "usbd_driver.h"
#include "usb_device.h"
#include <string.h>
#include "stm32f407xx_gpio_driver.h"
#include "logger.h"
extern UsbDevice usb_device;
extern void dwt_delay_ms(uint32_t ms);
static void initialize_gpio_pins()
{
    //Enable clock for GPIOB
    //SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

    MODIFY_REG(GPIOA->AFR[1], 
        (GPIO_AFRH_AFSEL11 | GPIO_AFRH_AFSEL12), 
        (_VAL2FLD(GPIO_AFRH_AFSEL11, 0xA) | _VAL2FLD(GPIO_AFRH_AFSEL12, 0xA))
    );

	MODIFY_REG(GPIOA->MODER,
		GPIO_MODER_MODER11 | GPIO_MODER_MODER12,
		_VAL2FLD(GPIO_MODER_MODER11, 2) | _VAL2FLD(GPIO_MODER_MODER12, 2)
	);
    GPIO_IRQPriorityConfig(OTG_FS_IRQn, 0);
    GPIO_IRQInterruptConfig(OTG_FS_IRQn, ENABLE);
}

static void initialize_core()
{
    //Enable clock for USB core
    //SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_OTGHSEN);
	SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_OTGFSEN);

    //Configure the USB core to run in device mode, and to use the embedded full-speed PHY
    /* For bit6 0: USB 2.0 high-speed ULPI PHY, 1:USB 1.1 full-speed serial transceiver */
    MODIFY_REG(USB_OTG_FS->GUSBCFG, 
        (USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | USB_OTG_GUSBCFG_TRDT), 
        (USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL | _VAL2FLD(USB_OTG_GUSBCFG_TRDT, 0x09))
    );


    //Configure the device to run in full speed mode 0x200003, HAL = 0x200200
    MODIFY_REG(USB_OTG_FS_DEVICE->DCFG, 
        USB_OTG_DCFG_DSPD, 
        _VAL2FLD(USB_OTG_DCFG_DSPD, 0x03)
    );

    //Enable VBUS sensing device, 0x90000, HAL = 0x10000
    CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_NOVBUSSENS);
    SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_VBUSBSEN);

    //Unmask the main USB core interrupts 0x80043818
    SET_BIT(USB_OTG_FS->GINTMSK, 
        (USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_SOFM |
        USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_WUIM | USB_OTG_GINTMSK_IEPINT | 
        USB_OTG_GINTMSK_OEPINT | USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTSTS_USBSUSP));

    //Clear all pending core interrupts 0x4001020
    WRITE_REG(USB_OTG_FS->GINTSTS, 0xFFFFFFFF);

    // Unmask USB global interrupt 0x1, HAL = 0x0
    SET_BIT(USB_OTG_FS->GAHBCFG, USB_OTG_GAHBCFG_GINT);

    // Unmask transfer completed interrupt of all endpoints.
    SET_BIT(USB_OTG_FS_DEVICE->DOEPMSK, USB_OTG_DOEPMSK_XFRCM);
    SET_BIT(USB_OTG_FS_DEVICE->DIEPMSK, USB_OTG_DIEPMSK_XFRCM);
}

static void set_device_address(uint8_t address)
{
    MODIFY_REG(USB_OTG_FS_DEVICE->DCFG,
        USB_OTG_DCFG_DAD,
        _VAL2FLD(USB_OTG_DCFG_DAD, address)
    );
}

static void connect()
{
    // Power the transceivers on, 0x90000, 0: power down active, HAL = 0x10000
    SET_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);

    //Connect the device to the bus, 0
    CLEAR_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);
}

static void disconnect()
{
    //Disconnect the device to the bus
    SET_BIT(USB_OTG_FS_DEVICE->DCTL, USB_OTG_DCTL_SDIS);

    // Power the transceivers off
    CLEAR_BIT(USB_OTG_FS->GCCFG, USB_OTG_GCCFG_PWRDWN);
}

static void read_packet(void *buffer, uint16_t size)
{
    // Note: There is only one RX FIFO, FIFO is on the unit of one word
    volatile uint32_t *fifo = FIFO(0);
    
    for(; size >= 4; size -= 4 , buffer += 4)
    {
        // Pop one 32-bit word of data (until there is less than one word remaining)
        volatile int32_t data = *fifo;

        // Stores the data in the buffer
        *((uint32_t*)buffer) = data;
    }

    if(size > 0)
    {
        // Pops the last remaining bytes(which are less than one word)
        volatile uint32_t data = *fifo;
        for(; size > 0 ; size --, buffer++, data >>= 8)
        {
            // Stores the data in the buffer with the correct alignment
            *((uint8_t*)buffer) = (data & 0xFF);
        }
    }
}
static void write_packet(uint8_t endpoint_number, const void *buffer, uint16_t size)
{
    volatile uint32_t *fifo = FIFO(endpoint_number);
    USB_OTG_INEndpointTypeDef *in_endpoint = IN_ENDPOINT(endpoint_number);


    //Configures the transmission (1 packet that has 'size' bytes)
    MODIFY_REG(in_endpoint->DIEPTSIZ,
        USB_OTG_DIEPTSIZ_PKTCNT | USB_OTG_DIEPTSIZ_XFRSIZ,
        _VAL2FLD(USB_OTG_DIEPTSIZ_PKTCNT, 1)  | _VAL2FLD(USB_OTG_DIEPTSIZ_XFRSIZ, size)
    );


    //Enable the transmission after clearing both STALL and NACK of endpoint
    MODIFY_REG(in_endpoint->DIEPCTL,
        USB_OTG_DIEPCTL_STALL,
        USB_OTG_DIEPCTL_CNAK  | USB_OTG_DIEPCTL_EPENA
    );

    //Get the size in term of 32-bit word
    size = (size + 3)/4;


    for(; size > 0 ; size --, buffer += 4 )
    {
        //Pushes data to the fifo
        *fifo = *((uint32_t*)buffer);
    }

}


/*
+--------------------------------+
|         TX FIFO #n packet      |   DIEPTXFn[31:0]
+--------------------------------+
|               ...              | 
|                                |
+--------------------------------+
|         TX FIFO #1 packet      |   DIEPTXF1[31:0]
+--------------------------------+
|         TX FIFO #0 packet      |   GNPTXFSIZ[31:0]
+--------------------------------+
|           RX packet            |   GRXFSIZ[31:0]
+--------------------------------+   (RX start address fixed to 0)
*/

/*
Updates the start address of all FIFOs according to the size of each FIFOs
*/
static void refresh_fifo_start_address()
{
    // The first changeable start address begins after the region of RX FIFO
    uint16_t start_address = _FLD2VAL(USB_OTG_GRXFSIZ_RXFD, USB_OTG_FS->GRXFSIZ) * 4;

    // Updates the start address of the TX FIFO 0
    MODIFY_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,
        USB_OTG_TX0FSA,
        _VAL2FLD(USB_OTG_TX0FSA, start_address) 
    );

    // The next start address is after when the last TX FIFO ends
    start_address += _FLD2VAL(USB_OTG_TX0FD, USB_OTG_FS->DIEPTXF0_HNPTXFSIZ) * 4;

    // Updates the start address of the reset TX FIFOs
    for(uint8_t txfifo_number = 0; txfifo_number < ENDPOINT_COUNT - 1; txfifo_number++)
    {
        MODIFY_REG(USB_OTG_FS->DIEPTXF[txfifo_number],
            USB_OTG_NPTXFSA,
            _VAL2FLD(USB_OTG_NPTXFSA, start_address) 
        );
        start_address += _FLD2VAL(USB_OTG_NPTXFD, USB_OTG_FS->DIEPTXF[txfifo_number]) * 4;
    }
}


/*
Configure the RX FIFO of all OUT endpoint
RX FIFO is shared between all OUT endpoint
*/
static void configure_rxfifo_size(uint16_t size)
{
    /*
        Configures the space required to save status packets in Rx FIFO and gets the size in terms of 32-bit 
        10: for SETUP packets
        1:  for status information
        size/4 : in unit of word
    */
    size = 10 + (2*(size/4 +1));

    // Configures the depth of the  FIFO

    MODIFY_REG(USB_OTG_FS->GRXFSIZ,
        USB_OTG_GRXFSIZ_RXFD,
        _VAL2FLD(USB_OTG_GRXFSIZ_RXFD, size) 
    );

    refresh_fifo_start_address();
}

/*
Configure the TX FIFO of an IN endpoint
Any change on any FIFO will update the registers of all TXFIFOs to adapt the start offsets
*/
static void configure_txfifo_size(uint8_t endpoint_number, uint16_t size)
{
    // Gets the FIFO size in term of 32-bit words
    size = (size + 3)/4;

    // Configures the depth of the TX FIFO
    if(endpoint_number == 0)
    {
        MODIFY_REG(USB_OTG_FS->DIEPTXF0_HNPTXFSIZ,
            USB_OTG_TX0FD,
            _VAL2FLD(USB_OTG_TX0FD, size) 
        );
    }
    else
    {
        MODIFY_REG(USB_OTG_FS->DIEPTXF[endpoint_number - 1],
            USB_OTG_NPTXFD,
            _VAL2FLD(USB_OTG_NPTXFD, size) 
        );
    }
    refresh_fifo_start_address();
}

/*
Flushes the RX FIFO of all OUT endpoints
*/
static void flush_rxfifo()
{
    SET_BIT(USB_OTG_FS->GRSTCTL, USB_OTG_GRSTCTL_RXFFLSH);
}

/*
Flushes the TX FIFO of an IN endpoint
*/
static void flush_txfifo(uint8_t endpoint_number)
{
    MODIFY_REG(USB_OTG_FS->GRSTCTL,
        USB_OTG_GRSTCTL_TXFNUM,
        _VAL2FLD(USB_OTG_GRSTCTL_TXFNUM, endpoint_number) | USB_OTG_GRSTCTL_TXFFLSH
    );
}

static void configure_endpoint0(uint8_t endpoint_size)
{
    //Unmask all interrupts for IN and OUT endpoint0
    SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, (1<<0 | 1<<16));

    int size_f4;
    if(endpoint_size == 64)
        size_f4 = 0;
    else if(endpoint_size == 32)
        size_f4 = 1;
    else if(endpoint_size == 16)
        size_f4 = 2;
    else if(endpoint_size == 8)
        size_f4 = 3;

    //Configure the maximum packet size, activates the endpoint, and NACK the endpoint(cannot send data)
    MODIFY_REG(IN_ENDPOINT(0)->DIEPCTL,
        USB_OTG_DIEPCTL_MPSIZ,
        USB_OTG_DIEPCTL_USBAEP | _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, size_f4) | USB_OTG_DIEPCTL_SNAK
    );

    /*
    Clear NACK, and enables endpoint data transmission, 

    The maximum packet size for control OUT endpoint 0 is the same as what is programmed in
    control IN endpoint 0.

    OUT endpoint 0 is always active by default and it can't be deactivated
    */ 
    SET_BIT(OUT_ENDPOINT(0)->DOEPCTL, USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);

    // 64 bytes is the maximum packet size for full speed USB devices.
    configure_rxfifo_size(64);
    configure_txfifo_size(0, endpoint_size);
}

static void configure_in_endpoint(uint8_t endpoint_number, UsbEndpointType endpoint_type, uint16_t endpoint_size)
{
    //Unmask all interrupts for the targeted IN endpoint
    SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, 1 << endpoint_number);
    
    /* 
    Activates the endpoint, sets endpoint handshake to NACK(not ready to send data), sets DATA0 packet identifier
    Configure its type, its maximum packet size, and assigns it a TX FIFO
    */
   MODIFY_REG(IN_ENDPOINT(endpoint_number)->DIEPCTL,
        USB_OTG_DIEPCTL_MPSIZ | USB_OTG_DIEPCTL_EPTYP | USB_OTG_DIEPCTL_TXFNUM,
        USB_OTG_DIEPCTL_USBAEP | _VAL2FLD(USB_OTG_DIEPCTL_MPSIZ, endpoint_size) | USB_OTG_DIEPCTL_SNAK | 
        _VAL2FLD(USB_OTG_DIEPCTL_EPTYP, endpoint_type) | _VAL2FLD(USB_OTG_DIEPCTL_TXFNUM, endpoint_number) |USB_OTG_DIEPCTL_SD0PID_SEVNFRM
    );

   configure_txfifo_size(endpoint_number, endpoint_size);
}

static void configure_out_endpoint(uint8_t endpoint_number, UsbEndpointType endpoint_type, uint16_t endpoint_size)
{
    //Unmask all interrupts for the targeted OUT endpoint
    SET_BIT(USB_OTG_FS_DEVICE->DAINTMSK, 1 << 16 << endpoint_number);

   MODIFY_REG(OUT_ENDPOINT(endpoint_number)->DOEPCTL,
        USB_OTG_DOEPCTL_MPSIZ | USB_OTG_DOEPCTL_EPTYP,
        USB_OTG_DOEPCTL_USBAEP | _VAL2FLD(USB_OTG_DOEPCTL_MPSIZ, endpoint_size) | /* USB_OTG_DOEPCTL_SNAK | */
        _VAL2FLD(USB_OTG_DOEPCTL_EPTYP, endpoint_type) | USB_OTG_DOEPCTL_SD0PID_SEVNFRM
    );
}

static void deconfigure_endpoint(uint8_t endpoint_number)
{
    USB_OTG_INEndpointTypeDef  *in_endpoint = IN_ENDPOINT(endpoint_number);
    USB_OTG_OUTEndpointTypeDef *out_endpoint = OUT_ENDPOINT(endpoint_number);

    // Mask all interrupts for the targeted IN and OUT endpoints
    CLEAR_BIT(USB_OTG_FS_DEVICE->DAINTMSK, (1 << endpoint_number) | (1 << 16 << endpoint_number));

    // Clear all interrupts of the endpoint
    //SET_BIT(in_endpoint->DIEPINT, 0x29ff);
    SET_BIT(in_endpoint->DIEPINT, 0x28FB);
    //SET_BIT(out_endpoint->DOEPINT, 0x715F);
    SET_BIT(out_endpoint->DOEPINT, 0x313B);

    // Disable the endpoints if possible
    if(in_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
    {
        // Disables endpoint transmission
        SET_BIT(in_endpoint->DIEPCTL, USB_OTG_DIEPCTL_EPDIS);
    }

    // Deactivate the endpoint
    CLEAR_BIT(in_endpoint->DIEPCTL, USB_OTG_DIEPCTL_USBAEP);

    /*
    OUT endpoint 0 cannot be deconfigured 
    because it must always be active and enabled, so it's always ready to receive data from the host.
    */
    if(endpoint_number != 0)
    {
        if(out_endpoint->DOEPCTL & USB_OTG_DOEPCTL_EPENA)
        {
            // Disables endpoint transmission
            SET_BIT(out_endpoint->DOEPCTL, USB_OTG_DOEPCTL_EPDIS);
        }
        // Deactivate the endpoint
        CLEAR_BIT(out_endpoint->DOEPCTL, USB_OTG_DOEPCTL_USBAEP);
    }

    // Flushes the FIFOs
    flush_txfifo(endpoint_number);
    flush_rxfifo();
}


static void usbrst_handler()
{
    //log_info("USB reset signal was detected.");
    for(uint8_t i = 0; i < ENDPOINT_COUNT; i++)
    {
        deconfigure_endpoint(i);
    }

    usb_events.on_usb_reset_received();
}

static void enum_done_handler()
{
    //log_info("USB device speed enumeration done.");
    configure_endpoint0(64); /* 64 bytes is the maximum packet size*/
}

static void iepint_handler()
{
    // Finds the endpoint caused the interrupt
    uint8_t endpoint_number = ffs(USB_OTG_FS_DEVICE->DAINT) - 1;

    if(IN_ENDPOINT(endpoint_number)->DIEPINT & USB_OTG_DIEPINT_XFRC)
    {
        usb_events.on_in_transfer_completed(endpoint_number);

        //Clear interrupt
        SET_BIT(IN_ENDPOINT(endpoint_number)->DIEPINT, USB_OTG_DIEPINT_XFRC);
    }
    if(IN_ENDPOINT(endpoint_number)->DIEPINT & USB_OTG_DIEPINT_TXFE)
    {
        ;
    }
}

static void oepint_handler()
{
    // Finds the endpoint caused the interrupt
    uint8_t endpoint_number = ffs(USB_OTG_FS_DEVICE->DAINT >> 16) - 1;
    
    if(OUT_ENDPOINT(endpoint_number)->DOEPINT & USB_OTG_DOEPINT_XFRC)
    {

        usb_events.on_out_transfer_completed(endpoint_number);

        //Clear interrupt
        SET_BIT(OUT_ENDPOINT(endpoint_number)->DOEPINT, USB_OTG_DOEPINT_XFRC);
    }
}

static void rxflvl_handler() //RX FIFO not empty
{
    
    //Pop the status information word from RX FIFO
    
    uint32_t receive_status = USB_OTG_FS_GLOBAL->GRXSTSP; // status information will be received before actual data

    // The endpoint that received the data  
    uint8_t endpoint_number = _FLD2VAL(USB_OTG_GRXSTSP_EPNUM, receive_status);
    // The count of bytes in the received packet
    uint8_t bcnt = _FLD2VAL(USB_OTG_GRXSTSP_BCNT, receive_status);
    // The endpoint that received the data  
    uint8_t pktsts = _FLD2VAL(USB_OTG_GRXSTSP_PKTSTS, receive_status);
    /*
    For SETUP stage has completed and OUT stage has completed
    When a tranfer is completed on an OUT or IN endpoint, the USB core disables the endpoint.
    Which means the transmissions on the endpoint will be disabled and consequently the endpoint won't be received any data.
    So we need to re-enable the endpoint after every single transfer.
    */
    switch(pktsts)
    {
        case PKTSTS_SETUP_PACKET: /* SETUP packet (includes data) 6 */
            usb_events.on_setup_data_received(endpoint_number, bcnt);
            break;
        case PKTSTS_OUT_PACKET: /* OUT packet (includes data) 2 */
            usb_events.on_out_data_received(endpoint_number, bcnt);
            break;
        case PKTSTS_SETUP_CMPLT: /* SETUP stage has completed 4 */
            //Re-enable the transmission on the endpoint
            SET_BIT(OUT_ENDPOINT(endpoint_number)->DOEPCTL, USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
            break;
        case PKTSTS_OUT_CMPLT: /* OUT stage has completed 3 */
            //Re-enable the transmission on the endpoint
            SET_BIT(OUT_ENDPOINT(endpoint_number)->DOEPCTL, USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
            break;
    }
}
static void gintsts_handler()
{

    volatile uint32_t gintsts = USB_OTG_FS_GLOBAL->GINTSTS;
    /* 
        Although we will set NACK bit for all OUT endpoints in usb reset handler, it still can receive SETUP packets.
        The main purpose of the reset signal is to reset the configuration of the USB core and get it in a default state 
        SETUP packet cannot be NACK, so it doesn't matter if you set the endpoint zero to not receive any data, 
        it will still receive SETUP packets
    */
    
    if(gintsts & USB_OTG_GINTSTS_USBRST)
    {
        usbrst_handler();
        //Clear the interrupt
        SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_USBRST);
    }
    if(gintsts & USB_OTG_GINTSTS_ENUMDNE)
    {
        enum_done_handler();
        //Clear the interrupt
        SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_ENUMDNE);
    }

    if(gintsts & USB_OTG_GINTSTS_RXFLVL)
    {
        rxflvl_handler();
        //Clear the interrupt
        SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_RXFLVL);
    }
    if(gintsts & USB_OTG_GINTSTS_IEPINT)
    {
        iepint_handler();
        //Clear the interrupt
        SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_IEPINT);
    }
    if(gintsts & USB_OTG_GINTSTS_OEPINT)
    {
        oepint_handler();
        //Clear the interrupt
        SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_OEPINT);
    }
    if(gintsts & USB_OTG_GINTSTS_SRQINT)
    {
        //Clear the interrupt
        SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_SRQINT);
    }
    if(gintsts & USB_OTG_GINTSTS_SOF)
    {
        //Clear the interrupt 
        SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_SOF);
    }
    if(gintsts & USB_OTG_GINTSTS_USBSUSP)
    {
        usb_device.device_state = USB_DEVICE_STATE_SUSPENDED;
        //Clear the interrupt 
        SET_BIT(USB_OTG_FS_GLOBAL->GINTSTS, USB_OTG_GINTSTS_USBSUSP);
    }
    usb_events.on_usb_polled();
}

const UsbDriver usb_driver = 
{
    .initialize_core = &initialize_core,
    .initialize_gpio_pins = &initialize_gpio_pins,
    .set_device_address = &set_device_address,
    .connect = &connect,
    .disconnect = &disconnect,
    .flush_rxfifo = &flush_rxfifo,
    .flush_txfifo = &flush_txfifo,
    .configure_in_endpoint = &configure_in_endpoint,
    .configure_out_endpoint = &configure_out_endpoint,
    .read_packet = &read_packet,
    .write_packet = &write_packet,
    .poll = &gintsts_handler
};