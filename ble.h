/*
 * ble.h
 *
 *  Created on: 18 gen 2021
 *      Author: UTPM9 (modificaiton by ADS)
 */

#ifndef BLE_H_
#define BLE_H_

//#include "ble_commands.h"
#include "stm32f407.h"
#define     BLE_GPIO_PORT                       GPIO_E
#define     BLE_INT_Pin                         GPIO_PIN_NO_9
#define     BLE_RST_Pin                         GPIO_PIN_NO_10
#define     BLE_CS_Pin                          GPIO_PIN_NO_11

#define 	SPI2_SCK 			                GPIO_PIN_NO_13
#define 	SPI2_MISO 			                GPIO_PIN_NO_14
#define 	SPI2_MOSI 			                GPIO_PIN_NO_15

#define     BLE_OK                              0
#define     ADVERTISE_THEN_NON_DISCOVERABLE     0
#define EVENT_STARTUP_SIZE 6
#define ACI_GATT_INIT_COMPLETE_SIZE 7
#define SET_ATTRIBUTES(n) (n)
#define SET_CONTENT_LENGTH(n) (n)

extern uint8_t NORDIC_UART_SERVICE_HANDLE[2];

extern uint8_t READ_CHAR_HANDLE[];
extern uint8_t WRITE_CHAR_HANDLE[];

extern uint8_t* rxEvent;

void ble_gpio_init(void);
void SPI2_GPIOInits(void);
void SPI2_Inits(void);
void xnucleo_init(void);

//function for starting the BLE protocol
void ble_init(void);

void standbyBle();

//function that gets a pending event and save the data in the pointer *container
int fetchBleEvent(uint8_t *container, int size);

//check if the event that was fetched is what I expected
int checkEventResp(uint8_t *event, uint8_t *reference, int size);

void sendCommand(uint8_t *command,int size);

void catchBLE(void);

void setConnectable(void);

int BLE_command(uint8_t* command, int size, uint8_t* result, int sizeRes, int returnHandles);

void addService(uint8_t* UUID, uint8_t* handle, int attributes);

void addCharacteristic(uint8_t* UUID,uint8_t* handleChar, uint8_t* handleService, uint8_t maxsize, uint8_t proprieties,uint8_t secPermissions,uint8_t gattEvtMask,uint8_t encryKeySize,uint8_t isVariable);

void updateCharValue(uint8_t* handleService,uint8_t* handleChar, int offset, int size,uint8_t* data);
void reset_connection_state(void);
void disconnectBLE(void);

void setDiscoverability(uint8_t mode);
#endif
