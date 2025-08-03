/*
 * ble_commands.h
 *
 *  Created on: 26 gen 2021
 *      Author: UTPM9
 */

/***
 * Commands:
 * Bytes | Description
 * 0	 | HCI Packet Type (Indicates this is a command packet) (0x01 is command, 0x04 is event)
   1-2	 | Opcode
   3 	 | Parameter Length (n bytes follow this field)
 * */

/***
 * Events:
 * Bytes | Description
 * 0	 | HCI Packet Type (Indicates this is a command packet) (0x01 is command, 0x04 is event)
   1	 | Event Code (Command Complete is 0x0E)
   2	 | Parameter Length (n bytes follow this field)
   3	 | Number of HCI Command Packets Allowed
   4-5	 | Opcode of Command sent (Little Endian format)
   6     | Status
 * */

 #ifndef INC_BLE_COMMANDS_H_
 #define INC_BLE_COMMANDS_H_
 
 #include <stdint.h>
 
 #define READABLE 0x02
 #define NOTIFIBLE 0x10
 #define WRITABLE  0x04
 #define WRITE_WITHOUT_RESP   0x04

 #endif /* INC_BLE_COMMANDS_H_ */
 