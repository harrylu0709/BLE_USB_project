#include "LIS3DSH.h"
#include "led.h"
#include "ble.h"

#define SYSTICK_TIM_CLK   				16000000UL //16MHz
#define NON_DISCOVER_SEC				10
#define LED_PERIOD_MS	  				500
#define DETECT_MOVING_PERIOD	 	  	(NON_DISCOVER_SEC*(1000/LED_PERIOD_MS))
#define MSG_RATE			  		    2000
#define DWT_CTRL    					(*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  					(*(volatile uint32_t*)0xE0001004)
#define DEMCR       					(*(volatile uint32_t*)0xE000EDFC)
#define READ_Z_AXIS						0

extern void dwt_delay_ms(uint32_t ms);
extern uint8_t gatt_flag;
extern int dataAvailable;
extern volatile uint8_t is_discoverable;