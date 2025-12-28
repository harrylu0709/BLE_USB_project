

#include <stdio.h>
#include "main.h"
#include "usbd_framework.h"
#include "usb_device.h"
// #include "usbd_core.h"
// #include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "logger.h"
// #include "stm32f4xx_hal.h"
// #include "stm32f4xx_hal_def.h"
// #include "stm32f4xx_hal_pcd.h"

TIM_Handle_t Timer5_Handle;
//USBD_HandleTypeDef hUsbDeviceFS;
UsbDevice usb_device;
extern int16_t connectionHandler[2];
extern uint8_t user_disconnect_flag;
extern I2C_Handle_t g_ds1307I2CHandle;
//extern TIM_HandleTypeDef        htim6;
//extern PCD_HandleTypeDef 		hpcd_USB_OTG_FS;
/*
 * Embedded Systems Programming on ARM Cortex-M3/M4 Processor lecture 79

 * Section 4.4 of Cortex -M4 Devices Generic User Guide
*/

//semihosting init function
extern void initialise_monitor_handles(void);
uint32_t *pNVIC_ISPR1 = (uint32_t*)0xE000E204;
volatile int32_t moving_cnt = DETECT_MOVING_PERIOD;
volatile uint8_t dataAvailable = 0;
volatile uint8_t is_discoverable = 0;
// void SystemClock_Config(void);
// void HAL_EnableCompensationCell(void);
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void update_msg(unsigned char *msg);
uint8_t gatt_flag = 0;
int16_t acc_x;
int16_t acc_y;
int16_t acc_z;
int lost_cnt_sec = 0;
int dwt_cyc_unit = 0;
uint32_t rx_buffer[16];
UsbDevice usb_device;
/*
	part1
	https://docs.google.com/document/d/16My2VHtrzub1NycOldHII0mE8Xq-i0LB490brx3gpcc/edit?tab=t.0#heading=h.uu6cpqw6zjbz

	part2
	https://docs.google.com/document/d/17jYCwIollEr9pZSDwi219dB4Bd-WEjr2jH_KUeVLsSc/edit?tab=t.0#heading=h.lc5fcf8se9xg

	part3
	https://docs.google.com/document/d/1LF4pFkJgdbv1WE2SsijvQHM_YJ8eBC7BEsMf994AXnU/edit?tab=t.0#heading=h.lc5fcf8se9xg

https://github.com/ucsd-cse190b-w25/project-3-adding-low-energy-radio-communication-teamone?tab=readme-ov-file

For stm32f4 USB
https://github.com/STMicroelectronics/STM32CubeF4

target remote localhost:3333
monitor reset init
monitor flash write_image erase final_sh.elf
monitor arm semihosting enable
monitor reset halt
monitor resume

	*/


void TIM5_init(void)
{
	Timer5_Handle.pTIMx = TIM_5;

	if(((RCC_->CFGR >> 2) & 0x3) == 0)
		Timer5_Handle.Prescalar = 7999;
	else
		Timer5_Handle.Prescalar = 35999;	
		
	timer_set_ms(&Timer5_Handle, 1000);
	timer_init(&Timer5_Handle);
}	

void dwt_init(void) 
{
    DEMCR |= (1 << 24);     // Enable trace and debug
    DWT_CTRL |= 1;          // Enable the cycle counter
    DWT_CYCCNT = 0;         // Reset the counter
	//dwt_cyc_unit = 72;//HAL_RCC_GetHCLKFreq()/1000000;
}

void dwt_delay_us(uint32_t us) 
{
	
    uint32_t cycles = us * 72;//us * dwt_cyc_unit;  // 16 MHz = 16 cycles per microsecond
    uint32_t start = DWT_CYCCNT;
    while ((DWT_CYCCNT - start) < cycles);
}

void dwt_delay_ms(uint32_t ms) 
{
    while (ms--) 
	{
        dwt_delay_us(1000);
    }
}


int twos_complement_to_signed(int value, int bitWidth) 
{
    if (value & (1U << (bitWidth - 1))) 
	{
        // Negative number
        value |= ~((1U << bitWidth) - 1);  // Sign-extend the value
    }
    return value;
}

uint8_t Read_movement(void)
{
	int res_x;
	int res_y;
#if READ_Z_AXIS
	int res_z;
#endif
	LIS3DSH_read_xyz(&acc_x, &acc_y, &acc_z);
	res_x = twos_complement_to_signed(acc_x, 16);
	res_y = twos_complement_to_signed(acc_y, 16);
#if READ_Z_AXIS
	res_z = twos_complement_to_signed(acc_z, 16);
#endif
	
	if(res_x > ACC_TH_X || res_x  < -ACC_TH_X
		|| res_y > ACC_TH_Y || res_y  < -ACC_TH_Y
#if READ_Z_AXIS
		|| res_z > ACC_TH_Z || res_z  < -ACC_TH_Z
#endif
	  )
	{
		//moving_cnt = DETECT_MOVING_PERIOD;
		return 1;
	}
	return 0;
}

#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_NO_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIO_C

#define OTG_FS_OverCurrent_Pin GPIO_PIN_NO_5
#define OTG_FS_OverCurrent_GPIO_Port GPIO_D
GPIO_Handle_t	USB_FS_GPIO;
void USB_FS_GPIO_INIT(void)
{
    USB_FS_GPIO.pGPIOx = OTG_FS_PowerSwitchOn_GPIO_Port;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinNumber = OTG_FS_PowerSwitchOn_Pin;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    GPIO_Init(&USB_FS_GPIO);
	GPIO_WriteToOutputPin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, 1);

    USB_FS_GPIO.pGPIOx = OTG_FS_OverCurrent_GPIO_Port;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinNumber = OTG_FS_OverCurrent_Pin;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USB_FS_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&USB_FS_GPIO);
}

int main(void)
{
	
	initialise_monitor_handles();
	//return 0;
	// HAL_Init();
	// //Set_PLL_Clock();
	// SystemClock_Config();

	// //USB_FS_GPIO_INIT();

	// if(USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
	// {
	// 	printf("fail 1\n");
	// }else printf("ok\n");
	
	// if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
	// {
	// 	printf("fail 2\n");
	// }else printf("ok\n");

	// if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
	// {
	// 	printf("fail 3\n");
	// }else printf("ok\n");

	// if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
	// {
	// 	printf("fail 4\n");
	// }else printf("ok\n");

	SystemInit();
	dwt_init();

	usb_device.ptr_out_buffer = rx_buffer;
	usbd_initialize(&usb_device);
	LIS3DSH_init();
	led_init();

	xnucleo_init();

	GPIO_WriteToOutputPin(BLE_GPIO_PORT,BLE_RST_Pin,GPIO_PIN_RESET);
	dwt_delay_ms(10);
	GPIO_WriteToOutputPin(BLE_GPIO_PORT,BLE_RST_Pin,GPIO_PIN_SET);
	
	ble_init();
	dwt_delay_ms(10);
	GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_BLUE, 0);
	TIM5_init();
#if ADVERTISE_THEN_NON_DISCOVERABLE
	setDiscoverability(0);
#endif
    // /* Manually trigger TIM5 interrupt */
	//*pNVIC_ISPR1 |= (1 << (IRQ_NO_TIM5 % 32));

	uint8_t nonDiscoverable = 0;

	while(1)
	{
		//printf("%d\n",hUsbDeviceFS.dev_state);
		if(moving_cnt > 0)
		{
			//setDiscoverability(0);
			Read_movement();
		}
		else
		{
			//if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
			if(usb_device.device_state == USB_DEVICE_STATE_CONFIGURED)
			{
				leds_set(1);
				int i = MSG_RATE;
				int move_flag = 1;				
				while (i--) 
				{
					dwt_delay_ms(1);
					if(Read_movement() || (usb_device.device_state == USB_DEVICE_STATE_SUSPENDED))
					{
						moving_cnt = DETECT_MOVING_PERIOD;
						lost_cnt_sec = 0;
						move_flag = 0;
						GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_GREEN, 0);
						break;
					}
    			}				
				if(move_flag)
				{
					unsigned char msg_str[] = "freeze for        \n";
					update_msg(msg_str);
					// if(cdc_transmit(msg_str, sizeof(msg_str)) != USBD_OK)
					// {
					// 	moving_cnt = DETECT_MOVING_PERIOD;
					// 	GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_GREEN, 0);
					// 	break;
					// }
					cdc_transmit(msg_str, sizeof(msg_str));
				}
			}
			else
			{
				if(!is_discoverable)
				{
					setDiscoverability(1);
					leds_set(1);
				}	
				if(!nonDiscoverable && GPIO_ReadFromInputPin(BLE_GPIO_PORT, BLE_INT_Pin))
				{
					catchBLE();
				}
				else
				{
					int i = MSG_RATE;
					int move_flag = 1;				
					while (i--) 
					{
						dwt_delay_ms(1);
						if(Read_movement())
						{
							moving_cnt = DETECT_MOVING_PERIOD;
							lost_cnt_sec = 0;
							move_flag = 0;
							GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_GREEN, 0);
							GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_BLUE, 0);
							disconnectBLE();
							break;
						}
						if(usb_device.device_state == USB_DEVICE_STATE_CONFIGURED)
						{
							lost_cnt_sec = 0;
							move_flag = 0;
							disconnectBLE();
							GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_GREEN, 0);
							GPIO_WriteToOutputPin(LED_GPIO_PORT, LED_GPIO_BLUE, 0);
							moving_cnt = DETECT_MOVING_PERIOD;
							break;
						}
						if(user_disconnect_flag)
						{
							lost_cnt_sec = 0;
							move_flag = 0;
							user_disconnect_flag = 0;
							reset_connection_state();
							moving_cnt = DETECT_MOVING_PERIOD;
							break;
						}
					}
					if(move_flag)
					{
						unsigned char msg_str[] = "freeze for        ";
						update_msg(msg_str);
						if(gatt_flag)	updateCharValue(NORDIC_UART_SERVICE_HANDLE, WRITE_CHAR_HANDLE, 0, sizeof(msg_str), msg_str);
					}
				}
			}
			// Wait for interrupt, only uncomment if low power is needed
			//__WFI();
		}
	}
	return 0;
}

void update_msg(unsigned char *msg)
{
	lost_cnt_sec += (MSG_RATE/1000);

	if(lost_cnt_sec >= 100)
	{
		msg[11] = (lost_cnt_sec / 100) +'0';
		msg[12] = (lost_cnt_sec / 10) % 10 +'0';
		msg[13] = (lost_cnt_sec % 10)+'0';
		memcpy(msg + 14, " sec", 4);
	}
	else if(lost_cnt_sec >=10)
	{
		msg[11] = (lost_cnt_sec / 10) % 10 +'0';
		msg[12] = (lost_cnt_sec % 10)+'0';
		memcpy(msg + 13, " sec", 4);
	}
	else
	{
		msg[11] = (lost_cnt_sec % 10)+'0';
		memcpy(msg + 12, " sec", 4);
	}
}

void TIM5_IRQHandler(void)
{
	if(moving_cnt > 0)
	{
		moving_cnt--;
		GPIO_ToggleOutputPin(LED_GPIO_PORT, LED_GPIO_RED);
	}
	TIM_IRQHandling(&Timer5_Handle);
 	// Timer5_Handle.pTIMx->SR &= ~(1<<0);
}

// void TIM6_DAC_IRQHandler(void)
// {
// 	HAL_TIM_IRQHandler(&htim6);
// }

void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */
  /* USER CODE END OTG_FS_IRQn 0 */
  //HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);

  usb_driver.poll();
  //GPIO_ToggleOutputPin(GPIO_D, LED_GPIO_BLUE);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

void EXTI9_5_IRQHandler(void)
{
	dataAvailable=1;
	GPIO_IRQHandling(BLE_INT_Pin); //clear the pending event from exti line
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	return;
}

// void SystemClock_Config(void)
// {
// //   printf("set clock 1\n");
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Configure the main internal regulator output voltage
//   */
//   __HAL_RCC_PWR_CLK_ENABLE();
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLM = 4;
//   RCC_OscInitStruct.PLL.PLLN = 72;
//   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//   RCC_OscInitStruct.PLL.PLLQ = 3;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     //Error_Handler();
// 	printf("fail\n");
//   }
// //   printf("set clock 2\n");
//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != 0x00)
//   {
//     //Error_Handler();
// 	printf("fail\n");
//   }
// //   printf("set clock ok\n");
// }

// void HAL_EnableCompensationCell(void)
// {
//   *(volatile uint32_t *)CMPCR_CMP_PD_BB = (uint32_t)ENABLE;
// }


// void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
// {
//   uint32_t reg_value;
//   uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);             /* only values 0..7 are used          */

//   reg_value  =  SCB->AIRCR;                                                   /* read old register configuration    */
//   reg_value &= ~((uint32_t)(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)); /* clear bits to change               */
//   reg_value  =  (reg_value                                   |
//                 ((uint32_t)0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
//                 (PriorityGroupTmp << 8U)                      );              /* Insert write key and priorty group */
//   SCB->AIRCR =  reg_value;
// }


// void HAL_MspInit(void)
// {
//   /* USER CODE BEGIN MspInit 0 */

//   /* USER CODE END MspInit 0 */

//   __HAL_RCC_SYSCFG_CLK_ENABLE();
//   __HAL_RCC_PWR_CLK_ENABLE();
// }

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   /* USER CODE BEGIN Callback 0 */

//   /* USER CODE END Callback 0 */
//   if (htim->Instance == TIM6) 
//   {
//     HAL_IncTick();
//   }
//   /* USER CODE BEGIN Callback 1 */

//   /* USER CODE END Callback 1 */
// }
// #endif