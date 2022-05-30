/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  *
  * Copyright (c) 2016 Mori
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  * associated documentation files (the "Software"), to deal in the Software without restriction,
  * including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
  * subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
  * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  *
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stdio.h"
#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbh_hub.h"

#include "log.h"
#include "stdio.h"


USBH_HandleTypeDef hUSBHost[5];

static void USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId);
static void hub_process();
static void system_clock_config(void);
/*!
 * \brief Initialize the SWO trace port for debug message printing
 * \param portBits Port bit mask to be configured
 * \param cpuCoreFreqHz CPU core clock frequency in Hz
 */
//void SWO_Init(uint32_t portBits, uint32_t cpuCoreFreqHz) {
//  uint32_t SWOSpeed = 64000; /* default 64k baud rate */
//  uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1; /* SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock */
//
//  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk; /* enable trace in core debug */
//  *((volatile unsigned *)(ITM_BASE + 0x400F0)) = 0x00000002; /* "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO NRZ, 1: SWO Manchester encoding) */
//  *((volatile unsigned *)(ITM_BASE + 0x40010)) = SWOPrescaler; /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */
//  *((volatile unsigned *)(ITM_BASE + 0x00FB0)) = 0xC5ACCE55; /* ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC */
//  ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk; /* ITM Trace Control Register */
//  ITM->TPR = ITM_TPR_PRIVMASK_Msk; /* ITM Trace Privilege Register */
//  ITM->TER = portBits; /* ITM Trace Enable Register. Enabled tracing on stimulus ports. One bit per stimulus port. */
//  *((volatile unsigned *)(ITM_BASE + 0x01000)) = 0x400003FE; /* DWT_CTRL */
//  *((volatile unsigned *)(ITM_BASE + 0x40304)) = 0x00000100; /* Formatter and Flush Control Register */
//}

//-------------------------------------------------------------------------------------------------
/*
    Initialize the SWO trace port for debug message printing
    portMask : Stimulus bit mask to be configured
    cpuCoreFreqHz : CPU core clock frequency in Hz
    baudrate : SWO frequency in Hz
*/
static volatile int bItmAvailable;
void swoInit (uint32_t portMask, uint32_t cpuCoreFreqHz, uint32_t baudrate)
{
    uint32_t SWOPrescaler = (cpuCoreFreqHz / baudrate) - 1u ; // baudrate in Hz, note that cpuCoreFreqHz is expected to match the CPU core clock

    CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;      // Debug Exception and Monitor Control Register (DEMCR): enable trace in core debug
    DBGMCU->CR  = 0x00000027u ;                         // DBGMCU_CR : TRACE_IOEN DBG_STANDBY DBG_STOP  DBG_SLEEP
    TPI->SPPR   = 0x00000002u ;                         // Selected PIN Protocol Register: Select which protocol to use for trace output (2: SWO)
    TPI->ACPR   = SWOPrescaler ;                        // Async Clock Prescaler Register: Scale the baud rate of the asynchronous output
    ITM->LAR    = 0xC5ACCE55u ;                         // ITM Lock Access Register: C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC
    ITM->TCR    = 0x0001000Du ;                         // ITM Trace Control Register
    ITM->TPR    = ITM_TPR_PRIVMASK_Msk ;                // ITM Trace Privilege Register: All stimulus ports
    ITM->TER    = portMask ;                            // ITM Trace Enable Register: Enabled tracing on stimulus ports. One bit per stimulus port.
    DWT->CTRL   = 0x400003FEu ;                         // Data Watchpoint and Trace Register
    TPI->FFCR   = 0x00000100u ;                         // Formatter and Flush Control Register

    // ITM/SWO works only if enabled from debugger.
    // If ITM stimulus 0 is not free, don't try to send data to SWO
    if (ITM->PORT [0].u8 == 1)
    {
        bItmAvailable = 1 ;
    }
}


int main(void)
{
	uint32_t i = 0;

	HAL_Init();
	system_clock_config();

#if LOG_MODE == 1
    BSP_UART_Init();
    LOG_INIT(USARTx, 3800000);
#elif LOG_MODE == 2
	swoInit (0x1, SystemCoreClock , 2000000);
#endif

	BSP_LED_Init(LED4);



	LOG("\033[2J\033[H");
	LOG(" ");
	LOG("APP RUNNING...");
	LOG("MCU-ID %08X", DBGMCU->IDCODE);
	printf("hello world");
	printf("2\r\n");
	memset(&hUSBHost[0], 0, sizeof(USBH_HandleTypeDef));

	hUSBHost[0].valid   = 1;
	hUSBHost[0].address = USBH_DEVICE_ADDRESS;
	hUSBHost[0].Pipes   = USBH_malloc(sizeof(uint32_t) * USBH_MAX_PIPES_NBR);

	USBH_Init(&hUSBHost[0], USBH_UserProcess, ID_USB_HOST_FS);
	USBH_RegisterClass(&hUSBHost[0], USBH_HID_CLASS);
	USBH_RegisterClass(&hUSBHost[0], USBH_HUB_CLASS);

	USBH_Start(&hUSBHost[0]);

	while(1)
	{
		if (i++ > 150000)
			i = 0;

		if(i > 0 && i <= 2500)
			BSP_LED_On(LED4);
		else
			BSP_LED_Off(LED4);

		hub_process();
	}
}

void hub_process()
{
	static uint8_t current_loop = -1;
	static USBH_HandleTypeDef *_phost = 0;

	if(_phost != NULL && _phost->valid == 1)
	{
		USBH_Process(_phost);

		if(_phost->busy)
			return;
	}

	while(1)
	{
		current_loop++;

		if(current_loop > MAX_HUB_PORTS)
			current_loop = 0;

		if(hUSBHost[current_loop].valid)
		{
			_phost = &hUSBHost[current_loop];
			USBH_LL_SetupEP0(_phost);

			if(_phost->valid == 3)
			{
//LOG("PROCESSING ATTACH %d", _phost->address);
				_phost->valid = 1;
				_phost->busy  = 1;
			}

			break;
		}
	}

#if 0
	if(_phost != NULL && _phost->valid)
	{
		HID_MOUSE_Info_TypeDef *minfo;
		minfo = USBH_HID_GetMouseInfo(_phost);
		if(minfo != NULL)
		{
LOG("BUTTON %d", minfo->buttons[0]);
		}
		else
		{
			HID_KEYBD_Info_TypeDef *kinfo;
			kinfo = USBH_HID_GetKeybdInfo(_phost);
			if(kinfo != NULL)
			{
LOG("KEYB %d", kinfo->keys[0]);
			}
		}
	}
#endif
}

void USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId)
{
	switch (vId)
	{
		case HOST_USER_SELECT_CONFIGURATION:
			break;

		case HOST_USER_CLASS_SELECTED:
			break;

		case HOST_USER_CLASS_ACTIVE:
			break;

		case HOST_USER_CONNECTION:
			break;

		case HOST_USER_DISCONNECTION:
			break;

		case HOST_USER_UNRECOVERED_ERROR:
			break;

		default:
			break;
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void system_clock_config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is
     	 clocked below the maximum system frequency, to update the voltage scaling value
     	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14);
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14);
	}
}
