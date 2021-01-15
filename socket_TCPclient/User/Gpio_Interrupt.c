/**********************************************************************
* $Id$		Gpio_Interrupt.c		2011-06-02
*//**
* @file		Gpio_Interrupt.c
* @brief	This example used to test GPIO interrupt function
* @version	1.0
* @date		02. June. 2011
* @author	NXP MCU SW Application Team
*
* Copyright(C) 2011, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
**********************************************************************/
#include "LPC177x_8x.h"
#include "lpc177x_8x_gpio.h"
#include "lpc177x_8x_pinsel.h"
#include "bsp.h"


/** @defgroup GPIO_Interrupt	GPIO Interrupt
 * @ingroup GPIO_Examples
 * @{
 */

/************************** PRIVATE DEFINITIONS *************************/

#define LED1_PORT	(BRD_LED_1_CONNECTED_PORT)
#define LED1_BYTE	((uint32_t)BRD_LED_1_CONNECTED_PIN / 8)
#define LED1_BIT	(1 << ((uint32_t)BRD_LED_1_CONNECTED_PIN % 8))

#define LED2_PORT	(BRD_LED_2_CONNECTED_PORT)
#define LED2_BYTE	((uint32_t)BRD_LED_2_CONNECTED_PIN / 8)
#define LED2_BIT	(1 << ((uint32_t)BRD_LED_2_CONNECTED_PIN % 8))

#define LED3_PORT	(BRD_LED_3_CONNECTED_PORT)
#define LED3_BYTE	((uint32_t)BRD_LED_3_CONNECTED_PIN / 8)
#define LED3_BIT	(1 << ((uint32_t)BRD_LED_3_CONNECTED_PIN % 8))

#define LED4_PORT	(BRD_LED_4_CONNECTED_PORT)
#define LED4_BYTE	((uint32_t)BRD_LED_4_CONNECTED_PIN / 8)
#define LED4_BIT	(1 << ((uint32_t)BRD_LED_4_CONNECTED_PIN % 8))


/************************** PRIVATE FUNCTIONS *************************/
void GPIO_IRQHandler(void);

void delay (void);

/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/
/*********************************************************************//**
 * @brief		External interrupt 3 handler sub-routine
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void GPIO_IRQHandler(void)
{	
													
	if(GPIO_GetIntStatus(2, 19, 1))
	{
		GPIO_ClearInt(2, 1<<19);
		if(FIO_ByteReadValue(LED2_PORT,LED2_BYTE)&(LED2_BIT))
			FIO_ByteClearValue(LED2_PORT, LED2_BYTE, LED2_BIT);
		else
			FIO_ByteSetValue(LED2_PORT, LED2_BYTE, LED2_BIT);
	}
	else if(GPIO_GetIntStatus(2, 10, 1))
	{
		GPIO_ClearInt(BRD_PIO_USED_INTR_PORT, BRD_PIO_USED_INTR_MASK);
		if(FIO_ByteReadValue(LED3_PORT,LED3_BYTE)&(LED3_BIT))
			FIO_ByteClearValue(LED3_PORT, LED3_BYTE, LED3_BIT);
		else
			FIO_ByteSetValue(LED3_PORT, LED3_BYTE, LED3_BIT);
	}
}

/*-------------------------PRIVATE FUNCTIONS------------------------------*/
/*********************************************************************//**
 * @brief		Delay function
 * @param[in]	None
 * @return 		None
 **********************************************************************/
void delay (void)
{
  unsigned int i;

  for (i = 0; i < 0x1000000; i++)
  {

  }
}


/*-------------------------MAIN FUNCTION------------------------------*/
/*********************************************************************//**
 * @brief		c_entry: Main program body
 * @param[in]	None
 * @return 		int
 **********************************************************************/
int c_entry (void)
{								//第2端口			第3组			第3位								1为输出
	FIO_ByteSetDir(LED1_PORT, LED1_BYTE, LED1_BIT, GPIO_DIRECTION_OUTPUT);
	FIO_ByteSetDir(LED2_PORT, LED2_BYTE, LED2_BIT, GPIO_DIRECTION_OUTPUT);	
	FIO_ByteSetDir(LED3_PORT, LED3_BYTE, LED3_BIT, GPIO_DIRECTION_OUTPUT);
	FIO_ByteSetDir(LED4_PORT, LED4_BYTE, LED4_BIT, GPIO_DIRECTION_OUTPUT);

	// Turn off all LEDs
	FIO_ByteSetValue(LED1_PORT, LED1_BYTE, LED1_BIT);
	FIO_ByteSetValue(LED2_PORT, LED2_BYTE, LED2_BIT);
	FIO_ByteSetValue(LED3_PORT, LED3_BYTE, LED3_BIT);
	FIO_ByteSetValue(LED4_PORT, LED4_BYTE, LED4_BIT);

	// Enable GPIO interrupt
	GPIO_IntCmd(2, 1<<19 | 1<<10, 1);
	
	NVIC_SetPriority(GPIO_IRQn, 1);	
	NVIC_EnableIRQ(GPIO_IRQn);

	while (1)
	{
		FIO_ByteClearValue(LED1_PORT, LED1_BYTE, LED1_BIT);
		FIO_ByteSetValue(LED4_PORT, LED4_BYTE, LED4_BIT);

		delay();

				
		FIO_ByteSetValue(LED1_PORT, LED1_BYTE, LED1_BIT);
		FIO_ByteClearValue(LED4_PORT, LED4_BYTE, LED4_BIT);
		delay();
	}
}


/* With ARM and GHS toolsets, the entry point is main() - this will
   allow the linker to generate wrapper code to setup stacks, allocate
   heap area, and initialize and copy code and data segments. For GNU
   toolsets, the entry point is through __start() in the crt0_gnu.asm
   file, and that startup code will setup stacks and data */
int main(void)
{
    return c_entry();
}

/*
 * @}
*/
