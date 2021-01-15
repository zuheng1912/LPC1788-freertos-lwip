/**********************************************************************
* $Id$		bsp.h			2011-06-02
*//**
* @file		bsp.h
* @brief	Contains basic information about the board that can
* 			be using with the current code package. It may
* 			include some header file for the components mounted
* 			on the board. Or else some neccessary hardware (IOs)
* 			settings for the using board may be involved.
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


/* Peripheral group ----------------------------------------------------------- */
/** @defgroup BoardSupport Board Support
 * @ingroup LPC177x_8xCMSIS_Board_Support
 * @{
 */

#ifndef __BSP_H
#define __BSP_H


//List the board that have been supported
/** Default board */
#define _DEFAULT_BOARD				(0)
/** LPC1788 OEM board connects with QVGA base board v1.2 */
#define _QVGA_BOARD					(1)
/** LPC1788 OEM Board rev A and OEM Base Board rev A */
#define _EA_PA_BOARD				(2)
/** LPC1788 IAR Start Kit Rev.B */
#define _IAR_OLIMEX_BOARD			(3)

/** Current using board definition */
#define _CURR_USING_BRD				_EA_PA_BOARD


//List the NandFlash that have been supported_RUNNING_NANDFLASH_NONE
#define _RUNNING_NANDFLASH_NONE			(0)
#define _RUNNING_NANDFLASH_K9F1G08U0C	(1)
#define _RUNNING_NANDFLASH_K9F1G08U0A	(2)

#define _CUR_USING_NANDFLASH			(_RUNNING_NANDFLASH_K9F1G08U0C)

#if (_CURR_USING_BRD == _QVGA_BOARD)
//Driver for PHY of LAN DP83848C IC
#include "phylan_dp83848c.h"


//ADC input preset on this board
#define BRD_ADC_PREPARED_CHANNEL		(ADC_CHANNEL_2)
#define BRD_ADC_PREPARED_INTR			(ADC_ADINTEN2)

#define BRD_ADC_PREPARED_CH_PORT		(0)
#define BRD_ADC_PREPARED_CH_PIN			(25)
#define BRD_ADC_PREPARED_CH_FUNC_NO		(1)


//LED indicators preset
#define BRD_LED_1_CONNECTED_PORT		(1)
#define BRD_LED_1_CONNECTED_PIN			(13)
#define BRD_LED_1_CONNECTED_MASK		(1 << BRD_LED_1_CONNECTED_PIN)

#define BRD_LED_2_CONNECTED_PORT		(0)
#define BRD_LED_2_CONNECTED_PIN			(13)
#define BRD_LED_2_CONNECTED_MASK		(1 << BRD_LED_2_CONNECTED_PIN)


//PIO interrupt preset
#define BRD_PIO_USED_INTR_PORT			(0)
#define BRD_PIO_USED_INTR_PIN			(25)
#define BRD_PIO_USED_INTR_MASK			(1 << BRD_PIO_USED_INTR_PIN)


//MCI power active levell
#define BRD_MCI_POWERED_ACTIVE_LEVEL	(0)


//Timer preset
#define BRD_TIMER_USED				(LPC_TIM0)
#define BRD_TIM_INTR_USED			(TIMER0_IRQn)

#define BRD_TIM_CAP_LINKED_PORT		(1)
#define BRD_TIM_CAP_LINKED_PIN		(26)


// NandFlash preset
#define _CUR_USING_NANDFLASH			(_RUNNING_NANDFLASH_K9F1G08U0A)

#endif

#if (_CURR_USING_BRD == _EA_PA_BOARD)
//Driver for PHY of LAN LAN8720 IC
#include "phylan_lan8720.h"


//ADC input preset on this board
#define BRD_ADC_PREPARED_CHANNEL		(ADC_CHANNEL_0)
#define BRD_ADC_PREPARED_INTR			(ADC_ADINTEN0)

#define BRD_ADC_PREPARED_CH_PORT		(0)
#define BRD_ADC_PREPARED_CH_PIN			(23)
#define BRD_ADC_PREPARED_CH_FUNC_NO		(1)

#define BRD_DAC_PREPARED_CH_PORT		(0)
#define BRD_DAC_PREPARED_CH_PIN			(26)
#define BRD_DAC_CONNECTED_MASK		(1 << BRD_DAC_PREPARED_CH_PIN)
#define BRD_DAC_PREPARED_CH_FUNC_NO		(2)

//LED indicators preset
#define BRD_LED_1_CONNECTED_PORT		(2)
#define BRD_LED_1_CONNECTED_PIN			(27)
#define BRD_LED_1_CONNECTED_MASK		(1 << BRD_LED_1_CONNECTED_PIN)

#define BRD_LED_2_CONNECTED_PORT		(2)
#define BRD_LED_2_CONNECTED_PIN			(26)
#define BRD_LED_2_CONNECTED_MASK		(1 << BRD_LED_2_CONNECTED_PIN)

#define BRD_LED_3_CONNECTED_PORT		(2)
#define BRD_LED_3_CONNECTED_PIN			(23)
#define BRD_LED_3_CONNECTED_MASK		(1 << BRD_LED_3_CONNECTED_PIN)

#define BRD_LED_4_CONNECTED_PORT		(2)
#define BRD_LED_4_CONNECTED_PIN			(22)
#define BRD_LED_4_CONNECTED_MASK		(1 << BRD_LED_4_CONNECTED_PIN)

//PIO interrupt preset
#define BRD_PIO_USED1_INTR_PORT			(2)
#define BRD_PIO_USED1_INTR_PIN			(19)
#define BRD_PIO_USED1_INTR_MASK			(1 << BRD_PIO_USED1_INTR_PIN)

#define BRD_PIO_USED2_INTR_PORT			(4)
#define BRD_PIO_USED2_INTR_PIN			(26)
#define BRD_PIO_USED2_INTR_MASK			(1 << BRD_PIO_USED2_INTR_PIN)

#define BRD_PIO_USED_INTR_PORT			(2)
#define BRD_PIO_USED_INTR_PIN			(10)
#define BRD_PIO_USED_INTR_MASK			(1 << BRD_PIO_USED_INTR_PIN)

#define BRD_PIO_USED3_INTR_PORT			(4)
#define BRD_PIO_USED3_INTR_PIN			(27)
#define BRD_PIO_USED3_INTR_MASK			(1 << BRD_PIO_USED3_INTR_PIN)


//MCI power active levell
#define BRD_MCI_POWERED_ACTIVE_LEVEL	(0)


//Timer preset
#define BRD_TIMER_USED				(LPC_TIM3)
#define BRD_TIM_INTR_USED			(TIMER3_IRQn)

#define BRD_TIM_CAP_LINKED_PORT		(2)
#define BRD_TIM_CAP_LINKED_PIN		(22)


// NandFlash preset
//#define _CUR_USING_NANDFLASH			(_RUNNING_NANDFLASH_K9F1G08U0C)

#endif

#if (_CURR_USING_BRD == _IAR_OLIMEX_BOARD)
//Driver for PHY of LAN KS8721B IC
#include "phylan_ks8721b.h"


//ADC input preset on this board
#define BRD_ADC_PREPARED_CHANNEL		(ADC_CHANNEL_7)
#define BRD_ADC_PREPARED_INTR			(ADC_ADINTEN7)

#define BRD_ADC_PREPARED_CH_PORT		(0)
#define BRD_ADC_PREPARED_CH_PIN			(13)
#define BRD_ADC_PREPARED_CH_FUNC_NO		(3)


//LED indicators preset
#define BRD_LED_1_CONNECTED_PORT		(1)
#define BRD_LED_1_CONNECTED_PIN			(13)
#define BRD_LED_1_CONNECTED_MASK		(1 << BRD_LED_1_CONNECTED_PIN)

#define BRD_LED_2_CONNECTED_PORT		(1)
#define BRD_LED_2_CONNECTED_PIN			(18)
#define BRD_LED_2_CONNECTED_MASK		(1 << BRD_LED_2_CONNECTED_PIN)

//PIO interrupt preset
#define BRD_PIO_USED_INTR_PORT			(0)
#define BRD_PIO_USED_INTR_PIN			(13)
#define BRD_PIO_USED_INTR_MASK			(1 << BRD_PIO_USED_INTR_PIN)


//MCI power active levell
#define BRD_MCI_POWERED_ACTIVE_LEVEL	(1)


//Timer preset
#define BRD_TIMER_USED				(LPC_TIM2)
#define BRD_TIM_INTR_USED			(TIMER2_IRQn)

#define BRD_TIM_CAP_LINKED_PORT		(0)
#define BRD_TIM_CAP_LINKED_PIN		(4)

#endif


#endif//BSP_H

/**
 * @}
 */
