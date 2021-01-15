/**********************************************************************
* $Id$		norflash_sst39vf3201.h			2011-06-02
*//**
* @file		norflash_sst39vf3201.h
* @brief	Contains all macro definitions and function prototypes
*			support for external NOR Flash SamSung SST39VF3201
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
/** @defgroup NorFlash_SST39VF3201 NorFlash SST39VF3201
 * @ingroup LPC177x_8xCMSIS_Board_Support
 * @{
 */
#ifndef __NORFLASH_SST39VF3201_H_
#define __NORFLASH_SST39VF3201_H_

#include "lpc177x_8x_emc.h"

/*****************************************************************************
 * Defines and typedefs
 ****************************************************************************/
#define NOR_FLASH_BASE		0x90000000
#define NOR_FLASH_SIZE		0x00080000

#define GET_ADDR(addr)	(volatile uint16_t *)(NOR_FLASH_BASE | (addr<<1))

#define SECTOR_SIZE		0x800	/* Must be 2048 words for 39VF160 */
#define BLOCK_SIZE		0x8000	/* Must be 32K words for 39VF160  */

#define SST_ID			0xBF    /* SST Manufacturer's ID code   */
#define SST_39VF160		0x734B  /* SST 39VF160 device code      */

#define PROGRAM_TIMEOUT	0x08000000

extern void NORFLASHInit( void );
extern void NORFLASHErase( void );
extern uint32_t NORFLASHCheckID( void );
extern uint32_t NORFLASHWriteWord( uint32_t Addr, uint16_t Data );
extern uint32_t ToggleBitCheck( uint32_t Addr, uint16_t Data );

#endif /* __NORFLASH_SST39VF3201_H_ */

/**
 * @}
 */
