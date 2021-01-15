/*----------------------------------------------------------------------------
 *      LPC2378 Ethernet Definitions
 *----------------------------------------------------------------------------
 *      Name:    EMAC.H
 *      Purpose: Philips LPC2378 EMAC hardware definitions
 *----------------------------------------------------------------------------
 *      Copyright (c) 2006 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __EMAC_H
#define __EMAC_H
#include "lpc_types.h"

#define MYMAC_1         0x1E            /* our ethernet (MAC) address        */
#define MYMAC_2         0x30            /* (MUST be unique in LAN!)          */
#define MYMAC_3         0x6c
#define MYMAC_4         0xa2
#define MYMAC_5         0x45
#define MYMAC_6         0x5e

/* Defined MAC address */   
#define emacETHADDR0   0xD8
#define emacETHADDR1   0x7A
#define emacETHADDR2   0xA5
#define emacETHADDR3   0x00
#define emacETHADDR4   0x00
#define emacETHADDR5   0x01
 
/* Defined IP address */
#define	 emacIPADDR0  192
#define	 emacIPADDR1  168
#define	 emacIPADDR2  1
#define	 emacIPADDR3  100

/* Defined gateway address */
#define emacGATEWAY_ADDR0  192
#define emacGATEWAY_ADDR1  168
#define emacGATEWAY_ADDR2  1
#define emacGATEWAY_ADDR3  1

/* Defined mask address */
#define emacNET_MASK0  255
#define emacNET_MASK1  255
#define emacNET_MASK2  255
#define emacNET_MASK3  0

// prototypes
void           Init_EMAC(void);
uint16_t       ReadHalfWordBE_EMAC(void);
void           CopyToFrame_EMAC(void *Source, unsigned int Size);
void           CopyFromFrame_EMAC(void *Dest, uint16_t Size);
void           DummyReadFrame_EMAC(uint16_t Size);
void           ReadFrame_EMAC(void *Dest, uint16_t Size);
uint16_t       StartReadFrame(void);
void           EndReadFrame(void);
unsigned int   CheckFrameReceived(void);
unsigned int   Rdy4Tx(void);
void           SendFrame(void *Source, unsigned int Size);
void           FrameReceiveCallback(uint16_t* pData, uint32_t size);
void           ErrorReceiveCallback(int32_t errCode);

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

