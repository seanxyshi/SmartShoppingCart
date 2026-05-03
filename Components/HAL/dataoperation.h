/*
 *******************************************************************************
 *
 *******************************************************************************
 */
#ifndef _SMARTCART_DATAOPERATION_H
#define _SMARTCART_DATAOPERATION_H

#include  "stm32f10x.h"

void FillString(u8 *pString,u32 nStrLen,u8 nFillData);
u16 U16HighLowByteSwap(u16 nByte);
u32 U32HighLowByteSwap(u32 nByte);

#endif  //_SMARTCART_DATAOPERATION_H