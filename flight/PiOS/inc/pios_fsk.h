/**
 ******************************************************************************
 * @addtogroup PIOS PIOS
 * @{
 * @addtogroup
 * @brief
 * @{
 *
 * @file       PIOS_fsk.h
 * @author     The Tau Labs Team, http://www.taulabls.org Copyright (C) 2013.
 * @brief
 * @see        The GNU Public License (GPL) Version 3
 *****************************************************************************/
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef PIOS_FSK_H_
#define PIOS_FSK_H_

#include <pios.h>
#include <pios_stm32.h>
#include <pios_tim_priv.h>

struct pios_fsk_cfg
{
	uint32_t baudRate; // Bits per second // 1225
	uint32_t highFreq; // High frequency in Hz // 7350
	uint32_t lowFreq; // Low frequency in Hz // 4900
	uint32_t txBufferSize; // Maximum tx buffer size required // 32

	struct pios_tim_clock_cfg tim_base_init;
	TIM_OCInitTypeDef tim_oc_init;
	GPIO_InitTypeDef gpio_init;
	uint32_t remap;
	struct pios_tim_channel channel;
};

extern int32_t PIOS_FSK_TX_Init(struct pios_fsk_cfg * cfg);
extern uint32_t PIOS_FSK_TX_Write(const void * pData, uint32_t length);

#endif /* PIOS_FSK_H_ */
