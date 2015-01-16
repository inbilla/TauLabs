/**
 ******************************************************************************
 * @addtogroup PIOS PIOS
 * @{
 * @addtogroup
 * @brief
 * @{
 *
 * @file       PIOS_fsk.c
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

#include "PIOS_fsk.h"

/* Project Includes */
#include "pios.h"

static const struct pios_fsk_cfg * fsk_cfg;

int32_t PIOS_Fsk_Init(const struct pios_fsk_cfg * cfg)
{
	uintptr_t tim_id;
	if (PIOS_TIM_InitChannels(&tim_id, cfg->channels, cfg->num_channels, NULL, 0)) {
		return -1;
	}
}

uint32_t PIOS_Fsk_Write(const void * pData, uint32_t length)
{

}

void PIOS_Fsk_ISR_Edge()
{

}

