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

#include "pios_fsk.h"

/* Project Includes */
#include "pios.h"
#include "pios_dma.h"

struct pios_fsk_tim_cmd
{
	uint32_t ARR;
	//uint16_t RCR;
	//uint16_t RESERVED;
};

struct pios_fsk_tim_cmd_byte
{
	struct pios_fsk_tim_cmd bit[8];
};

struct pios_fsk_device
{
	// Configuration
	struct pios_fsk_cfg * cfg;

	// Buffer management (ring)
	struct pios_fsk_tim_cmd_byte * buffer;
	uint32_t nextWrite;
	uint32_t nextRead;

	// Timing information
	uint32_t bitHalfPeriod[2];
	uint32_t bitRepeat[2];
};

static struct pios_fsk_device fsk_dev;

static void PIOS_Fsk_tim_overflow_cb (uintptr_t tim_id, uintptr_t context, uint8_t channel, uint16_t count);
static void PIOS_Fsk_tim_edge_cb (uintptr_t tim_id, uintptr_t context, uint8_t chan_idx, uint16_t count);

const static struct pios_tim_callbacks tim_callbacks = {
	.overflow = PIOS_Fsk_tim_overflow_cb,
	.edge     = PIOS_Fsk_tim_edge_cb,
};

extern uintptr_t pios_com_aux_id;
void PIOS_DebugMsg( const char * msg)
{
	PIOS_WDG_Clear();
	PIOS_COM_SendFormattedString(pios_com_aux_id, msg);
}

void PIOS_SpinDebugMsg( char * msg )
{
	while(1)
	{
		PIOS_COM_SendFormattedString(pios_com_aux_id, msg);
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(200);
	}
}

static void PIOS_Fsk_DMA_Handler(void)
{
	PIOS_DebugMsg("DMA IRQ handler\n");
	if (DMA_GetFlagStatus(DMA2_IT_TC1))
	{
		DMA_ClearFlag(DMA2_IT_TC1);
		PIOS_DebugMsg("\tDMA Clear TC\n");
	}
	else if (DMA_GetFlagStatus(DMA2_IT_HT1))
	{
		DMA_ClearFlag(DMA2_IT_HT1);
		PIOS_DebugMsg("\tDMA Clear HT\n");
	}
	else
	{
		// Probably due to transfer errors
		DMA_ClearFlag((DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1));
		PIOS_DebugMsg("DMA Clear ALL\n");
	}
}

int32_t PIOS_Fsk_Init(struct pios_fsk_cfg * cfg)
{
	PIOS_DebugMsg("FSK Init\n");
	uintptr_t tim_id;
	if (PIOS_TIM_InitChannels(&tim_id, &cfg->channel, 1, &tim_callbacks, (uintptr_t)&fsk_dev)) {
		return -1;
	}

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	if (sizeof(struct pios_fsk_tim_cmd) == 2*2 &&
		0x4000004C == (uint32_t)(&TIM2->DMAR))
	{
		PIOS_DebugMsg("FSK What you expect\n");
	}
	else
	{
		PIOS_DebugMsg("FSK NOT what you expect\n");
	}

	fsk_dev.cfg = cfg;
	const uint32_t totalBufferSize = cfg->txBufferSize * sizeof(struct pios_fsk_tim_cmd_byte);
	void * rawBuffer = PIOS_malloc(totalBufferSize + 1);
	fsk_dev.buffer = (struct pios_fsk_tim_cmd_byte*)(((uint32_t)rawBuffer + 1) & 0xFFFFFFFE);

	const struct pios_tim_channel * chan = &cfg->channel;

	// Extrapolate device numbers
	{
		// Prescalers are assumed to be pre configured to 1 count = 1 us.
		const uint32_t usPerBit = 1000000 / cfg->baudRate;

		fsk_dev.bitHalfPeriod[0] = 1000000 / (cfg->lowFreq * 2);
		fsk_dev.bitHalfPeriod[1] = 1000000 / (cfg->highFreq * 2);

		fsk_dev.bitRepeat[0] = usPerBit / fsk_dev.bitHalfPeriod[0];
		fsk_dev.bitRepeat[1] = usPerBit / fsk_dev.bitHalfPeriod[1];
	}

	uint32_t bitSelection  = 0;

	PIOS_DebugMsg("FSK Config Timer\n");
	// (Re)Configure the timer
	{
		TIM_Cmd(chan->timer, DISABLE);
		// So configure the clock
		TIM_TimeBaseInitTypeDef tim_base = *fsk_dev.cfg->tim_base_init.time_base_init;
		tim_base.TIM_Period = fsk_dev.bitHalfPeriod[bitSelection];
		tim_base.TIM_RepetitionCounter = fsk_dev.bitRepeat[bitSelection];

		//TIM_ITConfig(chan->timer, TIM_IT_Update, ENABLE);

		TIM_TimeBaseInit(chan->timer, &tim_base);
	}

	PIOS_DebugMsg("FSK Config Timer Channel\n");
	// Configure the timer channel and GPIO pins
	{
		// Adjust the functionality defined:
		cfg->tim_oc_init.TIM_OCMode = TIM_OCMode_Toggle;
		cfg->tim_oc_init.TIM_OutputState = TIM_OutputState_Disable;
		cfg->tim_oc_init.TIM_OutputNState = TIM_OutputNState_Enable;
		cfg->tim_oc_init.TIM_Pulse = 0;//fsk_dev.bitPeriod[bitSelection] / 2;
		cfg->tim_oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
		cfg->tim_oc_init.TIM_OCNPolarity = TIM_OCPolarity_High;
		cfg->tim_oc_init.TIM_OCIdleState = TIM_OCIdleState_Reset;
		cfg->tim_oc_init.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

		/* Set up for output compare function */
		switch(chan->timer_chan) {
			case TIM_Channel_1:
				TIM_OC1Init(chan->timer, (TIM_OCInitTypeDef*)&cfg->tim_oc_init);
				TIM_OC1PreloadConfig(chan->timer, TIM_OCPreload_Enable);
				break;
			case TIM_Channel_2:
				TIM_OC2Init(chan->timer, (TIM_OCInitTypeDef*)&cfg->tim_oc_init);
				TIM_OC2PreloadConfig(chan->timer, TIM_OCPreload_Enable);
				break;
			case TIM_Channel_3:
				TIM_OC3Init(chan->timer, (TIM_OCInitTypeDef*)&cfg->tim_oc_init);
				TIM_OC3PreloadConfig(chan->timer, TIM_OCPreload_Enable);
				break;
			case TIM_Channel_4:
				TIM_OC4Init(chan->timer, (TIM_OCInitTypeDef*)&cfg->tim_oc_init);
				TIM_OC4PreloadConfig(chan->timer, TIM_OCPreload_Enable);
				break;
		}

		TIM_ARRPreloadConfig(chan->timer, ENABLE);
		TIM_CtrlPWMOutputs(chan->timer, ENABLE);
	}

	PIOS_DebugMsg("FSK Config NVIC \n");
	// Configure DMA channel
	{
		NVIC_InitTypeDef NVICInit = {
			.NVIC_IRQChannel                   = DMA2_Channel1_IRQn,
			.NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
			.NVIC_IRQChannelSubPriority        = 0,
			.NVIC_IRQChannelCmd                = ENABLE,
			};
		NVIC_Init(&NVICInit);
		NVIC_EnableIRQ(DMA2_Channel1_IRQn);
		PIOS_DebugMsg("FSK Config DMA\n");

		PIOS_DMA_Install_Interrupt_handler(DMA2_Channel1, &PIOS_Fsk_DMA_Handler);

		/* DeInitialize the DMA1 Stream2 */
		DMA_DeInit(DMA2_Channel1);

		DMA_InitTypeDef DMA_InitStructure;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&chan->timer->DMAR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)fsk_dev.buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_BufferSize = 2*2 + 0*totalBufferSize;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

		//DMA_Cmd(DMA2_Channel1, DISABLE);
		//while (DMA2_Channel1->CR & DMA_S2CR_EN);
		DMA_Init(DMA2_Channel1, &DMA_InitStructure);

		DMA_ITConfig(DMA2_Channel1, (DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1), ENABLE);
		//DMA_ITConfig(DMA2_Channel1, DMA2_FLAG_TC1 | DMA2_FLAG_GL1, ENABLE);
		DMA_ClearFlag((DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1));

		TIM_DMAConfig(chan->timer, TIM_DMABase_ARR, TIM_DMABurstLength_2Transfers);
		TIM_DMACmd(chan->timer, TIM_DMA_Update, ENABLE);
	}

	// Initialize the buffer's contents
	PIOS_DebugMsg("FSK Init Buffer contents\n");
	{
		for (uint32_t i = 0; i < cfg->txBufferSize; ++i)
		{
			for (uint32_t bit = 0; bit < 8; ++bit)
			{
				uint32_t bitSelection = bit % 2;
				fsk_dev.buffer[i].bit[bit].ARR = fsk_dev.bitHalfPeriod[bitSelection]*2;
				//fsk_dev.buffer[i].bit[bit].RCR = fsk_dev.bitRepeat[bitSelection];
			}
		}
	}

	// Enable the timer
	{
		DMA_ClearFlag((DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1));
		DMA_Cmd(DMA2_Channel1, ENABLE);
		PIOS_DebugMsg("DMA Enable\n");
		PIOS_DELAY_WaitmS(300);
		PIOS_DebugMsg("DMA Enable 2 Wait\n");
		PIOS_DELAY_WaitmS(300);
		PIOS_DebugMsg("DMA Enable 3 Wait\n");
		TIM_Cmd(chan->timer, ENABLE);

		PIOS_DebugMsg("Timer Enable\n");
	}


	// TEST CODE:
	{
		char buffer[32] = {0xFF};
		PIOS_Fsk_Write( &buffer[0], 32 );
	}
	PIOS_DebugMsg("End FSK init\n");
	//PIOS_SpinDebugMsg("EndInitSpin\n");
	return 0;
}

uint32_t PIOS_Fsk_Write(const void * pData, uint32_t length)
{
	//

	return 0;
}

static void PIOS_Fsk_tim_overflow_cb (uintptr_t tim_id, uintptr_t context, uint8_t channel, uint16_t count)
{
	PIOS_DebugMsg("TIM Overflow IRQ Handler\n");
#if 0 // PWM code
	struct pios_pwm_dev * pwm_dev = (struct pios_pwm_dev *)context;

	if (!PIOS_PWM_validate(pwm_dev)) {
		/* Invalid device specified */
		return;
	}

	if (channel >= pwm_dev->cfg->num_channels) {
		/* Channel out of range */
		return;
	}

	pwm_dev->us_since_update[channel] += count;
	if(pwm_dev->us_since_update[channel] >= PWM_SUPERVISOR_TIMEOUT) {
		pwm_dev->CaptureState[channel] = 0;
		pwm_dev->RiseValue[channel] = 0;
		pwm_dev->FallValue[channel] = 0;
		pwm_dev->CaptureValue[channel] = PIOS_RCVR_TIMEOUT;
		pwm_dev->us_since_update[channel] = 0;
	}
#endif
	return;
}

static void PIOS_Fsk_tim_edge_cb (uintptr_t tim_id, uintptr_t context, uint8_t chan_idx, uint16_t count)
{
	PIOS_DebugMsg("TIM Edge IRQ Handler\n");

#if 0 // PWM Code
	/* Recover our device context */
	struct pios_pwm_dev * pwm_dev = (struct pios_pwm_dev *)context;

	if (!PIOS_PWM_validate(pwm_dev)) {
		/* Invalid device specified */
		return;
	}

	if (chan_idx >= pwm_dev->cfg->num_channels) {
		/* Channel out of range */
		return;
	}

	const struct pios_tim_channel * chan = &pwm_dev->cfg->channels[chan_idx];

	if (pwm_dev->CaptureState[chan_idx] == 0) {
		pwm_dev->RiseValue[chan_idx] = count;
		pwm_dev->us_since_update[chan_idx] = 0;
	} else {
		pwm_dev->FallValue[chan_idx] = count;
	}

	// flip state machine and capture value here
	/* Simple rise or fall state machine */
	TIM_ICInitTypeDef TIM_ICInitStructure = pwm_dev->cfg->tim_ic_init;
	if (pwm_dev->CaptureState[chan_idx] == 0) {
		/* Switch states */
		pwm_dev->CaptureState[chan_idx] = 1;

		/* Switch polarity of input capture */
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		TIM_ICInitStructure.TIM_Channel = chan->timer_chan;
		TIM_ICInit(chan->timer, &TIM_ICInitStructure);
	} else {
		/* Capture computation */
		if (pwm_dev->FallValue[chan_idx] > pwm_dev->RiseValue[chan_idx]) {
			pwm_dev->CaptureValue[chan_idx] = (pwm_dev->FallValue[chan_idx] - pwm_dev->RiseValue[chan_idx]);
		} else {
			pwm_dev->CaptureValue[chan_idx] = ((chan->timer->ARR - pwm_dev->RiseValue[chan_idx]) + pwm_dev->FallValue[chan_idx]);
		}

		/* Switch states */
		pwm_dev->CaptureState[chan_idx] = 0;

		/* Increase supervisor counter */
		pwm_dev->CapCounter[chan_idx]++;

		/* Switch polarity of input capture */
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_Channel = chan->timer_chan;
		TIM_ICInit(chan->timer, &TIM_ICInitStructure);
	}
#endif
}
