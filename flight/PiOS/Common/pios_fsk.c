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

struct pios_fsk_device
{
	// Configuration
	struct pios_fsk_cfg * cfg;

	// Buffer management (ring)
	void * buffer;
	uint32_t nextWrite;
	uint32_t nextRead;
	char currentBit;

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

int32_t PIOS_Fsk_Init(struct pios_fsk_cfg * cfg)
{
	uintptr_t tim_id;
	if (PIOS_TIM_InitChannels(&tim_id, &cfg->channel, 1, &tim_callbacks, (uintptr_t)&fsk_dev)) {
		return -1;
	}

	fsk_dev.cfg = cfg;

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
	// (Re)Configure the timer
	{
		// So configure the clock
		TIM_TimeBaseInitTypeDef tim_base = *fsk_dev.cfg->tim_base_init.time_base_init;
		tim_base.TIM_Period = fsk_dev.bitHalfPeriod[bitSelection];
		tim_base.TIM_RepetitionCounter = fsk_dev.bitRepeat[bitSelection];

		TIM_TimeBaseInit(chan->timer, &tim_base);
	}

	// Configure the timer channel and GPIO pins
	{
		// Adjust the functionality defined:
		cfg->tim_oc_init.TIM_OCMode = TIM_OCMode_Toggle;
		cfg->tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
		cfg->tim_oc_init.TIM_OutputNState = TIM_OutputNState_Disable;
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
		TIM_Cmd(chan->timer, ENABLE);
	}

#if 0 // SERVO CODE
	/* Configure the channels to be in output compare mode */
	for (uint8_t i = 0; i < cfg->num_channels; i++) {
		const struct pios_tim_channel * chan = &cfg->channels[i];

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
		TIM_Cmd(chan->timer, ENABLE);
	}

	/* Allocate memory */
	output_timer_frequency_scaler = PIOS_malloc(servo_cfg->num_channels * sizeof(typeof(output_timer_frequency_scaler)));
	// Check that memory was successfully allocated, and return if not
	if (output_timer_frequency_scaler == NULL) {
		return -1;
	}
	memset(output_timer_frequency_scaler, 0, servo_cfg->num_channels * sizeof(typeof(output_timer_frequency_scaler)));
#endif


	// TEST CODE:
	{
		char buffer[32] = {0xFF};
		PIOS_Fsk_Write( &buffer[0], 32 );
	}

	return 0;
}

uint32_t PIOS_Fsk_Write(const void * pData, uint32_t length)
{
	//

	return 0;
}

static void PIOS_Fsk_tim_overflow_cb (uintptr_t tim_id, uintptr_t context, uint8_t channel, uint16_t count)
{
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
