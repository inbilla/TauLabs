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

/**
 * AFSK Protocol:
 * n x HIGH = Carrier setup
 * for each byte:
 *   1 x LOW = Start bit
 *   8 x XXX = Data bits (LSB first)
 *   1 x HIGH = Stop bit
 * 1 x HIGH = Push bit
 */


#include "pios_fsk.h"

/* Project Includes */
#include "pios.h"
#include "pios_dma.h"

#pragma pack(push, 1)

struct pios_fsk_tx_tim_cmd
{
	uint16_t ARR;
	uint16_t RCR;
};

#pragma pack(pop)

struct pios_fsk_tx_command_buffer
{
	// Command buffer management (ring)
	struct pios_fsk_tx_tim_cmd * buffer;
	uint32_t length;

	// Staging buffer info
	struct pios_fsk_tx_tim_cmd stagingBuffer[10];
	uint32_t nextStagingCmd;
	uint32_t numStagingCmds;
};

void PIOS_FSK_TX_cmd_init( struct pios_fsk_tx_command_buffer * cmdBuffer, uint32_t size );
void PIOS_FSK_TX_cmd_generateBits( struct pios_fsk_tx_tim_cmd * cmd, uint32_t polarity, uint16_t count );
uint32_t PIOS_FSK_TX_cmd_generateByte( struct pios_fsk_tx_tim_cmd * cmdBuffer, uint8_t byte );

struct pios_fsk_tx_data_buffer
{
	uint8_t * buffer;
	uint32_t nextRead;
	uint32_t nextWrite;
	uint32_t length;
};

void PIOS_FSK_TX_data_init( struct pios_fsk_tx_data_buffer * dataBuffer, uint32_t size );
uint32_t PIOS_FSK_TX_data_readByte( struct pios_fsk_tx_data_buffer * dataBuffer, uint8_t * byte );
uint32_t PIOS_FSK_TX_data_write( struct pios_fsk_tx_data_buffer* dataBuffer, const uint8_t * data, uint32_t length );
uint32_t PIOS_FSK_TX_data_empty( struct pios_fsk_tx_data_buffer* dataBuffer );
uint32_t PIOS_FSK_TX_data_available( struct pios_fsk_tx_data_buffer* dataBuffer );

struct pios_fsk_tx_device
{
	// Configuration
	struct pios_fsk_cfg * cfg;

	struct pios_fsk_tx_command_buffer cmdBuffer;
	struct pios_fsk_tx_data_buffer dataBuffer;

	// Timing information
	uint32_t bitHalfPeriod[2];
	uint32_t bitRepeat[2];
};

static struct pios_fsk_tx_device fsk_dev;

static void PIOS_FSK_TX_tim_overflow_cb (uintptr_t tim_id, uintptr_t context, uint8_t channel, uint16_t count);
static void PIOS_FSK_TX_tim_edge_cb (uintptr_t tim_id, uintptr_t context, uint8_t chan_idx, uint16_t count);
static void PIOS_FSK_TX_DMA_Handler(void);

const static struct pios_tim_callbacks tim_callbacks = {
	.overflow = PIOS_FSK_TX_tim_overflow_cb,
	.edge     = PIOS_FSK_TX_tim_edge_cb,
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

#define ASSERT_IMPL( exp, file, lineNum ) \
	do{\
		if (!(exp))\
		{\
			PIOS_COM_SendFormattedString( pios_com_aux_id, "Assert Failed: ");\
			/*PIOS_COM_SendFormattedString( pios_com_aux_id, "%s", file);*/\
			PIOS_COM_SendFormattedString( pios_com_aux_id, "@%d - ", lineNum);\
			/*PIOS_COM_SendFormattedString( pios_com_aux_id, "%s", #exp);*/\
			PIOS_COM_SendFormattedString( pios_com_aux_id, "\n");\
		}\
	}while(0)

#define ASSERT( exp ) ASSERT_IMPL( exp, __FILE__, __LINE__ )

int32_t PIOS_FSK_TX_Init(struct pios_fsk_cfg * cfg)
{
	PIOS_DebugMsg("FSK Init\n");
	uintptr_t tim_id;
	if (PIOS_TIM_InitChannels(&tim_id, &cfg->channel, 1, &tim_callbacks, (uintptr_t)&fsk_dev)) {
		return -1;
	}

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	PIOS_COM_SendFormattedString(pios_com_aux_id, "sizeof(fsk_tim_cmd): %d\n", sizeof(struct pios_fsk_tx_tim_cmd));

	ASSERT( sizeof(struct pios_fsk_tx_tim_cmd) == 2*2 );
	ASSERT( 0x4000004C == (uint32_t)(&TIM2->DMAR) );

	fsk_dev.cfg = cfg;

	// Initialize the buffers
	PIOS_FSK_TX_data_init( &fsk_dev.dataBuffer, cfg->txBufferSize );
	PIOS_FSK_TX_cmd_init( &fsk_dev.cmdBuffer, 128 );

	const struct pios_tim_channel * chan = &cfg->channel;

	// Extrapolate device numbers
	{
		// Prescalers are assumed to be pre configured to 1 count = 1 us.
		const uint32_t usPerBit = 1000000 / cfg->baudRate;

		fsk_dev.bitHalfPeriod[0] = 1000000 / (cfg->lowFreq * 2);
		fsk_dev.bitHalfPeriod[1] = 1000000 / (cfg->highFreq * 2);

		fsk_dev.bitRepeat[0] = usPerBit / fsk_dev.bitHalfPeriod[0] / 2 - 1;
		fsk_dev.bitRepeat[1] = usPerBit / fsk_dev.bitHalfPeriod[1] / 2 - 1;
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

		PIOS_DMA_Install_Interrupt_handler(DMA2_Channel1, &PIOS_FSK_TX_DMA_Handler);

		/* DeInitialize the DMA1 Stream2 */
		DMA_DeInit(DMA2_Channel1);

		DMA_InitTypeDef DMA_InitStructure;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&chan->timer->DMAR);
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)fsk_dev.cmdBuffer.buffer;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
		DMA_InitStructure.DMA_BufferSize = fsk_dev.cmdBuffer.length * 2; // num elements to transfer
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

		DMA_ITConfig(DMA2_Channel1, (DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1), DISABLE);
		DMA_ClearFlag((DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1));
		DMA_ITConfig(DMA2_Channel1, DMA2_FLAG_HT1 | DMA2_FLAG_TC1, ENABLE);

		TIM_DMAConfig(chan->timer, TIM_DMABase_ARR, TIM_DMABurstLength_2Transfers);
		TIM_DMACmd(chan->timer, TIM_DMA_Update, ENABLE);
	}

	// Initialize the buffer's contents
	PIOS_DebugMsg("FSK Init Buffer contents\n");
	{
		for (uint32_t i = 0; i < fsk_dev.cmdBuffer.length; ++i)
		{
			uint32_t bitSelection = 0; // LOW as default
			PIOS_FSK_TX_cmd_generateBits( &fsk_dev.cmdBuffer.buffer[i], bitSelection, 1 );

			//fsk_dev.cmdBuffer.buffer[i].ARR = 2 + 2 * i;
			//fsk_dev.cmdBuffer.buffer[i].RCR = 2 + 2 * i + 1;
		}
	}

	uint32_t halfPeriod = fsk_dev.bitHalfPeriod[0];
	uint32_t repeat = fsk_dev.bitRepeat[0];
	PIOS_COM_SendFormattedString(pios_com_aux_id, "\tTest (%d, %d)\n", halfPeriod, repeat);
	halfPeriod = fsk_dev.bitHalfPeriod[1];
	repeat = fsk_dev.bitRepeat[1];
	PIOS_COM_SendFormattedString(pios_com_aux_id, "\tTest (%d, %d)\n", halfPeriod, repeat);

	// Enable the timer
	{
		DMA_ClearFlag((DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1));
		DMA_Cmd(DMA2_Channel1, ENABLE);
		PIOS_DebugMsg("DMA Enable\n");

		TIM_Cmd(chan->timer, ENABLE);
		PIOS_DebugMsg("Timer Enable\n");
	}


	// TEST CODE:
	char buffer[32] = {
		0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
		0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
		0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
		0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
		};
	{
		PIOS_FSK_TX_Write( &buffer[0], 32 );
	}
	PIOS_DebugMsg("End FSK init\n");
	//PIOS_SpinDebugMsg("EndInitSpin\n");

	while (1)
	{
		PIOS_WDG_Clear();
		PIOS_COM_SendFormattedString(pios_com_aux_id, "TIM ARR: %d, RCR %d\n", TIM8->ARR, (uint32_t)TIM8->RCR);
		PIOS_DELAY_WaitmS(200);
		PIOS_WDG_Clear();
		PIOS_DELAY_WaitmS(200);
		PIOS_WDG_Clear();
		PIOS_FSK_TX_Write( &buffer[0], 32 );
	}

	return 0;
}

void PIOS_FSK_TX_cmd_generateBits( struct pios_fsk_tx_tim_cmd * cmd, uint32_t polarity, uint16_t count )
{
	cmd->ARR = fsk_dev.bitHalfPeriod[polarity];
	cmd->RCR = fsk_dev.bitRepeat[polarity] * count;
}

uint32_t PIOS_FSK_TX_cmd_generateByte( struct pios_fsk_tx_tim_cmd * cmdBuffer, uint8_t byte )
{
	// Returns number of commands generated
	// Worst case, a byte will require 10 discrete timer commands
	// L|HLHLHLHL|H
	// Best case, a byte will require 2 timer commands
	// L|LLLLLLLL|H
	// L|HHHHHHHH|H
	// Generate the least commands as possible
	uint8_t curPolarity = 0;
	uint8_t count = 1;
	uint8_t generatedCmds = 0;
	uint32_t byteAndStop = byte | 0x00000100;
	for (uint8_t i = 0; i < 8 + 1; ++i)
	{
		uint8_t polarity = (byteAndStop & (1 << i)) != 0;

		if (polarity == curPolarity)
		{
			count++;
		}
		else
		{
			PIOS_FSK_TX_cmd_generateBits( &cmdBuffer[generatedCmds++], curPolarity, count );
			curPolarity = polarity;
			count = 1;
		}
	}
	PIOS_FSK_TX_cmd_generateBits( &cmdBuffer[generatedCmds++], curPolarity, count );
	return generatedCmds;
}

void PIOS_FSK_TX_cmd_init( struct pios_fsk_tx_command_buffer * cmdBuffer, uint32_t size )
{
	// ASSUMPTION: CmdBuffer length is longer than a whole staging buffer
	// so total length of cmdbuffer must be at least 2 staging buffers long
	// since we update in halves
	ASSERT( size >= 2 * 10 );

	// Buffer must be aligned to 16bit boundary
	cmdBuffer->length = size;
	const uint32_t cmdBufferSize = size * sizeof(struct pios_fsk_tx_tim_cmd);
	void * rawBuffer = PIOS_malloc(cmdBufferSize + 1);
	cmdBuffer->buffer = (struct pios_fsk_tx_tim_cmd*)(((uint32_t)rawBuffer + 1) & 0xFFFFFFFE);
	cmdBuffer->nextStagingCmd = 0;
	cmdBuffer->numStagingCmds = 0;
}


void PIOS_FSK_TX_data_init( struct pios_fsk_tx_data_buffer * dataBuffer, uint32_t size )
{
	// ASSUMPTION: One byte of this buffer is always going to be unused
	// when the buffer is "Full" because read == write when the
	// buffer is empty. So size must be larger than 1.
	// Otherwise the buffer will be both empty and full at the same time.
	ASSERT( size > 1 );

	dataBuffer->length = size;
	dataBuffer->buffer = (uint8_t*)PIOS_malloc( size );
	dataBuffer->nextWrite = 0;
	dataBuffer->nextRead = 0;
}

uint32_t PIOS_FSK_TX_data_empty( struct pios_fsk_tx_data_buffer* dataBuffer )
{
	return dataBuffer->nextRead == dataBuffer->nextWrite;
}

uint32_t PIOS_FSK_TX_data_available( struct pios_fsk_tx_data_buffer* dataBuffer )
{
	// Difference between write/read pointers.
	// add length on because mod of negative numbers is undefined
	// and once we have underflowed there is no guarantee
	// it has wrapped correctly, unless max_uint can be divided evenly by
	// length
	return (dataBuffer->nextRead - dataBuffer->nextWrite + dataBuffer->length - 1)
		% dataBuffer->length;
}

uint32_t PIOS_FSK_TX_data_write(
	struct pios_fsk_tx_data_buffer* dataBuffer,
	const uint8_t * data, uint32_t length )
{
	ASSERT( dataBuffer->nextWrite < dataBuffer->length );

	uint32_t available = PIOS_FSK_TX_data_available( dataBuffer );
	ASSERT( available < dataBuffer->length );
	if (available == 0)
	{
		return 0;
	}
	uint32_t totalToCopy = (length < available) ? length : available;

	// Potentially we need to write this data in 2 steps
	// The first step is from current position to the end of the buffer.
	// The second step is from the start of the buffer to the read position
	uint32_t numToEnd = dataBuffer->length - dataBuffer->nextWrite;
	uint32_t numToCopy = (numToEnd < totalToCopy) ? numToEnd : totalToCopy;
	memcpy( &dataBuffer->buffer[dataBuffer->nextWrite], data, numToCopy );

	dataBuffer->nextWrite = (dataBuffer->nextWrite + numToCopy) % dataBuffer->length;
	data += numToCopy;
	numToCopy = totalToCopy - numToCopy;

	if (numToCopy == 0)
	{
		// Complete copy done
		// We haven't wrapped.
		// So we're done
		ASSERT( dataBuffer->nextWrite < dataBuffer->length );
		ASSERT( dataBuffer->nextWrite != dataBuffer->nextRead );
		return totalToCopy;
	}

	// Copy the remaining data
	// Don't allow writes all the way up to the read marker
	// because that would mean nextWrite == nextRead again,
	// which is our empty condition.
	// The inital calculation of the available space should
	// cover this
	memcpy( &dataBuffer->buffer[0], data, numToCopy );
	dataBuffer->nextWrite = numToCopy;

	ASSERT( dataBuffer->nextWrite < dataBuffer->length );
	ASSERT( dataBuffer->nextWrite != dataBuffer->nextRead );

	return totalToCopy;
}

uint32_t PIOS_FSK_TX_data_readByte( struct pios_fsk_tx_data_buffer* dataBuffer, uint8_t * byte )
{
	if (PIOS_FSK_TX_data_empty( dataBuffer ))
	{
		return 0;
	}

	ASSERT( dataBuffer->nextRead < dataBuffer->length );

	*byte = dataBuffer->buffer[dataBuffer->nextRead];
	dataBuffer->nextRead = (dataBuffer->nextRead + 1) % dataBuffer->length;

	ASSERT( dataBuffer->nextRead < dataBuffer->length );

	return 1;
}

void PIOS_FSK_TX_updateCommandBuffer(
	struct pios_fsk_tx_data_buffer * dataBuffer,
	struct pios_fsk_tx_command_buffer * cmdBuffer,
	struct pios_fsk_tx_tim_cmd * buffer,
	uint32_t length)
{
	// When we update the command buffer, we update half of it,
	// because the other half is yet to be sent to the timer.
	// When we update the buffer, the entire half of the command buffer
	// must be filled, because the DMA controller WILL send the data.
	// if we have no data to fill it with, we should fill it
	// with stop bits. That way the next start bit will be ready.
	// If we have filled one half of the command buffer with stop bits
	// and we are now updating again, then we should fill the other half
	// as well, and disable the timer.
	// If we have disabled the timer, we should fill from the start of
	// the command buffer and then restart the timer to set it all off again.

	// Because we must always fill the command buffer, we may not be
	// able to fit all of a byte's commands into the buffer at a time.
	// So we have a 10 command long staging buffer that will always
	// fit a byte of commands, and when filling the command buffer
	// we always start from there for commands first before generating
	// the commands for more bytes.

	ASSERT( dataBuffer->nextWrite < dataBuffer->length );
	ASSERT( dataBuffer->nextRead < dataBuffer->length );
	ASSERT( cmdBuffer->nextStagingCmd <= cmdBuffer->numStagingCmds );

	// Begin filling from staging buffer
	uint32_t curOffset = cmdBuffer->numStagingCmds - cmdBuffer->nextStagingCmd;

	memcpy( buffer, &cmdBuffer->stagingBuffer[cmdBuffer->nextStagingCmd],
		curOffset * sizeof(struct pios_fsk_tx_tim_cmd));
	cmdBuffer->nextStagingCmd = 0;
	cmdBuffer->numStagingCmds = 0;

	// Now start generating directly into the command buffer
	while (curOffset < length)
	{
		uint8_t byte;
		if (PIOS_FSK_TX_data_readByte( dataBuffer, &byte ))
		{
			// If we have more than 10 cmds left
			uint32_t cmdsRemaining = length - curOffset;
			if (cmdsRemaining > 10)
			{
				// Generate directly into the command buffer
				curOffset += PIOS_FSK_TX_cmd_generateByte( &buffer[curOffset], byte);
			}
			else
			{
				// We need to generate via the staging buffer
				cmdBuffer->numStagingCmds = PIOS_FSK_TX_cmd_generateByte( &cmdBuffer->stagingBuffer[0], byte );
				// Now copy from staging into destination
				uint8_t copyComplete = cmdBuffer->numStagingCmds <= cmdsRemaining;
				uint32_t amountToCopy = (copyComplete) ? cmdBuffer->numStagingCmds : cmdsRemaining;
				memcpy( &buffer[curOffset], &cmdBuffer->stagingBuffer[0],
					amountToCopy * sizeof(struct pios_fsk_tx_tim_cmd));
				cmdBuffer->nextStagingCmd = amountToCopy;
				curOffset += amountToCopy;

				// If we didn't get to copy the entire staging buffer
				// then we are full!
				if (!copyComplete)
				{
					return;
				}
			}
		}
		else
		{
			// Fill the rest of the buffer with HIGH cmds
			for (;curOffset < length; ++curOffset)
			{
				PIOS_FSK_TX_cmd_generateBits( &buffer[curOffset], 1, 1 );
			}

			// Return because there is no more data to be generated
			return;
		}
	}

	// If we got here then one of the direct generations
	// into the command buffer managed to fill the buffer
	// just right
}

void PIOS_FSK_TX_handleDMAtransferComplete( uint32_t complete )
{
	//return;
	uint32_t updateLength = fsk_dev.cmdBuffer.length / 2;
	uint32_t offset = complete ? updateLength : 0;

	//uint32_t halfPeriod = fsk_dev.bitHalfPeriod[complete];
	//uint32_t repeat = fsk_dev.bitRepeat[complete];


	PIOS_COM_SendFormattedString(pios_com_aux_id, "\tWrite\n");
	//PIOS_COM_SendFormattedString(pios_com_aux_id, "\tWrite %d, %d, %d, (%d, %d)\n", offset, updateLength, complete, halfPeriod, repeat);

	//for (size_t i = 0; i < updateLength; ++i)
	//{
		//fsk_dev.cmdBuffer.buffer[i + offset].ARR = 100;//fsk_dev.bitHalfPeriod[i % 2];
		//fsk_dev.cmdBuffer.buffer[i + offset].RCR = 3;//fsk_dev.bitRepeat[i % 2] * 100;
		//fsk_dev.cmdBuffer.buffer[i + offset].ARR = 2 + 2 * i;
		//fsk_dev.cmdBuffer.buffer[i + offset].RCR = 2 + 2 * i + 1;
		//PIOS_FSK_TX_cmd_generateBits( &fsk_dev.cmdBuffer.buffer[offset + i], complete, 1 );
	//}

	PIOS_FSK_TX_updateCommandBuffer(
		&fsk_dev.dataBuffer,
		&fsk_dev.cmdBuffer,
		&fsk_dev.cmdBuffer.buffer[offset],
		updateLength);
}

static void PIOS_FSK_TX_DMA_Handler(void)
{
	if (DMA_GetFlagStatus(DMA2_IT_TC1))
	{
		DMA_ClearFlag(DMA2_IT_TC1);
		PIOS_FSK_TX_handleDMAtransferComplete( 1 );
	}
	else if (DMA_GetFlagStatus(DMA2_IT_HT1))
	{
		DMA_ClearFlag(DMA2_IT_HT1);
		PIOS_FSK_TX_handleDMAtransferComplete( 0 );
	}
	else
	{
		// Probably due to transfer errors
		DMA_ClearFlag((DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1));
		PIOS_DebugMsg("DMA Clear ALL\n");
	}
}

uint32_t PIOS_FSK_TX_Write(const void * pData, uint32_t length)
{
	// Copy the data into the buffer
	uint32_t copiedBytes = PIOS_FSK_TX_data_write(
		&fsk_dev.dataBuffer,
		(const uint8_t *)pData,
		length);

	return copiedBytes;
}

static void PIOS_FSK_TX_tim_overflow_cb (uintptr_t tim_id, uintptr_t context, uint8_t channel, uint16_t count)
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

static void PIOS_FSK_TX_tim_edge_cb (uintptr_t tim_id, uintptr_t context, uint8_t chan_idx, uint16_t count)
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