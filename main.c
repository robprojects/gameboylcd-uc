/**
 * Drive ili9341 display from gameboy using an STM32 blue pill
*/

#include <stdio.h>
#include "platform_config.h"
#include "ILI9341/ili9341_spi.h"
#include "ILI9341/ili9341_core.h"
#include "ILI9341/ili9341_graph.h"
#include "ILI9341/ili9341_text.h"


inline void ili9341_sendData16_rob(u16 *data, u32 n)
{
	ILI9341_DC_SET;
	dev_spi_start(m_spi);
	dev_spi_send16(m_spi, data, n);
	/*dev_spi_wait(m_spi);
	 does -> while(SPI_I2S_GetFlagStatus(spi->spi, SPI_I2S_FLAG_BSY) == SET);
	dev_spi_stop(m_spi);

	does -> spi->port->ODR |= spi->nss;*/
}


// 4 lines of buffer
#define RX_BUFFER_SIZE (160)
uint16_t Rx_Buffer[RX_BUFFER_SIZE*4];


int main(void)
{

	/* Initialize the delay timer */
	delay_init(SystemCoreClock);


	/* Initialize LCD */
	LCD_init();
	LCD_setOrientation(ORIENTATION_LANDSCAPE_MIRROR);
	LCD_fillScreen(GREEN);

	LCD_setAddressWindowToWrite(0, 0, (u16 ) (160-1),
			(u16 ) (143));

	dev_spi_set16(m_spi);

	/*
	 * Setup IO for Gameboy LCD
	 * PB7 - PIXCLK TIM4_CH2
	 * PB3 - VSYNC
	 * PB4 - HSYNC
	 * PB5 - DATA0
	 * PB6 - DATA1
	 *
	 * 160x144
	 */

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef gpioStructure;
	gpioStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	gpioStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioStructure);

	/* Timer used to trigger the DMA on every CLK rising edge */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 0;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 1024;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timerInitStructure);

	TIM_ICInitTypeDef icinit;
	icinit.TIM_Channel = TIM_Channel_2;
	icinit.TIM_ICFilter = 0;
	icinit.TIM_ICPolarity = TIM_ICPolarity_Rising;
	icinit.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	icinit.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM4, &icinit);

	TIM_DMAConfig(TIM4, TIM_DMABase_CCR2, TIM_DMABurstLength_1Transfer);
	TIM_DMACmd(TIM4, TIM_DMA_CC2, ENABLE);

	TIM_Cmd(TIM4, ENABLE);

	/* Setup DMA to capture Gameboy LCD signals on pixclk */
	DMA_InitTypeDef DMA_InitStruct;
	DMA_StructInit(&DMA_InitStruct);
	DMA_InitStruct.DMA_BufferSize = (RX_BUFFER_SIZE);
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)Rx_Buffer;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStruct.DMA_PeripheralBaseAddr = ((uint32_t)&GPIOB->IDR);
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA1_Channel4, &DMA_InitStruct);


	int buf = 0;

	while(1) {

		// wait for VSYNC
		//while(!(GPIO_ReadInputData(GPIOB) & (1<<2)));

		for(int line=0; line<144; line++) {
			// wait for HSYNC
			int s = 0;
			while(!(s&0x10)) s = GPIO_ReadInputData(GPIOB);

			// start DMA of line from gameboy
			DMA1_Channel4->CMAR = (uint32_t)&(Rx_Buffer[160*((buf+2)%4)]);
			DMA_Cmd(DMA1_Channel4, ENABLE);

			// write line to screen (don't wait for it to finish!)
			ili9341_sendData16_rob(&(Rx_Buffer[160*((buf+0)%4)]), 160);

			// modify data - expand two data bits into 16bit grayscale colour
			int b = 160*((buf+1)%4);
			for (int i=0; i<160; i++) {
				uint16_t p = Rx_Buffer[b+i];
				uint16_t val = (p & 0x60)>>5;
				Rx_Buffer[b+i] = ~((val << 3) | (val << 9) | (val << 14));
			}

			// wait for DMA of line from gameboy
			while(!DMA_GetFlagStatus(DMA1_FLAG_TC4));
			DMA_ClearFlag(DMA1_FLAG_TC4);
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA_SetCurrDataCounter(DMA1_Channel4, RX_BUFFER_SIZE);

			buf = (buf + 1)%4;
		}
	}
}


