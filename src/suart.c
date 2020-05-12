/*
 * suart.c
 *
 *  Created on: 25 Dec 2019
 *      Author: Steve Chang
 */
#include "stm32f10x.h"
#include "suart.h"

#include "freertos/include/FreeRTOS.h"
#include "freertos/include/task.h"
#include "freertos/include/queue.h"
#include "freertos/include/semphr.h"

#include "bool.h"
#include <string.h>

/*
*
*/

void Tim2_Init(uint32_t prescaler, uint32_t period)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* NVIC_PriorityGroup */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 11; /* FreeRTOS *FromISR() API can be called safely from priority 11 ~ 15 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  //基础设置，时基和比较输出设置，由于这里只需定时，所以不用OC比较输出
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  
  TIM_DeInit(TIM2);

  TIM_TimeBaseStructure.TIM_Period = period;//装载值
  //prescaler is 72, that is 72000000/72/1000 = 1000Hz;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;//分频系数
  //set clock division 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
  //count up
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  //clear the TIM2 overflow interrupt flag
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  //TIM2 overflow interrupt enable
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  //enable TIM2
  TIM_Cmd(TIM2, DISABLE);
}

void Tim2_Enable(void)
{
  TIM_Cmd(TIM2, ENABLE);
}

void Tim2_Disable(void)
{
  TIM_Cmd(TIM2, DISABLE);
}

//prescaler is 24, period 24, that is 72000000/25/25 = 115200Hz;

//prescaler is 74, period 24, that is 72000000/75/25 = 38400Hz;
//prescaler is 74, period 49, that is 72000000/75/50 = 19200Hz;
//prescaler is 74, period 99, that is 72000000/75/100 = 9600Hz;
//prescaler is 74, period 199, that is 72000000/75/200 = 4800Hz;
//prescaler is 74, period 399, that is 72000000/75/400 = 2400Hz;
//prescaler is 74, period 799, that is 72000000/75/800 = 1200Hz;

typedef enum {
  COM_IDLE,
  COM_START_BIT,
  COM_D0_BIT,
  COM_D1_BIT,
  COM_D2_BIT,
  COM_D3_BIT,
  COM_D4_BIT,
  COM_D5_BIT,
  COM_D6_BIT,
  COM_D7_BIT,
  COM_STOP_BIT
} UartState;

#define SUART_BUFFER_SIZE 16

static xQueueHandle decoder_evt_queue = NULL;
typedef struct {
  GPIO_TypeDef* GPIOx_tx;
  uint16_t GPIO_Pin_tx;
  GPIO_TypeDef* GPIOx_rx;
  uint16_t GPIO_Pin_rx;
  BaudRate baudRate;
  UartState state_tx, state_rx;
  uint8_t dataBits_tx, dataBits_rx;
  uint16_t queueSize_rx;
  xQueueHandle dataQueue_rx;
  SemaphoreHandle_t sem_tx, sem_rx;
} SoftUart;

static SoftUart suart0;

void SoftUart_Init(SoftUart *psu)
{
  switch(psu->baudRate) {
    case B1200: Tim2_Init(74, 799);
      break;
    case B2400: Tim2_Init(74, 399);
      break;
    case B4800: Tim2_Init(74, 199);
      break;
    case B9600: Tim2_Init(74, 99);
      break;
    case B19200: Tim2_Init(74, 49);
      break;
    case B38400: Tim2_Init(74, 24);
      break;
    case B115200: Tim2_Init(24, 24);
      break;
  }

  psu->state_tx = COM_IDLE;
  psu->state_rx = COM_IDLE;
  psu->dataBits_tx = 0;
  psu->dataBits_rx = 0;

  psu->dataQueue_rx = xQueueCreate(psu->queueSize_rx, sizeof(uint8_t));

  psu->sem_tx = xSemaphoreCreateBinary();
  psu->sem_rx = xSemaphoreCreateBinary();
}

void SoftUart0_Init(BaudRate br, uint16_t buffer_size)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  EXTI_InitTypeDef EXTI_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;

  // Step 1: Initialize GPIO as input for button
  // PA1 & PA2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1; /* TX */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2; /* RX */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // Step 2: Initialize EXTI for PA1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
  EXTI_InitStruct.EXTI_Line = EXTI_Line2;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);

  // Step 3: Initialize NVIC for EXTI2 IRQ channel

  //設定NVIC
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); /* 4 bits preemption */
  NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 12;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  /* USART2 DMA conflict with I2C1*/
  suart0.GPIOx_tx = GPIOA;
  suart0.GPIO_Pin_tx = GPIO_Pin_1;
  suart0.GPIOx_rx = GPIOA;
  suart0.GPIO_Pin_rx = GPIO_Pin_2;

  GPIO_SetBits(suart0.GPIOx_tx, suart0.GPIO_Pin_tx);
  //GPIO_ResetBits(suart0.GPIOx_rx, suart0.GPIO_Pin_rx);

  suart0.baudRate = br;
  if(buffer_size < SUART_BUFFER_SIZE)
  	buffer_size = SUART_BUFFER_SIZE;
  suart0.queueSize_rx = buffer_size;

  SoftUart_Init(&suart0);
}

void SoftUart0_Task(void *p)
{
  while(1) {
    if(xSemaphoreTake(suart0.sem_rx, portMAX_DELAY) == pdPASS) {
      if(suart0.state_rx == COM_IDLE) {
        suart0.state_rx = COM_START_BIT;
        suart0.dataBits_rx = 0;
        Tim2_Enable();
      }
    }
  }
}

void SoftUart_Write(SoftUart *psu, uint8_t *data, size_t len)
{
  size_t c = 0;
  while(c < len) {
    psu->state_tx = COM_START_BIT;
    psu->dataBits_tx = data[c++];
    Tim2_Enable();
    GPIO_ResetBits(psu->GPIOx_tx, psu->GPIO_Pin_tx); /* Transsmit START bit */
    xSemaphoreTake(psu->sem_tx, portMAX_DELAY); /* Wait for TX done ... */
  }
}

int SoftUart_Read(SoftUart *psu, uint8_t *data, size_t len)
{
  size_t c = 0;
  while(c < len) {
    if(xQueueReceive(psu->dataQueue_rx, &data[c], 0) == pdFALSE)
      break;
    c++;
  }
  return c;
}

int SoftUart_Poll(SoftUart *psu)
{
  uint8_t r;
  if(xQueuePeek(psu->dataQueue_rx, &r, portMAX_DELAY)) {
    return (psu->queueSize_rx - uxQueueSpacesAvailable(psu->dataQueue_rx));
  }
  return 0;
}

int SoftUart_Poll2(SoftUart *psu, uint16_t timeout_ms)
{
  uint8_t r;
  if(xQueuePeek(psu->dataQueue_rx, &r, timeout_ms / portTICK_RATE_MS)) {
    return (psu->queueSize_rx - uxQueueSpacesAvailable(psu->dataQueue_rx));
  }
  return 0;
}

void SoftUart0_Write(uint8_t *data, size_t len)
{
    SoftUart_Write(&suart0, data, len);
}

int SoftUart0_Read(uint8_t *data, size_t len)
{
    return SoftUart_Read(&suart0, data, len);
}

int SoftUart0_Poll(void)
{
    return SoftUart_Poll(&suart0);
}

int SoftUart0_Poll2(uint16_t timeout_ms)
{
    return SoftUart_Poll2(&suart0, timeout_ms);
}

void EXTI2_IRQHandler(void)
{
  BaseType_t r = pdFALSE;
  BaseType_t xHigherPriorityTaskWoken;

  if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line2); //清除标志
    r = xSemaphoreGiveFromISR(suart0.sem_rx, &xHigherPriorityTaskWoken);
  }

  if(r == pdTRUE)
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); /* Do context switch if necessary ... */
}

void TIM2_IRQHandler(void)
{
  	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, /*TIM_IT_Update*/ TIM_FLAG_Update);

		SoftUart *psu = &suart0;

		if(psu->state_rx != COM_IDLE) {
			psu->state_rx++;
			if(psu->state_rx == COM_STOP_BIT) {
				psu->state_rx = COM_IDLE;
				if(psu->state_tx == COM_IDLE) /* Both tx / rx idle, stop timer 2 */
		    		Tim2_Disable();
#if 1          
				if(xQueueIsQueueFullFromISR(psu->dataQueue_rx)) { /* Drop oldest one of buffer */
					uint8_t r;
					xQueueReceiveFromISR(psu->dataQueue_rx, &r, NULL);
				}
#endif
 				xQueueSendFromISR(psu->dataQueue_rx, &psu->dataBits_rx, NULL);
 			} else {
 				uint8_t bitOffset = psu->state_rx - COM_D0_BIT;
 				if(GPIO_ReadInputDataBit(psu->GPIOx_rx, psu->GPIO_Pin_rx))
 					psu->dataBits_rx |= (1 << bitOffset);
 				else
 					psu->dataBits_rx &= ~(1 << bitOffset);
 			}
    	}

    	if(psu->state_tx == COM_STOP_BIT) {
			psu->state_tx = COM_IDLE;
			if(psu->state_rx == COM_IDLE) /* Both tx / rx idle, stop timer 2 */
				Tim2_Disable();
			BaseType_t xHigherPriorityTaskWoken;
			if(xSemaphoreGiveFromISR(psu->sem_tx, &xHigherPriorityTaskWoken) == pdTRUE) /* */
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken); /* Do context switch if necessary ... */
    	}

		if(psu->state_tx != COM_IDLE) {
			psu->state_tx++;
			if(psu->state_tx == COM_STOP_BIT) {
				GPIO_SetBits(psu->GPIOx_tx, psu->GPIO_Pin_tx); /* Transsmit STOP bit */
			} else { /* Transsmit DATA */
				if(psu->dataBits_tx & (1 << (psu->state_tx - COM_D0_BIT)))
					GPIO_SetBits(psu->GPIOx_tx, psu->GPIO_Pin_tx);
				else
					GPIO_ResetBits(psu->GPIOx_tx, psu->GPIO_Pin_tx);
			}
    	}
  	}
}

