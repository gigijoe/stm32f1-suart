/*
 * This is freertos release 9.0.0 configured for a stm32f103vct6 board,
 * named Hy-MiniSTM32V. It should run on any stm32f103 with a few changes.
 * 
 * There are two tasks running. One is blinking two LEDs, the other 
 * is sending stuff via USART1.
 * 
 * Author: Nils Stec, stecdose@gmail.com
 * Date 2016-07-01
 * 
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "stm32f10x.h"
#include "bool.h"
#include "suart.h"

#include "freertos/include/FreeRTOS.h"
#include "freertos/include/task.h"
#include "freertos/include/queue.h"
#include "freertos/include/timers.h"
#include "freertos/include/semphr.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
*
*/

void Main_Task(void *p)
{
  static portTickType latestTick;
  while(1) {
    latestTick = xTaskGetTickCount();

    uint8_t r;
    while(SoftUart0_Read(&r, 1) > 0)
      SoftUart0_Write(&r, 1);

    vTaskDelayUntil(&latestTick, 10 / portTICK_RATE_MS); /* 100 Hz */
  }
}

/*
*
*/

static TimerHandle_t hLedTimer;

static void LedTimerCb(TimerHandle_t th)
{
  static uint16_t tick = 0;

  if(tick++ % 2)
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
  else
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

/*
*
*/

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

/*
  #define configMAX_SYSCALL_INTERRUPT_PRIORITY 191 （0xBF也即优先级11），
  故在中断优先级为0～10的中断，均不会被内核延迟，并且可嵌套但不能调用API函数。在11～15之间的中断可以调用以​FromISR结尾的API函数。
*/

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); /* 4 bits preemption. It's requirement of FreeRTOS */
#if 0
  /* Enable Prefetch Buffer */
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
  /* Flash 2 wait state */
  FLASH_SetLatency(FLASH_Latency_2);
  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  EE_Init();
#endif
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  // PB12 : LED
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB, GPIO_Pin_12);

  SoftUart0_Init(B9600, 256);
  xTaskCreate(SoftUart0_Task, (const char*)"SoftUart0_Task", 256, NULL, 5, NULL); /* heighest priority 5 */  

  xTaskCreate(Main_Task, (const char*)"Main_Task", 256, NULL, 3, NULL); /* heighest priority 5 */
  
  hLedTimer = xTimerCreate(
      "ledTimer", /* debug name of task */
      pdMS_TO_TICKS(500), /* period */
      pdTRUE, /* auto-reload */
      NULL, /* no timer ID */
      LedTimerCb
      );

  xTimerStart(hLedTimer, 0);

	// Start RTOS scheduler

	vTaskStartScheduler();

	for(;;) { // Never here ...
	}
		
	return 0;
}


void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
// 	printf("ERROR: vApplicationStackOverflowHook(): Task \"%s\" overflowed its stack\n", pcTaskName);
// 	fflush(stdout);
// 	assert(false);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

