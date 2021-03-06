/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "cmsis_os.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef CPU_OFF_IND
#define CPU_OFF_IND()
#define CPU_ON_IND()
#endif /* CPU_OFF_IND */
#ifndef IDLE_TASK_RESUMED
#define IDLE_TASK_RESUMED()
#define IDLE_TASK_SUSPENDED()
#endif /* IDLE_TASK_IND */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* RTOS Task Handles */
TaskHandle_t xTaskHandle_pre  = NULL;
TaskHandle_t xTaskHandle_com  = NULL;
TaskHandle_t xTaskHandle_post = NULL;
TaskHandle_t xTaskHandle_idle = NULL;
/* RTOS Queue Handles */
QueueHandle_t xQueueHandle_tx = NULL;   /* holds the messages to be transmitted over the eLWB network */
QueueHandle_t xQueueHandle_rx = NULL;   /* holds the messages received from the eLWB network */
QueueHandle_t xQueueHandle_retransmit = NULL;   /* required by the eLWB for retransmitting packets (D-ACK feature) */
/* Variables */
bool     round_finished   = false;
uint64_t active_time      = 0;
uint64_t wakeup_timestamp = 0;
uint64_t last_reset       = 0;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

extern void vTask_pre(void* argument);
extern void vTask_com(void* argument);
extern void vTask_post(void* argument);

/* USER CODE END FunctionPrototypes */

/* Pre/Post sleep processing prototypes */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */

  IDLE_TASK_RESUMED();

  if (!xTaskHandle_idle) {
    xTaskHandle_idle = xTaskGetCurrentTaskHandle();
  }

  /* if the application ends up in the RESET state, something went wrong -> reset the MCU */
  if (lpm_get_opmode() == OP_MODE_RESET) {
    NVIC_SystemReset();
  }

  IDLE_TASK_SUSPENDED();
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */

  (void)pcTaskName;
  (void)xTask;
  FATAL_ERROR("--- stack overflow detected! ---\r\n");
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
  FATAL_ERROR("--- malloc failed ---\r\n");
}
/* USER CODE END 5 */

/* USER CODE BEGIN PREPOSTSLEEP */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
  /* interrupts are disabled within this function */

  /* note: for tickless idle, the HAL tick needs to be suspended! */
  lpm_prepare();

  /* duty cycle measurement */
  active_time += lptimer_now() - wakeup_timestamp;
  CPU_OFF_IND();
}

void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
  /* interrupts are disabled within this function */

  CPU_ON_IND();
  wakeup_timestamp = lptimer_now();    /* reset duty cycle timer */

  lpm_resume();
}
/* USER CODE END PREPOSTSLEEP */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* RTOS functions ------------------------------------------------------------*/
void rtos_init(void)
{
  /* create RTOS queues */
  xQueueHandle_rx = xQueueCreate(RECEIVE_QUEUE_SIZE, DPP_MSG_PKT_LEN);
  if (xQueueHandle_rx == NULL) {
    Error_Handler();
  }
  xQueueHandle_tx = xQueueCreate(TRANSMIT_QUEUE_SIZE, DPP_MSG_PKT_LEN);
  if (xQueueHandle_tx == NULL) {
    Error_Handler();
  }
  xQueueHandle_retransmit = xQueueCreate(TRANSMIT_QUEUE_SIZE, DPP_MSG_PKT_LEN);
  if (xQueueHandle_retransmit == NULL) {
    Error_Handler();
  }

  /* create RTOS tasks */
  /* max. priority is (configMAX_PRIORITIES - 1), higher numbers = higher
   * priority; idle task has priority 0 */
  /* pre communication round task */
  if (xTaskCreate(vTask_pre,
                  "preTask",
                  PRE_TASK_STACK_SIZE,
                  NULL,
                  tskIDLE_PRIORITY + 2,
                  &xTaskHandle_pre) != pdPASS)     { Error_Handler(); }
  if (xTaskCreate(vTask_post,
                  "postTask",
                  POST_TASK_STACK_SIZE,
                  NULL,
                  tskIDLE_PRIORITY + 1,            /* lowest priority right after the idle task */
                  &xTaskHandle_post) != pdPASS)    { Error_Handler(); }
  if (xTaskCreate(vTask_com,
                  "elwbTask",
                  COM_TASK_STACK_SIZE,
                  NULL,
                  configMAX_PRIORITIES - 1,        /* highest priority task */
                  &xTaskHandle_com) != pdPASS)     { Error_Handler(); }
}

uint32_t rtos_get_cpu_dc(void)
{
  return (uint32_t)((active_time * 10000) / (lptimer_now() - last_reset));
}

void rtos_reset_cpu_dc(void)
{
  last_reset = lptimer_now();
  active_time = 0;
}

/* returns the highest watermark value */
uint32_t rtos_check_stack_usage(void)
{
  static unsigned long idleTaskStackWM = 0,
#if BOLT_ENABLE
                       preTaskStackWM  = 0,
#endif /* BOLT_ENABLE */
                       comTaskStackWM  = 0,
                       postTaskStackWM = 0;

  uint32_t max_wm = 0;

  unsigned long idleSWM = configMINIMAL_STACK_SIZE - (uxTaskGetStackHighWaterMark(xTaskHandle_idle));
  if (idleSWM > idleTaskStackWM) {
    idleTaskStackWM = idleSWM;
    uint32_t usage = idleTaskStackWM * 100 / configMINIMAL_STACK_SIZE;
    if (usage > STACK_WARNING_THRESHOLD) {
      LOG_WARNING("stack watermark of idle task reached %u%%", usage);
      EVENT_WARNING(EVENT_SX1262_STACK_WM, usage);
    } else {
      LOG_INFO("stack watermark of idle task increased to %u%%", usage);
    }
  }
  max_wm = idleTaskStackWM;

#if BOLT_ENABLE
  unsigned long preSWM  = PRE_TASK_STACK_SIZE - (uxTaskGetStackHighWaterMark(xTaskHandle_pre));
  if (preSWM > preTaskStackWM) {
    preTaskStackWM = preSWM;
    uint32_t usage = preTaskStackWM * 100 / PRE_TASK_STACK_SIZE;
    if (usage > STACK_WARNING_THRESHOLD) {
      LOG_WARNING("stack watermark of pre task reached %u%%", usage);
      EVENT_WARNING(EVENT_SX1262_STACK_WM, 0x00010000 | usage);
    } else {
      LOG_INFO("stack watermark of pre task increased to %u%%", usage);
    }
  }
  if (preTaskStackWM > max_wm) {
    max_wm = preTaskStackWM;
  }
#endif /* BOLT_ENABLE */

  unsigned long comSWM  = COM_TASK_STACK_SIZE - (uxTaskGetStackHighWaterMark(xTaskHandle_com));
  if (comSWM > comTaskStackWM) {
    comTaskStackWM = comSWM;
    uint32_t usage = comTaskStackWM * 100 / COM_TASK_STACK_SIZE;
    if (usage > STACK_WARNING_THRESHOLD) {
      LOG_WARNING("stack watermark of com task reached %u%%", usage);
      EVENT_WARNING(EVENT_SX1262_STACK_WM, 0x00020000 | usage);
    } else {
      LOG_INFO("stack watermark of com task increased to %u%%", usage);
    }
  }
  if (comTaskStackWM > max_wm) {
    max_wm = comTaskStackWM;
  }

  unsigned long postSWM = POST_TASK_STACK_SIZE - (uxTaskGetStackHighWaterMark(xTaskHandle_post));
  if (postSWM > postTaskStackWM) {
    postTaskStackWM = postSWM;
    uint32_t usage = postTaskStackWM * 100 / POST_TASK_STACK_SIZE;
    if (usage > STACK_WARNING_THRESHOLD) {
      LOG_WARNING("stack watermark of post task reached %u%%", usage);
      EVENT_WARNING(EVENT_SX1262_STACK_WM, 0x00030000 | usage);
    } else {
      LOG_INFO("stack watermark of post task increased to %u%%", usage);
    }
  }
  if (postTaskStackWM > max_wm) {
    max_wm = postTaskStackWM;
  }

  return max_wm;
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
