/*
 * task_elwb.c
 *
 *  Created on: Mar 25, 2020
 *      Author: rdaforno
 */

#include "main.h"
#include "protocol/elwb/elwb.h"


/* Private variables ---------------------------------------------------------*/


extern TaskHandle_t xTaskHandle_pre;
extern TaskHandle_t xTaskHandle_post;
extern QueueHandle_t xQueueHandle_rx;
extern QueueHandle_t xQueueHandle_tx;


/* communication task */
void vTask_com(void const * argument)
{
  LOG_INFO_CONST("eLWB task started");

  // TODO postpone start of eLWB on host node until a valid UNIX timestamp has been received!

  /* start eLWB */
  elwb_start(xTaskGetCurrentTaskHandle(), xTaskHandle_pre, xTaskHandle_post, xQueueHandle_rx, xQueueHandle_tx);
  FATAL_ERROR("eLWB task terminated");

  /* for debugging purposes only if eLWB is not used */
  while (1) {
    xTaskNotify(xTaskHandle_post, 0, eNoAction);    /* notify the post task */
    ELWB_SUSPENDED();
    vTaskDelay(MS_TO_RTOS_TICKS(900));
    ELWB_RESUMED();
    /* send a flood */
    const char payload[32] = "hello world!";
    gloria_start(NODE_ID, (uint8_t*)payload, sizeof(payload), 2, 1);
    /* wait some time */
    vTaskDelay(MS_TO_RTOS_TICKS(100));
    /* stop the flood */
    gloria_stop();
    LOG_INFO("com task executed");
  }
}

