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
  LOG_INFO("eLWB task started");

  /* load the UNIX timestamp from the RTC, convert it to microseconds and add the static startup delay */
  uint32_t currtime;
#if !FLOCKLAB
  currtime = rtc_get_unix_timestamp();
  LOG_INFO("timestamp %lu loaded from the RTC", currtime);
#else /* FLOCKLAB */
  currtime = BUILD_TIME;
  LOG_INFO("using build timestamp %lu", BUILD_TIME);
  rtc_set_unix_timestamp(BUILD_TIME);
#endif /* FLOCKLAB */
  elwb_sched_set_time((uint64_t)currtime * 1000000 + ELWB_CONF_STARTUP_DELAY * 1000);

  /* make sure the radio is awake */
  radio_wakeup();

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

