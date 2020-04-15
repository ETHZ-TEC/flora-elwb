/*
 * task_bolt.c
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#include "main.h"


#if BOLT_ENABLE

extern QueueHandle_t xQueueHandle_tx;
extern TIM_HandleTypeDef htim2;


/* Global variables ----------------------------------------------------------*/

uint64_t unix_timestamp     = 0;
uint64_t bolt_trq_timestamp = 0;
bool     timestamp_updated  = false;


/* Private define ------------------------------------------------------------*/

#ifndef PRE_TASK_RESUMED
#define PRE_TASK_RESUMED()
#define PRE_TASK_SUSPENDED()
#endif /* PRE_TASK_IND */


/* Private variables ---------------------------------------------------------*/
static uint8_t  bolt_read_buffer[BOLT_MAX_MSG_LEN];
static uint32_t timestamp = 0;


/* Functions -----------------------------------------------------------------*/


uint64_t get_unix_timestamp(void)
{
  // TODO fix this
  return unix_timestamp + ((lptimer_now() - bolt_trq_timestamp) *
                           (1000000 / LPTIMER_SECOND));
}


void GPIO_PIN_3_Callback(void)
{
  //led_toggle(LED_EVENT);
}

void bolt_treq_callback(void)
{
  led_toggle(LED_EVENT);
  timestamp = htim2.Instance->CCR4;
}

void update_time(void)
{
  if (timestamp_updated) {
    /* a UTC timestamp has been received -> update the network time */
    uint32_t elapsed = 0;
    /* adjust the timestamp to align with the next communication round
    * as closely as possible */
    uint64_t next_round = lptimer_get();
    elapsed = (next_round - bolt_trq_timestamp);

    /* convert to us */
    elapsed = elapsed * (1000000 / LPTIMER_SECOND);
    uint32_t new_time = (unix_timestamp + elapsed) / 1000000;
    uint32_t curr_time = elwb_sched_get_time();
    uint16_t diff = (new_time > curr_time) ? (new_time - curr_time) : (curr_time - new_time);

    /* only update if the difference is much larger than 1 second */
    if (diff > TIMESTAMP_MAX_DRIFT) {
      elwb_sched_set_time(new_time);
      LOG_INFO("timestamp adjusted to %lu", new_time);
      EVENT_INFO(EVENT_CC430_TIME_UPDATED, diff);
    } else {
      LOG_INFO("timestamp: %lu, drift: %u", new_time, diff);
    }
    timestamp_updated = 0;
  }
}

/* pre communication round task */
void vTask_pre(void const * argument)
{
  LOG_INFO("pre task started");

  /* check message size */
  if (sizeof(dpp_message_t) > DPP_MSG_PKT_LEN || DPP_MSG_PKT_LEN > BOLT_MAX_MSG_LEN) {
    FATAL_ERROR("invalid message size config");
  }

  if (!bolt_init()) {
    FATAL_ERROR("bolt_init failed!");
  }

  /* configure input capture for TIM2_CH4 (PA3) */
  hs_timer_capture4(bolt_treq_callback);

  /* Infinite loop */
  for (;;)
  {
    /* wait for notification token (= explicitly granted permission to run) */
    PRE_TASK_SUSPENDED();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    PRE_TASK_RESUMED();

    if (timestamp) {
      LOG_INFO("last timestamp was %lu", timestamp);
      timestamp = 0;
    }

    /* read from BOLT */
    uint32_t max_read_cnt = BOLT_TASK_MAX_READ_CNT,
             forwarded = 0;
    /* only read as long as there is still space in the transmit queue */
    while (max_read_cnt && uxQueueSpacesAvailable(xQueueHandle_tx)) {
      uint32_t len = bolt_read(bolt_read_buffer);
      if (!len) {
        if (BOLT_DATA_AVAILABLE) {
          LOG_ERROR_CONST("bolt read failed");
          EVENT_ERROR(EVENT_CC430_BOLT_ERROR, 0);
        }
        break;
      }
      if (!process_message((dpp_message_t*)bolt_read_buffer, true)) {
        forwarded++;
      }
      max_read_cnt--;
    }
    if (max_read_cnt < BOLT_TASK_MAX_READ_CNT) {
      LOG_INFO("%lu msg read from BOLT, %lu forwarded",
               BOLT_TASK_MAX_READ_CNT - max_read_cnt, forwarded);
    }
    LOG_VERBOSE_CONST("pre task executed");

  #if !IS_HOST
    /* --- send the timestamp if one has been requested --- */
    //TODO
    /*if(bolt_captured_trq) {
      send_timestamp(bolt_captured_trq);
      bolt_captured_trq = 0;
    }*/
  #else /* !IS_HOST */
    /* note: even though the time is updated here, it will only be used from
     *       the next round since the schedule has already been computed. */
    update_time();
  #endif /* !IS_HOST */
  }
}

#endif /* BOLT_ENABLE */
