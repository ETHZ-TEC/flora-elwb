/*
 * task_pre.c
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#include "main.h"


#if BOLT_ENABLE

/* Global variables ----------------------------------------------------------*/

extern QueueHandle_t xQueueHandle_tx;
extern TIM_HandleTypeDef htim2;


/* Private define ------------------------------------------------------------*/

#ifndef PRE_TASK_RESUMED
#define PRE_TASK_RESUMED()
#define PRE_TASK_SUSPENDED()
#endif /* PRE_TASK_IND */


/* Private variables ---------------------------------------------------------*/
static uint8_t  bolt_read_buffer[BOLT_MAX_MSG_LEN];
static uint64_t unix_timestamp     = 0;
static uint64_t bolt_trq_timestamp = 0;
static bool     timestamp_updated  = false;


/* Functions -----------------------------------------------------------------*/


void set_unix_timestamp(uint64_t ts_us)
{
  unix_timestamp = ts_us;
  timestamp_updated = true;
}

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

bool handle_trq(void)
{
  /* note: don't take the SW extension into consideration here */
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4)) {
    uint32_t elapsed_hs_ticks = htim2.Instance->CNT - htim2.Instance->CCR4;
    bolt_trq_timestamp = lptimer_now();
    bolt_trq_timestamp -= (elapsed_hs_ticks / (HS_TIMER_FREQUENCY / LPTIMER_SECOND));
    LOG_VERBOSE("timestamp request received %lums ago, timestamp is %llu", (elapsed_hs_ticks / (HS_TIMER_FREQUENCY / 1000)), bolt_trq_timestamp);

    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);      /* clear capture compare interrupt flag */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4OF);    /* clear capture overrun flag */

    return true;
  }
  return false;
}

void update_time(bool treq_received)
{
  if (treq_received && timestamp_updated) {
    /* a UNIX timestamp has been received -> update the network time */
    /* adjust the timestamp to align with the next communication round as closely as possible */
    uint64_t next_round  = lptimer_get();
    /* convert to microseconds */
    uint32_t elapsed_us  = (next_round - bolt_trq_timestamp) * (1000000 / LPTIMER_SECOND);
    /* round to the next full second */
    uint32_t new_time_s  = (unix_timestamp + elapsed_us) / 1000000;
    uint32_t curr_time_s = elwb_sched_get_time();
    uint16_t delta_s     = (new_time_s > curr_time_s) ? (new_time_s - curr_time_s) : (curr_time_s - new_time_s);

    /* only update if the difference is much larger than 1 second */
    if (delta_s > TIMESTAMP_MAX_DRIFT) {
      elwb_sched_set_time(new_time_s);
      LOG_INFO("timestamp adjusted to %lu", new_time_s);
      EVENT_INFO(EVENT_CC430_TIME_UPDATED, diff);
    } else {
      LOG_INFO("timestamp: %lu, drift: %u", new_time_s, delta_s);
    }
  }
  timestamp_updated = false;
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

  /* configure input capture for TIM2_CH4 (PA3) -> don't pass a callback function, we don't want an interrupt */
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_4);

  /* Infinite loop */
  for (;;)
  {
    /* wait for notification token (= explicitly granted permission to run) */
    PRE_TASK_SUSPENDED();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    PRE_TASK_RESUMED();

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

    /* --- handle timestamp request --- */

    bool trq_rcvd = handle_trq();

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
    update_time(trq_rcvd);
  #endif /* !IS_HOST */
  }
}

#endif /* BOLT_ENABLE */
