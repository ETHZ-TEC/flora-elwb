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
static uint64_t master_timestamp      = 0;
static uint64_t bolt_trq_timestamp    = 0;
static uint64_t bolt_trq_hs_timestamp = 0;
static bool     timestamp_updated     = false;
static bool     timestamp_requested   = false;


/* Functions -----------------------------------------------------------------*/


void set_master_timestamp(uint64_t ts_us)
{
  master_timestamp  = ts_us;
  timestamp_updated = true;
}

uint64_t get_master_timestamp(void)
{
#if TIMESTAMP_USE_HS_TIMER
  return master_timestamp + ((hs_timer_now() - bolt_trq_hs_timestamp) * 1000000 / HS_TIMER_FREQUENCY);
#else /* TIMESTAMP_USE_HS_TIMER */
  return master_timestamp + ((lptimer_now() - bolt_trq_timestamp) * 1000000 / LPTIMER_SECOND);
#endif /* TIMESTAMP_USE_HS_TIMER */
}

void GPIO_PIN_3_Callback(void)
{
  bolt_trq_timestamp  = lptimer_now() - 20;   /* subtract wakeup + ISR + function call delays (estimate only) */
  timestamp_requested = true;
}

void handle_trq(void)
{
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4)) {
    if (!lptimer_now_synced(&bolt_trq_timestamp, &bolt_trq_hs_timestamp)) {
      LOG_ERROR("failed to retrieve synchronized timestamps");
      return;
    }
    uint32_t curr_ticks       = (uint32_t)bolt_trq_hs_timestamp;
    uint32_t elapsed_hs_ticks = curr_ticks - htim2.Instance->CCR4;
    bolt_trq_hs_timestamp    -= elapsed_hs_ticks;
    bolt_trq_timestamp       -= (elapsed_hs_ticks / (HS_TIMER_FREQUENCY / LPTIMER_SECOND));
    LOG_VERBOSE("timestamp request received %lums ago, timestamp is %llu", (elapsed_hs_ticks / (HS_TIMER_FREQUENCY / 1000)), bolt_trq_timestamp);

    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);      /* clear capture compare interrupt flag */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4OF);    /* clear capture overrun flag */

    timestamp_requested = true;
  }
}

void update_time(void)
{
  static int32_t  average_drift         = 0;
  static uint64_t prev_trq_timestamp    = 0;
  static uint64_t prev_master_timestamp = 0;

  /* only update the time if a timestamp request has been registered and a new timestamp has been received */
  if (timestamp_requested && timestamp_updated) {
    /* first, calculate the drift */
    if (prev_master_timestamp) {
      int32_t master_ts_diff_us = (master_timestamp - prev_master_timestamp);
  #if TIMESTAMP_USE_HS_TIMER
      int32_t local_ts_diff_us = ((uint64_t)(bolt_trq_hs_timestamp - prev_trq_timestamp) * 1000000 / HS_TIMER_FREQUENCY);
  #else /* TIMESTAMP_USE_HS_TIMER */
      int32_t local_ts_diff_us = ((uint64_t)(bolt_trq_timestamp - prev_trq_timestamp) * 1000000 / LPTIMER_SECOND);
  #endif /* TIMESTAMP_USE_HS_TIMER */
      int32_t drift = (int64_t)(local_ts_diff_us - master_ts_diff_us) * 1000000 / master_ts_diff_us;
      //LOG_VERBOSE("diff master: %ldus, diff local: %ldus, drift: %ldppm", master_ts_diff_us, local_ts_diff_us, drift);
      if (drift > TIMESTAMP_TYPICAL_DRIFT || drift < -TIMESTAMP_TYPICAL_DRIFT) {
        LOG_WARNING("drift is larger than usual");
      }
      average_drift = (average_drift + drift) / 2;
      /* make sure the drift does not exceed the maximum allowed value */
      if (average_drift > TIMESTAMP_MAX_DRIFT) {
        average_drift = TIMESTAMP_MAX_DRIFT;
      } else if (average_drift < -TIMESTAMP_MAX_DRIFT) {
        average_drift = -TIMESTAMP_MAX_DRIFT;
      }
      /* note: a negative drift means the local time runs slower than the master clock */
      LOG_INFO("current drift: %ldppm, average drift: %ldppm", drift, average_drift);
      elwb_set_drift(average_drift);
    }

    /* calculate the global time at the point where the next flood starts */
  #if TIMESTAMP_USE_HS_TIMER
    uint64_t new_time_us = master_timestamp + (lptimer_get() - bolt_trq_hs_timestamp) * 1000000 / HS_TIMER_FREQUENCY;
  #else /* TIMESTAMP_USE_HS_TIMER */
    uint64_t new_time_us = master_timestamp + (lptimer_get() - bolt_trq_timestamp) * 1000000 / LPTIMER_SECOND;
  #endif /* TIMESTAMP_USE_HS_TIMER */

  #if TIMESTAMP_MAX_OFFSET_MS > 0
    /* calculate the difference between the actual time and the current network time */
    uint64_t curr_time_us = elwb_sched_get_time();
    int32_t delta = (int64_t)curr_time_us - (int64_t)new_time_us;
    if (delta > (TIMESTAMP_MAX_OFFSET_MS * 1000) || delta < -(TIMESTAMP_MAX_OFFSET_MS * 1000)) {
      elwb_sched_set_time(new_time_us);
      LOG_INFO("timestamp adjusted to %llu", new_time_us);
    } else {
      if (delta > 500) {
        /* slow down the clock speed to compensate the offset */
        elwb_set_drift(average_drift + 10);
      } else if (delta < -500) {
        /* speed up */
        elwb_set_drift(average_drift - 10);
      }
      LOG_INFO("current time offset: %ldus", delta);
    }
  #else
    /* adjust the network time */
    elwb_sched_set_time(new_time_us);
  #endif /* TIMESTAMP_MAX_OFFSET_MS */

  #if TIMESTAMP_USE_HS_TIMER
    prev_trq_timestamp   = bolt_trq_hs_timestamp;
  #else /* TIMESTAMP_USE_HS_TIMER */
    prev_trq_timestamp   = bolt_trq_timestamp;
  #endif /* TIMESTAMP_USE_HS_TIMER */
    prev_master_timestamp = master_timestamp;
  }
  timestamp_requested = false;
  timestamp_updated   = false;
}

/* pre communication round task */
void vTask_pre(void const * argument)
{
  LOG_INFO("pre task started");

  /* check message size */
  if (sizeof(dpp_message_t) > DPP_MSG_PKT_LEN || DPP_MSG_PKT_LEN > BOLT_MAX_MSG_LEN) {
    FATAL_ERROR("invalid message size config");
  }

  /* configure input capture for TIM2_CH4 (PA3) */
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_4);

  /* Infinite loop */
  for (;;)
  {
    /* wait for notification token (= explicitly granted permission to run) */
    PRE_TASK_SUSPENDED();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    PRE_TASK_RESUMED();

    /* read from BOLT */
    uint32_t max_read_cnt = TRANSMIT_QUEUE_SIZE,
             forwarded = 0;
    /* only read as long as there is still space in the transmit queue */
    while (max_read_cnt && uxQueueSpacesAvailable(xQueueHandle_tx) && BOLT_DATA_AVAILABLE) {
      uint32_t len = bolt_read(bolt_read_buffer);
      if (!len) {
        LOG_ERROR("bolt read failed");
        break;
      }
      if (!process_message((dpp_message_t*)bolt_read_buffer, true)) {
        forwarded++;
      }
      max_read_cnt--;
    }
    if (max_read_cnt < TRANSMIT_QUEUE_SIZE) {
      LOG_INFO("%lu msg read from BOLT, %lu forwarded", TRANSMIT_QUEUE_SIZE - max_read_cnt, forwarded);
    }

    /* --- handle timestamp request --- */

    handle_trq();

    if (!IS_HOST) {
      if (timestamp_requested) {
        send_timestamp(bolt_trq_timestamp);
        timestamp_requested = false;
      }
    } else {
      update_time();
    }

    LOG_VERBOSE("pre task executed");
  }
}

#endif /* BOLT_ENABLE */
