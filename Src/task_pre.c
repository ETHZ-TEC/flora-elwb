/*
 * Copyright (c) 2020 - 2021, ETH Zurich, Computer Engineering Group (TEC)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * pre-communication task (reads messages from BOLT, handles timestamp requests)
 */

#include "main.h"


/* Global variables ----------------------------------------------------------*/

extern QueueHandle_t xQueueHandle_tx;
extern TIM_HandleTypeDef htim2;


/* Private define ------------------------------------------------------------*/

#ifndef PRE_TASK_RESUMED
#define PRE_TASK_RESUMED()
#define PRE_TASK_SUSPENDED()
#endif /* PRE_TASK_IND */


#if BOLT_ENABLE

/* Private variables ---------------------------------------------------------*/
static uint64_t unix_timestamp_us     = 0;
static uint64_t bolt_trq_timestamp    = 0;
static uint64_t bolt_trq_hs_timestamp = 0;
static int32_t  average_drift_ppm     = 0;
static bool     timestamp_updated     = false;
static bool     timestamp_requested   = false;


/* Functions -----------------------------------------------------------------*/

static void init_time(void)
{
  /* load timestamp from RTC */
  unix_timestamp_us = rtc_get_unix_timestamp_ms() * 1000;
  LOG_INFO("UNIX timestamp %llu loaded from RTC", unix_timestamp_us);
}


void set_unix_timestamp_us(uint64_t ts_us)
{
  unix_timestamp_us = ts_us;
  timestamp_updated = true;
}


uint64_t get_unix_timestamp_us(void)
{
#if TIMESTAMP_USE_HS_TIMER
  return unix_timestamp_us + ((hs_timer_now() - bolt_trq_hs_timestamp) * (1000000LL - average_drift_ppm) / HS_TIMER_FREQUENCY);
#else /* TIMESTAMP_USE_HS_TIMER */
  return unix_timestamp_us + ((lptimer_now() - bolt_trq_timestamp) * (1000000LL - average_drift_ppm) / LPTIMER_SECOND);
#endif /* TIMESTAMP_USE_HS_TIMER */
}


void GPIO_PIN_3_Callback(void)
{
  bolt_trq_timestamp  = lptimer_now() - 20;   /* subtract wakeup + ISR + function call delays (estimate only) */
  timestamp_requested = true;
}


static void handle_treq(void)
{
  /* timer 2 capture compare flag 4 set? (COM_TREQ) */
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4)) {
    if (!lptimer_now_synced(&bolt_trq_timestamp, &bolt_trq_hs_timestamp)) {
      LOG_ERROR("failed to retrieve synchronized timestamps");
      return;
    }
    uint32_t curr_ticks       = (uint32_t)bolt_trq_hs_timestamp;
    uint32_t elapsed_hs_ticks = curr_ticks - htim2.Instance->CCR4;
    bolt_trq_hs_timestamp    -= elapsed_hs_ticks;
    bolt_trq_timestamp       -= HS_TIMER_TICKS_TO_LPTIMER(elapsed_hs_ticks);
    LOG_VERBOSE("timestamp request received %lums ago", HS_TIMER_TICKS_TO_MS(elapsed_hs_ticks));

    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4);      /* clear capture compare interrupt flag */
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC4OF);    /* clear capture overrun flag */

    timestamp_requested = true;
  }
}


static void update_time(void)
{
  static uint64_t prev_trq_timestamp     = 0;
  static uint64_t prev_unix_timestamp_us = 0;

  /* only update the time if a timestamp request has been registered and a new timestamp has been received */
  if (timestamp_requested && timestamp_updated) {
    /* first, calculate the drift */
    if (prev_unix_timestamp_us) {
      int32_t  unix_ts_diff_us = (unix_timestamp_us - prev_unix_timestamp_us);
  #if TIMESTAMP_USE_HS_TIMER
      int32_t local_ts_diff_us = (uint64_t)HS_TIMER_TICKS_TO_US(bolt_trq_hs_timestamp - prev_trq_timestamp);
  #else /* TIMESTAMP_USE_HS_TIMER */
      int32_t local_ts_diff_us = (uint64_t)HS_TIMER_TICKS_TO_US(bolt_trq_timestamp - prev_trq_timestamp);
  #endif /* TIMESTAMP_USE_HS_TIMER */
      int32_t drift_ppm        = (int32_t)((int64_t)(local_ts_diff_us - unix_ts_diff_us) * 1000000LL / unix_ts_diff_us);
      if (drift_ppm < TIMESTAMP_MAX_DRIFT && drift_ppm > -TIMESTAMP_MAX_DRIFT) {
        if (drift_ppm > TIMESTAMP_TYPICAL_DRIFT || drift_ppm < -TIMESTAMP_TYPICAL_DRIFT) {
          LOG_WARNING("drift is larger than usual");
        }
        if (average_drift_ppm == 0) {
          average_drift_ppm = drift_ppm;
        } else {
          average_drift_ppm = (average_drift_ppm + drift_ppm) / 2;
        }
        /* note: a negative drift means the local time runs too slow */
        LOG_VERBOSE("current drift compensation: %ldppm", average_drift_ppm);
        elwb_set_drift(average_drift_ppm);

      } else {
        LOG_WARNING("drift is too large (%ldppm)", drift_ppm);
        EVENT_WARNING(EVENT_SX1262_TSYNC_DRIFT, (uint32_t)drift_ppm);
      }
    }

    /* calculate the global time at the point where the next flood starts (note: use lptimer here in any case) */
    uint64_t new_time_us = unix_timestamp_us + (lptimer_get() - bolt_trq_timestamp) * 1000000 / LPTIMER_SECOND;

  #if TIMESTAMP_MAX_OFFSET_MS > 0
    /* calculate the difference between the actual time and the current network time */
    uint64_t curr_time_us = elwb_sched_get_time();
    int32_t delta = (int64_t)curr_time_us - (int64_t)new_time_us;
    if (delta > (TIMESTAMP_MAX_OFFSET_MS * 1000) || delta < -(TIMESTAMP_MAX_OFFSET_MS * 1000)) {
      elwb_sched_set_time(new_time_us);
      LOG_VERBOSE("timestamp adjusted to %llu", new_time_us);
      EVENT_INFO(EVENT_SX1262_TIME_UPDATED, delta);
    } else {
      if (delta > 500) {
        /* slow down the clock speed to compensate the offset */
        elwb_set_drift(average_drift_ppm + 10);
      } else if (delta < -500) {
        /* speed up */
        elwb_set_drift(average_drift_ppm - 10);
      }
      LOG_VERBOSE("current time offset: %ldus", delta);
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
    prev_unix_timestamp_us = unix_timestamp_us;
  }
  timestamp_requested = false;
  timestamp_updated   = false;
}

#endif /* BOLT_ENABLE */


/* pre communication round task */
void vTask_pre(void const * argument)
{
  LOG_VERBOSE("pre task started");

  /* check message size */
  if (sizeof(dpp_message_t) > DPP_MSG_PKT_LEN || DPP_MSG_PKT_LEN > BOLT_MAX_MSG_LEN) {
    FATAL_ERROR("invalid message size config");
  }

#if BOLT_ENABLE
  init_time();
#endif /* BOLT_ENABLE */

#if TIMESTAMP_USE_HS_TIMER
  /* configure input capture for TIM2_CH4 (PA3) */
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_4);
#endif /* TIMESTAMP_USE_HS_TIMER */

  /* Infinite loop */
  for (;;)
  {
    /* wait for notification token (= explicitly granted permission to run) */
    PRE_TASK_SUSPENDED();
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    PRE_TASK_RESUMED();

#if BOLT_ENABLE
    /* read from BOLT */
    static uint8_t  bolt_read_buffer[BOLT_MAX_MSG_LEN];
    uint32_t max_read_cnt = TRANSMIT_QUEUE_SIZE,
             forwarded = 0;
    /* only read as long as there is still space in the transmit queue */
    while (max_read_cnt && (((int32_t)uxQueueSpacesAvailable(xQueueHandle_tx) - TRANSMIT_QUEUE_MARGIN) > 0) && BOLT_DATA_AVAILABLE) {
      uint32_t len = bolt_read(bolt_read_buffer);
      if (!len) {
        /* not supposed to happen -> try to initialize BOLT */
        bolt_init();
        LOG_ERROR("bolt read failed");
        EVENT_ERROR(EVENT_SX1262_BOLT_ERROR, 0);
        break;
      }
      if (!process_message((dpp_message_t*)bolt_read_buffer, true)) {
        forwarded++;
      }
      max_read_cnt--;
    }
    if (max_read_cnt < TRANSMIT_QUEUE_SIZE) {
      LOG_VERBOSE("%lu msg read from BOLT, %lu forwarded", TRANSMIT_QUEUE_SIZE - max_read_cnt, forwarded);
    }

    /* handle timestamp request (only if BOLT enabled) */
    handle_treq();
    if (!IS_HOST) {
      if (timestamp_requested) {
        send_timestamp(bolt_trq_timestamp);
        timestamp_requested = false;
      }
    } else {
      update_time();
    }

  #if BASEBOARD_TREQ_WATCHDOG && BASEBOARD
    static uint64_t last_treq = 0;      /* hs timestamp of last request */
    if (bolt_trq_hs_timestamp > last_treq) {
      last_treq = bolt_trq_hs_timestamp;
    }
    /* only use time request watchdog when baseboard is enabled */
    if (BASEBOARD_IS_ENABLED()) {
      bool powercycle = false;
      /* check when was the last time we got a time request */
      if (HS_TIMER_TICKS_TO_S(hs_timer_now() - last_treq) > BASEBOARD_TREQ_WATCHDOG) {
        last_treq  = hs_timer_now();
        powercycle = true;
      }
      if (powercycle) {
        /* power cycle the baseboard */
        LOG_WARNING("power-cycling baseboard (TREQ watchdog)");
        BASEBOARD_DISABLE();
        /* enable pin must be kept low for ~1s -> schedule pin release */
        if (!schedule_bb_command((elwb_get_time(0) / 1000000) + 2, CMD_SX1262_BASEBOARD_ENABLE, 0)) {
          /* we must wait and release the reset here */
          LOG_WARNING("failed to schedule baseboard enable");
          delay_us(1000000);
          BASEBOARD_ENABLE();
        }
      }
    } else {
      last_treq = hs_timer_now();
    }
  #endif /* BASEBOARD_TREQ_WATCHDOG */
#endif /* BOLT_ENABLE */

    /* wake the radio */
    radio_wakeup();

    //LOG_VERBOSE("pre task executed");
  }
}
