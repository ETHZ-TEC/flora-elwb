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
 * post-communication task (serial printing, stats, periodic checks, ...)
 */

#include "main.h"


/* Global variables ----------------------------------------------------------*/

extern QueueHandle_t xQueueHandle_rx;
extern TaskHandle_t xTaskHandle_pre;
extern TaskHandle_t xTaskHandle_com;
extern TaskHandle_t xTaskHandle_post;
extern TaskHandle_t xTaskHandle_idle;

uint32_t health_msg_period = NODE_HEALTH_MSG_PERIOD;


/* Private define ------------------------------------------------------------*/

#ifndef POST_TASK_RESUMED
#define POST_TASK_RESUMED()
#define POST_TASK_SUSPENDED()
#endif /* POST_TASK_IND */


/* Private variables ---------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/

void vTask_post(void const * argument)
{
  static dpp_message_t msg_buffer;
  static uint32_t      last_health_pkt = 0;

  LOG_VERBOSE("post task started");

  send_node_info();

  /* Infinite loop */
  for(;;)
  {
    POST_TASK_SUSPENDED();
    xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY);
    POST_TASK_RESUMED();

    /* process all packets rcvd from the network (regardless of whether there is space in the BOLT queue) */
    uint16_t rcvd = 0,
             forwarded = 0;
    while (xQueueReceive(xQueueHandle_rx, (void*)&msg_buffer, 0)) {
      if (!process_message(&msg_buffer, false)) {
        forwarded++;
      }
      rcvd++;
    }
    if (rcvd) {
      LOG_INFO("%u msg rcvd from network, %u forwarded", rcvd, forwarded);
    }

    /* generate a node info message if necessary (must be here) */
    if (health_msg_period) {
      /* only send other messages once the node info msg has been sent! */
      uint64_t network_time;
      elwb_get_last_syncpoint(&network_time, 0);
      uint32_t div = (network_time / (1000000 * health_msg_period));
      if (div != last_health_pkt) {
        /* using a divider instead of the elapsed time will group the health
         * messages of all nodes together into one round */
        send_node_health();
        last_health_pkt = div;
      }
    }

#if BASEBOARD
    /* process pending commands */
    process_scheduled_bb_commands();
#endif /* BASEBOARD */

    /* update RTC time */
    uint32_t rtctime  = rtc_get_unix_timestamp();
    uint32_t currtime = elwb_get_time_sec();
    if (currtime > 1500000000 && rtctime != currtime) {
      rtc_set_unix_timestamp(elwb_get_time_sec());
      LOG_INFO("RTC timestamp updated to %lu, was %lu", currtime, rtctime);
    }

    /* check for critical stack usage or overflow */
    rtos_check_stack_usage();

    /* print some stats */
    LOG_INFO("CPU duty cycle:  %.2f%%    radio duty cycle (rx/tx):  %.2f%%/%.2f%%", (float)rtos_get_cpu_dc() / 100.0f, (float)radio_get_rx_dc() / 10000.0f, (float)radio_get_tx_dc() / 10000.0f);

    /* flush the log print queue */
#if !LOG_PRINT_IMMEDIATELY
    log_flush();
#endif /* LOG_PRINT_IMMEDIATELY */

    /* before telling the state machine to enter low-power mode, wait for the UART transmission to complete (must be done here, NOT in lpm_prepare) */
#if LOG_USE_DMA
    uart_wait_tx_complete(100);     // 100ms timeout
#endif /* LOG_USE_DMA */

    /* round finished, prepare for low-power mode */
    lpm_update_opmode(OP_MODE_EVT_DONE);
  }
}
