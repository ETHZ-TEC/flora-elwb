/**
  ******************************************************************************
  * Flora
  ******************************************************************************
  * @author Roman Trub
  * @file   task_post.c
  * @brief  Post task (runs after the communication round)
  *
  *
  ******************************************************************************
  */

#include "main.h"


extern QueueHandle_t xQueueHandle_rx;
extern TaskHandle_t xTaskHandle_pre;
extern TaskHandle_t xTaskHandle_com;
extern TaskHandle_t xTaskHandle_post;
extern TaskHandle_t xTaskHandle_idle;

/* Private define ------------------------------------------------------------*/

#ifndef POST_TASK_RESUMED
#define POST_TASK_RESUMED()
#define POST_TASK_SUSPENDED()
#endif /* POST_TASK_IND */


/* Private variables ---------------------------------------------------------*/

static dpp_message_t msg_buffer;
static bool          node_info_sent = false;
static uint32_t      health_msg_period = NODE_HEALTH_MSG_PERIOD;
static uint32_t      last_health_pkt = 0;


/* Functions -----------------------------------------------------------------*/

void vTask_post(void const * argument)
{
  static unsigned long idleTaskStackWM = 0,
                       preTaskStackWM  = 0,
                       comTaskStackWM  = 0,
                       postTaskStackWM = 0;

  LOG_INFO("Post task started");

  /* Infinite loop */
  for(;;)
  {
    POST_TASK_SUSPENDED();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* wait for notification token */
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
    if (!node_info_sent) {
      uint32_t network_time = elwb_get_time_sec();
      if (network_time > 1500000000) {    /* wait until we have a valid timestamp */
        send_node_info();
        node_info_sent = true;
      }
    } else if (health_msg_period) {
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

    /* process pending commands */
    process_commands();

    /* check stack watermarks (store the used words) */
    unsigned long idleSWM = configMINIMAL_STACK_SIZE - (uxTaskGetStackHighWaterMark(xTaskHandle_idle));
    unsigned long preSWM  = PRE_TASK_STACK_SIZE - (uxTaskGetStackHighWaterMark(xTaskHandle_pre));
    unsigned long comSWM  = COM_TASK_STACK_SIZE - (uxTaskGetStackHighWaterMark(xTaskHandle_com));
    unsigned long postSWM = POST_TASK_STACK_SIZE - (uxTaskGetStackHighWaterMark(xTaskHandle_post));

    if (idleSWM > idleTaskStackWM) {
      idleTaskStackWM = idleSWM;
      uint32_t usage = idleTaskStackWM * 100 / configMINIMAL_STACK_SIZE;
      if (usage > STACK_WARNING_THRESHOLD) {
        LOG_WARNING("stack watermark of idle task reached %u%%", usage);
      } else {
        LOG_INFO("stack watermark of idle task increased to %u%%", usage);
      }
    }
    if (preSWM > preTaskStackWM) {
      preTaskStackWM = preSWM;
      uint32_t usage = preTaskStackWM * 100 / PRE_TASK_STACK_SIZE;
      if (usage > STACK_WARNING_THRESHOLD) {
        LOG_WARNING("stack watermark of pre task reached %u%%", usage);
      } else {
        LOG_INFO("stack watermark of pre task increased to %u%%", usage);
      }
    }
    if (comSWM > comTaskStackWM) {
      comTaskStackWM = comSWM;
      uint32_t usage = comTaskStackWM * 100 / COM_TASK_STACK_SIZE;
      if (usage > STACK_WARNING_THRESHOLD) {
        LOG_WARNING("stack watermark of com task reached %u%%", usage);
      } else {
        LOG_INFO("stack watermark of com task increased to %u%%", usage);
      }
    }
    if (postSWM > postTaskStackWM) {
      postTaskStackWM = postSWM;
      uint32_t usage = postTaskStackWM * 100 / POST_TASK_STACK_SIZE;
      if (usage > STACK_WARNING_THRESHOLD) {
        LOG_WARNING("stack watermark of post task reached %u%%", usage);
      } else {
        LOG_INFO("stack watermark of post task increased to %u%%", usage);
      }
    }

    /* print some stats */
    LOG_INFO("CPU duty cycle: %u%%", (uint16_t)rtos_get_cpu_dc() / 100);
    LOG_VERBOSE("post task executed");

    /* flush the log print queue */
#if !LOG_PRINT_IMMEDIATELY
    log_flush();
#endif /* LOG_PRINT_IMMEDIATELY */

    /* round finished, prepare for low-power mode */
    update_opmode(OP_MODE_EVT_DONE);
  }
}
