/*
 * task_elwb.c
 *
 *  Created on: Mar 25, 2020
 *      Author: rdaforno
 */

#include "main.h"


/* Private variables ---------------------------------------------------------*/


extern TaskHandle_t xTaskHandle_pre;
extern TaskHandle_t xTaskHandle_post;
extern QueueHandle_t xQueueHandle_rx;
extern QueueHandle_t xQueueHandle_tx;
extern QueueHandle_t xQueueHandle_retransmit;

/* global variables for binary patching the config */
volatile uint16_t         host_id           = HOST_ID;
static volatile int8_t    gloria_power      = GLORIA_INTERFACE_POWER;
static volatile uint8_t   gloria_modulation = GLORIA_INTERFACE_MODULATION;
static volatile uint8_t   gloria_band       = GLORIA_INTERFACE_RF_BAND;
static volatile uint8_t   elwb_n_tx         = ELWB_CONF_N_TX;
static volatile uint8_t   elwb_num_hops     = ELWB_CONF_NUM_HOPS;
static volatile uint32_t  elwb_period       = ELWB_CONF_SCHED_PERIOD;

extern uint32_t           health_msg_period;


void listen_timeout(void)
{
  /* nothing to do */
}


void collect_radio_stats(uint16_t initiator_id, elwb_phases_t elwb_phase, elwb_packet_t* packet)
{
  if (initiator_id != NODE_ID) {
    /* check if schedule packet is valid (there are sporadic cases of SCHED1 packets receptions which have a valid PHY CRC but are not valid elwb packets) */
    if ((elwb_phase == ELWB_PHASE_SCHED1) && !ELWB_IS_SCHEDULE_PACKET(packet)) {
      return;
    }

    uint8_t  rx_cnt        = gloria_get_rx_cnt();
    uint8_t  rx_started    = gloria_get_rx_started_cnt();
    uint8_t  rx_idx        = 0;
    int8_t   snr           = -99;
    int16_t  rssi          = -99;
    uint8_t  payload_len   = 0;
    uint8_t  t_ref_updated = 0;
    uint64_t network_time  = 0;
    uint64_t t_ref         = 0;

    if (rx_cnt > 0 && ELWB_IS_PKT_HEADER_VALID(packet)) {
      rx_idx         = gloria_get_rx_index();
      snr            = gloria_get_snr();
      rssi           = gloria_get_rssi();
      payload_len    = gloria_get_payload_len();
      t_ref_updated  = gloria_is_t_ref_updated();
      if (t_ref_updated) {
        elwb_get_last_syncpoint(&network_time, &t_ref);
      }
    }

    /* print in json format */
    LOG_INFO("{"
             "\"initiator_id\":%d,"
             "\"elwb_phase\":%d,"
             "\"rx_cnt\":%d,"
             "\"rx_idx\":%d,"
             "\"rx_started\":%d,"
             "\"rssi\":%d,"
             "\"snr\":%d,"
             "\"payload_len\":%d,"
             "\"t_ref_updated\":%llu,"
             "\"network_time\":%llu,"
             "\"t_ref\":%llu"
             "}",
      initiator_id,
      elwb_phase,
      rx_cnt,
      rx_idx,
      rx_started,
      rssi,
      snr,
      payload_len,
      t_ref_updated,
      network_time,
      t_ref
    );
  }
}


/* communication task */
void vTask_com(void const * argument)
{
  LOG_VERBOSE("eLWB task started");

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

  /* set gloria config values */
  gloria_set_tx_power(gloria_power);
  gloria_set_modulation(gloria_modulation);
  gloria_set_band(gloria_band);

  /* set elwb config values */
  if (elwb_sched_set_period(elwb_period)) { // Note: period needs to be larger than max round duration (based on current values of )
    LOG_INFO("eLWB successfully set period to %lus", elwb_period);
  } else {
    LOG_WARNING("eLWB rejects setting period to %lus", elwb_period);
  }
  if (elwb_set_n_tx(elwb_n_tx)) {  // Note: Configured period needs to be large enough!
    LOG_INFO("eLWB successfully set n_tx to %u", elwb_n_tx);
  } else {
    LOG_WARNING("eLWB rejects setting n_tx to %u", elwb_n_tx);
  }
  if (elwb_set_num_hops(elwb_num_hops)) {  // Note: Configured period needs to be large enough!
    LOG_INFO("eLWB successfully set num_hops to %u", elwb_num_hops);
  } else {
    LOG_WARNING("eLWB rejects setting num_hops to %u", elwb_num_hops);
  }

  /* init eLWB */
  if (!elwb_init(xTaskGetCurrentTaskHandle(), xTaskHandle_pre, xTaskHandle_post, xQueueHandle_rx, xQueueHandle_tx, xQueueHandle_retransmit, listen_timeout, IS_HOST)) {
    FATAL_ERROR("eLWB init failed");
  }
  elwb_register_slot_callback(collect_radio_stats);

  /* print config in json format */
  LOG_INFO("{"
           "\"node_id\":%d,"
           "\"host_id\":%d,"
           "\"tx_power\":%d,"
           "\"modulation\":%d,"
           "\"rf_band\":%d,"
           "\"n_tx\":%d,"
           "\"num_hops\":%d,"
           "\"elwb_pkt_len\":%d,"
           "\"elwb_num_slots\":%d,"
           "\"elwb_period\":%lu,"
           "\"health_msg_period\":%lu"
           "}",
    NODE_ID,
    host_id,
    gloria_power,
    gloria_modulation,
    gloria_band,
    elwb_get_n_tx(),
    elwb_get_num_hops(),
    ELWB_CONF_MAX_PAYLOAD_LEN,
    ELWB_CONF_MAX_DATA_SLOTS,
    elwb_sched_get_period(),
    health_msg_period
  );

  /* start eLWB */
  elwb_start();
  FATAL_ERROR("eLWB task terminated");

  /* for debugging purposes only if eLWB is not used */
  while (1) {
    xTaskNotify(xTaskHandle_post, 0, eNoAction);    /* notify the post task */
    ELWB_SUSPENDED();
    vTaskDelay(MS_TO_RTOS_TICKS(900));
    ELWB_RESUMED();
    /* send a flood */
    const char payload[32] = "hello world!";
    gloria_start(NODE_ID == HOST_ID, (uint8_t*)payload, sizeof(payload), 2, 1);
    /* wait some time */
    vTaskDelay(MS_TO_RTOS_TICKS(100));
    /* stop the flood */
    gloria_stop();
    LOG_INFO("com task executed");
  }
}
