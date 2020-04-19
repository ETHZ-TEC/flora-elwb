/*
 * message.c
 *
 *  Created on: Apr 7, 2020
 *      Author: rdaforno
 */

/* functions related to DPP message handling */

#include "main.h"


extern QueueHandle_t xQueueHandle_tx;
extern QueueHandle_t xQueueHandle_rx;


/* Private variables ---------------------------------------------------------*/

static dpp_message_t      msg_buffer;
static uint16_t           rcvd_msg_cnt     = 0;
static event_msg_level_t  event_msg_level  = EVENT_MSG_LEVEL;
static event_msg_target_t event_msg_target = EVENT_MSG_TARGET;


/* Functions -----------------------------------------------------------------*/

/* Do not call this function from an interrupt context!
 * Note: data may be 0, in that case the function will use the payload in 
 *       the global struct msg_buffer                                            */
uint_fast8_t send_msg(uint16_t recipient,
                      dpp_message_type_t type,
                      const uint8_t* data,
                      uint8_t len,
                      bool send_to_bolt)
{
  /* separate sequence number for each interface */
  static uint16_t seq_no_lwb  = 0;
  static uint16_t seq_no_bolt = 0;

  /* check message length */
  if (len > DPP_MSG_PAYLOAD_LEN) {
    LOG_WARNING_CONST("invalid message length");
    EVENT_WARNING(EVENT_CC430_INV_MSG, ((uint32_t)type) << 16 | 0xff00 | len);
    return 0;
  }

  /* compose the message header */
  msg_buffer.header.device_id   = NODE_ID;
  msg_buffer.header.type        = type;
  msg_buffer.header.payload_len = len;
  if (!len) {
    switch(type) {
    case DPP_MSG_TYPE_COM_HEALTH:
      msg_buffer.header.payload_len = sizeof(dpp_com_health_t); break;
    case DPP_MSG_TYPE_CMD:
      msg_buffer.header.payload_len = 6; break;  /* default is 6 bytes */
    case DPP_MSG_TYPE_EVENT:
      msg_buffer.header.payload_len = sizeof(dpp_event_t); break;
    case DPP_MSG_TYPE_NODE_INFO:
      msg_buffer.header.payload_len = sizeof(dpp_node_info_t); break;
    case DPP_MSG_TYPE_TIMESYNC:
      msg_buffer.header.payload_len = sizeof(dpp_timestamp_t); break;
    default:
      break;
    }
  }
  msg_buffer.header.target_id = recipient;
  if (send_to_bolt) {
    msg_buffer.header.seqnr = seq_no_bolt++;
  } else {
    msg_buffer.header.seqnr = seq_no_lwb++;
  }
  msg_buffer.header.generation_time = elwb_get_time(0);

  /* copy the payload if valid */
  if (msg_buffer.header.payload_len && data) {
    memcpy(msg_buffer.payload, data, msg_buffer.header.payload_len);
  }
  /* calculate and append the CRC */
  uint16_t msg_buffer_len = DPP_MSG_LEN(&msg_buffer);
  uint16_t crc = crc16((uint8_t*)&msg_buffer, msg_buffer_len - 2, 0);
  DPP_MSG_SET_CRC16(&msg_buffer, crc);

  /* forward the message either to BOLT or the eLWB */
  if (send_to_bolt) {
    if (bolt_write((uint8_t*)&msg_buffer, msg_buffer_len)) {
      LOG_VERBOSE_CONST("msg written to BOLT");
      return 1;
    }
    LOG_INFO_CONST("msg dropped (BOLT queue full)");
  } else {
    if (xQueueSend(xQueueHandle_tx, &msg_buffer, 0)) {
      LOG_VERBOSE_CONST("msg added to transmit queue");
      return 1;
    }
    LOG_ERROR_CONST("msg dropped (TX queue full)");
  }
  return 0;
}

/* returns 1 if processed (or dropped), 0 if forwarded */
uint_fast8_t process_message(dpp_message_t* msg, bool rcvd_from_bolt)
{
  uint16_t msg_len = DPP_MSG_LEN(msg);

  /* check message type, length and CRC */
  if (msg->header.type & DPP_MSG_TYPE_MIN ||
      msg_len > DPP_MSG_PKT_LEN ||
      msg_len < (DPP_MSG_HDR_LEN + 2) ||
      msg->header.payload_len == 0 ||
      DPP_MSG_GET_CRC16(msg) != crc16((uint8_t*)msg, msg_len - 2, 0)) {
    LOG_ERROR("msg with invalid length or CRC (%ub, type %u)", msg_len, msg->header.type);
    EVENT_WARNING(EVENT_CC430_INV_MSG, ((uint32_t)msg_len) << 16 | msg->header.device_id);
    return 1;
  }
  LOG_VERBOSE("msg type: %u, src: %u, len: %uB", msg->header.type, msg->header.device_id, msg_len);

  /* only process the message if target ID matched the node ID */
  uint16_t forward     = (msg->header.target_id == DPP_DEVICE_ID_BROADCAST);
  uint8_t  cfg_changed = 0;

  if (msg->header.target_id == NODE_ID || forward) {
    rcvd_msg_cnt++;
    if (msg->header.type == DPP_MSG_TYPE_CMD) {
      LOG_VERBOSE("command received");
      uint8_t  successful = 0;
      //uint16_t arg1 = msg->cmd.arg16[0];

      switch(msg->cmd.type) {
      case DPP_COMMAND_RESET:
      case CMD_CC430_RESET:
        if (IS_HOST) {
          // only reset if message is not a broadcast message
          if (!forward) {
            //TODO trigger reset
          }
        } else {
          //TODO trigger reset
        }
        break;

      // TODO implement more commands

  #if IS_HOST
      /* commands that only the host can handle */
      //case CMD_X:
      //    successful = 1;
      //  break;
  #else
      /* commands only for source nodes */

  #endif /* IS_HOST */
      default:
        /* unknown command */
        forward = 1;  /* forward to BOLT */
        break;
      }
      /* command successfully executed? */
      if (successful) {
        LOG_INFO("cmd %u processed", msg->cmd.type);
        //uint32_t val = (((uint32_t)arg1) << 16 | msg->cmd.type);
        //EVENT_INFO(EVENT_CC430_CFG_CHANGED, val);
        /* if necessary, store the new config in the flash memory */
        if (cfg_changed) {
          //nvcfg_save(&cfg);   TODO
        }
      }
  #if IS_HOST

    /* message types only processed by the host */
    } else if (msg->header.type == DPP_MSG_TYPE_TIMESYNC) {
      set_master_timestamp(msg->timestamp);
      LOG_VERBOSE("timestamp %llu received", msg->timestamp);

  #endif /* IS_HOST */

    /* unknown message type */
    } else {

      if (!IS_HOST) {
        forward = 1;    /* source nodes forward messages */
        EVENT_WARNING(EVENT_CC430_MSG_IGNORED, msg->header.type);
      }
    }

  /* target id is not this node -> forward message */
  } else {
    forward = 1;
  }

  /* forward the message */
  if (forward) {
    if (rcvd_from_bolt) {
      /* forward to network */
      if (!xQueueSend(xQueueHandle_tx, (uint8_t*)msg, 0)) {
        LOG_ERROR_CONST("failed to insert msg into transmit queue");
      } else {
        LOG_VERBOSE("msg forwarded to network (type: %u, dest: %u)", msg->header.type, msg->header.target_id);
      }
    } else {
      /* forward to BOLT */
      if (!bolt_write((uint8_t*)msg, msg_len)) {
        LOG_ERROR("failed to write message to BOLT");
      } else {
        LOG_VERBOSE("msg forwarded to BOLT (type: %u, len: %uB)", msg->header.type, msg_len);
      }
    }
    return 0;
  }
  return 1;
}

void send_timestamp(uint64_t trq_timestamp)
{
  /* timestamp request: calculate the timestamp and send it over BOLT */
  msg_buffer.timestamp = elwb_get_time(&trq_timestamp);
  send_msg(NODE_ID, DPP_MSG_TYPE_TIMESYNC, 0, 0, true);
  LOG_INFO("timestamp %llu sent", msg_buffer.timestamp);
}

void send_node_info(void)
{
  memset((uint8_t*)&msg_buffer.node_info, 0, sizeof(dpp_node_info_t));
  msg_buffer.node_info.component_id = DPP_COMPONENT_ID_SX1262;
  msg_buffer.node_info.compiler_ver = (__GNUC__ * 1000000 + __GNUC_MINOR__ * 1000 + __GNUC_PATCHLEVEL__);
  msg_buffer.node_info.compile_date = 0;    // TODO
  msg_buffer.node_info.fw_ver       = FW_VERSION;
  msg_buffer.node_info.rst_cnt      = 0;    // TODO
  msg_buffer.node_info.rst_flag     = 0;    // TODO
  msg_buffer.node_info.sw_rev_id    = 0;    // TODO   hexstr_to_uint32(GIT_HEADREV_SHA);
  memcpy(msg_buffer.node_info.compiler_desc, "GCC", MIN(4, strlen("GCC")));
  memcpy(msg_buffer.node_info.fw_name, FW_NAME, MIN(8, strlen(FW_NAME)));
  memcpy(msg_buffer.node_info.mcu_desc, "STM32L433CC", MIN(12, strlen("STM32L433CC")));

  LOG_INFO("node info msg generated");
  /* note: host sends message towards BOLT */
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_NODE_INFO, 0, 0, IS_HOST);
}

void send_node_health(void)
{
  static uint8_t  tx_dropped_last = 0,
                  rx_dropped_last = 0;
  static uint16_t rx_cnt_last     = 0;

  const elwb_stats_t* stats = elwb_get_stats();

  /* collect ADC values */
  msg_buffer.com_health.core_temp     = 0;    // TODO
  msg_buffer.com_health.core_vcc      = 0;    // TODO
  msg_buffer.com_health.uptime        = LPTIMER_NOW_SEC();
  msg_buffer.com_health.msg_cnt       = rcvd_msg_cnt;
  rcvd_msg_cnt                        = 0;    /* reset value */
  msg_buffer.com_health.stack         = 0;    // TODO get max stack size from FreeRTOS

  /* radio / communication stats */
  msg_buffer.com_health.radio_snr     = 0;    // TODO
  msg_buffer.com_health.radio_rssi    = 0;    // TODO
  msg_buffer.com_health.radio_tx_pwr  = 0;    // TODO
  msg_buffer.com_health.radio_per     = 0;    // TODO
  if (rx_cnt_last > stats->pkt_rcv) {
    msg_buffer.com_health.rx_cnt      = (65535 - rx_cnt_last) + stats->pkt_rcv;
  } else {
    msg_buffer.com_health.rx_cnt      = (stats->pkt_rcv - rx_cnt_last);
  }
  rx_cnt_last                         = stats->pkt_rcv;
  msg_buffer.com_health.tx_queue      = uxQueueMessagesWaiting(xQueueHandle_tx);
  msg_buffer.com_health.rx_queue      = uxQueueMessagesWaiting(xQueueHandle_rx);
  msg_buffer.com_health.tx_dropped    = stats->txbuf_drop - tx_dropped_last;
  tx_dropped_last = stats->txbuf_drop;
  msg_buffer.com_health.rx_dropped    = stats->rxbuf_drop - rx_dropped_last;
  rx_dropped_last = stats->rxbuf_drop;

  /* duty cycle */
  msg_buffer.com_health.cpu_dc        = RTOS_getDutyCycle();
  RTOS_resetDutyCycle();
  msg_buffer.com_health.radio_rx_dc   = 0;    // TODO
  msg_buffer.com_health.radio_tx_dc   = 0;    // TODO

  /* the host must send to BOLT, all other nodes to the network */
  send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_COM_HEALTH, 0, 0, IS_HOST);

  LOG_INFO_CONST("health msg generated");
}

void send_event(event_msg_level_t level, dpp_event_type_t type, uint32_t val)
{
  if (event_msg_level < level) {
    return;
  }
  if (event_msg_target == EVENT_MSG_TARGET_UART) {
    if (level == EVENT_MSG_LEVEL_INFO) {
      LOG_INFO("event 0x%02x occurred (value: 0x%02lx)", type, val);
    } else if (level == EVENT_MSG_LEVEL_WARNING) {
      LOG_WARNING("event 0x%02x occurred (value: 0x%02lx)", type, val);
    } else if (level == EVENT_MSG_LEVEL_ERROR) {
      LOG_ERROR("event 0x%02x occurred (value: 0x%02lx)", type, val);
    } else if (level == EVENT_MSG_LEVEL_VERBOSE) {
      LOG_VERBOSE("event 0x%02x occurred (value: 0x%02lx)", type, val);
    }
  } else {
    dpp_event_t event;
    event.type = type;
    event.value = val;
    if (event_msg_target == EVENT_MSG_TARGET_BOLT) {
      send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_EVENT, (uint8_t*)&event, 0, true);
      LOG_VERBOSE_CONST("event msg sent to BOLT");
    } else if (event_msg_target == EVENT_MSG_TARGET_NETWORK) {
      send_msg(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_EVENT, (uint8_t*)&event, 0, false);
      LOG_VERBOSE_CONST("event msg sent to LWB");
    } else {
      LOG_WARNING_CONST("invalid event target");
    }
  }
}

void set_event_level(event_msg_level_t level)
{
  event_msg_level = level;
}

void set_event_target(event_msg_target_t target)
{
  event_msg_target = target;
}

