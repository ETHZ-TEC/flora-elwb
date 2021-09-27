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
 * DPP message processing
 */

#include "main.h"


extern QueueHandle_t xQueueHandle_tx;
extern QueueHandle_t xQueueHandle_rx;
extern uint32_t health_msg_period;


/* Private variables ---------------------------------------------------------*/

static dpp_message_t      msg_buffer;
static uint32_t           rcvd_msg_cnt     = 0;
static event_msg_level_t  event_msg_level  = EVENT_MSG_LEVEL;
static event_msg_target_t event_msg_target = EVENT_MSG_TARGET;

#if BASEBOARD
LIST_CREATE(pending_bb_cmds, sizeof(scheduled_cmd_t), BASEBOARD_CMD_QUEUE_SIZE);    // list to store the pending baseboard enable/disable commands
#endif /* BASEBOARD */


/* Functions -----------------------------------------------------------------*/

/* returns true if processed (or dropped), false otherwise (forwarded) */
bool process_message(dpp_message_t* msg, bool rcvd_from_bolt)
{
  /* check message type, length and CRC */
  if (!ps_validate_msg(msg)) {
    LOG_ERROR("msg with invalid length or CRC (sender %u, len %ub, type 0x%x)", msg->header.device_id, msg->header.payload_len, msg->header.type);
    EVENT_WARNING(EVENT_SX1262_INV_MSG, ((uint32_t)msg->header.payload_len) << 16 | msg->header.device_id);
    return true;
  }
  LOG_VERBOSE("msg type: %u, src: %u, len: %uB", msg->header.type, msg->header.device_id, msg->header.payload_len);

  /* only process the message if target ID matched the node ID */
  bool forward = (msg->header.target_id == DPP_DEVICE_ID_BROADCAST);

  if (msg->header.target_id == NODE_ID || msg->header.target_id == DPP_DEVICE_ID_BROADCAST) {
    rcvd_msg_cnt++;
    if (msg->header.type == DPP_MSG_TYPE_CMD) {
      bool successful  = false;
      bool cfg_changed = false;

      LOG_VERBOSE("command received");

      switch(msg->cmd.type) {
      case DPP_COMMAND_RESET:
      case CMD_SX1262_RESET:
        if (IS_HOST) {
          if (msg->header.target_id != DPP_DEVICE_ID_BROADCAST) {
            NVIC_SystemReset();
          }
        } else {
          NVIC_SystemReset();
        }
        break;

      case CMD_SX1262_SET_COM_PERIOD:
        if (!IS_HOST) {   /* only a host node can interpret this command */
           break;
        }
        successful = elwb_sched_set_period(msg->cmd.arg16[0]);
        break;

      case CMD_SX1262_REGISTER_NODE:
        if (!IS_HOST) {   /* only a host node can interpret this command */
           break;
        }
        successful = elwb_sched_add_node(msg->cmd.arg16[0]);
        break;

      case CMD_SX1262_SET_HEALTH_PERIOD:
        if (msg->cmd.arg16[0] >= 15) {
          health_msg_period = msg->cmd.arg16[0];
          successful = true;
        }
        break;

      case CMD_SX1262_SET_EVENT_LEVEL:
        if (msg->cmd.arg[0] < NUM_EVENT_MSG_LEVELS) {
          event_msg_level = msg->cmd.arg[0];
          successful = true;
        }
        break;

      case CMD_SX1262_SET_TX_POWER:
        gloria_set_tx_power(msg->cmd.arg[0]);
        successful = true;
        break;

      case CMD_SX1262_SET_MODULATION:
        gloria_set_modulation(msg->cmd.arg[0]);
        successful = true;
        break;

#if BASEBOARD
      case CMD_SX1262_BASEBOARD_ENABLE:
      case CMD_SX1262_BASEBOARD_DISABLE:
      {
	    scheduled_cmd_t sched_cmd;
        if (IS_HOST) {
          break;    /* host node is not supposed to turn off the baseboard */
        }
        sched_cmd.type           = msg->cmd.type;
        sched_cmd.scheduled_time = msg->cmd.arg32[0];
        sched_cmd.arg            = 0;
        if (msg->cmd.arg[4] & 1) {
          /* relative time -> append generation time */
          sched_cmd.scheduled_time += msg->header.generation_time / 1000000;
        }
        if (msg->cmd.type == CMD_SX1262_BASEBOARD_ENABLE) {
          sched_cmd.arg = (uint16_t)msg->cmd.arg[6] << 8 | msg->cmd.arg[5];
        }
        if (!list_insert(pending_bb_cmds, sched_cmd.scheduled_time, &sched_cmd)) {
          LOG_WARNING("failed to add command to queue");
          EVENT_WARNING(EVENT_SX1262_QUEUE_FULL, 10);
        } else {
          LOG_VERBOSE("baseboard command %u scheduled (time: %lu)", msg->cmd.type & 0xff, sched_cmd.scheduled_time);
        }
        successful = true;
      } break;

      case CMD_SX1262_BASEBOARD_ENABLE_PERIODIC:
        if (IS_HOST) {
          break;    /* host node is not supposed to turn off the baseboard */
        }
        config.bb_en.period = (uint32_t)msg->cmd.arg16[1] * 60;  /* convert to seconds */
        if (config.bb_en.period > 0) {
          config.bb_en.starttime = rtc_get_next_timestamp_at_daytime(elwb_get_time_sec(), msg->cmd.arg[0], msg->cmd.arg[1], 0);
          if (config.bb_en.starttime > 0) {
            LOG_INFO("periodic baseboard enable scheduled (next: %u, period: %us)", config.bb_en.starttime, config.bb_en.period);
          } else {
            LOG_WARNING("invalid parameters for periodic enable cmd");
          }
        } else {
          config.bb_en.starttime = 0;
          LOG_INFO("periodic baseboard enable cleared");
        }
        successful = true;
        cfg_changed = true;
        break;

      case CMD_SX1262_BASEBOARD_POWER_EXT3:
        if (msg->cmd.arg[0]) {
          PIN_SET(BASEBOARD_EXT3_SWITCH);
          LOG_INFO("EXT3 power enabled");
        } else {
          PIN_CLR(BASEBOARD_EXT3_SWITCH);
          LOG_INFO("EXT3 power disabled");
        }
        successful = true;
        break;
#endif /* BASEBOARD */

      default:
        /* unknown command */
        if (!IS_HOST) {
          forward = true;  /* forward to BOLT */
        }
        break;
      }
      /* command successfully executed? */
      if (successful) {
        LOG_INFO("cmd 0x%x processed", msg->cmd.type & 0xff);
        EVENT_INFO(EVENT_SX1262_CMD_EXECUTED, msg->cmd.type & 0xff);
      } else {
        EVENT_WARNING(EVENT_SX1262_INV_CMD, msg->cmd.type & 0xff);
      }

      if (cfg_changed) {
    #if NVCFG_ENABLE
        if (nvcfg_save(&config)) {
          LOG_INFO("config saved to NV memory");
        } else {
          LOG_ERROR("failed to save config");
        }
    #endif /* NVCFG_ENABLE */
      }

#if BOLT_ENABLE
    /* message types only processed by the host */
    } else if (msg->header.type == DPP_MSG_TYPE_TIMESYNC) {
      set_unix_timestamp_us(msg->timestamp);
      LOG_VERBOSE("timestamp %llu received", msg->timestamp);
#endif /* BOLT_ENABLE */

    /* unknown message type */
    } else {

      if (!IS_HOST) {
        forward = true;    /* source nodes forward messages */
      }
      EVENT_WARNING(EVENT_SX1262_MSG_IGNORED, msg->header.type);
    }

  /* target id is not this node -> forward message */
  } else {
    forward = true;
  }

  /* forward the message */
  if (forward) {
    if (rcvd_from_bolt) {
      /* forward to network */
      if (!xQueueSend(xQueueHandle_tx, (uint8_t*)msg, 0)) {
        LOG_ERROR("failed to insert msg into transmit queue");
      } else {
        LOG_VERBOSE("msg forwarded to network (type: %u, dest: %u)", msg->header.type, msg->header.target_id);
      }
    } else {
#if BOLT_ENABLE
      /* forward to BOLT */
      /* on host node: make sure the generation time is valid, otherwise replace it with the current time */
      if (IS_HOST && !(msg->header.type & DPP_MSG_TYPE_MIN) && (msg->header.generation_time < MIN_VALID_GENTIME_US)) {
        LOG_WARNING("invalid message generation time (%llu) replaced", msg->header.generation_time);
        msg->header.generation_time = elwb_get_time(0);
        ps_update_msg_crc(msg);
      }
      uint8_t msg_len = DPP_MSG_LEN(msg);
      if (!bolt_write((uint8_t*)msg, msg_len)) {
        LOG_ERROR("failed to write message to BOLT");
      } else {
        LOG_VERBOSE("msg forwarded to BOLT (type: %u, len: %uB)", msg->header.type, msg_len);
      }
#endif /* BOLT_ENABLE */
    }
    return false;
  }

  return true;
}


#if BASEBOARD

void process_scheduled_bb_commands(void)
{
  uint32_t curr_time = elwb_get_time_sec();
  const scheduled_cmd_t* next_cmd = list_get_head(pending_bb_cmds);
  if (next_cmd) {
    /* there are pending commands */
    /* anything that needs to be executed now? */
    while (next_cmd && next_cmd->scheduled_time <= curr_time) {
      switch (next_cmd->type) {
      case CMD_SX1262_BASEBOARD_ENABLE:
        BASEBOARD_ENABLE();
        LOG_INFO("baseboard enabled");
        send_command_to_app(CMD_BASEBOARD_WAKEUP_MODE, next_cmd->arg, 2);
        break;
      case CMD_SX1262_BASEBOARD_DISABLE:
        BASEBOARD_DISABLE();
        LOG_INFO("baseboard disabled");
        bolt_get_write_cnt(true);         // assume all messages have been read from bolt at this point -> reset counter
        break;
      default:
        break;
      }
      list_remove_head(pending_bb_cmds, 0);
      next_cmd = list_get_head(pending_bb_cmds);
    }
  }
  /* check the periodic baseboard enable */
  if (config.bb_en.starttime > 0 && config.bb_en.starttime <= curr_time) {
    BASEBOARD_ENABLE();
    while (config.bb_en.period > 0 && config.bb_en.starttime < curr_time) {
      config.bb_en.starttime += config.bb_en.period;
    }
    LOG_INFO("baseboard enabled (next wakeup in %lus)", config.bb_en.starttime - curr_time);
  }
}

#endif /* BASEBOARD */


/*
 * calling this function from an interrupt context can lead to erratic behaviour
 * Note: if data is 0, the data in the static msg_buffer will be used
 */
bool send_message(uint16_t recipient,
                  dpp_message_type_t type,
                  const uint8_t* data,
                  uint8_t len,
                  interface_t target)
{
  /* separate sequence number for each interface */
  static uint16_t seq_no_elwb = 0;
  static uint16_t seq_no_bolt = 0;

  /* check if in interrupt context */
  if (IS_INTERRUPT()) {
    LOG_WARNING("cannot send messages from ISR (msg of type %u dropped)", type);
    return false;
  }

  /* check message length */
  if (len > DPP_MSG_PAYLOAD_LEN) {
    LOG_WARNING("invalid message length");
    EVENT_WARNING(EVENT_SX1262_INV_MSG, ((uint32_t)type) << 16 | len);
    return false;
  }

  /* compose the message */
  uint8_t msg_buffer_len = ps_compose_msg(recipient, type, data, len, &msg_buffer);
  if (!msg_buffer_len) {
    return false;
  }

  /* adjust the sequence number (per interface) */
  if ((type & DPP_MSG_TYPE_MIN) == 0) {
    if (target == INTERFACE_BOLT) {
      msg_buffer.header.seqnr = seq_no_bolt++;
    } else if (target == INTERFACE_ELWB) {
      msg_buffer.header.seqnr = seq_no_elwb++;
    }
    ps_update_msg_crc(&msg_buffer);
  }

  /* forward the message either to BOLT or the eLWB */
  switch (target) {

  case INTERFACE_BOLT:
#if BOLT_ENABLE
    if (bolt_write((uint8_t*)&msg_buffer, msg_buffer_len)) {
      LOG_VERBOSE("msg written to BOLT");
      return true;
    }
    LOG_INFO("msg dropped (BOLT queue full)");
#endif /* BOLT_ENABLE */
    break;

  case INTERFACE_ELWB:
    if (xQueueSend(xQueueHandle_tx, &msg_buffer, 0)) {
      LOG_VERBOSE("msg added to transmit queue");
      return true;
    }
    LOG_ERROR("msg dropped (TX queue full)");
    break;

  default:
    LOG_WARNING("invalid target (msg dropped)");
    break;
  }

  return false;
}


/* send the timestamp to the APP processor */
void send_timestamp(uint64_t trq_timestamp)
{
  msg_buffer.timestamp = elwb_get_time(&trq_timestamp);
  send_message(NODE_ID, DPP_MSG_TYPE_TIMESYNC, 0, 0, INTERFACE_BOLT);     /* always send to bolt */
  LOG_INFO("timestamp %llu sent", msg_buffer.timestamp);
}


void send_node_info(void)
{
  uint32_t cfg_field = 0;
  ps_compose_nodeinfo(&msg_buffer, config.rst_cnt, cfg_field);

  LOG_INFO("node info msg generated");
  /* note: host sends message towards BOLT */
  send_message(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_NODE_INFO, 0, 0, (IS_HOST ? INTERFACE_BOLT : INTERFACE_ELWB));
}


void send_node_health(void)
{
  static uint8_t  rx_dropped_last = 0;
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
  msg_buffer.com_health.radio_rssi    = -stats->rssi_avg;
  msg_buffer.com_health.radio_tx_pwr  = 0;    // TODO
  msg_buffer.com_health.radio_per     = 0;    // TODO
  if (rx_cnt_last > stats->pkt_rcvd) {
    msg_buffer.com_health.rx_cnt      = (65535 - rx_cnt_last) + stats->pkt_rcvd;
  } else {
    msg_buffer.com_health.rx_cnt      = (stats->pkt_rcvd - rx_cnt_last);
  }
  rx_cnt_last                         = stats->pkt_rcvd;
  msg_buffer.com_health.tx_queue      = uxQueueMessagesWaiting(xQueueHandle_tx);
  msg_buffer.com_health.rx_queue      = uxQueueMessagesWaiting(xQueueHandle_rx);
  msg_buffer.com_health.tx_dropped    = 0;    // TODO
  msg_buffer.com_health.rx_dropped    = stats->pkt_dropped - rx_dropped_last;
  rx_dropped_last = stats->pkt_dropped;

  /* duty cycle */
  msg_buffer.com_health.cpu_dc        = rtos_get_cpu_dc();
  rtos_reset_cpu_dc();
  msg_buffer.com_health.radio_rx_dc   = radio_get_rx_dc() / 100;
  msg_buffer.com_health.radio_tx_dc   = radio_get_tx_dc() / 100;
  radio_dc_counter_reset();

  LOG_INFO("health msg generated");

  /* the host must send to BOLT, all other nodes to the network */
  send_message(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_COM_HEALTH, 0, 0, (IS_HOST ? INTERFACE_BOLT : INTERFACE_ELWB));
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
#if BOLT_ENABLE
      /* do not report errors about BOLT via BOLT */
      if (type != EVENT_SX1262_BOLT_ERROR) {
        if (!send_message(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_EVENT, (uint8_t*)&event, 0, INTERFACE_BOLT)) {
          LOG_ERROR("failed to send event of type %u", type);
        }
      }
#endif /* BOLT_ENABLE */
    } else if (event_msg_target == EVENT_MSG_TARGET_NETWORK) {
      if (!send_message(DPP_DEVICE_ID_SINK, DPP_MSG_TYPE_EVENT, (uint8_t*)&event, 0, INTERFACE_ELWB)) {
        LOG_ERROR("failed to send event of type %u", type);
      }
    } else {
      LOG_WARNING("invalid event target");
    }
  }
}


/* send a command to the app processor */
void send_command_to_app(dpp_command_type_t cmd, uint32_t arg, uint32_t len)
{
  msg_buffer.cmd.type     = cmd;
  msg_buffer.cmd.arg32[0] = arg;
  send_message(NODE_ID, DPP_MSG_TYPE_CMD, 0, len, INTERFACE_BOLT);    /* always send to bolt */
}


#if BASEBOARD

bool schedule_bb_command(uint32_t sched_time, dpp_command_type_t cmd_type, uint16_t arg)
{
  scheduled_cmd_t cmd;

  cmd.scheduled_time = sched_time;
  cmd.type           = cmd_type;
  cmd.arg            = arg;

  return list_insert(pending_bb_cmds, sched_time, &cmd);
}

#endif /* BASEBOARD */


void set_event_level(event_msg_level_t level)
{
  event_msg_level = level;
}


void set_event_target(event_msg_target_t target)
{
  event_msg_target = target;
}

