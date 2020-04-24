/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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

#ifndef __MESSAGE_H
#define __MESSAGE_H


/* --- definitions --- */

#ifndef EVENT_MSG_ENABLE
#define EVENT_MSG_ENABLE          0
#endif /* EVENT_MSG_ENABLE */

#ifndef EVENT_MSG_TARGET
#define EVENT_MSG_TARGET          EVENT_MSG_TARGET_UART
#endif /* EVENT_MSG_TARGET */

/* default event message level */
#ifndef EVENT_MSG_LEVEL
#define EVENT_MSG_LEVEL           EVENT_MSG_LEVEL_INFO
#endif /* EVENT_MSG_LVEL */


/* --- typedefs --- */

/* event notification level (equivalent to debug_level_t scale) */
typedef enum {
  EVENT_MSG_LEVEL_QUIET,      /* no notifications */
  EVENT_MSG_LEVEL_ERROR,      /* report only errors */
  EVENT_MSG_LEVEL_WARNING,    /* report warnings and errors */
  EVENT_MSG_LEVEL_INFO,       /* report information, warnings and errors */
  EVENT_MSG_LEVEL_VERBOSE,    /* report all */
  NUM_EVENT_MSG_LEVELS,
} event_msg_level_t;

typedef enum {
  EVENT_MSG_TARGET_NONE,
  EVENT_MSG_TARGET_UART,
  EVENT_MSG_TARGET_BOLT,
  EVENT_MSG_TARGET_NETWORK,
} event_msg_target_t;

typedef struct {
  dpp_command_type_t  type;
  uint16_t            arg;
  uint32_t            scheduled_time;
} scheduled_cmd_t;


/* --- macros --- */

#define EVENT_INFO(evt, val)      send_event(EVENT_MSG_LEVEL_INFO, evt, val)
#define EVENT_WARNING(evt, val)   send_event(EVENT_MSG_LEVEL_WARNING, evt, val)
#define EVENT_ERROR(evt, val)     send_event(EVENT_MSG_LEVEL_ERROR, evt, val)
#define EVENT_VERBOSE(evt, val)   send_event(EVENT_MSG_LEVEL_VERBOSE, evt, val)


/* --- function prototypes --- */

uint_fast8_t  process_message(dpp_message_t* msg, bool rcvd_from_bolt);
void          process_commands(void);     /* process pending commands */
void          send_node_health(void);
void          send_node_info(void);
void          send_timestamp(uint64_t trq_timestamp);
uint_fast8_t  send_msg(uint16_t recipient, dpp_message_type_t type, const uint8_t* data, uint8_t len, bool send_to_bolt);
void          send_event(event_msg_level_t level, dpp_event_type_t type, uint32_t val);
void          set_event_level(event_msg_level_t level);
void          set_event_target(event_msg_target_t target);


#endif /* __MESSAGE_H */
