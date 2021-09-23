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
 * Flora eLWB changelog
 */

/*

v0.2.6
- code refactoring, some adjustments to message handling and debug prints
- timestamping fixed

v0.2.5
- minor code refactoring
- data collection updated

v0.2.4
- data collection added
- several bugfixes in eLWB protocol

v0.2.3
- minor bugfixes (startup phase, sx1262 image calibration)
- schedule init moved to elwb_init()
- max. number of nodes increased to 30

v0.2.2
- BOLT SPI clock speed reduced (resolved issue with corrupted messages)
- UART speed on FlockLab reduced to 460800
- updated to the latest Flora lib

v0.2.1
- applied necessary changes to make code compatible with latest flora lib
- buffer overflow issue fixed in eLWB

v0.2.0
- header with a configurable network ID added to all eLWB packets (unique IDs allows multiple networks to coexist)
- issue fixed where a data packet could have been interpreted as an eLWB schedule packet

v0.1.19
- elwb stats adjusted

v0.1.18
- flora lib update
- elwb debug stats printing fixed (values swapped)

v0.1.17
- flora lib update (toa calculation and radio constants)

v0.1.16
- print radio frequency and modulation at startup
- bugfix in eLWB (condition for exiting bootstrap mode adjusted)

v0.1.15
- option added to force overwrite node ID (in conjunction with non-volatile config)
- TX power parameter added to app_config
- simple baseboard detection added based on the BASEBOARD_ENABLE pin
- baseboard on/off command is now only accepted by source nodes

v0.1.14
- issue fixed where a previously set node ID would persist even after reprogramming the node
- config parameter ELWB_NUM_HOPS added, is passed to GLORIA_INTERFACE_FLOOD_DURATION() for slot calculation
- ELWB_CONF_SCHED_PERIOD_MIN set to 5

v0.1.13
- only update RTC in post task if eLWB time is reasonable (t >> 0)
- post task notification before starting the eLWB removed (caused issues)

v0.1.12
- bugfix in D-ACK feature of eLWB (payload overwritten)
- default RF mode changed to FSK

v0.1.11
- minor change to log printing in eLWB and more error checks added
- use FSK on FlockLab

v0.1.10
- bugfix in eLWB

v0.1.9
- use low-power mode SLEEP if SWO is enabled
- add NOPs when ISR IND not used
- new event message: stack watermark warning
- default communication band changed to LoRa SF5
- eLWB slot size is now calculated at compile time

v0.1.8
- issue fixed where periodic baseboard wakeup would not work properly when a jump in time occurs
- potential issue fixed in eLWB where a recovery on the host node would not be possible if the scheduled wakeup time happens to be in the past
- log printing: separate seconds and milliseconds (relative timestamp since start) with a dot
- eLWB listen timeout callback function added
- new commands added: register node, set communication base period, set health period, set event message level, set TX power, set modulation, EXT3 power enable
- new event added: BOLT error
- re-init BOLT in case a read operation fails, and timeout increased from 60 to 100us
- elwb_start split into two functions (init and start), startup is now relative to the current time
- eLWB D-ACK implemented and enabled
- FlockLab pin mappings adjusted for rev1.1 adapter
- issue in eLWB fixed (t_slot_ofs overflow)

v0.1.7
- unused timer tim15 removed
- use the pre task regardless of whether BOLT is enabled
- pre task now handles the radio wakeup
- post task puts the radio into sleep mode (saves some energy if the post task has a lot of work to do)
- minor adjustments to Gloria timings
- minor bugfixes in eLWB (slot alignment)
- max. preprocess task runtime reduced from 100 to 50ms
- if lptimer expiration is in the past, schedule expiration asap
- bugfix in hs_timer_schedule(), caused TX misalignment during a Gloria flood

v0.1.6
- changelog added


 */
