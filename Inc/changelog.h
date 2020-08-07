/**
  ******************************************************************************
  * Flora eLWB changelog
  ******************************************************************************
  * @file   changelog.h
  * @brief  version history and changelog
  *
  *
  ******************************************************************************
  */

/*

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
