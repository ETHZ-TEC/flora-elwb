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

v0.1.8
- issue fixed where periodic baseboard wakeup would not work properly when a jump in time occurs
- potential issue fixed in eLWB where a recovery on the host node would not be possible if the scheduled wakeup time happens to be in the past
- log printing: separate seconds and milliseconds (relative timestamp since start) with a dot
- eLWB listen timeout callback function added
- new commands added: register node, set communication base period, set health period, set event message level, set TX power, set modulation
- new events added: BOLT error
- re-init BOLT in case a read operation fails, and timeout increased from 60 to 100us
- elwb_start split into two functions (init and start), startup is now relative to the current time
- eLWB D-ACK implemented and enabled

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
