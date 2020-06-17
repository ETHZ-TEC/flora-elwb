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
