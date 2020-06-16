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
- disable lptimer overflow interrupt in LPM (-> handled in compare interrupt)
- unused tim15 removed
- use the pre task regardless of whether BOLT is enabled or not
- pre task now handles the radio wakeup
- post task puts the radio into sleep mode (saves some energy if the post task has a lot of work to do)
- minor adjustments to Gloria timing

v0.1.6
- changelog added


 */
