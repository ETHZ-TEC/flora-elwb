/**
  ******************************************************************************
  * Flora
  ******************************************************************************
  * @file   app_config.h
  * @brief  application config file
  *
  *
  ******************************************************************************
  */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H


/* --- adjustable parameters --- */

/* general */
#define FW_NAME                         "DPP2eLWB"  /* max. 8 chars */
#define FW_VERSION_MAJOR                0           /* 0..6 */
#define FW_VERSION_MINOR                1           /* 0..99 */
#define FW_VERSION_PATCH                0           /* 0..99 */
#define FLOCKLAB                        1
#define BASEBOARD                       0
#define BOLT_ENABLE                     (!FLOCKLAB)
#define SWO_ENABLE                      0
#define CLI_ENABLE                      0
#define LOW_POWER_MODE                  LP_MODE_STOP2  /* low-power mode to use between rounds during periods of inactivity */

/* network parameters */
#define HOST_ID                         2
#if !FLOCKLAB
  #define NODE_ID                       HOST_ID
#endif /* FLOCKLAB */
#ifndef IS_HOST
  #define IS_HOST                       (NODE_ID == HOST_ID)
#endif /* IS_HOST */

/* time sync and drift compensation */
#define TIMESTAMP_TYPICAL_DRIFT         100   /* typical drift +/- in ppm (if exceeded, a warning will be issued) */
#define TIMESTAMP_MAX_DRIFT             150   /* max. allowed drift in ppm (higher values will be capped) */
#define TIMESTAMP_MAX_OFFSET_MS         10    /* max. allowed offset in ms that the host tries to compensate; if larger, a jump in time occurs. set to 0 to always make a jump */
#define TIMESTAMP_USE_HS_TIMER          0     /* use hs_timer for timestamping events on the TREQ pin */

/* data collection config */
#define NODE_HEALTH_MSG_PERIOD          300

/* memory */
#define PRE_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE)      /* in # words of 4 bytes */
#define COM_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE * 2)  /* in # words of 4 bytes */
#define POST_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE)      /* in # words of 4 bytes */
#define STACK_WARNING_THRESHOLD         80                              /* a warning will be generated once the stack usage of a task exceeds this value (in percent) */
#define TRANSMIT_QUEUE_SIZE             20                              /* #messages */
#define RECEIVE_QUEUE_SIZE              ELWB_CONF_MAX_DATA_SLOTS        /* #messages */
#define COMMAND_QUEUE_SIZE              10

/* Gloria config */
#define GLORIA_INTERFACE_MODULATION     10   /* FSK 250kbit/s */
#define GLORIA_INTERFACE_RF_BAND        48   /* 869.46 MHz (see table in radio_constants.c for options) */

/* eLWB config */
#define ELWB_ENABLE                     1
#define ELWB_CONF_N_TX                  2
#define ELWB_CONF_T_SCHED               (ELWB_TIMER_SECOND / 50)      /* 20ms */
#define ELWB_CONF_T_DATA                (ELWB_TIMER_SECOND / 50)      /* 20ms */
#define ELWB_CONF_T_CONT                (ELWB_TIMER_SECOND / 100)     /* 10ms */
#define ELWB_ON_WAKEUP()                update_opmode(OP_MODE_EVT_WAKEUP)
#define ELWB_IS_HOST()                  IS_HOST

/* misc */
#define HS_TIMER_COMPENSATE_DRIFT       0

/* logging */
#define LOG_ENABLE                      1
#define LOG_LEVEL                       LOG_LEVEL_VERBOSE
#if SWO_ENABLE
  #define LOG_PRINT_FUNC                swo_print
  //#define LOG_PRINT_IMMEDIATELY         1
#endif /* SWO_ENABLE */

/* debugging */
#if !BASEBOARD
  #define CPU_ON_IND()                  //PIN_SET(COM_GPIO1)  /* pin to indicate activity (e.g. to calculate the duty cycle) */
  #define CPU_OFF_IND()                 //PIN_CLR(COM_GPIO1)
  #define LPM_ON_IND()                  //PIN_CLR(COM_GPIO1)
  #define LPM_OFF_IND()                 //PIN_SET(COM_GPIO1)
  #define IDLE_TASK_RESUMED()           //PIN_SET(COM_GPIO2)
  #define IDLE_TASK_SUSPENDED()         //PIN_CLR(COM_GPIO2)
  #if FLOCKLAB
    #define ISR_ON_IND()                PIN_SET(FLOCKLAB_INT1)
    #define ISR_OFF_IND()               PIN_CLR(FLOCKLAB_INT1)
    #define ELWB_RESUMED()              PIN_SET(FLOCKLAB_LED1)
    #define ELWB_SUSPENDED()            PIN_CLR(FLOCKLAB_LED1)
    #define PRE_TASK_RESUMED()          PIN_SET(FLOCKLAB_LED1)
    #define PRE_TASK_SUSPENDED()        PIN_CLR(FLOCKLAB_LED1)
    #define POST_TASK_RESUMED()         PIN_SET(FLOCKLAB_LED1)
    #define POST_TASK_SUSPENDED()       PIN_CLR(FLOCKLAB_LED1)
    #define GLORIA_START_IND()          led_on(LED_SYSTEM); PIN_SET(FLOCKLAB_LED3)
    #define GLORIA_STOP_IND()           led_off(LED_SYSTEM); PIN_CLR(FLOCKLAB_LED3)
  #else /* FLOCKLAB */
    #define ISR_ON_IND()                PIN_SET(COM_GPIO2)
    #define ISR_OFF_IND()               PIN_CLR(COM_GPIO2)
    #define ELWB_RESUMED()              PIN_SET(COM_GPIO1)
    #define ELWB_SUSPENDED()            PIN_CLR(COM_GPIO1)
    #define PRE_TASK_RESUMED()          PIN_SET(COM_GPIO1)
    #define PRE_TASK_SUSPENDED()        PIN_CLR(COM_GPIO1)
    #define POST_TASK_RESUMED()         PIN_SET(COM_GPIO1)
    #define POST_TASK_SUSPENDED()       PIN_CLR(COM_GPIO1)
    #define GLORIA_START_IND()          led_on(LED_SYSTEM); PIN_SET(COM_GPIO1)
    #define GLORIA_STOP_IND()           led_off(LED_SYSTEM); PIN_CLR(COM_GPIO1)
  #endif /* FLOCKLAB */
#else /* BASEBOARD */
  #define GLORIA_START_IND()            led_on(LED_SYSTEM)
  #define GLORIA_STOP_IND()             led_off(LED_SYSTEM)
#endif /* BASEBOARD */


/* --- parameter checks --- */

#if BOLT_ENABLE && (BOLT_MAX_MSG_LEN < DPP_MSG_PKT_LEN)
#error "BOLT_MAX_MSG_LEN is too small"
#endif

#if FLOCKLAB && BASEBOARD
#error "can't use target FLOCKLAB and BASEBOARD at the same time"
#endif

#if FLOCKLAB && (HOST_ID < 2 || HOST_ID > 12)
#error "HOST_ID is invalid for target FLOCKLAB"
#endif

#endif /* __APP_CONFIG_H */
