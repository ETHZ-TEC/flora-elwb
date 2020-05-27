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
#define FW_VERSION_PATCH                4           /* 0..99 */
#define FLOCKLAB                        0
#define BASEBOARD                       1
#define BOLT_ENABLE                     (!FLOCKLAB)
#define SWO_ENABLE                      0
#define CLI_ENABLE                      0
#define LOW_POWER_MODE                  LP_MODE_SLEEP  //STOP2  /* low-power mode to use between rounds during periods of inactivity */

/* network parameters */
#if BASEBOARD
  #define HOST_ID                       103
#else
  #define HOST_ID                       2
#endif /* BASEBOARD */
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
#define TIMESTAMP_USE_HS_TIMER          (LOW_POWER_MODE == LP_MODE_SLEEP)   /* don't use hs_timer for timestamping events on the TREQ pin if LPM != SLEEP */

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

/* non-volatile config storage */
#define NVCFG_ENABLE                    1    /* use non-volatile config storage */
#define NVCFG_BLOCK_SIZE                16   /* note: must be sizeof(nv_config_t)! */

/* Gloria config */
#define GLORIA_INTERFACE_MODULATION     10   /* FSK 250kbit/s */
#define GLORIA_INTERFACE_RF_BAND        48   /* 869.46 MHz (see table in radio_constants.c for options) */

/* eLWB config */
#define ELWB_ENABLE                     1
#define ELWB_CONF_STARTUP_DELAY         4000
#define ELWB_CONF_N_TX                  2
#define ELWB_CONF_T_SCHED               (ELWB_TIMER_SECOND / 50)      /* 20ms */
#define ELWB_CONF_T_DATA                (ELWB_TIMER_SECOND / 50)      /* 20ms */
#define ELWB_CONF_T_CONT                (ELWB_TIMER_SECOND / 100)     /* 10ms */
#define ELWB_CONF_SCHED_PERIOD_IDLE     15
#define ELWB_CONF_SCHED_PERIOD_MAX      120
#define ELWB_ON_WAKEUP()                update_opmode(OP_MODE_EVT_WAKEUP)
#define ELWB_IS_HOST()                  IS_HOST
#if FLOCKLAB
  #define ELWB_CONF_T_PREPROCESS        0     /* no pre task */
#else
  #define ELWB_CONF_T_PREPROCESS        (ELWB_TIMER_SECOND / 10)      /* 100ms */
#endif /* FLOCKLAB */

/* baseboard */
#if BASEBOARD
  #define BASEBOARD_TREQ_WATCHDOG       3600  /* if != 0, the baseboard will be power-cycled if no time request has been received within the specified #seconds */
#else /* BASEBOARD_TREQ_WATCHDOG */
  #define BASEBOARD_TREQ_WATCHDOG       0
#endif /* BASEBOARD_TREQ_WATCHDOG */

/* misc */
#define HS_TIMER_COMPENSATE_DRIFT       0
#define HS_TIMER_INIT_FROM_RTC          0
#define LPTIMER_RESET_WDG_ON_OVF        1
#define LPTIMER_CHECK_EXP_TIME          1
#define UART_FIFO_BUFFER_SIZE           1     /* not used */
#define CONFIG_ENABLE                   0     /* not used */

/* logging */
#define LOG_ENABLE                      1
#define LOG_LEVEL                       LOG_LEVEL_VERBOSE
#if BASEBOARD
  #define LOG_ADD_TIMESTAMP             0       /* don't print the timestamp on the baseboard */
  #define LOG_USE_COLORS                0
  #define LOG_LEVEL_ERROR_STR           "<3>"  /* use syslog severity level number instead of strings */
  #define LOG_LEVEL_WARNING_STR         "<4>"
  #define LOG_LEVEL_INFO_STR            "<6>"
  #define LOG_LEVEL_VERBOSE_STR         "<7>"
#endif /* BASEBOARD */
#if FLOCKLAB
  #define LOG_ADD_TIMESTAMP             0       /* don't print the timestamp on FlockLab */
#endif /* FLOCKLAB */
#if SWO_ENABLE
  #define LOG_PRINT_FUNC                swo_print
  //#define LOG_PRINT_IMMEDIATELY         1
#endif /* SWO_ENABLE */

/* debugging */
#if !BASEBOARD
  #define CPU_ON_IND()                  //PIN_SET(COM_GPIO1)  /* pin to indicate activity (e.g. to calculate the duty cycle) */
  #define CPU_OFF_IND()                 //PIN_CLR(COM_GPIO1)
  #define LPM_ON_IND()                  PIN_CLR(COM_GPIO1)
  #define LPM_OFF_IND()                 PIN_SET(COM_GPIO1)
  #define IDLE_TASK_RESUMED()           //PIN_SET(COM_GPIO2)
  #define IDLE_TASK_SUSPENDED()         //PIN_CLR(COM_GPIO2)
  #if FLOCKLAB
    #define ISR_ON_IND()                PIN_SET(FLOCKLAB_INT1)
    #define ISR_OFF_IND()               PIN_CLR(FLOCKLAB_INT1)
    #define ELWB_RESUMED()              PIN_SET(FLOCKLAB_LED1)
    #define ELWB_SUSPENDED()            PIN_CLR(FLOCKLAB_LED1)
    #define POST_TASK_RESUMED()         PIN_SET(FLOCKLAB_LED1); PIN_CLR(FLOCKLAB_LED1); PIN_SET(FLOCKLAB_LED1)
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

#if BASEBOARD_TREQ_WATCHDOG > 0 && BASEBOARD_TREQ_WATCHDOG < 120
#error "BASEBOARD_TREQ_WATCHDOG must be >= 120"
#endif

#endif /* __APP_CONFIG_H */
