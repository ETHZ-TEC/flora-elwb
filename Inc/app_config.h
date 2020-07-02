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
#define FW_VERSION_PATCH                8           /* 0..99 */

#define FLOCKLAB                        0           /* set to 1 to run on FlockLab */
#define BASEBOARD                       0           /* set to 1 if the comboard will be installed on a baseboard */
#define FLOCKLAB_SWD                    0           /* set to 1 to reserve SWDIO / SWDCLK pins for debugging (GPIOs not available for tracing) */
#define SWO_ENABLE                      0           /* set to 1 to enable data tracing or serial printing via SWO pin */
#define CLI_ENABLE                      0           /* command line interface */

/* network parameters */
#if BASEBOARD
  #define HOST_ID                       103  //110
#else
  #define HOST_ID                       2
#endif /* BASEBOARD */
#if !FLOCKLAB
  #define NODE_ID                       HOST_ID
#endif /* FLOCKLAB */
#define IS_HOST                         (NODE_ID == HOST_ID)

/* energy (low-power mode) */
#if SWO_ENABLE
  #define LOW_POWER_MODE                LP_MODE_SLEEP  /* low-power mode to use between rounds during periods of inactivity */
  #define TIMESTAMP_USE_HS_TIMER        1              /* use hs_timer for timestamping events on the TREQ pin for better accuracy */
#else /* SWO_ENABLE */
  #define LOW_POWER_MODE                LP_MODE_STOP2  /* low-power mode to use between rounds during periods of inactivity */
  #define TIMESTAMP_USE_HS_TIMER        0              /* don't use hs_timer for timestamping events on the TREQ pin if LPM != SLEEP */
#endif /* SWO_ENABLE */

/* time sync and drift compensation */
#define TIMESTAMP_TYPICAL_DRIFT         100   /* typical drift +/- in ppm (if exceeded, a warning will be issued) */
#define TIMESTAMP_MAX_DRIFT             150   /* max. allowed drift in ppm (higher values will be capped) */
#define TIMESTAMP_MAX_OFFSET_MS         10    /* max. allowed offset in ms that the host tries to compensate; if larger, a jump in time occurs. set to 0 to always make a jump */

/* data collection config */
#define NODE_HEALTH_MSG_PERIOD          300

/* memory */
#define PRE_TASK_STACK_SIZE             256                             /* in # words of 4 bytes */
#define COM_TASK_STACK_SIZE             512                             /* in # words of 4 bytes */
#define POST_TASK_STACK_SIZE            512                             /* in # words of 4 bytes */
#define STACK_WARNING_THRESHOLD         80                              /* a warning will be generated once the stack usage of a task exceeds this value (in percent) */
#define TRANSMIT_QUEUE_SIZE             20                              /* #messages */
#define RECEIVE_QUEUE_SIZE              ELWB_CONF_MAX_DATA_SLOTS        /* #messages */
#define COMMAND_QUEUE_SIZE              10

/* non-volatile config storage */
#define NVCFG_ENABLE                    1    /* use non-volatile config storage */
#define NVCFG_BLOCK_SIZE                16   /* note: must be sizeof(nv_config_t)! */

/* Gloria config */
#define GLORIA_INTERFACE_MODULATION     7    /* 7 = LoRa SF5, 10 = FSK 250kbit/s (see radio_constants.c for details) */
#if FLOCKLAB
  #define GLORIA_INTERFACE_RF_BAND      46   /* 869.01 MHz (see table in radio_constants.c for options) */
#else
  #define GLORIA_INTERFACE_RF_BAND      48   /* 869.46 MHz (see table in radio_constants.c for options) */
#endif /* FLOCKLAB */

/* eLWB config */
#define ELWB_ENABLE                     1
#define ELWB_CONF_N_TX                  2
#if GLORIA_INTERFACE_MODULATION < 8          /* LoRa modulations require longer slot lengths */
  #define ELWB_CONF_T_SCHED             (ELWB_TIMER_SECOND / 20)      /* 50ms */
  #define ELWB_CONF_T_DATA              (ELWB_TIMER_SECOND / 20)      /* 50ms */
  #define ELWB_CONF_T_CONT              (ELWB_TIMER_SECOND / 25)      /* 40ms */
#else
  #define ELWB_CONF_T_SCHED             (ELWB_TIMER_SECOND / 50)      /* 20ms */
  #define ELWB_CONF_T_DATA              (ELWB_TIMER_SECOND / 50)      /* 20ms */
  #define ELWB_CONF_T_CONT              (ELWB_TIMER_SECOND / 100)     /* 10ms */
#endif /* GLORIA_INTERFACE_MODULATION */
#define ELWB_CONF_T_GAP                 (ELWB_TIMER_SECOND / 200)     /* 5ms */
#define ELWB_CONF_SCHED_PERIOD_IDLE     15
#define ELWB_CONF_SCHED_PERIOD_MAX      120
#define ELWB_CONF_DATA_ACK              1
#define ELWB_ON_WAKEUP()                update_opmode(OP_MODE_EVT_WAKEUP)
#define ELWB_IS_HOST()                  IS_HOST
#define ELWB_CONF_T_PREPROCESS          (ELWB_TIMER_SECOND / 20)      /* 50ms */

/* baseboard */
#if BASEBOARD
  #define BASEBOARD_TREQ_WATCHDOG       3600  /* if != 0, the baseboard will be power-cycled if no time request has been received within the specified #seconds */
#else /* BASEBOARD_TREQ_WATCHDOG */
  #define BASEBOARD_TREQ_WATCHDOG       0
#endif /* BASEBOARD_TREQ_WATCHDOG */

/* misc */
#define HS_TIMER_COMPENSATE_DRIFT       0
#define HS_TIMER_INIT_FROM_RTC          0
#define LPTIMER_RESET_WDG_ON_OVF        0
#define LPTIMER_RESET_WDG_ON_EXP        1
#define LPTIMER_CHECK_EXP_TIME          1
#define UART_FIFO_BUFFER_SIZE           1     /* not used */
#define BOLT_ENABLE                     (!FLOCKLAB) /* BOLT is not available on FlockLab */

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
  //#define LOG_PRINT_FUNC                swo_print
  //#define LOG_PRINT_IMMEDIATELY         1
#endif /* SWO_ENABLE */

/* debugging */
#if !BASEBOARD
  #if FLOCKLAB
    #define ISR_ON_IND()                PIN_SET(FLOCKLAB_INT1)    // if unused, insert 2x NOP here
    #define ISR_OFF_IND()               PIN_CLR(FLOCKLAB_INT1)
    #define ELWB_RESUMED()              //PIN_SET(FLOCKLAB_INT2)
    #define ELWB_SUSPENDED()            //PIN_CLR(FLOCKLAB_INT2)
    #define POST_TASK_RESUMED()         //PIN_SET(FLOCKLAB_INT2)
    #define POST_TASK_SUSPENDED()       //PIN_CLR(FLOCKLAB_INT2)
    #define GLORIA_START_IND()          led_on(LED_SYSTEM); PIN_SET(FLOCKLAB_INT2)
    #define GLORIA_STOP_IND()           led_off(LED_SYSTEM); PIN_CLR(FLOCKLAB_INT2)
    #define RADIO_TX_START_IND()        PIN_SET(FLOCKLAB_LED3)
    #define RADIO_TX_STOP_IND()         PIN_CLR(FLOCKLAB_LED3)
    #define RADIO_RX_START_IND()        PIN_SET(FLOCKLAB_LED3)
    #define RADIO_RX_STOP_IND()         PIN_CLR(FLOCKLAB_LED3)
  #else /* FLOCKLAB */
    #define CPU_ON_IND()                //PIN_SET(COM_GPIO1)
    #define CPU_OFF_IND()               //PIN_CLR(COM_GPIO1)
    #define LPM_ON_IND()                //PIN_CLR(COM_GPIO1)
    #define LPM_OFF_IND()               //PIN_SET(COM_GPIO1)
    #define IDLE_TASK_RESUMED()         //PIN_SET(COM_GPIO2)
    #define IDLE_TASK_SUSPENDED()       //PIN_CLR(COM_GPIO2)
    #define ISR_ON_IND()                PIN_SET(COM_GPIO2)      // if unused, insert 2x NOP here
    #define ISR_OFF_IND()               PIN_CLR(COM_GPIO2)
    #define ELWB_RESUMED()              //PIN_SET(COM_GPIO1)
    #define ELWB_SUSPENDED()            //PIN_CLR(COM_GPIO1)
    #define PRE_TASK_RESUMED()          //PIN_SET(COM_GPIO1)
    #define PRE_TASK_SUSPENDED()        //PIN_CLR(COM_GPIO1)
    #define POST_TASK_RESUMED()         //PIN_SET(COM_GPIO1)
    #define POST_TASK_SUSPENDED()       //PIN_CLR(COM_GPIO1)
    #define GLORIA_START_IND()          led_on(LED_SYSTEM)
    #define GLORIA_STOP_IND()           led_off(LED_SYSTEM)
    #define RADIO_TX_START_IND()        PIN_SET(COM_GPIO1)
    #define RADIO_TX_STOP_IND()         PIN_CLR(COM_GPIO1)
    #define RADIO_RX_START_IND()        //PIN_SET(COM_GPIO1)
    #define RADIO_RX_STOP_IND()         //PIN_CLR(COM_GPIO1)
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

#if FLOCKLAB && (HOST_ID < 1 || HOST_ID > 25)
#error "HOST_ID is invalid for target FLOCKLAB"
#endif

#if BASEBOARD_TREQ_WATCHDOG > 0 && BASEBOARD_TREQ_WATCHDOG < 120
#error "BASEBOARD_TREQ_WATCHDOG must be >= 120"
#endif

#endif /* __APP_CONFIG_H */
