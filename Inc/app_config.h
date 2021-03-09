/**
  ******************************************************************************
  * Flora event-based Low-power Wireless Bus - config
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
#define FW_VERSION_MINOR                2           /* 0..99 */
#define FW_VERSION_PATCH                4           /* 0..99 */

#define FLOCKLAB                        1           /* set to 1 to run on FlockLab */
#define BASEBOARD                       0           /* set to 1 if the comboard will be installed on a baseboard */
#define FLOCKLAB_SWD                    0           /* set to 1 to reserve SWDIO / SWDCLK pins for debugging (GPIOs not available for tracing) */
#define SWO_ENABLE                      0           /* set to 1 to enable data tracing or serial printing via SWO pin */

/* network parameters */
#if BASEBOARD
  #define HOST_ID                       103         /* note: host ID is only used to determine whether a node is a host node (irrelevant for source nodes) */
#else
  #define HOST_ID                       2           /* note: host ID is only used to determine whether a node is a host node (irrelevant for source nodes); config will be overwritten by binary patching! */
#endif /* BASEBOARD */
#if !FLOCKLAB
  #define NODE_ID                       HOST_ID
#endif /* FLOCKLAB */
#define IS_HOST                         (NODE_ID == host_id)
#define WRITE_NODE_ID                   0           /* 1 = force node ID overwrite, 0 = use ID stored in NV config if available */

/* energy (low-power mode) */
#if SWO_ENABLE
  #define LOW_POWER_MODE                LP_MODE_SLEEP  /* low-power mode to use between rounds during periods of inactivity */
  #define TIMESTAMP_USE_HS_TIMER        1              /* use hs_timer for timestamping events on the TREQ pin for better accuracy */
#else /* SWO_ENABLE */
  #define LOW_POWER_MODE                LP_MODE_STOP2  /* low-power mode to use between rounds during periods of inactivity */
  #define TIMESTAMP_USE_HS_TIMER        0              /* don't use hs_timer for timestamping events on the TREQ pin  to safe energy */
#endif /* SWO_ENABLE */

/* time sync and drift compensation */
#define TIMESTAMP_TYPICAL_DRIFT         100   /* typical drift +/- in ppm (if exceeded, a warning will be issued) */
#define TIMESTAMP_MAX_DRIFT             150   /* max. allowed drift in ppm (higher values will be capped) */
#define TIMESTAMP_MAX_OFFSET_MS         10    /* max. allowed offset in ms that the host tries to compensate; if larger, a jump in time occurs. set to 0 to always make a jump */

/* data collection config */
#define NODE_HEALTH_MSG_PERIOD          15   /* in seconds */

/* memory */
#define PRE_TASK_STACK_SIZE             256                             /* in # words of 4 bytes */
#define COM_TASK_STACK_SIZE             400                             /* in # words of 4 bytes */
#define POST_TASK_STACK_SIZE            300                             /* in # words of 4 bytes */
#define STACK_WARNING_THRESHOLD         80                              /* a warning will be generated once the stack usage of a task exceeds this value (in percent) */
#define TRANSMIT_QUEUE_SIZE             20                              /* #messages */
#define RECEIVE_QUEUE_SIZE              ELWB_CONF_MAX_DATA_SLOTS        /* #messages */
#define COMMAND_QUEUE_SIZE              10

/* non-volatile config storage */
#define NVCFG_ENABLE                    1    /* use non-volatile config storage */
#define NVCFG_BLOCK_SIZE                16   /* note: must be sizeof(nv_config_t)! */

/* Gloria config */
#define GLORIA_INTERFACE_POWER          1    /* transmit power in dBm (max. value is 14 for most RF bands); keep non-zero init for binary patching!; config will be overwritten by binary patching! */
#if FLOCKLAB
  #define GLORIA_INTERFACE_MODULATION   7    /* 7 = LoRa SF5, 10 = FSK 250kbit/s (see radio_constants.c for details); config will be overwritten by binary patching! */
  #define GLORIA_INTERFACE_RF_BAND      46   /* 869.01 MHz (see table in radio_constants.c for options); config will be overwritten by binary patching! */
#elif BASEBOARD
  /* configuration for the deployment */
  #define GLORIA_INTERFACE_MODULATION   10   /* 7 = LoRa SF5, 10 = FSK 250kbit/s (see radio_constants.c for details); config will be overwritten by binary patching! */
  #define GLORIA_INTERFACE_RF_BAND      43   /* 869.46 MHz (see table in radio_constants.c for options); config will be overwritten by binary patching! */
#else
  #define GLORIA_INTERFACE_MODULATION   10   /* 7 = LoRa SF5, 10 = FSK 250kbit/s (see radio_constants.c for details); config will be overwritten by binary patching! */
  #define GLORIA_INTERFACE_RF_BAND      48   /* 869.46 MHz (see table in radio_constants.c for options); config will be overwritten by binary patching! */
#endif /* FLOCKLAB */

/* eLWB config */
#define ELWB_ENABLE                     1
#define ELWB_CONF_NETWORK_ID            0x3333
#define ELWB_CONF_N_TX                  2    /* number of transmissions */
#define ELWB_NUM_HOPS                   6    /* network diameter in number of hops */
#define ELWB_CONF_T_SCHED               GLORIA_INTERFACE_FLOOD_DURATION(ELWB_CONF_N_TX, ELWB_NUM_HOPS, DPP_MSG_PKT_LEN / 2)   /* note: use estimated max. packet length in bytes to calculate slot length */
#define ELWB_CONF_T_DATA                GLORIA_INTERFACE_FLOOD_DURATION(ELWB_CONF_N_TX, ELWB_NUM_HOPS, DPP_MSG_PKT_LEN / 2)   /* note: use estimated max. packet length in bytes to calculate slot length */
#define ELWB_CONF_T_CONT                GLORIA_INTERFACE_FLOOD_DURATION(ELWB_CONF_N_TX, ELWB_NUM_HOPS, 2)
#define ELWB_CONF_T_GAP                 ELWB_MS_TO_TICKS(5)
#define ELWB_CONF_SCHED_PERIOD_IDLE     15
#define ELWB_CONF_DATA_ACK              1
#define ELWB_CONF_MAX_NODES             30
#define ELWB_CONF_MAX_DATA_SLOTS        ELWB_CONF_MAX_NODES
#define ELWB_ON_WAKEUP()                update_opmode(OP_MODE_EVT_WAKEUP)
#define ELWB_CONF_T_PREPROCESS          (ELWB_TIMER_SECOND / 20)      /* 50ms */
#define ELWB_CONF_SCHED_NODE_LIST       1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32  /* nodes to pre-register in the scheduler, also include HOST_ID here! */

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
#define BOLT_ENABLE                     (!FLOCKLAB) /* BOLT is not available on FlockLab */
#define CLI_ENABLE                      0           /* command line interface */

/* logging */
#define LOG_ENABLE                      1
#define LOG_LEVEL                       LOG_LEVEL_VERBOSE
#define LOG_USE_DMA                     1
#define LOG_BUFFER_SIZE                 4096
#if LOG_USE_DMA
  #define UART_FIFO_BUFFER_SIZE         LOG_BUFFER_SIZE
#endif /* LOG_USE_DMA */
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
  #define LOG_PRINT_IMMEDIATELY         1       /* enable immediate printing to get accurate timestamps on FlockLab */
#endif /* FLOCKLAB */
#if SWO_ENABLE
  //#define LOG_PRINT_FUNC                swo_print
  //#define LOG_PRINT_IMMEDIATELY         1
#endif /* SWO_ENABLE */

/* debugging */
#if !BASEBOARD
  #if FLOCKLAB
    #define ISR_ON_IND()                bool nested = PIN_STATE(FLOCKLAB_INT1); (void)nested; PIN_SET(FLOCKLAB_INT1)    /* if unused, insert 2x NOP here */
    #define ISR_OFF_IND()               if (!nested) PIN_CLR(FLOCKLAB_INT1)
    #define CPU_ON_IND()                //PIN_SET(FLOCKLAB_INT2)
    #define CPU_OFF_IND()               //PIN_CLR(FLOCKLAB_INT2)
    #define ELWB_RESUMED()              //PIN_SET(FLOCKLAB_INT2)
    #define ELWB_SUSPENDED()            //PIN_CLR(FLOCKLAB_INT2)
    #define POST_TASK_RESUMED()         //PIN_SET(FLOCKLAB_INT2)
    #define POST_TASK_SUSPENDED()       //PIN_CLR(FLOCKLAB_INT2)
    #define GLORIA_START_IND()          led_on(LED_SYSTEM); PIN_SET(FLOCKLAB_INT2)
    #define GLORIA_STOP_IND()           led_off(LED_SYSTEM); PIN_CLR(FLOCKLAB_INT2)
    #define RADIO_TX_START_IND()        PIN_SET(FLOCKLAB_LED2)
    #define RADIO_TX_STOP_IND()         PIN_CLR(FLOCKLAB_LED2)
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

// #if FLOCKLAB && (HOST_ID < 1 || HOST_ID > 25)
// #error "HOST_ID is invalid for target FLOCKLAB"
// #endif

#if BASEBOARD_TREQ_WATCHDOG > 0 && BASEBOARD_TREQ_WATCHDOG < 120
#error "BASEBOARD_TREQ_WATCHDOG must be >= 120"
#endif

#endif /* __APP_CONFIG_H */
