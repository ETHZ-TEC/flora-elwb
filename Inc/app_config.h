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
#define FW_VERSION                      00100       /* major [0-5], minor [0-99], patch [0-99] */
#define FLOCKLAB                        0
#define BASEBOARD                       0
#define BOLT_ENABLE                     (!FLOCKLAB)
#define SWO_ENABLE                      0
#define CLI_ENABLE                      0
#define LOW_POWER_MODE                  LP_MODE_STOP2  /* low-power mode to use between rounds during periods of inactivity */

/* RTOS */
#define PRE_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE)  /* in # words of 4 bytes */
#define COM_TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE * 2)  /* in # words of 4 bytes */
#define POST_TASK_STACK_SIZE            (configMINIMAL_STACK_SIZE)  /* in # words of 4 bytes */
#define STACK_WARNING_THRESHOLD         80     /* a warning will be generated once the stack usage of a task exceeds this value (in percent) */

/* network parameters */
#define HOST_ID                         2
#if FLOCKLAB
  #define NODE_ID                       FLOCKLAB_NODE_ID
#else /* FLOCKLAB */
  #define NODE_ID                       3
#endif /* FLOCKLAB */
#define IS_HOST                         (HOST_ID == NODE_ID)
#define BOLT_TASK_MAX_READ_CNT          10    /* max. # messages that are read from BOLT in one period */
#define TIMESTAMP_TYPICAL_DRIFT         100   /* typical drift +/- in ppm (if exceeded, a warning will be issued) */
#define TIMESTAMP_MAX_DRIFT             200   /* max. allowed drift in ppm (higher values will be capped) */
#define TIMESTAMP_USE_HS_TIMER          0     /* use hs_timer for timestamping events on the TREQ pin */
#define NODE_HEALTH_MSG_PERIOD          300

/* queue size */
#define TRANSMIT_QUEUE_SIZE             20   /* #messages */
#define RECEIVE_QUEUE_SIZE              20   /* #messages */

/* Gloria config */
#define GLORIA_INTERFACE_MODULATION     10   /* FSK 250kbit/s */
#define GLORIA_INTERFACE_RF_BAND        40   /* 868 MHz (see table in radio_constants.c for options) */

/* timer */
#define HS_TIMER_COMPENSATE_DRIFT       0

/* eLWB config */
#define ELWB_ENABLE                     1
#define ELWB_CONF_N_TX                  2
#define ELWB_CONF_T_SCHED               (ELWB_TIMER_SECOND / 50)      /* 20ms */
#define ELWB_CONF_T_DATA                (ELWB_TIMER_SECOND / 50)      /* 20ms */
#define ELWB_CONF_T_CONT                (ELWB_TIMER_SECOND / 100)     /* 10ms */
#define ELWB_ON_WAKEUP()                op_mode = op_mode_sm[op_mode][OP_MODE_EVT_WAKEUP]; resume_from_lpm()
#define ELWB_RESUMED()                  PIN_SET(COM_GPIO1)
#define ELWB_SUSPENDED()                PIN_CLR(COM_GPIO1)

/* logging */
#define LOG_ENABLE                      1
#define LOG_LEVEL                       LOG_LEVEL_VERBOSE
#define LOG_PRINT_IMMEDIATELY           0
#if SWO_ENABLE
  #define LOG_PRINT_FUNC                swo_print
  #LOG_PRINT_IMMEDIATELY                1
#endif /* SWO_ENABLE */

/* debugging */
#define CPU_ON_IND()                    //PIN_SET(COM_GPIO1)  /* pin to indicate activity (e.g. to calculate the duty cycle) */
#define CPU_OFF_IND()                   //PIN_CLR(COM_GPIO1)
#define LPM_ON_IND()                    //PIN_CLR(COM_GPIO1)
#define LPM_OFF_IND()                   //PIN_SET(COM_GPIO1)
#define IDLE_TASK_RESUMED()             //PIN_SET(COM_GPIO2)
#define IDLE_TASK_SUSPENDED()           //PIN_CLR(COM_GPIO2)
#define PRE_TASK_RESUMED()              PIN_SET(COM_GPIO1)
#define PRE_TASK_SUSPENDED()            PIN_CLR(COM_GPIO1)
#define POST_TASK_RESUMED()             PIN_SET(COM_GPIO1)
#define POST_TASK_SUSPENDED()           PIN_CLR(COM_GPIO1)
#define GLORIA_START_IND()              led_on(LED_SYSTEM); PIN_SET(COM_GPIO2)
#define GLORIA_STOP_IND()               led_off(LED_SYSTEM); PIN_CLR(COM_GPIO2)


/* --- parameter checks --- */

#if FLOCKLAB && (BOLT_ENABLE || SWO_ENABLE)
#error "invalid config"
#endif

#if BOLT_MAX_MSG_LEN < DPP_MSG_PKT_LEN
#error "BOLT_MAX_MSG_LEN is too small"
#endif

#if BOLT_ENABLE && FLOCKLAB
#error "BOLT is not supported on FlockLab"
#endif

#endif /* __APP_CONFIG_H */
