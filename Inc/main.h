/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* include flora lib (also includes app_config.h) */
#include "flora_lib.h"

/* FreeRTOS files */
#include "cmsis_os.h"

/* project files */
#include "message.h"

#ifdef __GNUC__
  /* only include this file on Linux */
  #include "gitrev.h"
#else
  #define GIT_REV     0
  #define BUILD_TIME  0
#endif /* __GNUC__ */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* note: size must be equal to NVCFG_BLOCK_SIZE! */
typedef struct
{
  uint16_t    node_id;      /* node ID */
  uint16_t    rst_cnt;      /* reset counter */
  periodic_t  bb_en;        /* periodic baseboard enable schedule */
  uint32_t    reserved;
} nv_config_t;              /* non-volatile configuration */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern nv_config_t config;
extern volatile uint16_t host_id; // for binary patching

#if BOLT_ENABLE
extern SPI_HandleTypeDef hspi1;
#endif /* BOLT_ENABLE */
#ifdef HAL_IWDG_MODULE_ENABLED
extern IWDG_HandleTypeDef hiwdg;
#endif /* HAL_IWDG_MODULE_ENABLED */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define FATAL_ERROR(str)    __disable_irq(); \
                            uart_tx_direct("FATAL ERROR: " str "\r\n", sizeof(str) + 15); \
                            led_on(LED_EVENT); \
                            delay_us(10000000); \
                            NVIC_SystemReset()

#define RTOS_STARTED()      (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
#define MS_TO_HAL_TICKS(ms) (((ms) * HAL_GetTickFreq()) / 1000)
#define MS_TO_RTOS_TICKS(ms)  ((ms) / portTICK_PERIOD_MS)       // = pdMS_TO_TICKS()


/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void      set_unix_timestamp_us(uint64_t ts_us);
uint64_t  get_unix_timestamp_us(void);

void      rtos_init(void);
uint32_t  rtos_get_cpu_dc(void);     /* get duty cycle */
void      rtos_reset_cpu_dc(void);   /* reset duty cycle */
uint32_t  rtos_check_stack_usage(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RADIO_DIO1_WAKEUP_Pin GPIO_PIN_13
#define RADIO_DIO1_WAKEUP_GPIO_Port GPIOC
#define RADIO_DIO1_WAKEUP_EXTI_IRQn EXTI15_10_IRQn
#define BOLT_IND_Pin GPIO_PIN_0
#define BOLT_IND_GPIO_Port GPIOA
#define COM_TREQ_Pin GPIO_PIN_3
#define COM_TREQ_GPIO_Port GPIOA
#define APP_IND_Pin GPIO_PIN_4
#define APP_IND_GPIO_Port GPIOA
#define BOLT_SCK_Pin GPIO_PIN_5
#define BOLT_SCK_GPIO_Port GPIOA
#define BOLT_MISO_Pin GPIO_PIN_6
#define BOLT_MISO_GPIO_Port GPIOA
#define BOLT_MOSI_Pin GPIO_PIN_7
#define BOLT_MOSI_GPIO_Port GPIOA
#define BOLT_ACK_Pin GPIO_PIN_0
#define BOLT_ACK_GPIO_Port GPIOB
#define BOLT_REQ_Pin GPIO_PIN_1
#define BOLT_REQ_GPIO_Port GPIOB
#define BOLT_MODE_Pin GPIO_PIN_2
#define BOLT_MODE_GPIO_Port GPIOB
#define RADIO_NSS_Pin GPIO_PIN_12
#define RADIO_NSS_GPIO_Port GPIOB
#define RADIO_SCK_Pin GPIO_PIN_13
#define RADIO_SCK_GPIO_Port GPIOB
#define RADIO_MISO_Pin GPIO_PIN_14
#define RADIO_MISO_GPIO_Port GPIOB
#define RADIO_MOSI_Pin GPIO_PIN_15
#define RADIO_MOSI_GPIO_Port GPIOB
#define RADIO_NRESET_Pin GPIO_PIN_8
#define RADIO_NRESET_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define RADIO_BUSY_Pin GPIO_PIN_11
#define RADIO_BUSY_GPIO_Port GPIOA
#define RADIO_ANT_SW_Pin GPIO_PIN_12
#define RADIO_ANT_SW_GPIO_Port GPIOA
#define COM_PROG2_Pin GPIO_PIN_13
#define COM_PROG2_GPIO_Port GPIOA
#define COM_PROG_Pin GPIO_PIN_14
#define COM_PROG_GPIO_Port GPIOA
#define RADIO_DIO1_Pin GPIO_PIN_15
#define RADIO_DIO1_GPIO_Port GPIOA
#define COM_GPIO2_Pin GPIO_PIN_3
#define COM_GPIO2_GPIO_Port GPIOB
#define COM_GPIO1_Pin GPIO_PIN_3
#define COM_GPIO1_GPIO_Port GPIOH
#define LED_GREEN_Pin GPIO_PIN_8
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
