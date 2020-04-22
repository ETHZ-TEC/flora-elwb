/*
 * lpm.c
 * Low-power mode management and application state machine
 *
 *  Created on: Apr 21, 2020
 *      Author: keepcoding
 */

#include "main.h"


/* --- defines / macros --- */

#ifndef LPM_ON_IND
#define LPM_ON_IND()
#define LPM_OFF_IND()
#endif /* LPM_ON_IND */


/* --- globals --- */

extern LPTIM_HandleTypeDef hlptim1;
extern TIM_HandleTypeDef   htim2;
extern TIM_HandleTypeDef   htim15;
extern TIM_HandleTypeDef   htim16;
extern SPI_HandleTypeDef   hspi1;
extern SPI_HandleTypeDef   hspi2;
extern UART_HandleTypeDef  huart1;

extern void SystemClock_Config(void);        /* defined in main.c */


/* --- private variables --- */

/* operating mode and state machine */
static op_mode_t op_mode = OP_MODE_RESET;
static const op_mode_t op_mode_state_machine[NUM_OP_MODES][NUM_OP_MODE_EVENTS] =
{
  /* event:             OP_MODE_EVT_INIT, OP_MODE_EVT_WAKEUP, OP_MODE_EVT_DONE, OP_MODE_EVT_STOPPED, OP_MODE_EVT_RESTORED */
  /* state RESET: */  { OP_MODE_ACTIVE,   OP_MODE_RESET,      OP_MODE_RESET,    OP_MODE_RESET,       OP_MODE_RESET   },
  /* state ACTIVE: */ { OP_MODE_RESET,    OP_MODE_ACTIVE,     OP_MODE_IDLE,     OP_MODE_RESET,       OP_MODE_RESET   },
  /* state IDLE: */   { OP_MODE_RESET,    OP_MODE_ACTIVE,     OP_MODE_IDLE,     OP_MODE_LPM,         OP_MODE_RESET   },
  /* state LPM: */    { OP_MODE_RESET,    OP_MODE_WOKEN,      OP_MODE_RESET,    OP_MODE_RESET,       OP_MODE_RESET   },
  /* state WOKEN: */  { OP_MODE_RESET,    OP_MODE_WOKEN,      OP_MODE_RESET,    OP_MODE_RESET,       OP_MODE_ACTIVE  },
  /* NOTE: for all invalid transitions, go back to RESET state */
};
static lp_mode_t lp_mode = LOW_POWER_MODE;    /* selected low-power mode */


/* --- functions --- */

op_mode_t get_opmode(void)
{
  return op_mode;
}

void update_opmode(op_mode_event_t evt)
{
  op_mode = op_mode_state_machine[op_mode][evt];

  /* wakeup event? -> automatically resume from lpm */
  if (evt == OP_MODE_EVT_WAKEUP) {
    lpm_resume();
  }
}

/* prepare the MCU for low power mode */
void lpm_prepare(void)
{
  /* only enter a low-power mode if the application is in idle state */
  if (op_mode == OP_MODE_IDLE)
  {
    if (lp_mode == LP_MODE_SLEEP) {
      /* do not update op_mode since we are already in IDLE and we are not entering a real LPM state */
      HAL_SuspendTick();
    }
    else if ( (lp_mode == LP_MODE_STOP2)   ||
              (lp_mode == LP_MODE_STANDBY) ||
              (lp_mode == LP_MODE_SHUTDOWN)     )
    {
      /* make sure the radio is in sleep mode */
      radio_sleep(false);

      /* notes on stop mode:
      * - SRAM1, SRAM2 and all registers content are retained
      * - all clocks in the VCORE domain are stopped, the PLL, the MSI, the HSI16 and the HSE are disabled
      * - LSE and LPTIM keep running
      * - all I/O pins keep the state
      */
      __set_BASEPRI( (TICK_INT_PRIORITY + 1) << (8 - __NVIC_PRIO_BITS) );   /* or:  __disable_irq() */
      __DSB();
      __ISB();

      __HAL_RCC_PWR_CLK_ENABLE();

      CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);  /* suspend FreeRTOS SysTick */
      HAL_SuspendTick();                                  /* suspend HAL tick */

      /* disable all unused peripherals */
      __HAL_TIM_DISABLE(&htim2);
      __HAL_TIM_DISABLE(&htim15);
      __HAL_TIM_DISABLE(&htim16);
      __HAL_UART_DISABLE(&huart1);
      __HAL_SPI_DISABLE(&hspi1);
      __HAL_SPI_DISABLE(&hspi2);

      /* configure HSI as clock source (16MHz) */
      RCC_OscInitTypeDef RCC_OscInitStruct = {0};
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
      RCC_OscInitStruct.OscillatorType        = RCC_OSCILLATORTYPE_HSI;
      RCC_OscInitStruct.HSIState              = RCC_HSI_ON;
      RCC_OscInitStruct.HSICalibrationValue   = RCC_HSICALIBRATION_DEFAULT;
      RCC_OscInitStruct.PLL.PLLState          = RCC_PLL_NONE;
      if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
      /* select HSI as system clock */
      RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
      RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_HSI;
      if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
      HAL_RCCEx_WakeUpStopCLKConfig(RCC_STOP_WAKEUPCLOCK_HSI);
      /* disable MSI */
      RCC_OscInitStruct.OscillatorType    = RCC_OSCILLATORTYPE_MSI;
      RCC_OscInitStruct.MSIState          = RCC_MSI_OFF;
      RCC_OscInitStruct.PLL.PLLState      = RCC_PLL_NONE;
      if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

      /* configure unused GPIOs for minimal current drain (make sure there are no floating inputs) */
      // NOTE1: Not necessary at the moment since pins are either not used or by default already configured as input
      // NOTE2: For some constellation the analog mode causes a higher power consumpton than leaving it configured as input
      // NOTE3: restore of RADIO_DIO1_Pin breaks wakeup from LPM, reason unknown (PA15 is connected to PC13 in hardware on the COM board)
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = RADIO_DIO1_Pin;
      // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = RADIO_BUSY_Pin;
      // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = BOLT_SCK_Pin|BOLT_MOSI_Pin|BOLT_MISO_Pin;
      // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = UART_TX_Pin|UART_RX_Pin;
      // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      //// disabling Radio SPI pins causes floating pin in low-power mode (reason unknown)
      // GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      // GPIO_InitStruct.Pin = RADIO_SCK_Pin|RADIO_MISO_Pin|RADIO_MOSI_Pin;   // SPI for radio: RADIO_SCK_Pin|RADIO_MISO_Pin|RADIO_MOSI_Pin
      // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
      //// RADIO_DIO1 (PC13) should not be disabled in low power mode -> it would no longer be possible to wake up from STOP2 with a radio interrupt
      // // GPIO_InitStruct.Pin = RADIO_DIO1_WAKEUP_Pin;        // HSE pins: GPIO_PIN_14|GPIO_PIN_15
      // // HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

      /* turn off LEDs */
      led_off(LED_EVENT);
      led_off(LED_SYSTEM);
      PIN_CLR(COM_GPIO1);     /* has external pulldown */

  #if BOLT_ENABLE
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      /* configure BOLT TREQ in EXTI mode */
      GPIO_InitStruct.Pin = COM_TREQ_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      HAL_GPIO_Init(COM_TREQ_GPIO_Port, &GPIO_InitStruct);
      HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  #endif /* BOLT_ENABLE */

      /* disable and clear unused interrupts */
      HAL_NVIC_DisableIRQ(USART1_IRQn);
      HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
      HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
      HAL_NVIC_DisableIRQ(SPI1_IRQn);
      HAL_NVIC_DisableIRQ(SPI2_IRQn);
      HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
      HAL_NVIC_DisableIRQ(TIM2_IRQn);

      /* configure RF_DIO1 on PC13 interrupt for wakeup from LPM */
      __HAL_GPIO_EXTI_CLEAR_IT(RADIO_DIO1_WAKEUP_Pin); // important for low-power consumption in STOP2 mode -> see README
      if(lp_mode == LP_MODE_STOP2) {
        HAL_NVIC_SetPriority(RADIO_DIO1_WAKEUP_EXTI_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(RADIO_DIO1_WAKEUP_EXTI_IRQn);
      }

      /* disable LPTIM1 (necessary to enter STANDBY or SHUTDOWN) (still did not work so far) */
      if (lp_mode == LP_MODE_STANDBY || lp_mode == LP_MODE_SHUTDOWN) {
        __HAL_LPTIM_DISABLE(&hlptim1);
      }

      if (lp_mode == LP_MODE_STOP2) {
        /* Clear flags of all WAKEUP lines (necessary for STOP2 as well?) */
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

        MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STOP2);    /* set Stop mode 2 */
      }
      else if (lp_mode == LP_MODE_STANDBY) {
        /* Clear flags of all WAKEUP lines (necessary to successfully enter STANDBY mode after sending with radio) */
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

        MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_STANDBY);  /* set Standby mode */
      }
      else if (lp_mode == LP_MODE_SHUTDOWN) {
        /* Clear flags of all WAKEUP lines (necessary to successfully enter SHUTDOWN mode after sending with radio) */
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

        MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, PWR_CR1_LPMS_SHUTDOWN); /* set Shutdown mode */
      }
      SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));        /* set SLEEPDEEP bit */

      LPM_ON_IND();

      /* clock gating (-> shouldn't be necessary if clock source is disabled!) */
      /*__HAL_RCC_GPIOA_CLK_DISABLE();
      __HAL_RCC_GPIOB_CLK_DISABLE();
      __HAL_RCC_GPIOC_CLK_DISABLE();
      __HAL_RCC_GPIOD_CLK_DISABLE();
      __HAL_RCC_GPIOE_CLK_DISABLE();
      __HAL_RCC_GPIOH_CLK_DISABLE();
      __HAL_RCC_DMA1_CLK_DISABLE();
      __HAL_RCC_FLASH_CLK_DISABLE();
      __HAL_RCC_SYSCFG_CLK_DISABLE();
      __HAL_RCC_PWR_CLK_DISABLE();*/

      update_opmode(OP_MODE_EVT_STOPPED);
      __set_BASEPRI(0);     /* re-enable interrupts */
    }
  }
}


void lpm_resume(void)
{
  if (op_mode == OP_MODE_IDLE) {
    /* MCU was in sleep mode, only tick needs to be restored */
    HAL_ResumeTick();
    /* do not update op_mode since we are already in IDLE */
  }
  else if (op_mode == OP_MODE_WOKEN) {
    /* MCU was in STOP2, STANDBY, or SHUTDOWN mode, different components need to be restored */

    /* make sure the following code runs atomically */
    __set_BASEPRI( (TICK_INT_PRIORITY + 1) << (8 - __NVIC_PRIO_BITS) );   /* mask interrupts */
    __DSB();
    __ISB();

    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

    /* make sure all required clocks are enabled */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_FLASH_CLK_ENABLE();

    SystemClock_Config();                               /* restore clock config (and resume HAL tick) */

    /* restore GPIO config */
    // NOTE1: Not necessary at the moment since pins are either not used or by default already configured as input
    // NOTE2: For some constellation the analog mode causes a higher power consumpton than leaving it configured as input
    // NOTE3: restore of RADIO_DIO1_Pin breaks wakeup from LPM, reason unknown (PA15 is connected to PC13 in hardware on the COM board)
    // GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO_InitStruct.Pin  = RADIO_BUSY_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // GPIO_InitStruct.Pin = BOLT_SCK_Pin|BOLT_MOSI_Pin|BOLT_MISO_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // GPIO_InitStruct.Pin = UART_TX_Pin|UART_RX_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //// disabling Radio SPI pins causes floating pin in low-power mode (reason unknown)
    // GPIO_InitStruct.Pin = RADIO_SCK_Pin|RADIO_MISO_Pin|RADIO_MOSI_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // GPIO_InitStruct.Pin = RADIO_DIO1_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    // HAL_GPIO_Init(RADIO_DIO1A15_GPIO_Port, &GPIO_InitStruct);

    /* restore peripherals */
    __HAL_TIM_ENABLE(&htim2);
    __HAL_TIM_ENABLE(&htim15);
    __HAL_TIM_ENABLE(&htim16);
    __HAL_UART_ENABLE(&huart1);
    __HAL_SPI_ENABLE(&hspi1);
    __HAL_SPI_ENABLE(&hspi2);

  #if BOLT_ENABLE
    /* disable BOLT TREQ EXTI */
    HAL_NVIC_DisableIRQ(EXTI3_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(COM_TREQ_Pin);
  #endif /* BOLT_ENABLE */

    /* re-enable interrupts */
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    /* disable RF_DIO1 on PC13 interrupt (only neede for wakeup from LPM) */
    HAL_NVIC_DisableIRQ(RADIO_DIO1_WAKEUP_EXTI_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(RADIO_DIO1_WAKEUP_Pin);

    /* resume FreeRTOS SysTick and correct the tick count */
    SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk);

    update_opmode(OP_MODE_EVT_RESTORED);
    LPM_OFF_IND();
    __set_BASEPRI(0);   /* enable interrupts */
    HAL_ResumeTick();

    /* wake the radio */
    radio_wakeup();
  }
}
