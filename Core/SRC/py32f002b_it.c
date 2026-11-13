/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    py32f002b_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the License ; You may not use this file except in compliance with the
  * License.You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "py32f002b_it.h"
/* USER CODE BEGIN Includes */
#include "app_ears.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Public variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Private */

/* USER CODE END Private */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/*****************************************************************************/
/*           Cortex-M Processor Interruption and Exception Handlers          */
/*****************************************************************************/

/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NMI_Handler 0 */

  /* USER CODE END NMI_Handler 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_NMI_Handler 0 */

    /* USER CODE END W1_NMI_Handler 0 */
  }
}
/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_Handler 0 */
  /* 死循环是"看不见"的：定时器照样输出中位脉宽，看起来和待机一模一样。
   * 这里改成让舵机持续小幅快摆 —— 只要看到耳朵在不停地抖，
   * 就说明 CPU 进了 HardFault（固件崩了），而不是"没有信号"。 */
  Ear_PanicPark();
  /* USER CODE END HardFault_Handler 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_Handler 0 */

    /* USER CODE END W1_HardFault_Handler 0 */
  }
}
/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_Handler 0 */

  /* USER CODE END SVC_Handler 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_SVC_Handler 0 */

    /* USER CODE END W1_SVC_Handler 0 */
  }
}
/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_Handler 0 */

  /* USER CODE END PendSV_Handler 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_PendSV_Handler 0 */

    /* USER CODE END W1_PendSV_Handler 0 */
  }
}
/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_Handler 0 */

  /* USER CODE END SysTick_Handler 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_Handler 1 */

  /* USER CODE END SysTick_Handler 1 */
}

/******************************************************************************/
/* Puya Peripheral Interrupt Handlers */
/* Add here the Interrupt Handlers for the used peripherals. */
/* For the available peripheral interrupt handler names, */
/* please refer to the startup file. */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
