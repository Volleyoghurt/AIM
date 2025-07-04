/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint16_t ConvertVoltageToRaw(float voltage, float lsb);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCALERT_Pin GPIO_PIN_5
#define VCALERT_GPIO_Port GPIOA
#define VCALERT_EXTI_IRQn EXTI9_5_IRQn
#define LinkStatus2_Pin GPIO_PIN_6
#define LinkStatus2_GPIO_Port GPIOA
#define LinkStatus2_EXTI_IRQn EXTI9_5_IRQn
#define LinkStatus0_Pin GPIO_PIN_0
#define LinkStatus0_GPIO_Port GPIOB
#define LinkStatus0_EXTI_IRQn EXTI0_IRQn
#define LinkStatus1_Pin GPIO_PIN_1
#define LinkStatus1_GPIO_Port GPIOB
#define LinkStatus1_EXTI_IRQn EXTI1_IRQn
#define TMPALERT_Pin GPIO_PIN_8
#define TMPALERT_GPIO_Port GPIOA
#define TMPALERT_EXTI_IRQn EXTI9_5_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef enum {
    STATE_INIT,
    STATE_READ,
    STATE_TRANSMIT,
	STATE_DISPLAY,
    STATE_WAIT,
	STATE_ERROR,
} State_t;

void Error_Handler(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
