/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
//cambia da PC3 a PA2 in definitivo
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODER_SW_GPIO_IN_Pin GPIO_PIN_13
#define ENCODER_SW_GPIO_IN_GPIO_Port GPIOC
#define CH_EN_CMD_GPIO_OUT_Pin GPIO_PIN_3
#define CH_EN_CMD_GPIO_OUT_GPIO_Port GPIOC
#define CH_EN_BUTTON_GPIO_IN_Pin GPIO_PIN_0
#define CH_EN_BUTTON_GPIO_IN_GPIO_Port GPIOA
#define SDC_FUNGO_GPIO_IN_Pin GPIO_PIN_1
#define SDC_FUNGO_GPIO_IN_GPIO_Port GPIOA
#define STAT2_LED_GPIO_OUT_Pin GPIO_PIN_4
#define STAT2_LED_GPIO_OUT_GPIO_Port GPIOA
#define STAT1_LED_GPIO_OUT_Pin GPIO_PIN_5
#define STAT1_LED_GPIO_OUT_GPIO_Port GPIOA
#define WARN_LED_GPIO_OUT_Pin GPIO_PIN_6
#define WARN_LED_GPIO_OUT_GPIO_Port GPIOA
#define STAT3_LED_GPIO_OUT_Pin GPIO_PIN_7
#define STAT3_LED_GPIO_OUT_GPIO_Port GPIOA
#define AMS_DRIVER_GPIO_OUT_Pin GPIO_PIN_5
#define AMS_DRIVER_GPIO_OUT_GPIO_Port GPIOC
#define IMD_DRIVER_GPIO_OUT_Pin GPIO_PIN_0
#define IMD_DRIVER_GPIO_OUT_GPIO_Port GPIOB
#define NTC_ADC_IN_Pin GPIO_PIN_1
#define NTC_ADC_IN_GPIO_Port GPIOB
#define BUZZER_PWM_OUT_Pin GPIO_PIN_14
#define BUZZER_PWM_OUT_GPIO_Port GPIOB
#define FAN_PWM_OUT_Pin GPIO_PIN_15
#define FAN_PWM_OUT_GPIO_Port GPIOB
#define R_CAN1_RX_Pin GPIO_PIN_5
#define R_CAN1_RX_GPIO_Port GPIOB
#define R_CAN1_TX_Pin GPIO_PIN_6
#define R_CAN1_TX_GPIO_Port GPIOB
#define R_CAN2_RX_Pin GPIO_PIN_8
#define R_CAN2_RX_GPIO_Port GPIOB
#define D_CAN2_TX_Pin GPIO_PIN_9
#define D_CAN2_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LOG_UART huart2
#define init_fsm_error 0
#define fsm_start_error 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
