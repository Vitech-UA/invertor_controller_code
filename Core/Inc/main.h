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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define cEN_ENERGY_MONITOR_Pin GPIO_PIN_13
#define cEN_ENERGY_MONITOR_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOA
#define xMEAS_CURRENT_Pin GPIO_PIN_6
#define xMEAS_CURRENT_GPIO_Port GPIOA
#define xMEAS_VBAT_Pin GPIO_PIN_7
#define xMEAS_VBAT_GPIO_Port GPIOA
#define cEN_CHARGER_PWR_Pin GPIO_PIN_0
#define cEN_CHARGER_PWR_GPIO_Port GPIOB
#define dEXT_U_DET_Pin GPIO_PIN_1
#define dEXT_U_DET_GPIO_Port GPIOB
#define dPOWER_STATE_Pin GPIO_PIN_10
#define dPOWER_STATE_GPIO_Port GPIOB
#define EG_TX_Pin GPIO_PIN_11
#define EG_TX_GPIO_Port GPIOB
#define cEN_CHARGER_FAN_Pin GPIO_PIN_12
#define cEN_CHARGER_FAN_GPIO_Port GPIOB
#define BTN_OK_Pin GPIO_PIN_13
#define BTN_OK_GPIO_Port GPIOB
#define BTN_BK_Pin GPIO_PIN_14
#define BTN_BK_GPIO_Port GPIOB
#define BTN_UP_Pin GPIO_PIN_15
#define BTN_UP_GPIO_Port GPIOB
#define xCHRG_NTC_Pin GPIO_PIN_8
#define xCHRG_NTC_GPIO_Port GPIOA
#define BTN_XX_Pin GPIO_PIN_9
#define BTN_XX_GPIO_Port GPIOA
#define BTN_DW_Pin GPIO_PIN_11
#define BTN_DW_GPIO_Port GPIOA
#define xI_CHRG_Pin GPIO_PIN_12
#define xI_CHRG_GPIO_Port GPIOA
#define cEN_BUZZER_Pin GPIO_PIN_4
#define cEN_BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
