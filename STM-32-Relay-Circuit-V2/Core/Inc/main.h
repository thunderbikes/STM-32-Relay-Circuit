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
#include "stm32f2xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HVC__EN_Pin GPIO_PIN_0
#define HVC__EN_GPIO_Port GPIOC
#define HVC__ENC1_Pin GPIO_PIN_1
#define HVC__ENC1_GPIO_Port GPIOC
#define CHRG__EN_Pin GPIO_PIN_2
#define CHRG__EN_GPIO_Port GPIOC
#define CHRG__ENC3_Pin GPIO_PIN_3
#define CHRG__ENC3_GPIO_Port GPIOC
#define VSENSE_OUT_Pin GPIO_PIN_0
#define VSENSE_OUT_GPIO_Port GPIOA
#define BMS_OUT1_Pin GPIO_PIN_1
#define BMS_OUT1_GPIO_Port GPIOA
#define BMS_OUT2_Pin GPIO_PIN_2
#define BMS_OUT2_GPIO_Port GPIOA
#define BMS_OUT3_Pin GPIO_PIN_3
#define BMS_OUT3_GPIO_Port GPIOA
#define PRECHRG_EN_Pin GPIO_PIN_4
#define PRECHRG_EN_GPIO_Port GPIOC
#define IGNITION_SW_Pin GPIO_PIN_5
#define IGNITION_SW_GPIO_Port GPIOC
#define CHARGE_SW_Pin GPIO_PIN_0
#define CHARGE_SW_GPIO_Port GPIOB
#define DEBUG_1_Pin GPIO_PIN_1
#define DEBUG_1_GPIO_Port GPIOB
#define DEBUG_2_Pin GPIO_PIN_2
#define DEBUG_2_GPIO_Port GPIOB
#define PUMP_EN_Pin GPIO_PIN_7
#define PUMP_EN_GPIO_Port GPIOC
#define TEMP_A_Pin GPIO_PIN_8
#define TEMP_A_GPIO_Port GPIOC
#define MCU_OK_Pin GPIO_PIN_8
#define MCU_OK_GPIO_Port GPIOA
#define SAFETY_LOOP_ST_Pin GPIO_PIN_9
#define SAFETY_LOOP_ST_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWDCLK_Pin GPIO_PIN_14
#define SWDCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define HVC__AUX_Pin GPIO_PIN_4
#define HVC__AUX_GPIO_Port GPIOB
#define HVC__AUXB5_Pin GPIO_PIN_5
#define HVC__AUXB5_GPIO_Port GPIOB
#define PRECHRG_AUX_Pin GPIO_PIN_6
#define PRECHRG_AUX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
