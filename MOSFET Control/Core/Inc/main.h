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
#define IMD_IO_L_Pin GPIO_PIN_13
#define IMD_IO_L_GPIO_Port GPIOC
#define HVC_POS_Pin GPIO_PIN_0
#define HVC_POS_GPIO_Port GPIOC
#define HVC_NEG_Pin GPIO_PIN_1
#define HVC_NEG_GPIO_Port GPIOC
#define P_CHARGE_Pin GPIO_PIN_2
#define P_CHARGE_GPIO_Port GPIOC
#define CHARGE_POS_Pin GPIO_PIN_3
#define CHARGE_POS_GPIO_Port GPIOC
#define HV_SENSE_Pin GPIO_PIN_0
#define HV_SENSE_GPIO_Port GPIOA
#define BMS_INPUT1_Pin GPIO_PIN_1
#define BMS_INPUT1_GPIO_Port GPIOA
#define BMS_INPUT2_Pin GPIO_PIN_2
#define BMS_INPUT2_GPIO_Port GPIOA
#define BMS_INPUT3_Pin GPIO_PIN_3
#define BMS_INPUT3_GPIO_Port GPIOA
#define CHARGE_NEG_Pin GPIO_PIN_4
#define CHARGE_NEG_GPIO_Port GPIOC
#define IGNITION_Pin GPIO_PIN_5
#define IGNITION_GPIO_Port GPIOC
#define BMS_OK_L_Pin GPIO_PIN_0
#define BMS_OK_L_GPIO_Port GPIOB
#define IMD_OK_L_Pin GPIO_PIN_1
#define IMD_OK_L_GPIO_Port GPIOB
#define CHARGE_ENABLE_Pin GPIO_PIN_6
#define CHARGE_ENABLE_GPIO_Port GPIOC
#define PUMP_ENABLE_Pin GPIO_PIN_7
#define PUMP_ENABLE_GPIO_Port GPIOC
#define CTRL_OK_Pin GPIO_PIN_8
#define CTRL_OK_GPIO_Port GPIOA
#define SHDWN_ST_Pin GPIO_PIN_9
#define SHDWN_ST_GPIO_Port GPIOA
#define IMD_IO_H_Pin GPIO_PIN_12
#define IMD_IO_H_GPIO_Port GPIOC
#define R1_AUX_Pin GPIO_PIN_4
#define R1_AUX_GPIO_Port GPIOB
#define R2_AUX_Pin GPIO_PIN_5
#define R2_AUX_GPIO_Port GPIOB
#define R3_AUX_Pin GPIO_PIN_6
#define R3_AUX_GPIO_Port GPIOB
#define R4_AUX_Pin GPIO_PIN_7
#define R4_AUX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
