/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_it.h"
#include "stdlib.h"
#include "stm32f0xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RL1_Pin GPIO_PIN_0
#define RL1_GPIO_Port GPIOA
#define RL2_Pin GPIO_PIN_1
#define RL2_GPIO_Port GPIOA
#define RL3_Pin GPIO_PIN_2
#define RL3_GPIO_Port GPIOA
#define RL4_Pin GPIO_PIN_3
#define RL4_GPIO_Port GPIOA
#define RL5_Pin GPIO_PIN_4
#define RL5_GPIO_Port GPIOA
#define RL6_Pin GPIO_PIN_5
#define RL6_GPIO_Port GPIOA

#define CHARGE_INPUT_Pin GPIO_PIN_1
#define CHARGE_INPUT_GPIO_Port GPIOB
#define MAIN_INPUT_Pin GPIO_PIN_8
#define MAIN_INPUT_GPIO_Port GPIOA
#define FORWARD_INPUT_Pin GPIO_PIN_7
#define FORWARD_INPUT_GPIO_Port GPIOA
#define REVERSE_INPUT_Pin GPIO_PIN_0
#define REVERSE_INPUT_GPIO_Port GPIOB

#define BMS_Input1_Pin GPIO_PIN_4
#define BMS_Input1_GPIO_Port GPIOB
#define BMS_Input2_Pin GPIO_PIN_3
#define BMS_Input2_GPIO_Port GPIOB
#define BMS_Input3_Pin GPIO_PIN_15
#define BMS_Input3_GPIO_Port GPIOA
#define PUMP_OUTPUT_Pin GPIO_PIN_8
#define PUMP_OUTPUT_GPIO_Port GPIOA

#define R1_AUX_Pin GPIO_PIN_8
#define R1_AUX_GPIO_Port GPIOB
#define R2_AUX_Pin GPIO_PIN_7
#define R2_AUX_GPIO_Port GPIOB
#define R3_AUX_Pin GPIO_PIN_6
#define R3_AUX_GPIO_Port GPIOB
#define R6_AUX_Pin GPIO_PIN_5
#define R6_AUX_GPIO_Port GPIOB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
uint8_t receivedData[8];
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void MX_CAN_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config(); // Make sure this is implemented correctly
    MX_GPIO_Init();
    MX_CAN_Init();

    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        // Handle Error
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        // Handle Error
    }

    while (1)
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // Check if the ID is 0x6B0
            if (RxHeader.StdId == 0x6B0)
            {
                // Copy the data into the receivedData array
                for (int i = 0; i < 8; i++)
                {
                    receivedData[i] = RxData[i];
                }
                // At this point, receivedData contains the bytes from the last received message with ID 0x6B0
                // You can process this data as needed
            }
        }
        // Other tasks...
    }
}

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // Configure the CAN filter
      CAN_FilterTypeDef canFilter;

      canFilter.FilterBank = 0;
      canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
      canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
      canFilter.FilterIdHigh = 0x0000; // Set your ID
      canFilter.FilterIdLow = 0x0000;
      canFilter.FilterMaskIdHigh = 0xFFFF <<5 ;//all ids
      canFilter.FilterMaskIdLow = 0x0000;
      canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
      canFilter.FilterActivation = ENABLE;
      canFilter.SlaveStartFilterBank = 14;

      if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK)
      {
          // Filter configuration Error
          Error_Handler();
      }

      // Start the CAN peripheral
      if (HAL_CAN_Start(&hcan) != HAL_OK)
      {
          // Start Error
          Error_Handler();
      }

      // Activate CAN RX notification
      if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
      {
          // Notification Error
          Error_Handler();
      }

  /* USER CODE END CAN_Init 2 */

}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY1_Pin|RELAY2_Pin|RELAY3_Pin|RELAY4_Pin
                          |RELAY5_Pin|RELAY6_Pin|Pump_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY1_Pin RELAY2_Pin RELAY3_Pin RELAY4_Pin
                           RELAY5_Pin RELAY6_Pin Pump_Output_Pin */
  GPIO_InitStruct.Pin = RELAY1_Pin|RELAY2_Pin|RELAY3_Pin|RELAY4_Pin
                          |RELAY5_Pin|RELAY6_Pin|Pump_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Main_Input_Pin Charge_Input_Pin BMS_Input3_Pin */
  GPIO_InitStruct.Pin = Main_Input_Pin|Charge_Input_Pin|BMS_Input3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_Input2_Pin BMS_Input1_Pin R1_Aux_Pin R3_Aux_Pin
                           R2_Aux_Pin */
  GPIO_InitStruct.Pin = BMS_Input2_Pin|BMS_Input1_Pin|R1_Aux_Pin|R3_Aux_Pin
                          |R2_Aux_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
