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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RL1_Pin GPIO_PIN_0
#define RL1_GPIO_Port GPIOA
#define RL3_Pin GPIO_PIN_2
#define RL3_GPIO_Port GPIOA
#define RL4_Pin GPIO_PIN_3
#define RL4_GPIO_Port GPIOA
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

/* USER CODE BEGIN PV */
int charge_state = GPIO_PIN_RESET;
int main_state = GPIO_PIN_RESET;
int forward_state = GPIO_PIN_RESET;
int reverse_state = GPIO_PIN_RESET;
int relay1_aux = GPIO_PIN_RESET;
int relay2_aux = GPIO_PIN_RESET;
int relay3_aux = GPIO_PIN_RESET;
int relay6_aux = GPIO_PIN_RESET;
int operation = 0; // Added this line
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 *
 */
int park(int forward_input, int reverse_input) {
    return (forward_input == GPIO_PIN_RESET && reverse_input == GPIO_PIN_RESET) ? 1 : 0;
}

void allRelaysOpen() {
    HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RL6_GPIO_Port, RL6_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

void allDigitalRead() {
    charge_state = HAL_GPIO_ReadPin(CHARGE_INPUT_GPIO_Port, CHARGE_INPUT_Pin);
    main_state = HAL_GPIO_ReadPin(MAIN_INPUT_GPIO_Port, MAIN_INPUT_Pin);
    forward_state = HAL_GPIO_ReadPin(FORWARD_INPUT_GPIO_Port, FORWARD_INPUT_Pin);
    reverse_state = HAL_GPIO_ReadPin(REVERSE_INPUT_GPIO_Port, REVERSE_INPUT_Pin);
}

void allAuxDigitalRead(){
	relay1_aux = HAL_GPIO_ReadPin(R1_AUX_GPIO_Port, R1_AUX_Pin);
	relay2_aux = HAL_GPIO_ReadPin(R2_AUX_GPIO_Port, R2_AUX_Pin);
	relay3_aux = HAL_GPIO_ReadPin(R3_AUX_GPIO_Port, R3_AUX_Pin);
	relay6_aux = HAL_GPIO_ReadPin(R6_AUX_GPIO_Port, R6_AUX_Pin);
}

int ignition_aux(){
	allAuxDigitalRead();
	if(relay1_aux == GPIO_PIN_RESET && relay2_aux == GPIO_PIN_SET && relay3_aux == GPIO_PIN_SET && relay6_aux == GPIO_PIN_RESET) {
		return 1;
	} else {
		return 0;
	}
}

int charging_aux(){
	allAuxDigitalRead();
	if(relay1_aux == GPIO_PIN_RESET && relay2_aux == GPIO_PIN_RESET && relay3_aux == GPIO_PIN_RESET && relay6_aux == GPIO_PIN_SET) {
		return 1;
	} else {
		return 0;
	}
}

int discharging_aux(){
	allAuxDigitalRead();
	if(relay1_aux == GPIO_PIN_RESET && relay2_aux == GPIO_PIN_RESET && relay3_aux == GPIO_PIN_RESET && relay6_aux == GPIO_PIN_RESET) {
		return 1;
	} else {
		return 0;
	}
}

int operation_aux(){
	allAuxDigitalRead();
	if(relay1_aux == GPIO_PIN_SET && relay2_aux == GPIO_PIN_SET && relay3_aux == GPIO_PIN_RESET && relay6_aux == GPIO_PIN_RESET) {
		return 1;
	} else {
		return 0;
	}
}


// Added this function
void printDebug(char* message) {
    printf("%s\n", message);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        allDigitalRead();

        int park_input = park(forward_state, reverse_state);

        if (park_input)
        {
            allRelaysOpen();
        }
        else
        {

            // CHARGING
            if (charge_state == GPIO_PIN_SET && park(forward_state, reverse_state) == 1)
            {
                allRelaysOpen();
                HAL_Delay(1000);
                HAL_GPIO_WritePin(RL6_GPIO_Port, RL6_Pin, GPIO_PIN_SET);
                HAL_Delay(1000);// delay before aux
                while (charge_state == GPIO_PIN_SET)
                {
                	if (ignition_aux() == 0){
                		printDebug("Auxilary Error during Ignition");
                		exit(0);
                	} else {
                		allDigitalRead();
                		printDebug("Charging");
                	}
                }
                allRelaysOpen();
                HAL_Delay(2000);
            }
            // IGNITION
            else if (main_state == GPIO_PIN_SET && park_input == 1 && operation == 0)
            {
                allRelaysOpen();
                HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, GPIO_PIN_SET);
                HAL_Delay(1000);// delay before aux
                if (ignition_aux() == 0){
                	printDebug("Auxilary Error during Ignition");
                	exit(0);
                } else {
                    printDebug("Ignition started");
                    HAL_Delay(5000);
                    operation = 1;
                    printDebug("Ignition completed");
                    allRelaysOpen();
                    HAL_Delay(2000);
                }
            }
            // OPERATION
            else if (main_state == GPIO_PIN_SET && operation == 1 && park(forward_state, reverse_state) == 0)
            {
                allRelaysOpen();
                HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, GPIO_PIN_SET);
                HAL_Delay(1000);// delay before aux
                while (main_state == GPIO_PIN_SET && operation == 1 && park(forward_state, reverse_state) == 0)
                {
                	if (operation_aux() == 0){
                		printDebug("Auxilary Error during Operation");
                		exit(0);xx
                	} else {
                    allDigitalRead();
                	}
                }
                allRelaysOpen();
                HAL_Delay(2000);
            }
            // DISCHARGE
            else if (main_state == GPIO_PIN_RESET && park(forward_state, reverse_state) == 1 && charge_state == GPIO_PIN_RESET && operation == 1)
            {
                allRelaysOpen();
                HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, GPIO_PIN_SET);
                HAL_Delay(1000);// delay before aux
                if (charging_aux() == 0){
                	printDebug("Auxilary Error during Discharge");
                	exit(0);
                } else {
                    HAL_Delay(5000);
                    allRelaysOpen();
                    operation = 0;
                    printDebug("Discharged");
                    HAL_Delay(2000);
                }

            }
            // EDGE CASES
            else if (main_state == GPIO_PIN_SET && park(forward_state, reverse_state) == 1)
            {
                main_state = HAL_GPIO_ReadPin(MAIN_INPUT_GPIO_Port, MAIN_INPUT_Pin);
                char str[2];
                sprintf(str, "%d", main_state);
                printDebug(str);
                while (main_state == GPIO_PIN_SET && park(forward_state, reverse_state) == 1)
                {
                    allDigitalRead();
                    HAL_Delay(1000);
                    allDigitalRead();
                    printDebug("In park but Bike stays on");
                }
            }
            // Error Case
            else
            {
                HAL_Delay(1000);
                printDebug("Not a valid state: ERROR");
            }
        }
  }
  /* USER CODE END 3 */
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
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

  /*Configure GPIO pins : Main_input_Pin Forward_input_Pin BMS_Input3_Pin */
  GPIO_InitStruct.Pin = Main_input_Pin|Forward_input_Pin|BMS_Input3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Reverse_input_Pin Charge_input_Pin BMS_Input2_Pin BMS_Input1_Pin
                           R6_Aux_Pin R3_Aux_Pin R2_Aux_Pin R1_Aux_Pin */
  GPIO_InitStruct.Pin = Reverse_input_Pin|Charge_input_Pin|BMS_Input2_Pin|BMS_Input1_Pin
                          |R6_Aux_Pin|R3_Aux_Pin|R2_Aux_Pin|R1_Aux_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
       /*Configure GPIO pin : CHARGE_INPUT_Pin */
       GPIO_InitStruct.Pin = CHARGE_INPUT_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       HAL_GPIO_Init(CHARGE_INPUT_GPIO_Port, &GPIO_InitStruct);

       /*Configure GPIO pin : MAIN_INPUT_Pin */
       GPIO_InitStruct.Pin = MAIN_INPUT_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       HAL_GPIO_Init(MAIN_INPUT_GPIO_Port, &GPIO_InitStruct);

       /*Configure GPIO pin : FORWARD_INPUT_Pin */
       GPIO_InitStruct.Pin = FORWARD_INPUT_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       HAL_GPIO_Init(FORWARD_INPUT_GPIO_Port, &GPIO_InitStruct);

       /*Configure GPIO pin : REVERSE_INPUT_Pin */
       GPIO_InitStruct.Pin = REVERSE_INPUT_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       HAL_GPIO_Init(REVERSE_INPUT_GPIO_Port, &GPIO_InitStruct);


       // Your existing code here...
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

   // Add your relay control logic or any other code here...

   // Other functions...

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
