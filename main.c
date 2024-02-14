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

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 *
 */

CAN_TxHeaderTypeDef TxHeader1, TxHeader2, TxHeader3;
uint8_t TxData1[8], TxData2[8], TxData3[8];
uint32_t TxMailbox;// Initialization code for CAN and Timer (not shown)

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) { // Replace TIMx with your timer instance
        // Prepare CAN messages
        // Message 1 -> Relay output Status
        TxHeader1.StdId = 0x321;
        TxHeader1.IDE = CAN_ID_STD;
        TxHeader1.RTR = CAN_RTR_DATA;
        TxHeader1.DLC = 6;

        TxData1[0] = HAL_GPIO_ReadPin(RL1_GPIO_Port, RL1_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData1[1] = HAL_GPIO_ReadPin(RL2_GPIO_Port, RL2_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData1[2] = HAL_GPIO_ReadPin(RL3_GPIO_Port, RL3_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData1[3] = HAL_GPIO_ReadPin(RL4_GPIO_Port, RL4_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData1[4] = HAL_GPIO_ReadPin(RL5_GPIO_Port, RL5_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData1[5] = HAL_GPIO_ReadPin(RL6_GPIO_Port, RL6_Pin) == GPIO_PIN_SET ? 1 : 0;

        // Message 2 -> Aux status + BMS inputs
        TxHeader2.StdId = 0x322;
        TxHeader2.IDE = CAN_ID_STD;
        TxHeader2.RTR = CAN_RTR_DATA;
        TxHeader2.DLC = 7;

        TxData2[0] = HAL_GPIO_ReadPin(R1_AUX_GPIO_Port, R1_AUX_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData2[1] = HAL_GPIO_ReadPin(R2_AUX_GPIO_Port, R2_AUX_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData2[2] = HAL_GPIO_ReadPin(R3_AUX_GPIO_Port, R3_AUX_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData2[3] = HAL_GPIO_ReadPin(R6_AUX_GPIO_Port, R6_AUX_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData2[4] = HAL_GPIO_ReadPin(BMS_Input1_GPIO_Port, RL1_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData2[5] = HAL_GPIO_ReadPin(BMS_Input2_GPIO_Port, RL1_Pin) == GPIO_PIN_SET ? 1 : 0;
        TxData2[6] = HAL_GPIO_ReadPin(BMS_Input3_GPIO_Port, RL1_Pin) == GPIO_PIN_SET ? 1 : 0;

       // Message 3 -> Input Switch Status + Pump Output
        TxHeader3.StdId = 0x323;
        TxHeader3.IDE = CAN_ID_STD;
        TxHeader3.RTR = CAN_RTR_DATA;
        TxHeader3.DLC = 5;

       TxData3[0] = HAL_GPIO_ReadPin(MAIN_INPUT_GPIO_Port, MAIN_INPUT_Pin) == GPIO_PIN_SET ? 1 : 0;
       TxData3[1] = HAL_GPIO_ReadPin(FORWARD_INPUT_GPIO_Port, FORWARD_INPUT_Pin) == GPIO_PIN_SET ? 1 : 0;
       TxData3[2] = HAL_GPIO_ReadPin(REVERSE_INPUT_GPIO_Port, REVERSE_INPUT_Pin) == GPIO_PIN_SET ? 1 : 0;
       TxData3[3] = HAL_GPIO_ReadPin(CHARGE_INPUT_GPIO_Port, CHARGE_INPUT_Pin) == GPIO_PIN_SET ? 1 : 0;
       TxData3[4] = HAL_GPIO_ReadPin(PUMP_OUTPUT_GPIO_Port, PUMP_OUTPUT_Pin) == GPIO_PIN_SET ? 1 : 0;


        // Queue CAN messages#
        //Error Handling // Message
        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader1, TxData1, &TxMailbox) != HAL_OK) {
          printf("Error sending message 1");
        }

        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader2, TxData2, &TxMailbox) != HAL_OK) {
          printf("Error sending message 2");
        }

        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader3, TxData3, &TxMailbox) != HAL_OK) {
          printf("Error sending message 3");
        }
    }
}




int park(int forward_input, int reverse_input) {
    return (forward_input == GPIO_PIN_RESET && reverse_input == GPIO_PIN_RESET) ? 1 : 0;
}

void allRelaysOpen() {
    HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RL5_GPIO_Port, RL4_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RL6_GPIO_Port, RL6_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

void allDigitalRead() {
    HAL_GPIO_ReadPin(CHARGE_INPUT_GPIO_Port, CHARGE_INPUT_Pin);
    HAL_GPIO_ReadPin(MAIN_INPUT_GPIO_Port, MAIN_INPUT_Pin);
    HAL_GPIO_ReadPin(FORWARD_INPUT_GPIO_Port, FORWARD_INPUT_Pin);
    HAL_GPIO_ReadPin(REVERSE_INPUT_GPIO_Port, REVERSE_INPUT_Pin);
}


void allAuxDigitalRead(){
	 HAL_GPIO_ReadPin(R1_AUX_GPIO_Port, R1_AUX_Pin);
	 HAL_GPIO_ReadPin(R2_AUX_GPIO_Port, R2_AUX_Pin);
	HAL_GPIO_ReadPin(R3_AUX_GPIO_Port, R3_AUX_Pin);
	HAL_GPIO_ReadPin(R6_AUX_GPIO_Port, R6_AUX_Pin);
}

int Reading_Pin(char aux_number){
    int state = 0; // Default state
    switch(aux_number){
        case '1':
            state = HAL_GPIO_ReadPin(R1_AUX_GPIO_Port, R1_AUX_Pin);
            break;
        case '2':
            state = HAL_GPIO_ReadPin(R2_AUX_GPIO_Port, R2_AUX_Pin);
            break;
        case '3':
            state = HAL_GPIO_ReadPin(R3_AUX_GPIO_Port, R3_AUX_Pin);
            break;
        case '6':
            state = HAL_GPIO_ReadPin(R6_AUX_GPIO_Port, R6_AUX_Pin);
            break;
        case 'm':
            state =HAL_GPIO_ReadPin(MAIN_INPUT_GPIO_Port, MAIN_INPUT_Pin);
            break;
        case 'c':
            state = HAL_GPIO_ReadPin(CHARGE_INPUT_GPIO_Port, CHARGE_INPUT_Pin);
            break;
        case 'r':
            state = HAL_GPIO_ReadPin(REVERSE_INPUT_GPIO_Port, REVERSE_INPUT_Pin);
            break;
        case 'f':
            state = HAL_GPIO_ReadPin(FORWARD_INPUT_GPIO_Port, FORWARD_INPUT_Pin);
            break;
        default:
            state = -1; // Indicate an invalid aux_number
            break;
    }
    return state; // Ensure that a value is returned here
}

int ignition_aux(){
	allAuxDigitalRead();
	if(Reading_Pin(1) == 1 && Reading_Pin(2) == 1 && Reading_Pin(3) == 1 && Reading_Pin(6) == 0) {
		return 1;
	} else {
		return 0;
	}
}

int charging_aux(){
	allAuxDigitalRead();
	if(Reading_Pin(1) == 0 && Reading_Pin(2) == 0 && Reading_Pin(3) == 0 && Reading_Pin(6) == 1) {
		return 1;
	} else {
		return 0;
	}
}

int discharging_aux(){
	allAuxDigitalRead();
	if(Reading_Pin(1) == 0 && Reading_Pin(2) == 0 && Reading_Pin(3) == 0 && Reading_Pin(6) == 0) {
		return 1;
	} else {
		return 0;
	}
}

int operation_aux(){
	allAuxDigitalRead();
	if(Reading_Pin(1) == 1 && Reading_Pin(2) == 1 && Reading_Pin(3) ==0 && Reading_Pin(6) == 0) {
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/*  while (1)
  {

        allDigitalRead();
        allAuxDigitalRead();
        HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, GPIO_PIN_SET);
        HAL_Delay(2000);
        HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, GPIO_PIN_SET);


        HAL_Delay(2000);
        if (AUX_Read(2)== 1){
        HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, GPIO_PIN_RESET);
        HAL_Delay(2000);
        HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, GPIO_PIN_RESET);

        break;
        }

        if (HAL_GPIO_ReadPin(R2_AUX_GPIO_Port,R2_AUX_Pin) == 1 ){
        	HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, GPIO_PIN_RESET);
        	break;
        }

      }
*/


  while (1)
   {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
             // CHARGING

             if (Reading_Pin('c') == 1 )
                          {
                              HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, GPIO_PIN_SET);
                              allDigitalRead();
                          }
             else                                        {
                                           HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, GPIO_PIN_RESET);
                                           allDigitalRead();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /*Configure GPIO pins : Main_input_Pin Forward_input_Pin BMS_Input3_Pin */
  GPIO_InitStruct.Pin = Main_input_Pin|Forward_input_Pin|BMS_Input3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Reverse_input_Pin CHARGE_INPUT_Pin BMS_Input2_Pin BMS_Input1_Pin
                           R1_Aux_Pin R3_Aux_Pin R2_Aux_Pin */
  GPIO_InitStruct.Pin = Reverse_input_Pin|CHARGE_INPUT_Pin|BMS_Input2_Pin|BMS_Input1_Pin
                          |R1_Aux_Pin|R3_Aux_Pin|R2_Aux_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
       /*Configure GPIO pin : CHARGE_INPUT_Pin */
       GPIO_InitStruct.Pin = CHARGE_INPUT_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       HAL_GPIO_Init(CHARGE_INPUT_GPIO_Port, &GPIO_InitStruct);

       /*Configure GPIO pin : MAIN_INPUT_Pin */
       GPIO_InitStruct.Pin = MAIN_INPUT_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       HAL_GPIO_Init(MAIN_INPUT_GPIO_Port, &GPIO_InitStruct);

       /*Configure GPIO pin : FORWARD_INPUT_Pin */
       GPIO_InitStruct.Pin = FORWARD_INPUT_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
       HAL_GPIO_Init(FORWARD_INPUT_GPIO_Port, &GPIO_InitStruct);

       /*Configure GPIO pin : REVERSE_INPUT_Pin */
       GPIO_InitStruct.Pin = REVERSE_INPUT_Pin;
       GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
       GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
