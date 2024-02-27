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
#define MAIN_INPUT_Pin GPIO_PIN_6
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


#define R1_AUX_Pin GPIO_PIN_5
#define R1_AUX_GPIO_Port GPIOB
#define R2_AUX_Pin GPIO_PIN_7
#define R2_AUX_GPIO_Port GPIOB
#define R3_AUX_Pin GPIO_PIN_6
#define R3_AUX_GPIO_Port GPIOB
//#define R6_AUX_Pin GPIO_PIN_5
//#define R6_AUX_GPIO_Port GPIOB


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 *
 */





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
	if(Reading_Pin(1) == 0 && Reading_Pin(2) == 1 && Reading_Pin(3) == 1) {
		return 1;
	} else {
		return 0;
	}
}

int charging_aux(){
	allAuxDigitalRead();
	if(Reading_Pin(1) == 0 && Reading_Pin(2) == 0 && Reading_Pin(3) == 0 ) {
		return 1;
	} else {
		return 0;
	}
}

int discharging_aux(){
	allAuxDigitalRead();
	if(Reading_Pin(1) == 0 && Reading_Pin(2) == 0 && Reading_Pin(3) == 0 ) {
		return 1;
	} else {
		return 0;
	}
}

int operation_aux(){
	allAuxDigitalRead();
	if(Reading_Pin(1) == 1 && Reading_Pin(2) == 1 && Reading_Pin(3) ==0 ) {
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
int park() {
    return (Reading_Pin('f') == 0 && Reading_Pin('r') == 0) ? 1 : 0;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	volatile int operation = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();



  while (1)
   {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

	  //park() = 0 = forward
	  // park() =  = park
         if ( park() == 0 && Reading_Pin('m') == 1 && operation == 1) //operation
                              {

        	              allRelaysOpen();
                          while (park() == 0 && Reading_Pin('m') == 1 && operation == 1 ){
                                  }
         }


         if (park() == 1 && Reading_Pin('m') == 1 && operation == 0) // ignition (increase 7 seconds)
                                       {
        	 	 	 	 	 	 	 	 allRelaysOpen();
                                           HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, GPIO_PIN_SET);
                                           HAL_Delay(1000);
                                           HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, GPIO_PIN_SET);
                                           HAL_Delay(1000);// delay before aux
                                           operation = 1;
                                           while (park() == 1  && Reading_Pin('m') == 1 && operation == 0)
                                           {
                                                                             	  }
                                           }


         if (operation == 1 && park() == 1 && Reading_Pin('m') == 1) { // in park but bike stays on
        	 allRelaysOpen();
             HAL_Delay(1000);
             HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, GPIO_PIN_SET);
             HAL_Delay(1000);
             HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, GPIO_PIN_SET);
        	 while (operation == 1 && park() == 1 && Reading_Pin('m') == 1 )
        	                                            {
        	                                                                              	  }
         }



         if (operation == 1 && park() == 1 && Reading_Pin('m') == 0 ) { //discharge
        	 HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, GPIO_PIN_RESET);
        	 HAL_Delay(1000);
        	 HAL_GPIO_WritePin(RL4_GPIO_Port, RL4_Pin, GPIO_PIN_SET);
        	 HAL_Delay(1000);
        	 HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, GPIO_PIN_RESET);
        	 HAL_Delay(1000);
        	 allRelaysOpen();
        	 operation = 0;
        	 while (operation == 1 && park() == 1 && Reading_Pin('m') == 0 ){

        	 }
         }

   }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
/*void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


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
*/
/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */




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
  GPIO_InitStruct.Pin = Main_Input_Pin|Forward_Input_Pin|BMS_Input3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Reverse_input_Pin CHARGE_INPUT_Pin BMS_Input2_Pin BMS_Input1_Pin
                           R1_Aux_Pin R3_Aux_Pin R2_Aux_Pin */
  GPIO_InitStruct.Pin = Reverse_Input_Pin|CHARGE_INPUT_Pin|BMS_Input2_Pin|BMS_Input1_Pin
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
