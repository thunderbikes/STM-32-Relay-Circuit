/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
float Read_ADC_Voltage(void);
// watch out for running out of onboard memory because of inline
static inline void check_high(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
static inline void check_low(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1_AUX_PORT GPIOB
#define R2_AUX_PORT GPIOB
#define R3_AUX_PORT GPIOB
#define R4_AUX_PORT GPIOB

#define BLINK_TEST_PORT GPIOA

#define IMD_IO_H_PORT
#define IMD_IO_L_PORT

#define HVC_POS_PORT GPIOC
#define HVC_NEG_PORT GPIOC
#define P_CHARGE_PORT GPIOC
#define HV_SENSE_PORT HV_SENSE_GPIO_Port
#define CHARGE_POS_PORT GPIOC
#define CHARGE_NEG_PORT GPIOC

#define BMS_INPUT1_PORT GPIOA
#define BMS_INPUT2_PORT GPIOA
#define BMS_INPUT3_PORT GPIOA

#define SP1_NSS_PORT
#define SP1_SCK_PORT
#define SP1_MISO_PORT
#define SP1_MOSI_PORT

#define IGNITION_PORT GPIOC
#define CHARGE_ENABLE_PORT GPIOC
#define PUMP_ENABLE_PORT GPIOC

#define BMS_OK_L_PORT GPIOB
#define IMD_OK_L_PORT GPIOB
#define SHDWN_ST_PORT GPIOA

#define CAN1_TX_PORT
#define CAN1_RX_PORT

typedef enum {
	UPDATING, SHUTDOWN, STANDBY, CHARGING, PRECHARGE, OPERATION, DISCHARGE
} AllStatus;

AllStatus Status = STANDBY;
int updating_counter = 0;
int operation = 0;
int ignition = 0;
int charging = 0;

float can_voltage =0.0;

uint32_t updating_start_tick = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
//i have not set up timers -ray

/* USER CODE BEGIN PFP */
void while_operation(void);
void set_discharge(void);
void set_precharge(void);  // Forward declaration
void set_charging(void);   // Forward declaration
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void allRelaysOpen() {
	HAL_GPIO_WritePin(HVC_POS_PORT, HVC_POS_Pin, GPIO_PIN_RESET); // AUX_1
	HAL_GPIO_WritePin(HVC_NEG_PORT, HVC_NEG_Pin, GPIO_PIN_RESET); // AUX_2
	HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_RESET); // AUX_3

	HAL_GPIO_WritePin(CHARGE_NEG_PORT, CHARGE_NEG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CHARGE_POS_PORT, CHARGE_POS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PUMP_ENABLE_PORT, PUMP_ENABLE_Pin, GPIO_PIN_RESET);

	HAL_Delay(1000);

	check_low(R1_AUX_PORT, R1_AUX_Pin);
	check_low(R2_AUX_PORT, R2_AUX_Pin);
	check_low(R3_AUX_PORT, R3_AUX_Pin);

}

void allDigitalRead() {
	HAL_GPIO_ReadPin(IGNITION_PORT, IGNITION_Pin);
	HAL_GPIO_ReadPin(CHARGE_ENABLE_PORT, CHARGE_ENABLE_Pin);

}

void allAuxDigitalRead() {
	HAL_GPIO_ReadPin(R1_AUX_PORT, R1_AUX_Pin);
	HAL_GPIO_ReadPin(R2_AUX_PORT, R2_AUX_Pin);
	HAL_GPIO_ReadPin(R3_AUX_PORT, R3_AUX_Pin);
}

int Reading_Pin(char aux_number) {
	int state = 0; // Default state
	switch (aux_number) {
	case '1':
		state = HAL_GPIO_ReadPin(R1_AUX_PORT, R1_AUX_Pin);
		break;
	case '2':
		state = HAL_GPIO_ReadPin(R2_AUX_PORT, R2_AUX_Pin);
		break;
	case '3':
		state = HAL_GPIO_ReadPin(R3_AUX_PORT, R3_AUX_Pin);
		break;
	case 'I':
		state = HAL_GPIO_ReadPin(IGNITION_PORT, IGNITION_Pin);
		break;
	case 'c':
		state = HAL_GPIO_ReadPin(CHARGE_ENABLE_PORT, CHARGE_ENABLE_Pin);
		break;
	case 'h':
		state = HAL_GPIO_ReadPin(HV_SENSE_GPIO_Port, HV_SENSE_Pin);
		break;
	default:
		state = -1; // Indicate an invalid aux_number
		break;
	}
	return state;
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (Reading_Pin('I') == 1){
		  while(1){
			  HAL_Delay(1000);
			  allRelaysOpen();
			  HAL_Delay(5000);

			  HAL_GPIO_WritePin(HVC_POS_PORT, HVC_POS_Pin, GPIO_PIN_SET); // AUX_1
			  HAL_Delay(5000);
			  HAL_GPIO_WritePin(HVC_POS_PORT, HVC_POS_Pin, GPIO_PIN_RESET); // AUX_1

			  HAL_GPIO_WritePin(HVC_NEG_PORT, HVC_NEG_Pin, GPIO_PIN_SET); // AUX_2
			  HAL_Delay(5000);
			  HAL_GPIO_WritePin(HVC_NEG_PORT, HVC_NEG_Pin, GPIO_PIN_RESET); // AUX_2

			  HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_SET); // AUX_3
			  HAL_Delay(5000);
			  HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_RESET); // AUX_3

			  HAL_Delay(10000);
			  HAL_GPIO_WritePin(CHARGE_NEG_PORT, CHARGE_NEG_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(CHARGE_POS_PORT, CHARGE_POS_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(PUMP_ENABLE_PORT, PUMP_ENABLE_Pin, GPIO_PIN_RESET);
			  HAL_Delay(1000);

		  }
    /* USER CODE BEGIN 3 */
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, HVC__EN_Pin|HVC__ENC1_Pin|CHRG__EN_Pin|CHRG__ENC3_Pin
                          |PRECHRG_EN_Pin|PUMP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_OK_GPIO_Port, MCU_OK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HVC__EN_Pin HVC__ENC1_Pin CHRG__EN_Pin CHRG__ENC3_Pin
                           PRECHRG_EN_Pin PUMP_EN_Pin */
  GPIO_InitStruct.Pin = HVC__EN_Pin|HVC__ENC1_Pin|CHRG__EN_Pin|CHRG__ENC3_Pin
                          |PRECHRG_EN_Pin|PUMP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_OUT1_Pin BMS_OUT2_Pin BMS_OUT3_Pin SAFETY_LOOP_ST_Pin
                           SWDIO_Pin SWDCLK_Pin */
  GPIO_InitStruct.Pin = BMS_OUT1_Pin|BMS_OUT2_Pin|BMS_OUT3_Pin|SAFETY_LOOP_ST_Pin
                          |SWDIO_Pin|SWDCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IGNITION_SW_Pin TEMP_A_Pin */
  GPIO_InitStruct.Pin = IGNITION_SW_Pin|TEMP_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CHARGE_SW_Pin DEBUG_1_Pin DEBUG_2_Pin SWO_Pin
                           HVC__AUX_Pin HVC__AUXB5_Pin PRECHRG_AUX_Pin */
  GPIO_InitStruct.Pin = CHARGE_SW_Pin|DEBUG_1_Pin|DEBUG_2_Pin|SWO_Pin
                          |HVC__AUX_Pin|HVC__AUXB5_Pin|PRECHRG_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_OK_Pin */
  GPIO_InitStruct.Pin = MCU_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_OK_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	(void) file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// Check if the interrupt comes from TIM2
	if(htim->Instance == TIM3){
		can_voltage = 103.6; 	// MANUAL SET VOLTAGE
		}
	if (htim->Instance == TIM2) {
		if (HAL_GPIO_ReadPin(SHDWN_ST_PORT, SHDWN_ST_Pin) == GPIO_PIN_SET) {
			Status = SHUTDOWN;
		}

		switch (Status) {
		case UPDATING:
			if (updating_start_tick == 0) { // Initialize start tick on first entry
				updating_start_tick = HAL_GetTick();
			}
				// Check if 30 seconds have elapsed
			if ((HAL_GetTick() - updating_start_tick) > 30000) {
				updating_start_tick = 0;  // Reset for next use
				Error_Handler();
			}
			break;
		case SHUTDOWN:
			if (HAL_GPIO_ReadPin(IMD_OK_L_PORT, IMD_OK_L_Pin)
					== GPIO_PIN_RESET) {
				printf("IMD NOT OK.\n");
			} else if (HAL_GPIO_ReadPin(BMS_OK_L_PORT, BMS_OK_L_Pin)
					== GPIO_PIN_RESET) {
				printf("BMS NOT OK.\n");
			} else {
				printf("Cause of shutdown is unknown.\n");
			}
			Error_Handler();
		case STANDBY:
			//i forgot which AUX pins we need to check for standby -ray
			if (HAL_GPIO_ReadPin(R1_AUX_PORT, R1_AUX_Pin) == GPIO_PIN_RESET) { //please double check all AUX pins -ray
				if (HAL_GPIO_ReadPin(R2_AUX_PORT, R2_AUX_Pin)
						== GPIO_PIN_RESET) { //also 3 if statements seems a bit more readable
					if (HAL_GPIO_ReadPin(R3_AUX_PORT, R3_AUX_Pin)
							== GPIO_PIN_RESET) {
						break;
					}
				}
			}
			Error_Handler();
		case CHARGING:
			//i dont know which AUX pins we need to check for charging -ray
			if (HAL_GPIO_ReadPin(R1_AUX_PORT, R1_AUX_Pin) == GPIO_PIN_RESET) { //please double check all AUX pins -ray
				if (HAL_GPIO_ReadPin(R2_AUX_PORT, R2_AUX_Pin)
						== GPIO_PIN_RESET) { //also 3 if statements seems a bit more readable
					if (HAL_GPIO_ReadPin(R3_AUX_PORT, R3_AUX_Pin)
							== GPIO_PIN_RESET) {
						break;
					}
				}
			}
			Error_Handler();
		case PRECHARGE:
			if (HAL_GPIO_ReadPin(R1_AUX_PORT, R1_AUX_Pin) == GPIO_PIN_RESET) { //please double check all AUX pins -ray
				if (HAL_GPIO_ReadPin(R2_AUX_PORT, R2_AUX_Pin) == GPIO_PIN_SET) { //also 3 if statements seems a bit more readable
					if (HAL_GPIO_ReadPin(R3_AUX_PORT, R3_AUX_Pin)
							== GPIO_PIN_SET) {
						break;
					}
				}
			}
			Error_Handler();
		case OPERATION:
			if (HAL_GPIO_ReadPin(R1_AUX_PORT, R1_AUX_Pin) == GPIO_PIN_SET) {
				if (HAL_GPIO_ReadPin(R2_AUX_PORT, R2_AUX_Pin) == GPIO_PIN_SET) {
					if (HAL_GPIO_ReadPin(R3_AUX_PORT, R3_AUX_Pin)
							== GPIO_PIN_RESET) {
						break;
					}
				}
			}
			Error_Handler();
		case DISCHARGE:
			if (HAL_GPIO_ReadPin(R1_AUX_PORT, R1_AUX_Pin) == GPIO_PIN_RESET) {
				if (HAL_GPIO_ReadPin(R2_AUX_PORT, R3_AUX_Pin)
						== GPIO_PIN_RESET) {
					if (HAL_GPIO_ReadPin(R3_AUX_PORT, R3_AUX_Pin)
							== GPIO_PIN_RESET) {
						break;
					}
				}
			}
			Error_Handler();
		default:
			Error_Handler();
		}
	}
}

// delays are now built into the checks -ray
static inline void check_high(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin){ //
	for (int i = 0; i < 5; ++i) {
		HAL_Delay(100);
        	if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
			return;
		}
	}
	Error_Handler();
}

static inline void check_low(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    for (int i = 0; i < 5; ++i) {
        HAL_Delay(100);
        if (!HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)) {
            return;
        }
    }
    Error_Handler(); // Trigger error handler if the pin state is not low after retries
}


//these functions use mostly copied code from old main loop -ray
void set_precharge() {
	uint32_t start_tick = HAL_GetTick(); // in milliseconds
	uint32_t timeout = 30000; // Timeout set for 30000 milliseconds or 30 seconds
	double hv_sense_voltage;
	bool voltage_reached = false;

	Status = UPDATING;
	allRelaysOpen();
	HAL_GPIO_WritePin(HVC_NEG_PORT, HVC_NEG_Pin, GPIO_PIN_SET);
	check_high(R2_AUX_PORT, R2_AUX_Pin);
	HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_SET);
	check_high(R3_AUX_PORT, R3_AUX_Pin);

	while (1) {
		hv_sense_voltage = Read_ADC_Voltage();
		if (hv_sense_voltage >= 0.9 * can_voltage) { // CANBUS VLAUE FROM ISR
			voltage_reached = true;
			Status = PRECHARGE; // Successfully precharged, update status
			while_operation();
			break; // Exit the loop on successful precharge
		}

		// Check for timeout
		if (HAL_GetTick() - start_tick >= timeout) {
			break; // Exit the loop after timeout
		}

		HAL_Delay(1000); // Delay for 1 second before checking again
	}

	if (!voltage_reached) {
		printf("Failed to reach the required voltage within 30 seconds.\n");
		Error_Handler(); // Handle the timeout error
	}
	return;
}

void while_operation() {
	if (HAL_GPIO_ReadPin(IGNITION_PORT, IGNITION_Pin) == GPIO_PIN_RESET) {
		set_discharge();
	} else {
		Status = UPDATING;
		HAL_GPIO_WritePin(HVC_POS_PORT, HVC_POS_Pin, GPIO_PIN_SET);
		check_high(R1_AUX_PORT, R1_AUX_Pin);

		HAL_Delay(1000); // Wait for 1 second to ensure stability

		HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_RESET);
		check_low(R3_AUX_PORT, R3_AUX_Pin);

		Status = OPERATION; // Set status to OPERATION
	}
	return;
}

void set_discharge() {
	Status = UPDATING;
	HAL_Delay(30000); // 30-second delay for any necessary pre-discharge preparations

	allRelaysOpen(); // Open all relays to ensure system is in a safe state before discharging

	HAL_GPIO_WritePin(CTRL_OK_GPIO_Port, CTRL_OK_Pin, GPIO_PIN_RESET);

	operation = 0; // Reset operation status
	Status = DISCHARGE; // Set status to DISCHARGE
	return;
}

void set_charging() {
	Status = UPDATING;
	allRelaysOpen();  // Open all relays to ensure the system is in a safe state

	HAL_GPIO_WritePin(CHARGE_NEG_PORT, CHARGE_NEG_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(CHARGE_POS_PORT, CHARGE_POS_Pin, GPIO_PIN_SET);

	Status = CHARGING; // Update system status to CHARGING after successful pin settings
}

float Read_ADC_Voltage(void) {
	uint32_t hv_sense_value;
	// Start the ADC
	HAL_ADC_Start(&hadc1);

	// Poll for conversion completion with a timeout of 10ms
	if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		hv_sense_value = HAL_ADC_GetValue(&hadc1);
		return (hv_sense_value / 4095.0) * 103.6;
	} else {
		// Handle the error or return a default value
		return -1.0f;  // Indicate an error in conversion
	}
}
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
