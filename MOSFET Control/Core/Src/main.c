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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> // Include for boolean type

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
float Read_ADC_Voltage(void);
static inline void check_and_handle_pin_state(GPIO_TypeDef *GPIOx,
		uint16_t GPIO_Pin);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PIN_SET_ERROR "Pin set error detected"

#define R1_AUX_PORT GPIOB
#define R2_AUX_PORT GPIOB
#define R3_AUX_PORT GPIOB
#define R4_AUX_PORT GPIOB

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

uint32_t updating_start_tick = 0;

//all possible status
// #define UPDATING = 0;
// #define SHUTDOWN = 1;
// #define STANDBY = 2;
// #define CHARGING = 3;
// #define PRECHARGE = 4;
// #define OPERATION = 5;
// #define DISCHARGE = 6;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void set_precharge(void);
void while_operation(void);
void set_discharge(void);
void set_charging(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void allRelaysOpen() {
	HAL_GPIO_WritePin(HVC_POS_PORT, HVC_POS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVC_NEG_PORT, HVC_NEG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVC_POS_PORT, HVC_POS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CHARGE_NEG_PORT, CHARGE_NEG_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CHARGE_POS_PORT, CHARGE_POS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PUMP_ENABLE_PORT, PUMP_ENABLE_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
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
int main(void) {

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
	MX_TIM2_Init();
	MX_CAN2_Init();
	MX_SPI1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (Reading_Pin('I') == 1 && operation == 0) {
			set_precharge();
		}
		//Charging
		if (Reading_Pin('c') == 1 && Reading_Pin('I') == 0 && operation == 0) {
			//make sure no accidental charging during operation ^^^-ray
			set_charging();
		}
		/* USER CODE BEGIN 3 */

		/* USER CODE END 3 */
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void) {

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 16;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	/* USER CODE END CAN2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
//Timer for 500ms for interrupts
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 959;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 49999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			HVC_POS_Pin | HVC_NEG_Pin | P_CHARGE_Pin | CHARGE_POS_Pin
					| CHARGE_NEG_Pin | PUMP_ENABLE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CTRL_OK_GPIO_Port, CTRL_OK_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : IMD_IO_L_Pin IGNITION_Pin CHARGE_ENABLE_Pin IMD_IO_H_Pin */
	GPIO_InitStruct.Pin = IMD_IO_L_Pin | IGNITION_Pin | CHARGE_ENABLE_Pin
			| IMD_IO_H_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : HVC_POS_Pin HVC_NEG_Pin P_CHARGE_Pin CHARGE_POS_Pin
	 CHARGE_NEG_Pin PUMP_ENABLE_Pin */
	GPIO_InitStruct.Pin = HVC_POS_Pin | HVC_NEG_Pin | P_CHARGE_Pin
			| CHARGE_POS_Pin | CHARGE_NEG_Pin | PUMP_ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : BMS_INPUT1_Pin BMS_INPUT2_Pin BMS_INPUT3_Pin SHDWN_ST_Pin */
	GPIO_InitStruct.Pin = BMS_INPUT1_Pin | BMS_INPUT2_Pin | BMS_INPUT3_Pin
			| SHDWN_ST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : BMS_OK_L_Pin IMD_OK_L_Pin R1_AUX_Pin R2_AUX_Pin
	 R3_AUX_Pin R4_AUX_Pin */
	GPIO_InitStruct.Pin = BMS_OK_L_Pin | IMD_OK_L_Pin | R1_AUX_Pin | R2_AUX_Pin
			| R3_AUX_Pin | R4_AUX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : CTRL_OK_Pin */
	GPIO_InitStruct.Pin = CTRL_OK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CTRL_OK_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : HV_SENSEA10_Pin */
	GPIO_InitStruct.Pin = HV_SENSEA10_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(HV_SENSEA10_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	(void) file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

//to my knowledge, this interrupt only checks 3 AUX pins not pump, charge, etc..
//I dont know how to set up interupt as on a timer -ray)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
// Check if the interrupt comes from TIM2
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
static inline void check_and_handle_pin_state(GPIO_TypeDef *GPIOx,
		uint16_t GPIO_Pin) {
	if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != GPIO_PIN_SET) {
		//print("%s at %s: %d\n", PIN_SET_ERROR, __FILE__, __LINE__); // Optional: Log the error with file and line info
		Error_Handler();
		return;
	}
}

//these funcitons use mostly copied code from old main loop -ray
void set_precharge() {
	uint32_t start_tick = HAL_GetTick(); // in milliseconds
	uint32_t timeout = 30000; // Timeout set for 30000 milliseconds or 30 seconds
	float hv_sense_voltage;
	bool voltage_reached = false;

	Status = UPDATING;
	allRelaysOpen();
	HAL_GPIO_WritePin(HVC_NEG_PORT, HVC_NEG_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_SET);

	while (1) {
		hv_sense_voltage = Read_ADC_Voltage();
		if (hv_sense_voltage >= 0.9 * 103.6) { // Assume 103.6 is the BMS battery voltage setting
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
		check_and_handle_pin_state(HVC_POS_PORT, HVC_POS_Pin);

		HAL_Delay(1000); // Wait for 1 second to ensure stability

		HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_RESET);
		check_and_handle_pin_state(P_CHARGE_PORT, P_CHARGE_Pin);

		Status = OPERATION; // Set status to OPERATION
	}
	return;
}

void set_discharge() {
	Status = UPDATING;
	HAL_Delay(30000); // 30-second delay for any necessary pre-discharge preparations

	allRelaysOpen(); // Open all relays to ensure system is in a safe state before discharging

	HAL_GPIO_WritePin(CTRL_OK_GPIO_Port, CTRL_OK_Pin, GPIO_PIN_RESET);
	check_and_handle_pin_state(CTRL_OK_GPIO_Port, CTRL_OK_Pin);

	operation = 0; // Reset operation status
	Status = DISCHARGE; // Set status to DISCHARGE
	return;
}

void set_charging() {
	Status = UPDATING;
	allRelaysOpen();  // Open all relays to ensure the system is in a safe state

	// Set CHARGE_NEG_PIN and check if it is set correctly
	HAL_GPIO_WritePin(CHARGE_NEG_PORT, CHARGE_NEG_Pin, GPIO_PIN_SET);
	check_and_handle_pin_state(CHARGE_NEG_PORT, CHARGE_NEG_Pin);

	// Set CHARGE_POS_PIN and check if it is set correctly
	HAL_GPIO_WritePin(CHARGE_POS_PORT, CHARGE_POS_Pin, GPIO_PIN_SET);
	check_and_handle_pin_state(CHARGE_POS_PORT, CHARGE_POS_Pin);

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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		allRelaysOpen();
		HAL_GPIO_WritePin(CTRL_OK_GPIO_Port, CTRL_OK_Pin, GPIO_PIN_RESET);
		exit(0);
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
