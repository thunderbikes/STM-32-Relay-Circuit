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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
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

void allAuxDigitalRead(){
	 HAL_GPIO_ReadPin(R1_AUX_PORT, R1_AUX_Pin);
	 HAL_GPIO_ReadPin(R2_AUX_PORT, R2_AUX_Pin);
	HAL_GPIO_ReadPin(R3_AUX_PORT, R3_AUX_Pin);
}

int Reading_Pin(char aux_number){
    int state = 0; // Default state
    switch(aux_number){
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
            state =HAL_GPIO_ReadPin(IGNITION_PORT, IGNITION_Pin);
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
	//
	float Read_ADC_Voltage(void) {
		uint32_t hv_sense_value;
		float hv_sense_voltage;
	    // Start the ADC
	    HAL_ADC_Start(&hadc1);

	    // Poll for conversion completion with a timeout of 10ms
	    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
	        // Get the converted value
	    	hv_sense_value = HAL_ADC_GetValue(&hadc1);
	    	hv_sense_voltage = (hv_sense_value / 4095.0) * 103.6;
	    	return hv_sense_voltage;
	    }

	    // Return 0 or some error code if reading fails
	    return 0;
	}

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //voltage = (adc_value / 4095.0) * 3.3
  //printf("HV Sense Voltage: %lu\r\n", hv_sense_value)
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //voltage = (adc_value / 4095.0) * 3.3
	  float hv_sense_voltage = Read_ADC_Voltage();
	  int operation = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (Reading_Pin('I') == 1 && operation == 0)
	  {
		  allRelaysOpen();
		  operation = 1;
		  HAL_GPIO_WritePin(HVC_NEG_PORT, HVC_NEG_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(PUMP_ENABLE_PORT, PUMP_ENABLE_Pin, GPIO_PIN_SET);


		  /*
		   * The hv sense voltage gives voltage between the motor controller, we divide that by bms_battert
		   * voltage from canbus and if it is greater that 90 PERCENT
		  */


		  //Operation Mode
		  HAL_GPIO_WritePin(HVC_POS_PORT, HVC_POS_Pin, GPIO_PIN_SET);
		  HAL_Delay(1000);
		  //NEED TO DO CHECKING AUX
		  HAL_GPIO_WritePin(P_CHARGE_PORT, P_CHARGE_Pin, GPIO_PIN_RESET);
	  }
	  //Discharge
	  if (Reading_Pin('I') == 0 && operation == 1 )
		  {
		  HAL_Delay(30000); // 30 second delay
		  HAL_GPIO_WritePin(CTRL_OK_GPIO_Port, CTRL_OK_Pin, GPIO_PIN_RESET);
		  allRelaysOpen();
		  operation = 0;

		  }
	  //Charging
	  if (Reading_Pin('c') == 1 )
	  {
		  allRelaysOpen();
		  HAL_GPIO_WritePin(CHARGE_NEG_PORT, CHARGE_NEG_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(CHARGE_POS_PORT, CHARGE_POS_Pin, GPIO_PIN_SET);

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, HVC_POS_Pin|HVC_NEG_Pin|P_CHARGE_Pin|CHARGE_POS_Pin
                          |CHARGE_NEG_Pin|PUMP_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CTRL_OK_GPIO_Port, CTRL_OK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMD_IO_L_Pin IGNITION_Pin CHARGE_ENABLE_Pin IMD_IO_H_Pin */
  GPIO_InitStruct.Pin = IMD_IO_L_Pin|IGNITION_Pin|CHARGE_ENABLE_Pin|IMD_IO_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HVC_POS_Pin HVC_NEG_Pin P_CHARGE_Pin CHARGE_POS_Pin
                           CHARGE_NEG_Pin PUMP_ENABLE_Pin */
  GPIO_InitStruct.Pin = HVC_POS_Pin|HVC_NEG_Pin|P_CHARGE_Pin|CHARGE_POS_Pin
                          |CHARGE_NEG_Pin|PUMP_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_INPUT1_Pin BMS_INPUT2_Pin BMS_INPUT3_Pin SHDWN_ST_Pin */
  GPIO_InitStruct.Pin = BMS_INPUT1_Pin|BMS_INPUT2_Pin|BMS_INPUT3_Pin|SHDWN_ST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BMS_OK_L_Pin IMD_OK_L_Pin R1_AUX_Pin R2_AUX_Pin
                           R3_AUX_Pin R4_AUX_Pin */
  GPIO_InitStruct.Pin = BMS_OK_L_Pin|IMD_OK_L_Pin|R1_AUX_Pin|R2_AUX_Pin
                          |R3_AUX_Pin|R4_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CTRL_OK_Pin */
  GPIO_InitStruct.Pin = CTRL_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CTRL_OK_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
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
