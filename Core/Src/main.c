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
#include "MPU9250-DMP.h"
#include "printf/printf.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
//#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
char buff[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ULTRA_FORW HAL_GPIO_ReadPin(ULTRASONIC_FORW_GPIO_Port, ULTRASONIC_FORW_Pin)
#define ULTRA_BACK HAL_GPIO_ReadPin(ULTRASONIC_BACK_GPIO_Port, ULTRASONIC_BACK_Pin)
#define START_B HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin)
#define STOP_B HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin)

#define DRIVER_MOVE_ID 0x01
#define DRIVER_LIFT_ID 0x02
uint8_t uart_buff[100];
uint8_t new_data = 0;
uint16_t data_size = 0;

int32_t start_stop = 0;
int32_t motor_speed = 0;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0,};
uint8_t RxData[8] = {0,};
uint32_t TxMailbox = 0;
volatile uint8_t NewCanMsg = 0;

enum CanGetMsgStatus {
	CAN_GET_MSG_WAIT					=  0,
	CAN_GET_MSG_OK                      =  1,
	CAN_GET_MSG_ERROR                   =  2,
};

typedef struct {
	int32_t encoder;
	float voltage;
} CanDataRecvTypeDef;

typedef struct {
	uint32_t canId;
	uint32_t canExtId;
	uint32_t canRTR;
	uint8_t canData[8];
} CanDataSendTypeDef;

extern float pitch_inside, roll_inside, yaw_inside;
extern float heading;
void printIMUData(void)
{
  snprintf(buff, sizeof buff, "%2.2f | %2.2f | %2.2f |  %2.2f\r\n", roll_inside, pitch_inside, yaw_inside, heading);
  HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uint32_t er = HAL_UART_GetError(huart);
	switch (er) {
		case HAL_UART_ERROR_PE: // ошибка четности
			__HAL_UART_CLEAR_PEFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		case HAL_UART_ERROR_NE:  // шум на линии
			__HAL_UART_CLEAR_NEFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		case HAL_UART_ERROR_FE:  // ошибка фрейма
			__HAL_UART_CLEAR_FEFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		case HAL_UART_ERROR_ORE:  // overrun error
			__HAL_UART_CLEAR_OREFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		case HAL_UART_ERROR_DMA:  // ошибка DMA
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		default:
			break;
		}
	if (huart->Instance == USART1) {
		new_data = 1;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_buff, sizeof(uart_buff));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART1) {
		new_data = 1;
		data_size = Size;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_buff, sizeof(uart_buff));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		Error_Handler();
	}
	NewCanMsg = CAN_GET_MSG_OK;
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    NewCanMsg = CAN_GET_MSG_ERROR;
}

uint8_t CanMsgRead(CanDataRecvTypeDef *canDataRecv) {

	return 1;
}


uint8_t CanMsgSend(CanDataSendTypeDef *canDataSend) {

	TxHeader.StdId = canDataSend->canId;
	TxHeader.ExtId = canDataSend->canExtId;
	TxHeader.RTR = canDataSend->canRTR;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = 0;
	//globData.can_mutex = 0;
	for (int i = 0; i < sizeof(TxData); i++) {
		TxData[i] = canDataSend->canData[i];
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		//HAL_UART_Transmit(&huart, (uint8_t*) "no_trans\r\n", 10, 100);
		return 0;
	}
	return 1;
}

void Moving() {

	CanDataSendTypeDef canDataSend;

	static int move_axis_en = 0;
	static int lift_axis_en = 0;
	if (start_stop) {
			canDataSend.canExtId = DRIVER_MOVE_ID + 0x06000000;
			canDataSend.canRTR = CAN_RTR_DATA;
			if (!move_axis_en) {
				move_axis_en = 1;
				canDataSend.canData[0] = 0x23;
				canDataSend.canData[1] = 0x0D;
				canDataSend.canData[2] = 0x20;
				canDataSend.canData[3] = 0x01;
				canDataSend.canData[4] = 0x00;
				canDataSend.canData[5] = 0x00;
				canDataSend.canData[6] = 0x00;
				canDataSend.canData[7] = 0x00;
				CanMsgSend(&canDataSend);
				HAL_Delay(10);
			}
			if (!lift_axis_en) {
				lift_axis_en = 1;
				canDataSend.canData[0] = 0x23;
				canDataSend.canData[1] = 0x0D;
				canDataSend.canData[2] = 0x20;
				canDataSend.canData[3] = 0x02;
				canDataSend.canData[4] = 0x00;
				canDataSend.canData[5] = 0x00;
				canDataSend.canData[6] = 0x00;
				canDataSend.canData[7] = 0x00;
				CanMsgSend(&canDataSend);
				HAL_Delay(10);
			}
			static int32_t ch_velocity = 0;
			//if (motor1_speed) { //(l_current_move_comm == MOVE_FORW) {
			//HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)"can_send\r\n", 10, 100);
			canDataSend.canData[0] = 0x23;
			canDataSend.canData[1] = 0x00;
			canDataSend.canData[2] = 0x20;
			canDataSend.canData[3] = 0x01;
			ch_velocity = motor_speed;
			canDataSend.canData[4] = ch_velocity >> 24;
			canDataSend.canData[5] = ch_velocity >> 16;
			canDataSend.canData[6] = ch_velocity >> 8;
			canDataSend.canData[7] = ch_velocity;
			CanMsgSend(&canDataSend);
			HAL_Delay(2);
			//}
			//else if (motor2_speed) {
			canDataSend.canData[0] = 0x23;
			canDataSend.canData[1] = 0x00;
			canDataSend.canData[2] = 0x20;
			canDataSend.canData[3] = 0x02;
			ch_velocity = motor_speed;
			canDataSend.canData[4] = ch_velocity >> 24;
			canDataSend.canData[5] = ch_velocity >> 16;
			canDataSend.canData[6] = ch_velocity >> 8;
			canDataSend.canData[7] = ch_velocity;
			CanMsgSend(&canDataSend);
			//}
		}
		else
		{
			canDataSend.canData[0] = 0x23;
			canDataSend.canData[1] = 0x00;
			canDataSend.canData[2] = 0x20;
			canDataSend.canData[3] = 0x01;
			canDataSend.canData[4] = 0x00;
			canDataSend.canData[5] = 0x00;
			canDataSend.canData[6] = 0x00;
			canDataSend.canData[7] = 0x00;
			CanMsgSend(&canDataSend);
			HAL_Delay(10);
			canDataSend.canData[0] = 0x23;
			canDataSend.canData[1] = 0x00;
			canDataSend.canData[2] = 0x20;
			canDataSend.canData[3] = 0x02;
			canDataSend.canData[4] = 0x00;
			canDataSend.canData[5] = 0x00;
			canDataSend.canData[6] = 0x00;
			canDataSend.canData[7] = 0x00;
			CanMsgSend(&canDataSend);
			move_axis_en = 0;
			lift_axis_en = 0;
		}
		HAL_Delay(20);
}

void read_uart() {
	if (new_data)
	{
		new_data = 0;
	}
//	HAL_UART_Transmit(&huart1, (uint8_t*)bms_jbd_request_msg0, sizeof(bms_jbd_request_msg0), 100);
}

void logic()
{
	if (start_stop)
	{

	}
	else
	{
		motor_speed = 0;
	}
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_buff, sizeof(uart_buff));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart1, (uint8_t*)"Start\r\n", 7, HAL_MAX_DELAY);
  if (MPU9250_begin() != INV_SUCCESS)
  {
	  while(1)
	  {
		  HAL_UART_Transmit(&huart1, (uint8_t*)"Error\r\n", 7, HAL_MAX_DELAY);
		  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
		  HAL_Delay(1500);
	  }
  }
  else
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"begin\r\n", 7, HAL_MAX_DELAY);
  }



  MPU9250_dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, 10);
  //uint32_t time_u = 0;
  while (1)
  {

	  if(MPU9250_fifoAvailable())
	  {
		  if (MPU9250_dmpUpdateFifo() == INV_SUCCESS)
		  {
			  MPU9250_computeEulerAngles(1);
			  //MPU9250_updateCompass();
			  //MPU9250_computeCompassHeading();
			  //printIMUData();
		  }
	  }
	  logic();
	  Moving();

	  //HAL_Delay(100);

//	  if((HAL_GetTick() - time_u) > 1000)
//	  {
////		  snprintf(buff, 20, "%2.2f | %2.2f | %2.2f\r\n", roll, pitch, yaw);
//		  sprintf(buff, "%d.%d | %d.%d | %d.%d\r\n", roll/10, rollDecimal, pitch/10, pitchDecimal, yaw/10, yawDecimal);
//	  	  HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
//	  	  time_u = HAL_GetTick();
//	  }
//	  HAL_Delay(10);
//
    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
	CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
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
  	  	sFilterConfig.FilterBank = 0;
        sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        sFilterConfig.FilterIdHigh = 0x0000;
        sFilterConfig.FilterIdLow = 0x0000;
        sFilterConfig.FilterMaskIdHigh = 0x0000;
        sFilterConfig.FilterMaskIdLow = 0x0000;
        sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
        sFilterConfig.FilterActivation = ENABLE;
        //sFilterConfig.SlaveStartFilterBank = 14;

        if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
        {
        	Error_Handler();
        }
        if (HAL_CAN_Start(&hcan) != HAL_OK) {
      	Error_Handler();
        }
        if (HAL_CAN_ActivateNotification(&hcan,
      		  CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF
  			  	  | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
      	Error_Handler();
        }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GY_CS_GPIO_Port, GY_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULTRASONIC_FORW_Pin ULTRASONIC_BACK_Pin START_Pin STOP_Pin */
  GPIO_InitStruct.Pin = ULTRASONIC_FORW_Pin|ULTRASONIC_BACK_Pin|START_Pin|STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GY_CS_Pin */
  GPIO_InitStruct.Pin = GY_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GY_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
