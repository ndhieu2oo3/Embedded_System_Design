	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
#include "stdio.h"
#include "string.h"
#include "CLCD_I2C.h"
#include "stdlib.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_14

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for readTask */
osThreadId_t readTaskHandle;
const osThreadAttr_t readTask_attributes = {
  .name = "readTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for sendTask */
osThreadId_t sendTaskHandle;
const osThreadAttr_t sendTask_attributes = {
  .name = "sendTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void displayFunction(void *argument);
void readFunction(void *argument);
void sendFunction(void *argument);

uint8_t temp, hum;
CLCD_I2C_Name LCD1;

typedef struct {
    uint8_t temp_data;
    uint8_t hum_data;
} myQueueData_t;

void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while (__HAL_TIM_GET_COUNTER(&htim4) < us);
}

void DHT11_Start(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

uint8_t DHT11_CheckResponse(void) {
    uint8_t Response = 0;
    delay_us(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        delay_us(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
            Response = 1;  // Response received
        }
    }
    delay_us(40);
    return Response;
}

uint8_t DHT11_ReadData(void) {
    uint8_t i, j;
    uint8_t data = 0;
    for (j = 0; j < 8; j++) {
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));
        delay_us(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
            data &= ~(1 << (7 - j));
        } else {
            data |= (1 << (7 - j));
        }
        while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));
    }
    return data;
}

void DHT11_GetData(uint8_t *humidity, uint8_t *temperature) {
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, Sum;
    DHT11_Start();
    if (DHT11_CheckResponse()) {
        Rh_byte1 = DHT11_ReadData();
        Rh_byte2 = DHT11_ReadData();
        Temp_byte1 = DHT11_ReadData();
        Temp_byte2 = DHT11_ReadData();
        Sum = DHT11_ReadData();
        if (Sum == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2)) {
            *humidity = Rh_byte1;
            *temperature = Temp_byte1;
        }
    }
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 0 */

void TASK1() {
    DHT11_GetData(&hum, &temp);
}

void TASK2() {
    CLCD_I2C_SetCursor(&LCD1, 0, 0);
    char temp_string[20];  
    sprintf(temp_string, "Temp: %d C", temp);  
    CLCD_I2C_WriteString(&LCD1, temp_string);
    CLCD_I2C_SetCursor(&LCD1, 0, 1);
    char hum_string[20];  
    sprintf(hum_string, "Humid: %d%%", hum);  
    CLCD_I2C_WriteString(&LCD1, hum_string);
}

void TASK3() {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Temp: %d C, Hum: %d%%\r\n", temp, hum);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

/* USER CODE END 0 */

void readFunction(void *argument) {
    myQueueData_t sensorData;
    while (1) {
        DHT11_GetData(&sensorData.hum_data, &sensorData.temp_data);
        osMessageQueuePut(myQueue01Handle, &sensorData, 0, osWaitForever);
        osDelay(2000);
    }
}

void displayFunction(void *argument) {
    myQueueData_t receivedData;
    while (1) {
        if (osMessageQueueGet(myQueue01Handle, &receivedData, NULL, osWaitForever) == osOK) {
            osMutexAcquire(myMutex01Handle, osWaitForever);
            CLCD_I2C_SetCursor(&LCD1, 0, 0);
            char temp_string[20];
            sprintf(temp_string, "Temp: %d C", receivedData.temp_data);
            CLCD_I2C_WriteString(&LCD1, temp_string);
            CLCD_I2C_SetCursor(&LCD1, 0, 1);
            char hum_string[20];
            sprintf(hum_string, "Humid: %d%%", receivedData.hum_data);
            CLCD_I2C_WriteString(&LCD1, hum_string);
            osMutexRelease(myMutex01Handle);
        }
        osDelay(500);
    }
}

void sendFunction(void *argument) {
    myQueueData_t receivedData;
    while (1) {
        if (osMessageQueueGet(myQueue01Handle, &receivedData, NULL, osWaitForever) == osOK) {
            osMutexAcquire(myMutex01Handle, osWaitForever);
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "Temp: %d C, Hum: %d%%\r\n", receivedData.temp_data, receivedData.hum_data);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            osMutexRelease(myMutex01Handle);
        }
        osDelay(1000);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();
		CLCD_I2C_Init(&LCD1, &hi2c1, 0x4e, 16, 2);

    osKernelInitialize();
    myQueue01Handle = osMessageQueueNew(16, sizeof(myQueueData_t), NULL);
    myMutex01Handle = osMutexNew(NULL);

    readTaskHandle = osThreadNew(readFunction, NULL, &readTask_attributes);
    displayTaskHandle = osThreadNew(displayFunction, NULL, &displayTask_attributes);
    sendTaskHandle = osThreadNew(sendFunction, NULL, &sendTask_attributes);

    osKernelStart();

    while (1) {
    }
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
		* @brief TIM4 Initialization Function
		* @param None
		* @retval None
		*/
	static void MX_TIM4_Init(void)
	{

		/* USER CODE BEGIN TIM4_Init 0 */

		/* USER CODE END TIM4_Init 0 */

		TIM_ClockConfigTypeDef sClockSourceConfig = {0};
		TIM_MasterConfigTypeDef sMasterConfig = {0};

		/* USER CODE BEGIN TIM4_Init 1 */

		/* USER CODE END TIM4_Init 1 */
		htim4.Instance = TIM4;
		htim4.Init.Prescaler = 7;
		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim4.Init.Period = 65535;
		htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
		{
			Error_Handler();
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
		{
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
		{
			Error_Handler();
		}
		/* USER CODE BEGIN TIM4_Init 2 */

		/* USER CODE END TIM4_Init 2 */

	}

	/**
		* @brief USART2 Initialization Function
		* @param None
		* @retval None
		*/
	static void MX_USART2_UART_Init(void)
	{

		/* USER CODE BEGIN USART2_Init 0 */

		/* USER CODE END USART2_Init 0 */

		/* USER CODE BEGIN USART2_Init 1 */

		/* USER CODE END USART2_Init 1 */
		huart2.Instance = USART2;
		huart2.Init.BaudRate = 115200;
		huart2.Init.WordLength = UART_WORDLENGTH_8B;
		huart2.Init.StopBits = UART_STOPBITS_1;
		huart2.Init.Parity = UART_PARITY_NONE;
		huart2.Init.Mode = UART_MODE_TX_RX;
		huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart2.Init.OverSampling = UART_OVERSAMPLING_16;
		if (HAL_UART_Init(&huart2) != HAL_OK)
		{
			Error_Handler();
		}
		/* USER CODE BEGIN USART2_Init 2 */

		/* USER CODE END USART2_Init 2 */

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
		__HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

		/*Configure GPIO pin : PB14 */
		GPIO_InitStruct.Pin = GPIO_PIN_14;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
	}

	/* USER CODE BEGIN 4 */

	/* USER CODE END 4 */

	/* USER CODE BEGIN Header_displayFunction */
	/**
		* @brief  Function implementing the displayTask thread.
		* @param  argument: Not used
		* @retval None
		*/
	/* USER CODE END Header_displayFunction */


	/**
		* @brief  Period elapsed callback in non blocking mode
		* @note   This function is called  when TIM2 interrupt took place, inside
		* HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
		* a global variable "uwTick" used as application time base.
		* @param  htim : TIM handle
		* @retval None
		*/
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
		/* USER CODE BEGIN Callback 0 */

		/* USER CODE END Callback 0 */
		if (htim->Instance == TIM2) {
			HAL_IncTick();
		}
		/* USER CODE BEGIN Callback 1 */

		/* USER CODE END Callback 1 */
	}

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
