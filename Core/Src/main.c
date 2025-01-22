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
#include "stdio.h"
#include "string.h"
#include "CLCD_I2C.h"
#include "stdlib.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_14

void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  while (__HAL_TIM_GET_COUNTER(&htim4) < us);
}

void DHT11_Start(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Cau hình chân DHT11 làm dau ra de gui tin hieu khoi dong
  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);

  // giuc muc LOW trong 18ms de start
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
  HAL_Delay(18);

  // Kéo chân lên muc cao trong 30us
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
  delay_us(30);

  // Chuy?n chân v? ch? d? d?u vào d? d?c ph?n h?i t? DHT11
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

uint8_t DHT11_CheckResponse(void) {
  uint8_t Response = 0;
  delay_us(40);

  // Wait for LOW response from DHT11
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
    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)));  // Wait for pin HIGH
    delay_us(40);

    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {  // If LOW, it means 0
      data &= ~(1 << (7 - j));
    } else {  // If HIGH, it means 1
      data |= (1 << (7 - j));
    }

    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN));  // Wait for pin LOW
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

    // kiem tra check sum
    if (Sum == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2)) {
      *humidity = Rh_byte1;
      *temperature = Temp_byte1;
    }
  }

  // Sau khi doc xong, cau hinh de chuan bi lan sau
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

CLCD_I2C_Name LCD1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void MX_TIM4_Init(void) {
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;  // 72MHz / (71+1) = 1MHz (1us per tick)
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_Base_Start(&htim4);
}

void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  HAL_UART_Init(&huart2);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CMD_BUFFER_SIZE 10

volatile uint8_t isRunning = 1;  // 1: chay; 0: dung
uint8_t uartBuffer[1];           // Nhan tung ki tu
char cmdBuffer[CMD_BUFFER_SIZE]; // Bo dem luu chuong trinh
uint8_t cmdIndex = 0;
uint8_t displayMode = 3;         // 1: only temp, 2: only humid, 3: both
uint32_t measureInterval = 2000; // Chu ki do, ms

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) 
    {
        if (uartBuffer[0] == '\r' || uartBuffer[0] == '\n')  // Neu nhan Enter
        {
            cmdBuffer[cmdIndex] = '\0';  // Ket thuc chuoi
            
            if (strcmp(cmdBuffer, "stop") == 0)  
            {
                isRunning = 0;
                char msg[] = "Stopped\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
            else if (strcmp(cmdBuffer, "start") == 0)  
            {
								displayMode =3;
                isRunning = 1;
                char msg[] = "Started\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
            else if (strcmp(cmdBuffer, "temp") == 0)  
            {
                displayMode = 1;
								isRunning = 1;
                char msg[] = "Mode: Temp Only\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
            else if (strcmp(cmdBuffer, "hum") == 0)  
            {
                displayMode = 2;
								isRunning = 1;
                char msg[] = "Mode: Humidity Only\r\n";
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
            else if (strncmp(cmdBuffer, "time", 4) == 0) 
            {
                uint32_t newTime = atoi(&cmdBuffer[4]);
                if (newTime >= 500 && newTime <= 10000)  // Gi?i h?n t? 500ms d?n 10s
                {
                    measureInterval = newTime;
                    char msg[30];
                    snprintf(msg, sizeof(msg), "New Time: %dums\r\n", measureInterval);
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                }
                else
                {
                    char msg[] = "Invalid Time (500-10000)\r\n";
                    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                }
            }

            cmdIndex = 0;  // reset bo dem lenh
        }
        else
        {
            if (cmdIndex < CMD_BUFFER_SIZE - 1)  
            {
                cmdBuffer[cmdIndex++] = uartBuffer[0]; // Nhan tung ki tu
            }
        }
        HAL_UART_Receive_IT(&huart2, uartBuffer, 1); // Nhan ki tu tiep theo
    }
}

int main(void)
{
    uint8_t temp, hum;
    
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();

    CLCD_I2C_Init(&LCD1, &hi2c1, 0x4e, 16, 2);

    // B?t d?u nh?n UART
    HAL_UART_Receive_IT(&huart2, uartBuffer, 1);
    
    while (1)
    {
        if (isRunning)
        {
            DHT11_GetData(&hum, &temp);
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "Temp: %d C, Hum: %d%%\r\n", temp, hum);
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
						CLCD_I2C_Clear(&LCD1);
            if (displayMode == 1)  // only temp
            {
							  
                CLCD_I2C_SetCursor(&LCD1, 0, 0);
                char temp_string[20];  
                sprintf(temp_string, "Temp: %d C", temp);  
                CLCD_I2C_WriteString(&LCD1, temp_string); 
            }
            else if (displayMode == 2)  // only humid
            {
								
                CLCD_I2C_SetCursor(&LCD1, 0, 0);
                char hum_string[20];  
                sprintf(hum_string, "Humid: %d%%", hum);  
                CLCD_I2C_WriteString(&LCD1, hum_string); 
            }
            else  // display both
            {
                CLCD_I2C_SetCursor(&LCD1, 0, 0);
                char temp_string[20];  
                sprintf(temp_string, "Temp: %d C", temp);  
                CLCD_I2C_WriteString(&LCD1, temp_string); 
                
                CLCD_I2C_SetCursor(&LCD1, 0, 1);
                char hum_string[20];  
                sprintf(hum_string, "Humid: %d%%", hum);  
                CLCD_I2C_WriteString(&LCD1, hum_string); 
            }
        }
        else 
        {
            CLCD_I2C_Clear(&LCD1);
            CLCD_I2C_SetCursor(&LCD1, 0, 0);
            CLCD_I2C_WriteString(&LCD1, "Stopped");
        }

        HAL_Delay(measureInterval);  // delay theo chu ki set up
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
