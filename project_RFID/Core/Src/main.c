/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rfid.h"
#include "stdio.h"
#include "string.h"
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

/* USER CODE BEGIN PV */

volatile uint8_t locked = 0; // 차후 택배가 들어왔음을 확인하기 위한 flag
volatile uint8_t accepted = 0;
static uint32_t prevTick = 0;
uint8_t test = 0;

uint8_t flag0 = 1;
uint8_t flag1 = 0;
uint8_t flag2 = 1;
uint8_t flag3 = 0;
uint8_t flag4 = 0;
uint8_t flag5 = 0;

uint8_t code[1];

uint8_t rx1_buffer[100]; // 블루투스용 넉넉한 버퍼
uint8_t rx6_buffer[100]; // PC용 넉넉한 버퍼

uint8_t prevAccepted = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  RC522_Init(); // RFID 활성화
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, 100);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx6_buffer, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if((now - prevTick) > 1)
    {
      prevTick = now;
      RFID_Task();

      if((accepted == 1) && (prevAccepted == 0))
      {
        flag0 = 1;
        flag1 = 0;
        char msg[] = "accepted";
        HAL_UART_Transmit(&huart6, (uint8_t*)msg, sizeof(msg)-1, 100);
      }
      prevAccepted = accepted;
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 1. UART1(블루투스) -> UART6(PC)
    if (huart->Instance == USART1)
    {
        // 받은 데이터 전송
        if(rx1_buffer[0] == '0')
        {
          flag0 = 1;
          flag1 = 0;
          accepted = 0;
        }
        else if(rx1_buffer[0] == '1')
        {
          flag0 = 0;
          flag1 = 1;
        }
        else if(rx1_buffer[0] == '2')
        {
          flag2 = 1;
          flag3 = 0;
        }
        else if(rx1_buffer[0] == '3')
        {
          flag2 = 0;
          flag3 = 1;
        }

        // flag 판단
        // 물체 판단
        if((flag0 == 1) && (locked == 1))
        {
          char msg[] = "Thief recored";
          locked = 0;
          code[0] = '4';
          flag4 = 1;
          HAL_UART_Transmit(&huart1, code, 1, 100);
          HAL_UART_Transmit(&huart6, (uint8_t*)msg, sizeof(msg)-1, 100);
        }

        if((flag1 == 1) && (locked == 0))
        {
          locked = 1;
        }

        // 사람 판단
        if(flag2 == 1)
        {
          if(flag5 == 1)
          {
            char msg[] = "Suspicious recored";
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, sizeof(msg)-1, 100);
            flag5 = 0;
          }
        }

        if(flag3 == 1)
        {
          if(flag4 == 1)
          {
            char msg[] = "\r\n";
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, sizeof(msg)-1, 100);
            flag4 = 0;
          }
          flag5 = 1;
          char msg[] = "Suspicious recording";
          HAL_UART_Transmit(&huart6, (uint8_t*)msg, sizeof(msg)-1, 100);
        }

        // DMA 수신 중단
        HAL_UART_AbortReceive(&huart1);

        // 버퍼 초기화
        memset(rx1_buffer, 0, 100);

        // 다시 수신 시작
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx1_buffer, 100);
    }

    // 2. UART6(PC) -> UART1(블루투스)
    if (huart->Instance == USART6)
    {
        HAL_UART_Transmit(&huart1, rx6_buffer, Size, 100);

        HAL_UART_AbortReceive(&huart6);

        memset(rx6_buffer, 0, 100);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx6_buffer, 100);
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
#ifdef USE_FULL_ASSERT
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
