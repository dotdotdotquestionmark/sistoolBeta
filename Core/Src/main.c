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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

#define SYSTICK_LOAD (SystemCoreClock/1000000U)


extern __IO uint32_t uwTick;
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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t RxData[64];
char txdata[35] = "stanley what the hell is going on\r\n";
char txdata1[35] = "i can use a pussy sandwich\r\n";
char txdata2[35] = "smoke weed everyday\r\n";
char txcomplete[31] = "tx is done sending what's next\r\n";
volatile uint8_t ActiveCommand = 0;

unsigned long blinkInterval;
uint8_t *inputData;
uint32_t cycleStart = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

typedef struct MOTOR {
  int MotorNumber;
  int Angle;
  int Speed;
  uint16_t DataPin;
};

struct MOTOR motor1;
struct MOTOR motor2;
struct MOTOR motor3;

void motorInitializer(){
  motor1.MotorNumber = 1;
  motor1.Angle = 0;
  motor1.Speed = 5;
  motor1.DataPin = GPIO_PIN_6;

  motor2.MotorNumber = 2;
  motor2.Angle = 0;
  motor2.Speed = 5;
  motor2.DataPin = GPIO_PIN_7;

  // thissun for the pincer
  motor3.MotorNumber = 3;
  motor3.Angle = 0;
  motor3.Speed = 5;
  motor3.DataPin = GPIO_PIN_8;
}

void inputHandler(){
  if(ActiveCommand){
    ActiveCommand = 0;
    if(RxData[0] == 'M'){
      char successMsg[] = "First character is 'M'! Hello there!\r\n";
      HAL_UART_Transmit_DMA(&huart2, (uint8_t*)successMsg, strlen(successMsg));
      //uint32_t TARGET = 0;

      int SPEED;
      int ANGLE = RxData[2];
      //char angleTen[] = RxData[3];
      // char angleOne[] = RxData[4];

      // int ANGLEHUNDRED = atoi(angleHundred);
      // int ANGLETEN = atoi(angleTen);
      // int ANGLEONE = atoi(angleOne);

      //int ANGLE = 0;//ANGLEHUNDRED*100+ANGLETEN*10+ANGLEONE;
      // atoi(ANGLE);
      // atoi(SPEED);

      if(RxData[1] == '1'){
        motor1.Angle = ANGLE;
        motor1.Speed = SPEED;
        char M1Detection[23] = "M1 was found, Heyyyy!\r\n";
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t*)M1Detection, strlen(M1Detection));
        int cycles = 1;
        for(int i = 0; i < cycles; i++){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
          HAL_Delay(200);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
          HAL_Delay(200);
        }
      }
      if(RxData[1] == '2'){
        motor2.Angle = ANGLE;
        motor2.Speed = SPEED;
        char M2Detection[23] = "M2 was found, Heyyyy!\r\n";
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t*)M2Detection, strlen(M2Detection));
        int cycles = 2;
        for(int i = 0; i < cycles; i++){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
          HAL_Delay(200);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
          HAL_Delay(200);
        }
      }
      if(RxData[1] == '3'){
        motor3.Angle = ANGLE;
        motor3.Speed = SPEED;
        char M3Detection[23] = "M3 was found, Heyyyy!\r\n";
        //HAL_UART_Transmit_DMA(&huart2, (uint8_t*)M3Detection, strlen(M3Detection));
        int cycles = 3;
        for(int i = 0; i < cycles; i++){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
          HAL_Delay(200);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
          HAL_Delay(200);
        }
      }
      
    }
  }
}


uint32_t micros(){
  const uint32_t reloadValue = SysTick->LOAD;
    
  // The millisecond counter provided by the HAL library.
  uint32_t msTicks;
    
  // The current value of the SysTick down-counter.
  uint32_t currentTickValue;

  do {
      msTicks = uwTick;
      currentTickValue = SysTick->VAL;
  } while (msTicks != uwTick);

  return (msTicks * 1000) + ((reloadValue - currentTickValue) / (SystemCoreClock / 1000000));

}


void motorDriver(struct MOTOR *motor){
  // adjust pulse rate of each motor
  // lets say hypothetically you get a micros function working 
  // call this when you get 
  int ANGLE = motor->Angle;
  int additionalPULSE = ANGLE*11;
  char printableANGLE[8];
  itoa (ANGLE, printableANGLE, 8);
  uint32_t PulseInterval = 2000+(uint32_t)additionalPULSE; 
  uint32_t CycleInterval = 20000; //time for 50 hz
  uint32_t currentTime = micros();
  uint16_t DATAPIN = motor->DataPin;
  if ((currentTime - cycleStart)>CycleInterval){
    HAL_GPIO_WritePin(GPIOA, DATAPIN, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    cycleStart = currentTime;
  }
  if ((currentTime - cycleStart)>PulseInterval){
    HAL_GPIO_WritePin(GPIOA, DATAPIN, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  }

}


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Receive_IT(&huart2, RxData, 32);

  //HAL_UART_Receive_DMA(&huart2, RxData, 32);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxData, 64);
  motorInitializer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

  while (1)
  {
    // look for data processing flag
    inputHandler();
    
    //motorDriver(&motor1);
    // motorDriver(&motor2);
    motorDriver(&motor3);

    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

    //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

    //char tx_buffer[50];
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    // HAL_Delay(10);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    // HAL_Delay(10);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 PA8 PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{

  if (huart->Instance == USART2) {

    HAL_UART_AbortReceive(huart);
    RxData[size] = '\0';
    // arm flag
    ActiveCommand = 1;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxData, 64);
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
