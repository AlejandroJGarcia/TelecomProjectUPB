
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE BEGIN Includes */

#define WELCOME_CLASS "\r\n==========================================\r\nWelcome Electronics&Telecom. Project Class\r\n==========================================\r\n"
#define WELCOME_MSG "Nucleo-Board Management Console\r\n"
#define MAIN_MENU   "Select the option you are interested in:\r\n 1. Toggle LD2 LED\r\n 2. Read USER BUTTON status\r\n 3. Clear screen and print this message\r\n 4. TEN IN BINARY\r\n 5. ELEVEN IN BINARY\r\n \r\n 6. TWELVE IN BINARY\r\n"
#define INTERRUP "INTERRUPT.!!!!!!!!!!!!!!!!!!!\r\n"
#define PROMPT "\r\n> "


/* USER CODE END Includes */
char readBuf[1];
__IO ITStatus UartReady = SET;
volatile bool status = 0;
int8_t opt;
int8_t option;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
volatile uint8_t myFlag = 0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
void performCriticalTasks(void);
void printClassWelcomeMessage(void);
void printWelcomeMessage(void);
uint8_t processUserInput(int8_t opt);
int8_t readUserInput(void);

/* USER CODE END PV */
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	 /* USER CODE BEGIN 1 */

	  /* USER CODE END 1 */

	  /* MCU Configuration----------------------------------------------------------*/

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
	  MX_USART2_UART_Init();
	  MX_USART1_UART_Init();
	  /* USER CODE BEGIN 2 */
	  HAL_NVIC_SetPriority(USART1_IRQn,0,0);
	  HAL_NVIC_EnableIRQ(USART1_IRQn);
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  printClassWelcomeMessage();

	   printWelcomeMessage();

	   while (1)
	   {

	   /* USER CODE END WHILE */
	     opt = readUserInput();
	     if(opt > 0)
	     {
	       processUserInput(opt);
	       if(opt == 3)
	       {
	     	  printWelcomeMessage();
	           //goto printMessage;
	       }
	     }
	     performCriticalTasks();
	     /* USER CODE BEGIN 3 */
	   }
	  /* USER CODE END 3 */
	   if (myFlag == 1){
	   			   printinter();
	   		   }
}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void performCriticalTasks(void){
	HAL_Delay(100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	/* Set transmission flag: transfer complete*/
	UartReady = SET;
}

void printClassWelcomeMessage(void){
	char *strings[] = {PROMPT, WELCOME_CLASS, PROMPT};

	for (uint8_t i = 0; i < 3; i++)
	{
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)strings[i], strlen(strings[i]));

		while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);
	}
}
void printinter(void){
	char *strings[] = {PROMPT, INTERRUP, PROMPT};

	for (uint8_t i = 0; i < 3; i++)
	{
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)strings[i], strlen(strings[i]));

		while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);
	}
}
void printWelcomeMessage(void){
	char *strings[] = {PROMPT, WELCOME_MSG, MAIN_MENU, PROMPT};

	for (uint8_t i = 0; i < 4; i++)
	{
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)strings[i], strlen(strings[i]));

		while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX || HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX);
	}
}

int8_t readUserInput(void){
	int8_t retVal = -1;

	if(UartReady == SET)
	{
		UartReady = RESET;
		HAL_UART_Receive_IT(&huart1, (uint8_t*)readBuf, 1);
		retVal = atoi(readBuf);
	}
	return retVal;
}

uint8_t processUserInput(int8_t opt){
	  char msg[30];

	  if(!(opt >=1 && opt <= 6))
	    return 0;

	  sprintf(msg, "%d", opt);
	  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	  switch(opt) {
		  case 1:
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			break;
		  case 2:
			sprintf(msg, "\r\nUSER BUTTON status: %s",
				HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET ? "PRESSED" : "RELEASED");
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			break;
		  case 3:
			return 2;
		  case 4:

			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
			  break;
		  case 5:

			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
			  break;
		  case 6:

			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
			  			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
  			  break;
	  }

	  HAL_UART_Transmit(&huart1, (uint8_t*)PROMPT, strlen(PROMPT), HAL_MAX_DELAY);
	  return 1;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  if(GPIO_Pin==B1_Pin){
	  myFlag = 1;
  }
  else{
	  myFlag=0;
  }

  /* NOTE: This function should not be modified, when the callback is needed,
            the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
