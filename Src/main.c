/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#define DEADB 47  // a number not in the main array thats not too big!
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
int ADCValue;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  int state=0,rev=0,flash_c=0,delay=0;
  int delay_d[40] = {0,0,2,2,4,6,8,10,20,30,   40,50,60,70,80,90,100,200,DEADB,DEADB,DEADB,DEADB,200,100,90,80,70,60,50,40,   30,20,10,8,6,4,2,2,0,0};
  int flash_d = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    HAL_Delay(delay);

    if(delay != DEADB)
    {
      if(rev) {
        switch(state){
        case 0:
          HAL_GPIO_WritePin( GPIOB , GPIO_PIN_3 , GPIO_PIN_RESET );  // B
          HAL_GPIO_WritePin( GPIOA , GPIO_PIN_2 , GPIO_PIN_RESET ); // A
          state++;
          break;
        case 1:
          HAL_GPIO_WritePin( GPIOA , GPIO_PIN_2 , GPIO_PIN_SET ); // A
          state++;
          break;
        case 2:
          HAL_GPIO_WritePin( GPIOB , GPIO_PIN_3 , GPIO_PIN_SET );  // B
          state++;
          break;
        case 3:
          HAL_GPIO_WritePin( GPIOA , GPIO_PIN_2 , GPIO_PIN_RESET ); // A
          state=0;
          break;
        }
      }else{
        switch(state){
        case 0:
          HAL_GPIO_WritePin( GPIOB , GPIO_PIN_3 , GPIO_PIN_RESET );  // B
          HAL_GPIO_WritePin( GPIOA , GPIO_PIN_2 , GPIO_PIN_RESET ); // A
          state++;
          break;
        case 1:
          HAL_GPIO_WritePin( GPIOA , GPIO_PIN_3 , GPIO_PIN_SET );
          state++;
          break;
        case 2:
          HAL_GPIO_WritePin( GPIOB , GPIO_PIN_2 , GPIO_PIN_SET );
          state++;
          break;
        case 3:
          HAL_GPIO_WritePin( GPIOA , GPIO_PIN_3 , GPIO_PIN_RESET );
          state=0;
          break;
        }
      }//IF

      // LED flashing
      flash_c++;
      flash_d = 200-delay + 1;
      flash_d /= 10;
      if(rev){
        if(flash_c >= flash_d){
          HAL_GPIO_TogglePin( GPIOB , GPIO_PIN_4);  // RD
          HAL_GPIO_WritePin( GPIOB , GPIO_PIN_5 , GPIO_PIN_RESET );  // GN
          flash_c=0;
        }
      }else{
        if(flash_c >= flash_d){
            HAL_GPIO_TogglePin( GPIOB , GPIO_PIN_5);  // GN
            HAL_GPIO_WritePin( GPIOB , GPIO_PIN_4 , GPIO_PIN_RESET );  // RD
            flash_c=0;
        }
      }

    }else{
      // deadband
      HAL_GPIO_WritePin( GPIOB , GPIO_PIN_3 , GPIO_PIN_RESET );  // B
      HAL_GPIO_WritePin( GPIOA , GPIO_PIN_2 , GPIO_PIN_RESET ); // A
      HAL_GPIO_WritePin( GPIOB , GPIO_PIN_4 , GPIO_PIN_RESET );  // RD
      HAL_GPIO_WritePin( GPIOB , GPIO_PIN_5 , GPIO_PIN_RESET );  // GN

    }

    // ADC DAMPLING
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      /* Start Conversation Error */
      // Error_Handler();
    }
    if (HAL_ADC_PollForConversion(&hadc1, 500) != HAL_OK)
    {
      /* End Of Conversion flag not set on time */
      // Error_Handler();
      ADCValue=-1;
    }
    else
    {
      /* ADC conversion completed */
      /*##-5- Get the converted value of regular channel ########################*/
      //0-4096
      ADCValue = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);

    //Condition ADC into delay array
    ADCValue /= 100; // 0- 40
    if(ADCValue > 19)
      rev=0;
    else
      rev=1;
    //validate
    if(ADCValue > 39){
      ADCValue = 39;
    }
    if(ADCValue < 0) {
      ADCValue = 0;
    }
    delay = delay_d[ADCValue];
  }//WHILE
  /* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

  /**Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

  /**Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
