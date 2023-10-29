/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <stdio.h>
#include "tm1637_drv.h"
#include "rtc_drv.h"
#include "clock.h"


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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay(int n);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//uint32_t source6 = GPIO_BSRR_BS8 | GPIO_BSRR_BR9;
//uint32_t source7 = GPIO_BSRR_BR8 | GPIO_BSRR_BS9;

//#define DELAY 100
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
//  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
//    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
//
//    RCC->APB2RSTR |= 1U << 4U;
//    RCC->APB2RSTR &= ~(1U << 4U);
//
//    GPIOC->CRH &= ~GPIO_CRH_MODE9_Msk;
//    GPIOC->CRH |= 2U << GPIO_CRH_MODE9_Pos ;
//    GPIOC->CRH &= ~(3U << GPIO_CRH_CNF9_Pos);
//
//    GPIOC->CRH &= ~(GPIO_CRH_MODE8_Msk);
//    GPIOC->CRH |= GPIO_CRH_MODE8_1;
//    GPIOC->CRH &= ~(3U << GPIO_CRH_CNF8_Pos);

//    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

//    TIM7->DIER |= 1U << 0U;
//    TIM7->PSC = 399U;
//    uint32_t freq = HAL_RCC_GetSysClockFreq();
//    uint16_t period = freq / 8000U;
//    TIM7->ARR = period;
//    TIM7->CR1 |= 1U << 0U;
//    HAL_NVIC_EnableIRQ(TIM7_IRQn);
//    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
//    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

//    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
//
//    TIM3->PSC = 399U;
//    TIM3->ARR = period;
//    TIM3->CCR3 = 9 * period / 10;
//    TIM3->CCR4 = 9 * period / 10;
//    TIM3->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
//    TIM3->CCER = TIM_CCER_CC3E | TIM_CCER_CC4P | TIM_CCER_CC4E;
//    TIM3->CR1 |= 1U << 0U;
//
//    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP;
//
//    GPIOC->CRH |= GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1 | GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;


//    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
//    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
//    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
//
//
//
//
//    DMA1_Channel6->CMAR = (uint32_t) &source6;
//    DMA1_Channel7->CMAR = (uint32_t) &source7;
//
//    DMA1_Channel6->CNDTR = 1U;
//    DMA1_Channel7->CNDTR = 1U;
//    DMA1_Channel6->CPAR = (uint32_t) &GPIOC->BSRR;
//    DMA1_Channel7->CPAR = (uint32_t) &GPIOC->BSRR;
//    DMA1_Channel6->CCR = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_DIR
//            | DMA_CCR_TCIE | DMA_CCR_EN | DMA_CCR_CIRC;
//    DMA1_Channel7->CCR = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_DIR
//            | DMA_CCR_TCIE | DMA_CCR_EN  | DMA_CCR_CIRC;
//
//    TIM16->PSC = 399U;
//    TIM17->PSC = 399U;
//    TIM16->ARR = period * 4 / 15;
//    TIM17->ARR = period * 4 / 14;
//    TIM16->DIER |= TIM_DIER_UDE;
//    TIM17->DIER |= TIM_DIER_UDE;
//    TIM16->CR1 |= TIM_CR1_CEN;
//    TIM17->CR1 |= TIM_CR1_CEN;

//    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
//    TIM7->ARR = period;
//    TIM7->DIER = TIM_DIER_UIE;
//    HAL_NVIC_EnableIRQ(TIM7_IRQn);
//
//    GPIOC->CRH = GPIO_CRH_MODE13_0 | GPIO_CRH_MODE8_0;
//    GPIOC->ODR = GPIO_ODR_ODR13 | GPIO_ODR_ODR8;
//
//    delay(DELAY);


    clock_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void delay(int n) {
    for (; n>0; n--);
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

/* USER CODE BEGIN 4 */



void DMA1_Channel6_IRQHandler(void) {
    DMA1->IFCR = DMA_IFCR_CGIF6;
}

void DMA1_Channel7_IRQHandler(void) {
    DMA1->IFCR = DMA_IFCR_CGIF7;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
