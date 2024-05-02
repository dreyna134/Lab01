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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */


  /* This is Blinky (original) */
	// __HAL_RCC_GPIOC_CLK_ENABLE();
	RCC->AHBENR |= 0x00080000; // Enable peripheral clock for port C (19th bit is set)
	RCC->AHBENR |= 0x00020000; // Enable peripheral clock for port A (17th bit is set)
	
	// Setup config struct to pass to init function
	/*GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};*/
	//HAL_GPIO_Init(GPIOC, &initStr); // Init pins PC8 & PC9
	
	// TODO to shift LEDs to blue/red: Change to pins PC6 and PC7!
	
	// Set LED pins PC8 and PC9 to general output mode (01 01)
	GPIOC->MODER |= 0x00005000; // Set MODER8 and MODER9 to have bits 0101 (both in general output mode)
	GPIOC->MODER &= ~(0x0000a000); // Ensure that 0 bits are actually 0
	
	// Set pins PC8 PC9 to push-pull output type in the OTYPER reg
	GPIOC->OTYPER &= ~(0x000000c0); // Set OT8 and OT9 to 0 (Output push-pull)
	
	// Set pins to low speed in OSPEEDR
	GPIOC->OSPEEDR &= ~(0x00000f000); // Set OSPEEDR8 and OSPEEDR9 to clear all bits such that pins are set to 00, low speed
	
	// Set pins to no pull-up/down resistors in the PUPDR reg
	GPIOC->PUPDR &= ~(0x0000f000); // Set PUPDR8 and 9 to clear all bits such that pins are set to 00, no pull-up, pull-down
	
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC8 high
	GPIOC->ODR |= 0x00000040; // Set ODR6 to be high
	GPIOC->ODR &= ~(0x00000080); // Set ODR7 to be low
	
	// USER BUTTON Config
	GPIOA->MODER &= ~((1 << 0) | (1 << 1)); // Clear bits 0 and 1. Sets MODER0 on port A to "Input Mode"
	
	GPIOA->OSPEEDR |= ~(0x00000001); // Set OSPEEDR0 bit 0 to be a '0'. Sets to "Low Speed"
	
	GPIOA->PUPDR &= ~(0x00000001); // Set PUPDR to pull-down (10). This line clears bit 0
	GPIOA->PUPDR |= 0x00000002; // Set PUPDR bit 1 to be '1'
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	uint32_t debouncer = 0;
  while (1)
  {
		//HAL_Delay(200);
		debouncer = (debouncer << 1);
		// Toggle the output state of both PC8 and PC9
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
		/* END OF BLINKY (Original) */
		
		// Toggle the pins PC8 and PC9 output states
		//GPIOC->ODR ^= 0x000000c0; // INVERT bits 6 and 7, effectively toggling both PC6 and 7 output states
		
		// Check button input state
		uint32_t mask = 0x00000001;
		uint32_t inputState = mask & GPIOA->IDR;
		
		// Button press check
		if (inputState)
			debouncer |= 0x01;
		
		// This code triggers ONCE when transitioning to steady high
		if (debouncer == 0x7FFFFFFF)
			GPIOC->ODR ^= 0x000000c0; // Invert bits 6 and 7, toggling PC6 and PC7 output states
		
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
