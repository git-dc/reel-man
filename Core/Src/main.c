/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for blink */
osThreadId_t blinkHandle;
uint32_t blinkBuffer[128];
osStaticThreadDef_t blinkControlBlock;
const osThreadAttr_t blink_attributes = { .name = "blink", .cb_mem =
		&blinkControlBlock, .cb_size = sizeof(blinkControlBlock), .stack_mem =
		&blinkBuffer[0], .stack_size = sizeof(blinkBuffer), .priority =
		(osPriority_t) osPriorityLow, };
/* Definitions for adcmon */
osThreadId_t adcmonHandle;
const osThreadAttr_t adcmon_attributes = { .name = "adcmon", .stack_size = 256
		* 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for lcdhandler */
osThreadId_t lcdhandlerHandle;
const osThreadAttr_t lcdhandler_attributes =
		{ .name = "lcdhandler", .stack_size = 256 * 4, .priority =
				(osPriority_t) osPriorityBelowNormal, };
/* USER CODE BEGIN PV */
const double PI = 3.14;

const double encoder_A_pulses_per_rot = 600.0; // 600 pulses per full rotation
const double wheel_A_diameter = 10.0; // in mm
const double wheel_A_rot_per_pulse = 2 * PI * wheel_A_diameter
		/ encoder_A_pulses_per_rot;

const double encoder_B_pulses_per_rot = 100; // 100 pulses per full rotation
const double wheel_B_diameter = 10; // in mm
const double wheel_B_rot_per_pulse = 2 * PI * wheel_B_diameter
		/ encoder_B_pulses_per_rot;

uint32_t reel_A_degrees = 0;
uint8_t count_to_5 = 0;

double reel_A_position = 0.0;
uint8_t encoder_A_phase_0 = 0;
uint8_t encoder_A_phase_1 = 0;
uint32_t reel_A_pulses = 0;

double reel_B_position = 0.0;
uint8_t encoder_B_phase_0 = 0;
uint8_t encoder_B_phase_1 = 0;
uint32_t reel_B_pulses = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void start_blink(void *argument);
void start_adcmon(void *argument);
void start_lcdhandler(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of blink */
	blinkHandle = osThreadNew(start_blink, NULL, &blink_attributes);

	/* creation of adcmon */
	adcmonHandle = osThreadNew(start_adcmon, NULL, &adcmon_attributes);

	/* creation of lcdhandler */
	lcdhandlerHandle = osThreadNew(start_lcdhandler, NULL,
			&lcdhandler_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PHASE_A0_Pin PHASE_A1_Pin PHASE_B0_Pin PHASE_B1_Pin */
	GPIO_InitStruct.Pin = PHASE_A0_Pin | PHASE_A1_Pin | PHASE_B0_Pin
			| PHASE_B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : POKE_POINT_Pin */
	GPIO_InitStruct.Pin = POKE_POINT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(POKE_POINT_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len) {
	for (int DataIdx = 0; DataIdx < len; DataIdx++)
		ITM_SendChar(*ptr++);

	return len;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PHASE_A0_Pin) {
		// set encoder A phase 0 state variable
		encoder_A_phase_0 = 1;
		if (encoder_A_phase_1) {
			// phase 1 ahead of phase 0 on encoder A:
			// means reel A is retracting
			encoder_A_phase_0 = 0;
			encoder_A_phase_1 = 0;
			reel_A_pulses++;
			reel_A_degrees += 6;
			reel_A_position -= wheel_A_rot_per_pulse;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
	} else if (GPIO_Pin == PHASE_A1_Pin) {
		// set encoder A phase 1 state variable
		encoder_A_phase_1 = 1;
		if (encoder_A_phase_0) {
			// phase 0 ahead of phase 1 on encoder A:
			// means reel A is feeding out
			encoder_A_phase_0 = 0;
			encoder_A_phase_1 = 0;
			reel_A_pulses++;
			reel_A_degrees += 6;
			reel_A_position -= wheel_A_rot_per_pulse;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_blink */
/**
 * @brief  Function implementing the blink thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_start_blink */
void start_blink(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(500);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_adcmon */
/**
 * @brief Function implementing the adcmon thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_start_adcmon */
void start_adcmon(void *argument) {
	/* USER CODE BEGIN start_adcmon */
	/* Infinite loop */
	for (;;) {

		osDelay(100);
	}
	/* USER CODE END start_adcmon */
}

/* USER CODE BEGIN Header_start_lcdhandler */
/**
 * @brief Function implementing the lcdhandler thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_start_lcdhandler */
void start_lcdhandler(void *argument) {
	/* USER CODE BEGIN start_lcdhandler */
	/* Infinite loop */
	for (;;) {
		printf("%lu Reel A degrees: %lu\n", HAL_GetTick(), reel_A_degrees);
		osDelay(50);
	}
	/* USER CODE END start_lcdhandler */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM4) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
