/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
//#include "ble_commands.h"
#include "ble.h"

// Other imports
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

/* Include memory map of our MCU */
#include <stm32l475xx.h>

#include "timer.h"
#include "i2c.h"
#include "lsm6dsl.h"
#include "leds.h"
#include "lptimer.h"

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(int MSI_Val);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

// Counter to keep track of every time a 50 ms duration elapses and the device is not moving.
volatile unsigned int start_i = 0;

// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void TIM2_IRQHandler() {
	// Checking whether TIM2 has had an update event, and then clearing the update event every time it happens.
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;

        start_i++;

        // Ensure that the counter does not go out of bounds, and resets smoothly.
        // Note: UINT8_MAX * 60 represents the start_i value at which the mins_lost value overflows and resets to 1.
        // We round down UINT_MAX to the highest multiple of this overflow value to ensure that the timer resets without breaking the pattern.
        if (start_i == (UINT_MAX - (UINT_MAX % (UINT8_MAX * 60)))) {
            start_i = 60;
        }
    }
}

void LPTIM1_IRQHandler() {
    if (LPTIM1->ISR & LPTIM_ISR_ARRM)
    {
        // Clear the interrupt flag
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF;
        start_i++;

        // Ensure that the counter does not go out of bounds, and resets smoothly.
        // Note: UINT8_MAX * 60 represents the start_i value at which the mins_lost value overflows and resets to 1.
        // We round down UINT_MAX to the highest multiple of this overflow value to ensure that the timer resets without breaking the pattern.
        if (start_i == (UINT_MAX - (UINT_MAX % (UINT8_MAX * 60)))) {
            start_i = 60;
        }
    }
    if (LPTIM1->ISR & LPTIM_ISR_ARROK){
    	LPTIM1->ICR |= LPTIM_ICR_ARROKCF;
    }
}

void disable_all_peripherals() {

	RCC->CR &= ~(
		RCC_CR_HSION |
		RCC_CR_HSEON
	);

	RCC->AHB1ENR &= ~(
		RCC_AHB1ENR_DMA1EN |
		RCC_AHB1ENR_DMA2EN |
		RCC_AHB1ENR_FLASHEN |
		RCC_AHB1ENR_CRCEN |
		RCC_APB1ENR1_DAC1EN
	);

	RCC->AHB2ENR &= ~(
		RCC_AHB2ENR_GPIOAEN |
		RCC_AHB2ENR_GPIOBEN |
		RCC_AHB2ENR_GPIOCEN |
		RCC_AHB2ENR_GPIODEN |
		RCC_AHB2ENR_GPIOEEN |
		RCC_AHB2ENR_GPIOFEN |
		RCC_AHB2ENR_GPIOGEN |
		RCC_AHB2ENR_GPIOHEN |
		RCC_AHB2ENR_ADCEN
	);

	RCC->APB1ENR1 &= ~(
		RCC_APB1ENR1_TIM2EN |
		RCC_APB1ENR1_TIM3EN |
		RCC_APB1ENR1_TIM4EN |
		RCC_APB1ENR1_TIM5EN |
		RCC_APB1ENR1_TIM6EN |
		RCC_APB1ENR1_TIM7EN |
		RCC_APB1ENR1_SPI2EN |
		RCC_APB1ENR1_SPI3EN |
		RCC_APB1ENR1_USART2EN |
		RCC_APB1ENR1_I2C1EN |
		RCC_APB1ENR1_I2C2EN |
		RCC_APB1ENR1_I2C3EN
	);

	RCC->APB2ENR &= ~(
		RCC_APB2ENR_SYSCFGEN |
		RCC_APB2ENR_SPI1EN |
		RCC_APB2ENR_USART1EN |
		RCC_APB2ENR_TIM1EN |
		RCC_APB2ENR_TIM8EN
	);

	// Disable USB OTG FS and HS (if applicable)
	RCC->AHB2ENR &= ~RCC_AHB2ENR_OTGFSEN;  // Disable USB OTG FS clock

	// Reset USB peripheral
	RCC->AHB2RSTR |= RCC_AHB2RSTR_OTGFSRST;  // Reset USB OTG FS
	RCC->AHB2RSTR &= ~RCC_AHB2RSTR_OTGFSRST; // Release reset

	// Optional: Set USB pins to Analog mode (if USB is not needed)
	GPIOA->MODER |= (3 << (2 * 11)); // PA11 (USB_DM) as Analog
	GPIOA->MODER |= (3 << (2 * 12)); // PA12 (USB_DP) as Analog
}

void enable_some_peripherals() {
	RCC->AHB2ENR |= (
		RCC_AHB2ENR_GPIOAEN |
		RCC_AHB2ENR_GPIOBEN |
		RCC_AHB2ENR_GPIOCEN |
		RCC_AHB2ENR_GPIODEN |
		RCC_AHB2ENR_GPIOEEN
	);
	RCC->APB1ENR1 |= (
		RCC_APB1ENR1_TIM6EN |
		RCC_APB1ENR1_SPI3EN |
		RCC_APB1ENR1_I2C2EN
	);
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config(7);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  /* Initialize Timer, I2C, and Accelerometer*/
//  timer_init(TIM2, 8);
//  timer_set_ms(TIM2, 1000, 8);
  lptimer_init();
  i2c_init();
  lsm6dsl_init();


  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  ble_init();

  HAL_Delay(10);

  uint8_t nonDiscoverable = 1;

  // Poll the accelerometer using the last known values
  int16_t last_x = 0, last_y = 0, last_z = 0;
  lsm6dsl_read_xyz(&last_x, &last_y, &last_z);
  int16_t data_x = 0, data_y = 0, data_z = 0;
  int secs_lost = 0;
  // lost = 0 when not lost, lost = 1 when lost
  uint8_t lost = 0;
  setDiscoverability(0);
  RCC->CFGR &= ~RCC_CFGR_STOPWUCK;
  while (1)
  {
      RCC->CFGR &= ~RCC_CFGR_STOPWUCK;
	  enable_some_peripherals();
	  HAL_ResumeTick();

	  if ((lost == 0 && start_i % 10 == 0) || (lost == 1)) {
		  lsm6dsl_read_xyz(&data_x, &data_y, &data_z);
		  // Threshold chosen as 0.1g based on the uncertainty and values given.
	      int threshold = 1639;

	      // Loop for if moving
		  if ((abs(last_x - data_x) > threshold || abs(last_y - data_y) > threshold || abs(last_z - data_z) > threshold)) {
			  // Reset timer since last movement.
			  start_i = 0;
			  lost = 0;
			  disconnectBLE();
			  if (!nonDiscoverable) {
				  setDiscoverability(0);
				  nonDiscoverable = 1;
			  }
		  }

		  last_x = data_x;
		  last_y = data_y;
		  last_z = data_z;
	  }

	  if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
		catchBLE();
	  } else if (nonDiscoverable) {
		standbyBle();
	  }

	  // Start beaconing at 1 min
	  if (start_i >= 60) {
		  lost = 1;
		  if (nonDiscoverable) {
			  setDiscoverability(1);
			  nonDiscoverable = 0;
		  }
		  if (start_i % 10 == 0) {
			  secs_lost = start_i;
			  HAL_Delay(1000);
			  unsigned char lost_str[] = "ArayTag missing for ";
			  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(lost_str), lost_str);
			  unsigned char time_str[20];  // Ensure enough space for the formatted string
			  sprintf(time_str, "%d s", secs_lost);
			  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(time_str), time_str);
		  }
	  }
	  HAL_SuspendTick();
	  // Wait for interrupt, only uncomment if low power is needed
	  disable_all_peripherals();
	  PWR->CR1 |= PWR_CR1_LPMS_STOP2;
	  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	  __WFI();
  }
}

/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(int MSI_Val)
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  if (MSI_Val == 7) {
	  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  } else {
	  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  }
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
