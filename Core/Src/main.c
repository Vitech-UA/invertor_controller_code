/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "invertor.h"

#include "utility.h"
#include "energy_monitor.h"
#include "st7789.h"
//#include "st7789.h"
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
void config_eg8010(void);
uint16_t update_moving_average(uint16_t *buffer, uint32_t *sum, uint8_t *index,
		uint16_t new_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MA_WINDOW_SIZE 16  // розмір ковзного вікна

uint16_t raw_adc_values[3] = { 0 };  // CH0, VBAT (CH4), IBAT (CH6)
uint8_t adc_cplt_flag = 0;

// Для VBAT (канал 4)
uint16_t vbat_buffer[MA_WINDOW_SIZE] = { 0 };
uint32_t vbat_sum = 0;
uint8_t vbat_index = 0;

// Для IBAT (канал 6)
uint16_t ibat_buffer[MA_WINDOW_SIZE] = { 0 };
uint32_t ibat_sum = 0;
uint8_t ibat_index = 0;

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
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	ST7789_Init();
	init_gpio_expander();
	i2c_scan_bus();
	config_eg8010();
	set_12V(true);
	set_bridge_power(true);
	//set_eg_pwm(true);
	set_energy_monitor_pwr(true);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &raw_adc_values, 3);

	float voltage = 0.0;
	float current = 0.0;
	float power = 0.0;
	float pf = 0.0;
	float freq = 0.0;
	float energy = 0.0;
	bool alrm = 0;
	PZEM004Tv30_init(0xF8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		if (adc_cplt_flag) {
//			HAL_ADC_Stop_DMA(&hadc1);
//			adc_cplt_flag = 0;
//
//			uint16_t vbat_avg = update_moving_average(vbat_buffer, &vbat_sum,
//					&vbat_index, raw_adc_values[1]);
//			uint16_t ibat_avg = update_moving_average(ibat_buffer, &ibat_sum,
//					&ibat_index, raw_adc_values[2]);
//
//			print_vbat(vbat_avg);
//			print_ibat(ibat_avg);
//
//			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &raw_adc_values, 3);
//		}

		voltage = PZEM004Tv30_voltage();
		power = PZEM004Tv30_power();
		current = PZEM004Tv30_current();
		freq = PZEM004Tv30_frequency();
		pf = PZEM004Tv30_pf();
		energy = PZEM004Tv30_energy();


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint16_t update_moving_average(uint16_t *buffer, uint32_t *sum, uint8_t *index,
		uint16_t new_value) {
	*sum -= buffer[*index];           // відняти старе значення
	buffer[*index] = new_value;       // оновити буфер новим значенням
	*sum += new_value;                // додати нове значення до суми

	*index = (*index + 1) % MA_WINDOW_SIZE;  // інкремент індексу по колу

	return (uint16_t) (*sum / MA_WINDOW_SIZE);
}

void config_eg8010(void) {
	// Конфіг DEAD_TIME
	EG_DEAD_TIME_t dead_time_config = EG_DEAD_TIME_1p5_US;
	config_eg_dead_time(dead_time_config);
	// Конфіг частоти 50 Гц
	set_invertor_freq(EG_FREQ_50HZ);
	set_invertor_softstart(true);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (hadc->Instance == ADC1) {
		adc_cplt_flag = 1;
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
