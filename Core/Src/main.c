/**
 * @brief  Main program part that makes the Armuro 1 robot drive through an obstacle course.
 *
 * Makes the Armuro 1 robot from the High Performance Humanoid Technologies (H2T)
 * chair at KIT go through an obstacle course where it first follows a fixed trajectory,
 * then follows a line and crosses a gap and goes around an obstacle.
 *
 * @author Lukas Probst
 */

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sensors.h"
#include "tasks.h"
#include "utility.h"

#include <stdio.h>

volatile uint32_t adc[6];
uint32_t buffer[6];

uint32_t encoder_left_cnt;
uint32_t encoder_right_cnt;

double speed_left;
double speed_right;

Linesensor left_linesensor_state;
Linesensor middle_linesensor_state;
Linesensor right_linesensor_state;

RaceState current_state;

/* Private function prototypes */
void SystemClock_Config(void);

/**
 * @brief  Sets the initial properties.
 *
 * @return None
 */
void setup()
{
	resetEncoderCnt();

	/* The generation of PWM signals must be activated */
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	setNormalSpeed();

	current_state = FOLLOW_TRAJECTORY;
}

/**
  * @brief  The application entry point.
  *
  * @return int
  */
int main(void)
{
  /* MCU Configuration */

  /* Reset of all peripherals, initialises the flash interface and the systick */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialise all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();

  setup();

  while (1)
  {
	  HAL_ADC_Start_DMA(&hadc1, buffer, 6);
	  /* Output of ADC values here */

	  SchmittTrigger();
	  detectColour();

	  /* Check if power switch is activated before starting the robot-routine (prevents driving when it is still connected via USB) */
	  if (BATTERY)
  	  {
		  switch (current_state)
		  {
		  	  case FOLLOW_TRAJECTORY:
				  task_followTrajectory();
				  break;
			  case FOLLOW_LINE:
				  task_followLine();
				  break;
			  case SEARCH_LINE:
				  task_searchLine();
				  break;
			  case AVOID_OBSTACLE:
				  task_avoidObstacle();
				  break;
			  case FINISH_LINE:
				  task_finishLine();
				  break;
		  }
  	  }
  }
}

/**
  * @brief  System Clock Configuration.
  *
  * @return None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Configure LSE Drive Capability */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**
   * Initialises the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* Initialises the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Enable MSI Auto calibration */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief  This function is executed in case of error occurrence.
  *
  * @return None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}
