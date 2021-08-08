/**
 * @brief  All functions regarding the sensors of the robot.
 *
 * @author Lukas Probst
 */

#include <stdio.h>

#include "usart.h"
#include "sensors.h"

/* Schmitt trigger thresholds for the wheel encoders */
#define LEFT_HIGH_THRESHOLD  2500
#define LEFT_LOW_THRESHOLD   1000
#define RIGHT_HIGH_THRESHOLD 2750
#define RIGHT_LOW_THRESHOLD  1000

/* Threshold to detect black with the brightness sensors */
#define BLACK_LOW_THRESHOLD 2500

typedef enum {LOW, HIGH} Threshold;
Threshold threshold_left_state;
Threshold threshold_right_state;

/**
 * @brief  Sends real-time data of the sensors over a UART interface of the
 * 	    microcontroller to the computer via USB.
 * @return None
 */
void outputSensor()
{
	char string_buf[100];
	uint32_t len = sprintf((char*) string_buf, "%lu,%lu,%lu,%lu,%lu,%lu\n", adc[0], adc[1], adc[2], adc[3], adc[4], adc[5]);
	HAL_UART_Transmit(&huart2,(uint8_t*) string_buf, len, 1000000);;
}

/**
 * @brief  When the conversion is complete, this function is called, which must be in the main function.
 *
 * @param  hadc1 ADC handle structure
 * @return None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	for (int i = 0; i < 6; i++)
	{
		adc[i] = buffer[i];
	}
	outputSensor();
}

/**
 * @brief  Converts the encoder values into digital signals with Schmitt trigger.
 *
 * Threshold values for "low" (white) and "high" (black) are used to detect the
 * current encoder signals.
 *
 * @return None
 */
void SchmittTrigger()
{
	/* Schmitt trigger for the left encoder */
	if (ENCODER_LEFT >= LEFT_HIGH_THRESHOLD && threshold_left_state == LOW)
	{
		encoder_left_cnt++;
		threshold_left_state = HIGH;
	}
	if (ENCODER_LEFT <= LEFT_LOW_THRESHOLD && threshold_left_state == HIGH)
	{
		encoder_left_cnt++;
		threshold_left_state = LOW;
	}

	/* Schmitt trigger for the right encoder */
	if (ENCODER_RIGHT >= RIGHT_HIGH_THRESHOLD && threshold_right_state == LOW)
	{
		encoder_right_cnt++;
		threshold_right_state = HIGH;
	}
	if (ENCODER_RIGHT <= RIGHT_LOW_THRESHOLD && threshold_right_state == HIGH)
	{
		encoder_right_cnt++;
		threshold_right_state = LOW;
	}
}

/**
 * @brief  Detects the colour of the three brightness sensors.
 *
 * @return None
 */
void detectColour()
{
	  if (LINESENSOR_LEFT > BLACK_LOW_THRESHOLD)
	  {
		  left_linesensor_state = BLACK;
	  }
	  else
	  {
		  left_linesensor_state = WHITE;
	  }

	  if (LINESENSOR_MIDDLE > BLACK_LOW_THRESHOLD)
	  {
		  middle_linesensor_state = BLACK;
	  }
	  else
	  {
		  middle_linesensor_state = WHITE;
	  }

	  if (LINESENSOR_RIGHT > BLACK_LOW_THRESHOLD)
	  {
		  right_linesensor_state = BLACK;
	  }
	  else
	  {
		  right_linesensor_state = WHITE;
	  }
}
