/**
 * @brief  Utility functions for the rest of the program.
 *
 * @author Lukas Probst
 */

#include <stdio.h>

#include "adc.h"
#include "sensors.h"
#include "driving.h"

/* Default motor speed */
#define NORMAL_SPEED_LEFT  0.5
#define NORMAL_SPEED_RIGHT 0.5
/* The driving speed is scaled so that it lies in the interval [-1, 1], i.e. 1 is the maximum value */
#define MAX_SPEED 1

/* Interval in which an LED blinks */
#define BLINK_INTERVAL 100

uint32_t last_switch_left = 0;
uint32_t last_switch_right = 0;
uint32_t last_switch_tail = 0;

/**
 * @brief  Makes LED2 blink.
 *
 * @return None
 */
void blinkLeftLED()
{
	uint32_t time = HAL_GetTick();
	if (time - last_switch_left >= BLINK_INTERVAL)
	{
		HAL_GPIO_TogglePin(LED_left_GPIO_Port, LED_left_Pin);
		last_switch_left = time;
	}
}

/**
 * @brief  Makes LED3 blink.
 *
 * @return None
 */
void blinkRightLED()
{
	uint32_t time = HAL_GetTick();
	if (time - last_switch_right >= BLINK_INTERVAL)
	{
		HAL_GPIO_TogglePin(LED_right_GPIO_Port, LED_right_Pin);
		last_switch_right = time;
	}
}

/**
 * @brief  Makes LED5 & LED6 (SMD LEDs) blink.
 *
 * @return None
 */
void blinkTailLight()
{
	uint32_t time = HAL_GetTick();
	if (time - last_switch_tail >= BLINK_INTERVAL)
	{
		HAL_GPIO_TogglePin(LED_SMD_GPIO_Port, LED_SMD_Pin);
		last_switch_tail = time;
	}
}

/**
 * @brief  Makes all LEDs blink.
 *
 * @return None
 */
void blinkAllLEDs()
{
	blinkLeftLED();
	blinkRightLED();
	blinkTailLight();
}

/**
 * @brief  Sets the normal driving speed.
 *
 * @return none
 */
void setNormalSpeed()
{
	speed_left = NORMAL_SPEED_LEFT;
	speed_right = NORMAL_SPEED_RIGHT;
}

/**
 * @brief  Sets the maximum driving speed.
 *
 * @return none
 */
void setMaxSpeed()
{
	speed_left = MAX_SPEED;
	speed_right = MAX_SPEED;
}

/**
 * @brief  Resets the encoder count (number of ticks).
 *
 * @return none
 */
void resetEncoderCnt()
{
	encoder_left_cnt = 0;
	encoder_right_cnt = 0;
}
