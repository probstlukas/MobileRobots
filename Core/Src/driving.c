/**
 * @brief  Driving functions of the robot.
 *
 * @author Lukas Probst
 */

#include <math.h>

#include "adc.h"
#include "utility.h"
#include "sensors.h"
#include "driving.h"

/* Maximum motor speed or rather PWM-value of the robot */
#define MAX_PWM (int) (pow(2, 16) - 1)

/**
 * @brief  Sets the robot in motion by specifying a value for both wheels that
 * 		   lies in the interval [-1, 1].
 *
 * @param  speed_left controls how fast and in which direction the left wheel turns
 * @param  speed_right controls how fast and in which direction the right wheel turns
 * @return None
 */
void drive(double speed_left, double speed_right)
{
	/* Left control */
	if(speed_left > 0)
	{
		TIM1->CCR2 = (int) (MAX_PWM * speed_left);
		HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, GPIO_PIN_RESET);
		blinkRightLED();
	}
	else if (speed_left < 0)
	{
		TIM1->CCR2 = (int) (MAX_PWM - MAX_PWM * speed_left);
		HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, GPIO_PIN_SET);
		blinkLeftLED();
	}
	else if (speed_left == 0)
	{
		TIM1->CCR2 = 0;
		/* Necessary in case robot was driving backwards before */
		HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, GPIO_PIN_RESET);
	}

	/* Right control */
	if(speed_right > 0)
	{
		TIM1->CCR3 = (int) (MAX_PWM * speed_right);
		HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, GPIO_PIN_RESET);
		blinkLeftLED();
	}
	else if (speed_right < 0)
	{
		TIM1->CCR3 = (int) (MAX_PWM - MAX_PWM * speed_right);
		HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, GPIO_PIN_SET);
		blinkRightLED();
	}
	else if (speed_right == 0)
	{
		TIM1->CCR3 = 0;
		/* Necessary in case robot was driving backwards before */
		HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, GPIO_PIN_RESET);
	}
}

/**
 * @brief  The two wheels of the robot do not always turn at the same speed. This function
 * 		   makes sure that the robot follows a straight line.
 *
 * The reasons for this are scatter in the manufacture of the motors or dirty gears. Therefore, the
 * wheels do not rotate at the same speed with the same control. In order to follow a defined path
 * (e.g. a straight line), the actual wheel rotations must be observed and the motor control adjusted
 * accordingly. The encoders and a regulated control of the motors serve this purpose.
 * This is done by a P-controller, which controls the actuator with a value that is proportional to
 * the deviation of the setpoint from the actual value. To make the robot drive straight, a position
 * controller is suitable, which looks at the position of both wheels at the same time.
 *
 * @return None
 */
void driveForward()
{
	float proportional_gain = 0.15f;
	int32_t error = encoder_left_cnt - encoder_right_cnt;
	double delta_speed_left = error * proportional_gain * speed_left;
	double delta_speed_right = error * proportional_gain * speed_right;

	drive(speed_left - delta_speed_left, speed_right + delta_speed_right);
}
